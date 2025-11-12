#include "can_bsp.h"
#include "fdcan.h"
#include "fdcan1_task.h"
#include "motor_config.h"
#include "string.h"
#if defined(osCMSIS) && (osCMSIS >= 0x20000U)
#include "cmsis_os2.h"
#endif
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;
void FDCAN1_Config(void) {
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; //
  sFilterConfig.FilterID2 = 0x00000000; //
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }
  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */

  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
    Error_Handler();
  }
}

void FDCAN2_Config(void) {
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

static void fdcan_tx_delay(uint32_t delay_ms) {
#if defined(osCMSIS) && (osCMSIS >= 0x20000U)
  if (osKernelGetState() == osKernelRunning) {
    osDelay(delay_ms);
    return;
  }
#endif
  HAL_Delay(delay_ms);
}

#define FDCAN_TX_MAX_ATTEMPTS 100U
#define FDCAN_TX_RETRY_DELAY_MS 1U

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data,
                       uint32_t len) {
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.Identifier = id; // CAN ID
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  if (len <= 8) {
    TxHeader.DataLength = len;
  } else if (len == 12) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  } else if (len == 16) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;

  } else if (len == 20) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_20;
  } else if (len == 24) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
  } else if (len == 48) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_48;
  } else if (len == 64) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  }

  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN; // CANFD
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  for (uint32_t attempt = 0; attempt < FDCAN_TX_MAX_ATTEMPTS; ++attempt) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0U) {
      fdcan_tx_delay(FDCAN_TX_RETRY_DELAY_MS);
      continue;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) == HAL_OK) {
      return 0;
    }

    fdcan_tx_delay(FDCAN_TX_RETRY_DELAY_MS);
  }

  return 1;
}

int64_t mybuff[7] = {0};
int64_t mybuff3[9] = {0};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (hfdcan->Instance == FDCAN1) {
      /* Retrieve Rx messages from RX FIFO0 */
      memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);

      switch (RxHeader1.Identifier) {
      case 0x1F:
        dm4310_fbdata(&fdcan1_bus.motor[14], g_Can1RxData,
                      RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x1E:
        dm4310_fbdata(&fdcan1_bus.motor[13], g_Can1RxData,
                      RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x1D:
        dm4310_fbdata(&fdcan1_bus.motor[12], g_Can1RxData,
                      RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x1C:
        dm4310_fbdata(&fdcan1_bus.motor[11], g_Can1RxData,
                      RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x1B:
        dm4310_fbdata(&fdcan1_bus.motor[10], g_Can1RxData,
                      RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x1A:
        dm4310_fbdata(&fdcan1_bus.motor[9], g_Can1RxData, RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x19:
        dm4310_fbdata(&fdcan1_bus.motor[8], g_Can1RxData, RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x18:
        dm4310_fbdata(&fdcan1_bus.motor[7], g_Can1RxData, RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x17:
        dm4310_fbdata(&fdcan1_bus.motor[6], g_Can1RxData, RxHeader1.DataLength);
        mybuff[0]++;
        break;
      case 0x16:
        dm4310_fbdata(&fdcan1_bus.motor[5], g_Can1RxData, RxHeader1.DataLength);
        mybuff[1]++;
        break;
      case 0x15:
        dm4310_fbdata(&fdcan1_bus.motor[4], g_Can1RxData, RxHeader1.DataLength);
        mybuff[2]++;
        break;
      case 0x14:
        dm4310_fbdata(&fdcan1_bus.motor[3], g_Can1RxData, RxHeader1.DataLength);
        mybuff[3]++;
        break;
      case 0x13:
        dm4310_fbdata(&fdcan1_bus.motor[2], g_Can1RxData, RxHeader1.DataLength);
        mybuff[4]++;
        break;
      case 0x12:
        dm4310_fbdata(&fdcan1_bus.motor[1], g_Can1RxData, RxHeader1.DataLength);
        mybuff[5]++;
        break;
      case 0x11:
        dm4310_fbdata(&fdcan1_bus.motor[0], g_Can1RxData, RxHeader1.DataLength);
        mybuff[6]++;
        break;

      default:
        break;
      }
    }
  }
}
int64_t mybuff2[7] = {0};
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
    if (hfdcan->Instance == FDCAN2) {
      while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
        memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2,
                                   g_Can2RxData) != HAL_OK) {
          break; // 出错直接退出，避免死循环
        }

        switch (RxHeader2.Identifier) {
        case 0x1F:
          dm4310_fbdata(&fdcan2_bus.motor[14], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x1E:
          dm4310_fbdata(&fdcan2_bus.motor[13], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x1D:
          dm4310_fbdata(&fdcan2_bus.motor[12], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x1C:
          dm4310_fbdata(&fdcan2_bus.motor[11], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x1B:
          dm4310_fbdata(&fdcan2_bus.motor[10], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x1A:
          dm4310_fbdata(&fdcan2_bus.motor[9], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x19:
          dm4310_fbdata(&fdcan2_bus.motor[8], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x18:
          dm4310_fbdata(&fdcan2_bus.motor[7], g_Can1RxData,
                        RxHeader1.DataLength);
          mybuff[0]++;
          break;
        case 0x17:
          dm4340_fbdata(&fdcan2_bus.motor[6], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[0]++;
          break;
        case 0x16:
          dm4340_fbdata(&fdcan2_bus.motor[5], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[1]++;
          break;
        case 0x15:
          dm4340_fbdata(&fdcan2_bus.motor[4], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[2]++;
          break;
        case 0x14:
          dm4340_fbdata(&fdcan2_bus.motor[3], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[3]++;
          break;
        case 0x13:
          dm4340_fbdata(&fdcan2_bus.motor[2], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[4]++;
          break;
        case 0x12:
          dm4340_fbdata(&fdcan2_bus.motor[1], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[5]++;
          break;
        case 0x11:
          dm4340_fbdata(&fdcan2_bus.motor[0], g_Can2RxData,
                        RxHeader2.DataLength);
          mybuff2[6]++;
          break;
        default:
          break;
        }
      }
    }
  }
}

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud) {
  uint32_t nom_brp = 0, nom_seg1 = 0, nom_seg2 = 0, nom_sjw = 0;
  uint32_t dat_brp = 0, dat_seg1 = 0, dat_seg2 = 0, dat_sjw = 0;

  /*	nominal_baud = 80M/brp/(1+seg1+seg2)
          sample point = (1+seg1)/(1+seg1+seg2)
          sjw :	1-128
          seg1:	2-256
          seg2: 	2-128
          brp :	1-512  */
  if (mode == CAN_CLASS) {
    switch (baud) {
    case CAN_BR_125K:
      nom_brp = 4;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_200K:
      nom_brp = 2;
      nom_seg1 = 174;
      nom_seg2 = 25;
      nom_sjw = 25;
      break; // sample point 87.5%
    case CAN_BR_250K:
      nom_brp = 2;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_500K:
      nom_brp = 1;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_1M:
      nom_brp = 1;
      nom_seg1 = 59;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 75%
    }
    dat_brp = 1;
    dat_seg1 = 29;
    dat_seg2 = 10;
    dat_sjw = 10;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  }
  /*	data_baud	 = 80M/brp/(1+seg1+seg2)
          sample point = (1+seg1)/(1+seg1+seg2)
          sjw :	1-16
          seg1:	1-32
          seg2: 	2-16
          brp :	1-32  */
  if (mode == CAN_FD_BRS) {
    switch (baud) {
    case CAN_BR_2M:
      dat_brp = 1;
      dat_seg1 = 29;
      dat_seg2 = 10;
      dat_sjw = 10;
      break; // sample point 75%
    case CAN_BR_2M5:
      dat_brp = 1;
      dat_seg1 = 25;
      dat_seg2 = 6;
      dat_sjw = 6;
      break; // sample point 81.25%
    case CAN_BR_3M2:
      dat_brp = 1;
      dat_seg1 = 19;
      dat_seg2 = 5;
      dat_sjw = 5;
      break; // sample point 80%
    case CAN_BR_4M:
      dat_brp = 1;
      dat_seg1 = 14;
      dat_seg2 = 5;
      dat_sjw = 5;
      break; // sample point 75%
    case CAN_BR_5M:
      dat_brp = 1;
      dat_seg1 = 13;
      dat_seg2 = 2;
      dat_sjw = 2;
      break; // sample point 87.5%
    }
    nom_brp = 1;
    nom_seg1 = 59;
    nom_seg2 = 20;
    nom_sjw = 20;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  }

  HAL_FDCAN_DeInit(hfdcan);

  hfdcan->Init.NominalPrescaler = nom_brp;
  hfdcan->Init.NominalTimeSeg1 = nom_seg1;
  hfdcan->Init.NominalTimeSeg2 = nom_seg2;
  hfdcan->Init.NominalSyncJumpWidth = nom_sjw;

  hfdcan->Init.DataPrescaler = dat_brp;
  hfdcan->Init.DataTimeSeg1 = dat_seg1;
  hfdcan->Init.DataTimeSeg2 = dat_seg2;
  hfdcan->Init.DataSyncJumpWidth = dat_sjw;

  HAL_FDCAN_Init(hfdcan);
}
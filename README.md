# Ludan Control Board - STM32H7 机器人控制板

## 📋 项目概述

本项目是一个基于STM32H723VGT6的高性能机器人控制板固件，专为多关节机器人系统设计。支持通过CAN总线控制多个电机，集成IMU传感器进行姿态感知，并提供了丰富的控制算法库。

**主要特性：**
- 支持双CAN总线，每条总线最多控制16个电机
- 集成BMI088 IMU传感器（加速度计+陀螺仪）
- 多种控制算法（PID、前馈、扰动观测器等）
- FreeRTOS实时操作系统
- USB CDC调试接口
- 实时电压监测与保护

---

## 🔧 硬件平台

### 主控芯片
- **型号**: STM32H723VGT6
- **内核**: ARM Cortex-M7
- **主频**: 480MHz
- **浮点单元**: FPU (fpv5-d16, hard float ABI)
- **Flash**: 1MB
- **RAM**: 
  - DTCM: 128KB
  - AXI SRAM: 320KB
  - SRAM1/2/3/4: 128KB
  - SRAM D2: 32KB
  - SRAM D3: 16KB

### 外部时钟
- **HSE**: 25MHz（外部高速晶振）
- **LSE**: 32.768KHz（外部低速晶振，用于RTC）

### 外设配置

#### CAN总线
- **FDCAN1**: 1Mbps，FD模式，BRS使能，用于电机控制总线1
- **FDCAN2**: 1Mbps，FD模式，BRS使能，用于电机控制总线2
- **FDCAN3**: 1Mbps，FD模式，BRS使能，预留扩展

#### 通信接口
- **USB OTG HS**: USB Device模式，CDC类，用于调试和数据传输
- **USART1**: 串口通信，波特率可配置，支持DMA
- **SPI2**: 用于BMI088 IMU传感器通信，支持DMA

#### 其他外设
- **ADC1**: 2通道，DMA循环模式
  - 通道0: ADC1_INP19 (PA5) - 电压检测
  - 通道1: ADC1_INP4 (PC4) - 预留
- **TIM3**: PWM输出，通道4 (PB1)
- **TIM12**: PWM输出，通道2 (PB15)
- **GPIO**: 
  - 电源控制输出
  - 蜂鸣器控制
  - 传感器片选信号（ACC_CS, GYRO_CS）
  - 传感器中断信号（ACC_INT, GYRO_INT）

---

## 💻 软件架构

### 构建系统
- **主构建系统**: CMake 3.22+
- **备用构建系统**: Makefile
- **编译器**: arm-none-eabi-gcc
- **C标准**: C11
- **C++标准**: C++17

### 实时操作系统
- **RTOS**: FreeRTOS (CMSIS-RTOS V2接口)
- **堆大小**: 30KB
- **FPU支持**: 已启用
- **任务调度**: 抢占式调度

### 任务结构

| 任务名称 | 优先级 | 堆栈大小 | 功能描述 |
|---------|--------|---------|---------|
| defaultTask | Normal | 512×4 bytes | USB设备初始化 |
| FDCAN1_TASK | High | 512×4 bytes | CAN1总线电机控制 |
| FDCAN2_TASK | High | 512×4 bytes | CAN2总线电机控制 |
| OBSERVE_TASK | High | 512×4 bytes | 数据观测与发送 |
| VBUS_CHECK_TASK | Normal | 512×4 bytes | 电压检测与保护 |

---

## 🏗️ 项目结构

```
ludan_control_board/
├── Core/                          # STM32CubeMX生成的HAL驱动代码
│   ├── Inc/                       # 头文件
│   │   ├── main.h                 # 主头文件，包含GPIO定义
│   │   ├── stm32h7xx_hal_conf.h   # HAL库配置
│   │   ├── FreeRTOSConfig.h       # FreeRTOS配置
│   │   └── ...                    # 其他外设头文件
│   └── Src/                       # 源文件
│       ├── main.c                 # 主程序入口
│       ├── freertos.c             # FreeRTOS任务定义
│       └── ...                    # 其他外设源文件
│
├── Drivers/                       # STM32驱动库
│   ├── CMSIS/                     # CMSIS核心文件
│   └── STM32H7xx_HAL_Driver/      # HAL驱动库
│
├── Middlewares/                   # 中间件
│   ├── ST/                        # ST官方中间件
│   │   └── STM32_USB_Device_Library/  # USB设备库
│   └── Third_Party/               # 第三方库
│       └── FreeRTOS/               # FreeRTOS源码
│
├── USB_DEVICE/                    # USB设备配置
│   ├── App/                       # USB应用层
│   └── Target/                    # USB目标配置
│
├── User/                          # 用户代码
│   ├── App/                       # 应用任务
│   │   ├── fdcan_bus.c/h          # CAN总线管理
│   │   ├── fdcan1_task.c/h        # CAN1任务
│   │   ├── fdcan2_task.c/h        # CAN2任务
│   │   ├── observe_task.c/h       # 观测任务
│   │   └── vbus_check.c/h         # 电压检测任务
│   │
│   ├── Bsp/                       # 板级支持包
│   │   ├── bsp_dwt.c/h            # DWT计时器
│   │   ├── bsp_PWM.c/h            # PWM控制
│   │   ├── bsp_usart1.c/h         # 串口1驱动
│   │   └── can_bsp.c/h            # CAN总线BSP
│   │
│   ├── Controller/                # 控制器算法
│   │   ├── controller.c/h         # 控制器接口
│   │   └── ...                    # PID、前馈等算法
│   │
│   ├── Algorithm/                 # 滤波和估计算法
│   │   ├── EKF/                   # 扩展卡尔曼滤波
│   │   ├── kalman/                # 卡尔曼滤波
│   │   ├── mahony/                # Mahony滤波
│   │   ├── PID/                   # PID算法
│   │   └── VMC/                   # VMC计算
│   │
│   ├── Devices/                   # 设备驱动
│   │   ├── BMI088/                # BMI088 IMU驱动
│   │   └── DM_Motor/              # DM电机驱动
│   │
│   └── Lib/                       # 工具库
│       └── user_lib.c/h           # 通用工具函数
│
├── cmake/                         # CMake配置
│   └── stm32cubemx/               # STM32CubeMX CMake配置
│
├── build/                         # 构建输出目录
│   └── Debug/                     # Debug版本输出
│
├── CMakeLists.txt                 # CMake主配置文件
├── Makefile                       # Makefile构建脚本
├── CMakePresets.json              # CMake预设配置
├── STM32H723XG_FLASH.ld           # 链接脚本
├── startup_stm32h723xx.s          # 启动文件
├── ludan_control_board.ioc        # STM32CubeMX配置文件
├── flash.jlink                    # J-Link烧录脚本
└── loadbin.bat                    # Windows批处理烧录脚本
```

---

## 🚀 构建流程

### 前置要求

1. **工具链安装**
   ```bash
   # 安装ARM交叉编译工具链
   # Windows: 下载并安装 ARM GNU Toolchain
   # Linux: 
   sudo apt-get install gcc-arm-none-eabi
   # macOS:
   brew install arm-none-eabi-gcc
   ```

2. **CMake安装** (版本 >= 3.22)
   ```bash
   # Windows: 从官网下载安装
   # Linux:
   sudo apt-get install cmake
   # macOS:
   brew install cmake
   ```

3. **可选工具**
   - STM32CubeMX (用于配置外设)
   - J-Link (用于烧录和调试)
   - OpenOCD (替代调试工具)

### 使用CMake构建（推荐）

#### Windows (PowerShell)
```powershell
# 1. 创建构建目录
mkdir build
cd build

# 2. 配置CMake（指定工具链路径，如果需要）
cmake .. -DCMAKE_BUILD_TYPE=Debug

# 3. 编译
cmake --build . --config Debug

# 4. 输出文件位置
# build/Debug/ludan_control_board.elf
# build/Debug/ludan_control_board.bin
# build/Debug/ludan_control_board.hex
```

#### Linux/macOS
```bash
# 1. 创建构建目录
mkdir -p build/Debug
cd build/Debug

# 2. 配置CMake
cmake ../.. -DCMAKE_BUILD_TYPE=Debug

# 3. 编译
make -j$(nproc)  # Linux
# 或
make -j$(sysctl -n hw.ncpu)  # macOS

# 4. 查看编译信息
make VERBOSE=1
```

### 使用Makefile构建

```bash
# 1. 直接编译
make

# 2. 指定调试模式
make DEBUG=1

# 3. 清理构建文件
make clean

# 4. 查看编译信息
make VERBOSE=1
```

### 编译输出说明

编译成功后，在 `build/Debug/` 目录下会生成：
- `ludan_control_board.elf` - ELF格式，用于调试
- `ludan_control_board.bin` - 二进制格式，用于烧录
- `ludan_control_board.hex` - Intel HEX格式，用于烧录
- `ludan_control_board.map` - 内存映射文件，用于分析代码大小

---

## 📥 烧录流程

### 使用J-Link烧录（Windows）

1. **准备J-Link驱动**
   - 安装SEGGER J-Link软件包
   - 连接J-Link调试器到目标板

2. **修改烧录脚本**
   编辑 `flash.jlink` 文件，确认路径正确：
   ```jlink
   device STM32H723VG
   if SWD
   speed 4000
   connect
   erase
   loadbin D:\WR\ludan_control_board\build\Debug\ludan_control_board.bin, 0x08000000
   r
   g
   q
   ```

3. **执行烧录**
   ```cmd
   # 方法1: 使用批处理脚本
   loadbin.bat

   # 方法2: 直接使用J-Link命令行
   JLink.exe -CommanderScript flash.jlink
   ```

### 使用OpenOCD烧录

```bash
# 1. 启动OpenOCD（需要配置文件）
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg

# 2. 在另一个终端使用GDB烧录
arm-none-eabi-gdb build/Debug/ludan_control_board.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset
(gdb) continue
```

### 使用STM32CubeProgrammer

1. 打开STM32CubeProgrammer
2. 选择连接方式（ST-Link/SWD）
3. 点击"Connect"
4. 选择"Open file"，加载 `ludan_control_board.bin` 或 `.hex`
5. 设置起始地址：`0x08000000`
6. 点击"Download"

---

## 🔄 系统启动流程

### 1. 硬件初始化阶段

```
系统复位
  ↓
启动文件 (startup_stm32h723xx.s)
  ↓
SystemInit() - 系统时钟配置
  ↓
main() 函数
```

### 2. HAL库初始化

```c
HAL_Init()                    // HAL库初始化
  ↓
SystemClock_Config()          // 系统时钟配置（480MHz）
  ↓
PeriphCommonClock_Config()    // 外设时钟配置
```

### 3. 外设初始化

```c
MX_GPIO_Init()                // GPIO初始化
MX_DMA_Init()                 // DMA初始化
MX_SPI2_Init()                // SPI2初始化（IMU）
MX_TIM3_Init()                // TIM3初始化（PWM）
MX_FDCAN3_Init()              // FDCAN3初始化
MX_USART1_UART_Init()         // USART1初始化
MX_FDCAN1_Init()              // FDCAN1初始化
MX_FDCAN2_Init()              // FDCAN2初始化
MX_ADC1_Init()                // ADC1初始化
MX_TIM12_Init()               // TIM12初始化（PWM）
```

### 4. 用户初始化

```c
Buzzer_OFF()                  // 关闭蜂鸣器
DWT_Init(480)                 // 初始化DWT计时器（480MHz）
HAL_TIM_Base_Start_IT(&htim3) // 启动TIM3中断
Power_OUT1_ON()               // 打开电源输出1
Power_OUT2_ON()               // 打开电源输出2
FDCAN1_Config()               // 配置FDCAN1
FDCAN2_Config()               // 配置FDCAN2
```

### 5. FreeRTOS启动

```c
osKernelInitialize()         // 初始化FreeRTOS内核
MX_FREERTOS_Init()            // 创建任务
  ├── defaultTask            // USB初始化任务
  ├── FDCAN1_TASK            // CAN1电机控制任务
  ├── FDCAN2_TASK            // CAN2电机控制任务
  ├── OBSERVE_TASK           // 观测任务
  └── VBUS_CHECK_TASK        // 电压检测任务
osKernelStart()              // 启动任务调度
```

---

## 📊 任务详细说明

### FDCAN1_TASK / FDCAN2_TASK

**功能**: 管理对应CAN总线上的电机控制

**执行流程**:
```
1. 延迟500ms（等待系统稳定）
   ↓
2. 初始化CAN总线 (fdcan1_init/fdcan2_init)
   ├── 初始化所有电机参数
   ├── 逐个使能电机（最多重试20次）
   │   ├── 发送使能命令
   │   ├── 等待25ms
   │   └── 检查反馈（enabled或state标志）
   └── 设置start_flag = 1
   ↓
3. 初始化电机反馈数据结构
   ├── dm6248p_fbdata_init (电机0-2)
   └── dm4340_fbdata_init (电机3-15)
   ↓
4. 循环执行（周期由CHASSR_TIME/CHASSL_TIME控制）
   └── 遍历所有电机，发送控制命令
       └── mit_ctrl_test() - MIT模式控制
```

**关键参数**:
- `MOTOR_ENABLE_MAX_RETRY`: 20（最大重试次数）
- `MOTOR_ENABLE_INTERVAL_MS`: 25ms（重试间隔）
- `CHASSR_TIME`: 1ms（FDCAN1控制周期）
- `CHASSL_TIME`: 1ms（FDCAN2控制周期）

### OBSERVE_TASK

**功能**: 收集所有电机数据并通过USB/串口发送

**执行流程**:
```
1. 构建数据帧
   ├── 帧头: FRAME_HEADER
   ├── FDCAN1总线电机数据（位置、速度、力矩）
   └── FDCAN2总线电机数据（位置、速度、力矩）
   ↓
2. 数据打包（每电机5字节）
   ├── 位置: 2字节 (p_int)
   ├── 速度: 1.5字节 (v_int，压缩编码)
   └── 力矩: 1.5字节 (t_int，压缩编码)
   ↓
3. 发送数据
   └── 通过USB CDC或串口发送
   ↓
4. 延迟等待（周期由OBSERVE_TIME控制）
```

**关键参数**:
- `FRAME_LENGTH`: 152字节（总帧长度）
- `LOAD_LENGTH`: 151字节（数据长度）
- `OBSERVE_TIME`: 1ms（640Hz采样率）

**数据格式**:
```
[帧头][电机1数据][电机2数据]...[电机N数据]
每电机5字节: [P_H][P_L][V_H|T_H][V_L|T_L][T_L]
```

### VBUS_CHECK_TASK

**功能**: 实时监测系统电压，提供低电压保护

**执行流程**:
```
1. ADC校准
   └── HAL_ADCEx_Calibration_Start()
   ↓
2. 启动DMA循环采集
   └── HAL_ADC_Start_DMA() - 连续采集2通道
   ↓
3. 循环检测
   ├── 读取ADC值并转换为电压
   │   └── vbus = (adc_val + calibration) * 3.3 / 65535 * 11.0
   ├── 电压判断
   │   ├── 6.0V < vbus < 22.6V: 蜂鸣器报警
   │   └── 6.0V < vbus < 22.2V: 低电压保护
   │       ├── loss_voltage = 1
   │       └── Power_OUT2_OFF() - 关闭部分电源
   └── 延迟等待
```

**关键参数**:
- `calibration_value`: 378（ADC校准值）
- `vbus_threhold_disable`: 22.2V（低电压保护阈值）
- `vbus_threhold_call`: 22.6V（报警阈值）

---

## 🎮 电机控制说明

### 支持的电机型号

| 电机型号 | 位置范围 | 速度范围 | 力矩范围 | 备注 |
|---------|---------|---------|---------|------|
| DM4310 | ±12.5 rad | ±30.0 rad/s | ±10.0 N·m | 小功率关节电机 |
| DM4340 | ±12.5 rad | ±10.0 rad/s | ±28.0 N·m | 中功率关节电机 |
| DM6006 | ±12.5 rad | ±45.0 rad/s | ±12.0 N·m | 高速关节电机 |
| DM8006 | ±12.5 rad | ±45.0 rad/s | ±20.0 N·m | 大功率关节电机 |
| DM3507 | ±12.5 rad | ±50.0 rad/s | ±10.0 N·m | 轮毂电机 |
| DM10010L | ±12.5 rad | ±25.0 rad/s | ±200.0 N·m | 大扭矩电机 |
| DM6248P | ±12.566 rad | ±20.0 rad/s | ±120.0 N·m | 高精度电机 |

### 控制模式

#### MIT模式（混合控制）
```c
mit_ctrl(hcan, motor_id, pos, vel, kp, kd, torq);
```
- **位置控制**: 通过kp参数设置位置刚度
- **速度控制**: 通过kd参数设置速度阻尼
- **力矩控制**: 直接设置目标力矩
- **应用场景**: 力控、柔顺控制

#### 位置模式
```c
pos_speed_ctrl(hcan, motor_id, pos, vel);
```
- **位置控制**: 设置目标位置
- **速度限制**: 限制最大速度
- **应用场景**: 位置伺服

#### 速度模式
```c
speed_ctrl(hcan, motor_id, vel);
```
- **速度控制**: 设置目标速度
- **应用场景**: 速度伺服

### 电机使能流程

```
1. 发送使能命令
   enable_motor_mode(hcan, motor_id, mode);
   ↓
2. 等待反馈（最多20次重试）
   ├── 检查 para.enabled == 1
   └── 检查 para.state == 1
   ↓
3. 使能成功
   └── 开始控制循环
```

---

## 🔬 控制算法库

### PID控制器

**特性**:
- 多种改进选项（位标志组合）
- 积分限幅
- 微分先行
- 梯形积分
- 输出滤波
- 变积分速率
- 微分滤波
- 误差处理

**使用示例**:
```c
PID_t pid;
PID_Init(&pid, max_out, integral_limit, deadband,
         kp, ki, kd, A, B, output_lpf_rc, 
         derivative_lpf_rc, ols_order, improve);
float output = PID_Calculate(&pid, measure, ref);
```

### 前馈控制

**功能**: 根据参考信号预测控制量，提高响应速度

**使用示例**:
```c
Feedforward_t ffc;
Feedforward_Init(&ffc, max_out, c, lpf_rc, 
                 ref_dot_ols_order, ref_ddot_ols_order);
float output = Feedforward_Calculate(&ffc, ref);
```

### 线性扰动观测器 (LDOB)

**功能**: 估计并补偿系统扰动

**使用示例**:
```c
LDOB_t ldob;
LDOB_Init(&ldob, max_d, deadband, c, lpf_rc,
          measure_dot_ols_order, measure_ddot_ols_order);
float disturbance = LDOB_Calculate(&ldob, measure, u);
```

### 跟踪微分器 (TD)

**功能**: 平滑跟踪输入信号并提取微分

**使用示例**:
```c
TD_t td;
TD_Init(&td, r, h0);
float output = TD_Calculate(&td, input);
```

### 滤波算法

- **扩展卡尔曼滤波 (EKF)**: 用于姿态估计
- **卡尔曼滤波**: 通用状态估计
- **Mahony滤波**: 轻量级姿态滤波

---

## 🔍 调试与监控

### USB CDC调试

**功能**: 通过USB虚拟串口进行数据通信

**使用方法**:
1. 连接USB线到USB OTG HS接口
2. 在PC上识别为COM端口
3. 使用串口工具（如PuTTY、串口助手）打开对应COM口
4. 波特率：115200（默认）

**数据格式**: 见OBSERVE_TASK说明

### 串口调试

**接口**: USART1 (PA9-TX, PA10-RX)

**配置**: 
- 波特率：可配置（需查看bsp_usart1.c）
- 数据位：8
- 停止位：1
- 校验位：无

### DWT计时器

**功能**: 提供微秒级高精度计时

**使用示例**:
```c
uint32_t cnt_last = 0;
float dt = DWT_GetDeltaT(&cnt_last);  // 获取时间差（秒）
float timeline = DWT_GetTimeline_ms(); // 获取系统时间（毫秒）
```

---

## ⚙️ 配置说明

### 系统时钟配置

- **系统时钟**: 480MHz
- **AHB时钟**: 240MHz
- **APB1/2/3/4时钟**: 120MHz
- **FDCAN时钟**: 80MHz（来自PLL2）

### FreeRTOS配置

- **堆大小**: 30KB
- **任务最大优先级**: 可配置
- **时间片**: 1个tick
- **FPU支持**: 启用

### CAN总线配置

- **波特率**: 1Mbps（标称）
- **数据波特率**: 可配置（FD模式）
- **帧格式**: CAN FD with BRS
- **过滤器**: 标准过滤器4个，扩展过滤器5个

---

## 🐛 常见问题

### 1. 电机无法使能

**可能原因**:
- CAN总线未正确连接
- 电机ID配置错误
- 电源电压不足
- CAN总线终端电阻未连接

**解决方法**:
- 检查CAN总线物理连接
- 确认电机ID与代码中配置一致
- 检查电源电压（应>22.6V）
- 确认CAN总线两端有120Ω终端电阻

### 2. 编译错误

**可能原因**:
- 工具链路径未配置
- CMake版本过低
- 缺少依赖文件

**解决方法**:
- 检查arm-none-eabi-gcc是否在PATH中
- 升级CMake到3.22或更高版本
- 确认所有源文件完整

### 3. 烧录失败

**可能原因**:
- 调试器连接问题
- 目标板未上电
- 烧录脚本路径错误

**解决方法**:
- 检查SWD连接（SWDIO, SWCLK, GND）
- 确认目标板已上电
- 修改flash.jlink中的文件路径

### 4. USB无法识别

**可能原因**:
- USB线质量问题
- USB驱动未安装
- USB配置错误

**解决方法**:
- 使用数据线（非仅充电线）
- 安装STM32 USB驱动
- 检查USB_DEVICE配置

---

## 📈 性能指标

- **电机控制频率**: 1kHz（每条总线）
- **数据观测频率**: 640Hz（OBSERVE_TIME=1）
- **电压检测频率**: 取决于任务调度
- **CAN总线负载**: <50%（正常情况）
- **CPU使用率**: <70%（正常情况）

---

## 🔧 改进建议

### 1. 代码质量改进

#### 1.1 错误处理机制
**现状**: 部分函数缺少错误返回值检查

**建议**:
- 为所有关键函数添加返回值检查
- 实现统一的错误码定义
- 添加错误日志记录机制
- 实现错误恢复策略

**示例**:
```c
typedef enum {
    ERR_OK = 0,
    ERR_CAN_TIMEOUT,
    ERR_MOTOR_NOT_RESPOND,
    ERR_VOLTAGE_LOW,
    // ...
} error_code_t;

error_code_t motor_enable_with_retry(...) {
    // 实现带错误返回的使能函数
}
```

#### 1.2 代码注释完善
**现状**: 部分函数缺少详细注释

**建议**:
- 为所有公共函数添加Doxygen风格注释
- 说明函数参数、返回值、使用场景
- 添加使用示例
- 标注线程安全性

**示例**:
```c
/**
 * @brief 使能指定电机
 * @param hcan CAN总线句柄
 * @param motor_id 电机ID (1-16)
 * @param mode 控制模式 (MIT_MODE/POS_MODE/SPEED_MODE)
 * @return error_code_t 错误码，ERR_OK表示成功
 * @note 此函数非线程安全，应在CAN任务中调用
 * @example
 *   error_code_t err = enable_motor_mode(&hfdcan1, 1, MIT_MODE);
 */
```

#### 1.3 配置参数集中管理
**现状**: 配置参数分散在各个文件中

**建议**:
- 创建统一的配置文件 `config.h`
- 使用宏定义或结构体管理配置
- 支持运行时配置（通过USB/串口）
- 添加配置验证机制

**示例**:
```c
// config.h
typedef struct {
    struct {
        uint8_t max_retry;
        uint16_t retry_interval_ms;
        uint16_t control_period_ms;
    } motor;
    
    struct {
        float threshold_disable;
        float threshold_alarm;
        uint16_t calibration_value;
    } voltage;
    
    // ...
} system_config_t;

extern const system_config_t g_config;
```

### 2. 功能增强

#### 2.1 电机参数自动识别
**现状**: 电机型号需要手动配置

**建议**:
- 实现电机自动识别机制
- 通过CAN反馈数据判断电机型号
- 自动加载对应参数配置
- 支持热插拔检测

**实现思路**:
```c
typedef enum {
    MOTOR_UNKNOWN = 0,
    MOTOR_DM4310,
    MOTOR_DM4340,
    // ...
} motor_type_t;

motor_type_t identify_motor(uint8_t *feedback_data);
void auto_config_motor(Joint_Motor_t *motor, motor_type_t type);
```

#### 2.2 参数在线调参
**现状**: PID等参数需要重新编译才能修改

**建议**:
- 实现USB/串口参数配置接口
- 支持PID参数实时调整
- 参数保存到Flash（可选）
- 提供参数导入/导出功能

**协议设计**:
```
[命令头][参数ID][参数值][校验和]
例如: SET_PID_KP 1 10.5
```

#### 2.3 数据记录功能
**现状**: 只能实时发送数据，无法记录

**建议**:
- 添加SD卡或Flash数据记录功能
- 支持触发式记录（按键/命令）
- 数据格式：CSV或二进制
- 支持数据回放分析

#### 2.4 安全保护机制
**现状**: 只有电压保护，缺少其他安全机制

**建议**:
- 添加看门狗监控
- 实现电机堵转检测
- 添加温度监控（电机、MCU）
- 实现急停功能
- 添加CAN总线故障检测

**实现示例**:
```c
typedef struct {
    uint32_t watchdog_timeout_ms;
    float motor_temp_max;
    float mcu_temp_max;
    uint32_t can_timeout_ms;
} safety_config_t;

void safety_monitor_task(void *arg);
```

### 3. 架构优化

#### 3.1 模块化设计
**现状**: 部分代码耦合度较高

**建议**:
- 实现更清晰的模块接口
- 使用回调函数解耦
- 实现事件驱动架构
- 添加模块间消息队列

**示例**:
```c
// 事件驱动架构
typedef enum {
    EVENT_MOTOR_ENABLED,
    EVENT_MOTOR_DISABLED,
    EVENT_VOLTAGE_LOW,
    // ...
} system_event_t;

void event_publish(system_event_t event, void *data);
void event_subscribe(system_event_t event, void (*handler)(void *));
```

#### 3.2 状态机设计
**现状**: 系统状态管理不够清晰

**建议**:
- 实现系统状态机
- 定义明确的状态转换
- 添加状态日志
- 支持状态查询

**状态定义**:
```c
typedef enum {
    SYS_STATE_INIT,
    SYS_STATE_READY,
    SYS_STATE_RUNNING,
    SYS_STATE_ERROR,
    SYS_STATE_EMERGENCY_STOP
} system_state_t;
```

#### 3.3 通信协议标准化
**现状**: 数据格式硬编码，不易扩展

**建议**:
- 定义标准通信协议（如自定义协议或ROS消息）
- 实现协议版本管理
- 添加数据校验（CRC）
- 支持多客户端连接

### 4. 性能优化

#### 4.1 CAN总线优化
**现状**: CAN总线可能存在拥塞

**建议**:
- 实现CAN消息优先级队列
- 优化消息发送时机
- 添加总线负载监控
- 实现消息丢失检测

#### 4.2 内存管理优化
**现状**: 使用静态分配，可能浪费内存

**建议**:
- 分析实际内存使用情况
- 优化任务堆栈大小
- 使用内存池管理
- 添加内存使用监控

#### 4.3 实时性优化
**现状**: 部分任务可能影响实时性

**建议**:
- 分析任务执行时间
- 优化关键路径代码
- 使用DMA减少CPU占用
- 实现任务优先级动态调整

### 5. 测试与验证

#### 5.1 单元测试
**建议**:
- 为关键算法添加单元测试
- 使用Unity等测试框架
- 实现自动化测试流程
- 添加代码覆盖率分析

#### 5.2 集成测试
**建议**:
- 实现硬件在环测试
- 添加电机模拟器支持
- 实现自动化测试脚本
- 记录测试数据用于分析

#### 5.3 压力测试
**建议**:
- 测试最大电机数量下的性能
- 测试长时间运行稳定性
- 测试异常情况恢复能力
- 测试极端电压情况

### 6. 文档完善

#### 6.1 API文档
**建议**:
- 使用Doxygen生成API文档
- 添加使用示例
- 说明注意事项
- 标注版本信息

#### 6.2 用户手册
**建议**:
- 编写硬件连接指南
- 添加故障排除指南
- 提供典型应用案例
- 制作视频教程

#### 6.3 开发文档
**建议**:
- 详细架构设计文档
- 算法原理说明
- 通信协议文档
- 版本更新日志

### 7. 工具链改进

#### 7.1 构建脚本优化
**建议**:
- 添加版本号自动生成
- 实现增量编译优化
- 添加编译选项配置
- 支持多配置构建（Debug/Release）

#### 7.2 调试工具
**建议**:
- 集成SEGGER RTT支持
- 添加性能分析工具
- 实现数据可视化工具
- 支持远程调试

#### 7.3 代码质量工具
**建议**:
- 集成静态代码分析（如cppcheck）
- 添加代码格式化工具（如clang-format）
- 实现代码审查检查清单
- 添加依赖管理工具

### 8. 硬件相关改进

#### 8.1 电源管理
**建议**:
- 实现更精细的电源管理
- 支持低功耗模式
- 添加电源状态指示
- 实现软启动功能

#### 8.2 扩展接口
**建议**:
- 预留更多GPIO接口
- 添加I2C接口支持更多传感器
- 预留SPI接口扩展
- 添加ADC通道用于更多监测

---

## 📝 版本历史

### v0.1 (2025-11-18)
- 初始版本
- 支持双CAN总线电机控制
- 集成BMI088 IMU
- 实现基本控制算法
- FreeRTOS任务框架

---

## 👥 贡献者

- **Zhengbi Yong** (zhengbi.yong@outlook.com) - 项目创建者和主要开发者

---

## 📄 许可证

本项目基于STM32 HAL库和FreeRTOS，遵循相应的开源许可证。

---

## 🔗 相关链接

- [STM32H7系列参考手册](https://www.st.com/resource/en/reference_manual/rm0433-stm32h7-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [FreeRTOS文档](https://www.freertos.org/Documentation/RTOS_book.html)
- [CMSIS文档](https://arm-software.github.io/CMSIS_5/General/html/index.html)

---

## 📧 联系方式

如有问题或建议，请通过以下方式联系：
- 邮箱: zhengbi.yong@outlook.com
- 项目Issues: [GitHub Issues] (如有)

---

**最后更新**: 2025-01-XX


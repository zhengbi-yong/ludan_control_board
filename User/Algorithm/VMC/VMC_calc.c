/**
 * @file VMC_calc.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-18
 * 
 * Zhengbi Yong
 * 
 */
#include "VMC_calc.h"

void VMC_init(vmc_leg_t *vmc)//���˳���ֵ
{
	vmc->l5=0.088f;//AE���� //��λΪm
	vmc->l1=0.0833f;//��λΪm
	vmc->l2=0.16f;//��λΪm
	vmc->l3=0.16f;//��λΪm
	vmc->l4=0.0833f;//��λΪm
}

void VMC_calc_1_right(vmc_leg_t *vmc,INS_t *ins,float dt)//����theta��d_theta��lqr�ã�ͬʱҲ�����ȳ�L0
{		
		static float PitchR=0.0f;
	  static float PithGyroR=0.0f;
	  PitchR=ins->Pitch;
	  PithGyroR=ins->Gyro[1];
	
	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D��y����
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B��y����
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D��x����
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B��x����
			
		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
		vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
		vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C��ֱ������
		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
		//C�㼫����
		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
		
	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0���ڼ���lqr��Ҫ��theta		
	  vmc->alpha=pi/2.0f-vmc->phi0 ;
		
		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0 ;
			vmc->first_flag=1;
		}
		vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//����phi0�仯�ʣ�d_phi0���ڼ���lqr��Ҫ��d_theta
		vmc->d_alpha=0.0f-vmc->d_phi0 ;
		
		vmc->theta=pi/2.0f-PitchR-vmc->phi0;//�õ�״̬����1
		vmc->d_theta=(-PithGyroR-vmc->d_phi0);//�õ�״̬����2
		
		vmc->last_phi0=vmc->phi0 ;
    
		vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//�ȳ�L0��һ�׵���
    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//�ȳ�L0�Ķ��׵���
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;
}


void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt)//����theta��d_theta��lqr�ã�ͬʱҲ�����ȳ�L0
{		
	  static float PitchL=0.0f;
	  static float PithGyroL=0.0f;
	  PitchL=0.0f-ins->Pitch;
	  PithGyroL=0.0f-ins->Gyro[1];
	
		vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D��y����
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B��y����
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D��x����
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B��x����
			
		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
		vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
		vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C��ֱ������
		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
		//C�㼫����
		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
			
	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0���ڼ���lqr��Ҫ��theta		
	  vmc->alpha=pi/2.0f-vmc->phi0 ;
		
		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0 ;
			vmc->first_flag=1;
		}
		vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//����phi0�仯�ʣ�d_phi0���ڼ���lqr��Ҫ��d_theta
		vmc->d_alpha=0.0f-vmc->d_phi0 ;
		
		vmc->theta=pi/2.0f-PitchL-vmc->phi0;//�õ�״̬����1
		vmc->d_theta=(-PithGyroL-vmc->d_phi0);//�õ�״̬����2
		
		vmc->last_phi0=vmc->phi0 ;

		vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//�ȳ�L0��һ�׵���
    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//�ȳ�L0�Ķ��׵���
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;
}

void VMC_calc_2(vmc_leg_t *vmc)//���������Ĺؽ��������
{
		vmc->j11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
		vmc->j21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
	
		vmc->torque_set[0]=vmc->j11*vmc->F0+vmc->j12*vmc->Tp;//�õ�RightFront��������������أ�F0Ϊ�����˻���ĩ�����ȵ����� 
		vmc->torque_set[1]=vmc->j21*vmc->F0+vmc->j22*vmc->Tp;//�õ�RightBack��������������أ�TpΪ������������� 

}

float averr[4]={0.0f};
float aver_fnr=0.0f;
uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins)
{

	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));
 
 
	averr[0]=averr[1];
	averr[1]=averr[2];
	averr[2]=averr[3];
	averr[3]=vmc->FN;
	
	aver_fnr=0.25f*averr[0]+0.25f*averr[1]+0.25f*averr[2]+0.25f*averr[3];//��֧�������о�ֵ�˲�
	
	
	if(aver_fnr<3.0f)
	{//�����

	  return 1;
	}
	else
	{
	  return 0;	
	}
}


float averl[4]={0.0f};
float aver_fnl=0.0f;
uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins)
{
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));
 
	averl[0]=averl[1];
	averl[1]=averl[2];
	averl[2]=averl[3];
	averl[3]=vmc->FN;
	
	aver_fnl=0.25f*averl[0]+0.25f*averl[1]+0.25f*averl[2]+0.25f*averl[3];//��֧�������о�ֵ�˲�
	
	if(aver_fnl<3.0f)
	{//�����
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

float LQR_K_calc(float *coe,float len)
{
   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}



/*
*********************************************************************************************************
*
*	ģ������ : �����������ģ��
*	�ļ����� : bsp_motor.h
*	��    �� : V1.0
*	˵    �� : 
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/  
#include "stdio.h"
#include "bsp_motor.h" 
#include "common.h"
#include "attitude_controller.h"
#include "velocity_controller.h"
/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/
//����motor��IO����
#define  MOTOR_PORT_CLK 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA)
#define  MOTOR_TIM        TIM3
#define  MOTOR_TIM_CLK    RCC_APB1Periph_TIM3
#define  MOTOR_Left_Pin1     GPIO_Pin_6
#define  MOTOR_Left_Pin2     GPIO_Pin_7
#define  MOTOR_Right_Pin1		 GPIO_Pin_0
#define  MOTOR_Right_Pin2    GPIO_Pin_1
#define  MOTOR_Left_PORT     GPIOA
#define  MOTOR_Right_PORT    GPIOB


//PWM�������
#define  MOTOR_Left_Out1			TIM3->CCR1
#define  MOTOR_Left_Out2			TIM3->CCR2
#define  MOTOR_Right_Out1			TIM3->CCR4
#define  MOTOR_Right_Out2			TIM3->CCR3
//PWM�����duty=1250/2000 *100%
#define  MOTOR_PWM_MAX      		1999
#define  MOTOR_PWM_MID      		1250
#define  MOTOR_PWM_MIN      		100
#define  MOTOR_THROTTLE_MIN     130
#define  MOTOR_THROTTLE_MID  		500
#define  MOTOR_THROTTLE_MAX     1000
/*********************************************************************
*
*       Global var
*
**********************************************************************
*/
_Motor _motor[MOTOR_MAX_NUM];


/*********************************************************************************************************
*	�� �� ��: Motor_GPIO_Config
*	����˵��: ����Motor�������PWMʱ�õ���I/O
*						TIM3 CH1--PA6   MOTOR_Left_OUT1
*						TIM3 CH2--PA7   MOTOR_Left_OUT2
*						TIM3 CH3--PB0   MOTOR_Right_OUT1
*						TIM3 CH4--PB1	  MOTOR_Right_OUT1
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/

static void Motor_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* ����motor�õ���TIMx CLK Ϊ 72MHZ */
  RCC_APB1PeriphClockCmd(MOTOR_TIM_CLK, ENABLE); 
  /* enable motor GPIO CLK */
  RCC_APB2PeriphClockCmd(MOTOR_PORT_CLK, ENABLE); 

  /*LEFT Motor GPIO Configuration: */
  GPIO_InitStructure.GPIO_Pin =  MOTOR_Left_Pin1 | MOTOR_Left_Pin2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR_Left_PORT, &GPIO_InitStructure);
	/*Right Motor GPIO Configuration: */
	GPIO_InitStructure.GPIO_Pin =  MOTOR_Right_Pin1 | MOTOR_Right_Pin2;
  GPIO_Init(MOTOR_Right_PORT, &GPIO_InitStructure);

}

 
/*********************************************************************************************************
*	�� �� ��: TIM_Mode_Config
*	����˵��: ����TIM4�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
*	��    �Σ�TIMxCLK/CK_PSC --> TIMxCNT --> TIMx_ARR --> TIMxCNT ���¼���
*           TIMx_CCR(��ƽ�����仯)
* 					�ź�����=(TIMx_ARR +1 ) * ʱ������   
* 					ռ�ձ�=TIMx_CCR/(TIMx_ARR +1)
*	�� �� ֵ: 
*********************************************************************************************************
*/
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM�źŵ�ƽ����ֵ */
 	u16 CCR_Val = MOTOR_PWM_MIN;     //�ź�����Ϊ2K,��ʼռ�ձ�Ϊ5%   

/* ----------------------------------------------------------------------- 
    TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel3 duty cycle = (TIM2_CCR3/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel4 duty cycle = (TIM2_CCR4/ TIM2_ARR+1)* 100% = 5%
  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 1999;       //����ʱ����0������1999����Ϊ2000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //����Ԥ��Ƶ��72M/72 = 10^6
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
  TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  TIM_OC1PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//ʹ���Զ���װ

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC2Init(MOTOR_TIM, &TIM_OCInitStructure);	  //ʹ��ͨ��2
  TIM_OC2PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//ʹ���Զ���װ

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);	 //ʹ��ͨ��3
  TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//ʹ���Զ���װ

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC4Init(MOTOR_TIM, &TIM_OCInitStructure);	//ʹ��ͨ��4
  TIM_OC4PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//ʹ���Զ���װ
	
	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);			 // ʹ��motor �õ�TIM3���ؼĴ���ARR

  /* TIM3 enable counter */
  TIM_Cmd(MOTOR_TIM, ENABLE);                   //ʹ�ܶ�ʱ��3
}


/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************************************************
*	�� �� ��: bsp_MotorInit
*	����˵��: ��ʼ����ʱ��PWM���
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void bsp_MotorInit(void)
{
	Motor_GPIO_Config();
	TIM_Mode_Config();	
}
/*********************************************************************************************************
*	�� �� ��: Motor_1_Forward
*	����˵��: ���1��ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_1_Forward(void)
{
	   MOTOR_Left_Out1 = MOTOR_PWM_MID;
	   MOTOR_Left_Out2 = 0;
}
/*********************************************************************************************************
*	�� �� ��: MOTOR_1_Reverse
*	����˵��: ���1��ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_1_Reverse(void)
{
	   MOTOR_Left_Out1 = 0;
	   MOTOR_Left_Out2 = MOTOR_PWM_MID;
}
/*********************************************************************************************************
*	�� �� ��: MOTOR_1_Stop
*	����˵��: ���1ֹͣ
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_1_Stop(void)
{
	   MOTOR_Left_Out1 = 0;
	   MOTOR_Left_Out2 = 0;
}
/*********************************************************************************************************
*	�� �� ��: MOTOR_2_Forward
*	����˵��: ���2��ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_2_Forward(void)
{
	   MOTOR_Right_Out1 = MOTOR_PWM_MID;
	   MOTOR_Right_Out2 = 0;
}
/*********************************************************************************************************
*	�� �� ��: MOTOR_2_Reverse
*	����˵��: ���2��ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_2_Reverse(void)
{
	   MOTOR_Right_Out1 = 0;
	   MOTOR_Right_Out2 = MOTOR_PWM_MID;
}
/*********************************************************************************************************
*	�� �� ��: MOTOR_2_Stop
*	����˵��: ���2ֹͣ
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_2_Stop(void)
{
	   MOTOR_Right_Out1 = 0;
	   MOTOR_Right_Out2 = 0;
}
/*********************************************************************************************************
*	�� �� ��: Motor_Tuning
*	����˵��: �������
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Motor_Tuning(void* motor, uint8_t motor_id)
{
		float vel_pid_out,yaw_pid_out;
		_Motor *m = motor;
	
		yaw_pid_out = YawStabilizer();
		vel_pid_out = VelocityCorrectPID(m->cur_speed, m->tar_speed);
	  
	  if(motor_id == MOTOR_LEFT)
		{
			 m->out = yaw_pid_out + vel_pid_out;
			 if(m->tar_speed > 0)
			 {
				 MOTOR_Left_Out1 = constrain((MOTOR_THROTTLE_MAX + m->out), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
				 MOTOR_Left_Out2 = 0;
			 }
			 else
			 {
				 MOTOR_Left_Out2 = constrain((MOTOR_THROTTLE_MAX + m->out), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
				 MOTOR_Left_Out1 = 0;
			 }
		}
		else
		{
			m->out = yaw_pid_out - vel_pid_out;
			if(m->tar_speed > 0)
			{
				 MOTOR_Right_Out1 = constrain((MOTOR_THROTTLE_MAX + m->out), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
				 MOTOR_Right_Out2 = 0;
			}
			else
			{
				 MOTOR_Right_Out2 = constrain((MOTOR_THROTTLE_MAX + m->out), MOTOR_PWM_MIN, MOTOR_PWM_MAX);
				 MOTOR_Right_Out1 = 0;

			}
		}
}
/*********************************************************************************************************
*	�� �� ��: Car_Forward
*	����˵��: С��ǰ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Car_Forward(void)
{	
		 MOTOR_Left_Out1 = MOTOR_PWM_MAX;
	   MOTOR_Left_Out2 = MOTOR_PWM_MIN;
	   MOTOR_Right_Out1 = MOTOR_PWM_MAX;
	   MOTOR_Right_Out2 = MOTOR_PWM_MIN;
}
/*********************************************************************************************************
*	�� �� ��: Car_Backward
*	����˵��: С������
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Car_Backward(void)
{
	   
	   MOTOR_Left_Out1 = MOTOR_PWM_MIN;
	   MOTOR_Left_Out2 = MOTOR_PWM_MAX;
	   MOTOR_Right_Out1 = MOTOR_PWM_MIN;
	   MOTOR_Right_Out2 = MOTOR_PWM_MAX;
}
/*********************************************************************************************************
*	�� �� ��: Car_Left
*	����˵��: С����ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Car_Left(void)
{	
		 MOTOR_Left_Out1 = MOTOR_PWM_MIN;
	   MOTOR_Left_Out2 = MOTOR_PWM_MAX;
	   MOTOR_Right_Out1 = MOTOR_PWM_MAX;
	   MOTOR_Right_Out2 = 0;

}
/*********************************************************************************************************
*	�� �� ��: Car_Right
*	����˵��: С����ת
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Car_Right(void)
{
	   
		 MOTOR_Left_Out1 = MOTOR_PWM_MAX;
	   MOTOR_Left_Out2 = 0;
	   MOTOR_Right_Out1 = MOTOR_PWM_MIN;
	   MOTOR_Right_Out2 = MOTOR_PWM_MAX;
}
/*********************************************************************************************************
*	�� �� ��: Car_Stop
*	����˵��: С��ֹͣ
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Car_Stop(void)
{
	   
	   MOTOR_Left_Out1 = 0;
	   MOTOR_Left_Out2 = 0;
	   MOTOR_Right_Out1 = 0;
	   MOTOR_Right_Out2 = 0;
}
/*******************************************************************************
* Function Name  : DispMotorData
* Description    : Display motor Data
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DispMotorData(void)
{
		printf("RightSpeed:%d\n", (int)_motor[MOTOR_RIGHT].cur_speed);
		printf("LeftSpeed:%d\n", (int)_motor[MOTOR_LEFT].cur_speed);
//		printf("LeftAngle:%d\n", (int)_motor[MOTOR_LEFT].cur_angle);
//		printf("RightAngle:%d\n", (int)_motor[MOTOR_RIGHT].cur_angle);

}	

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

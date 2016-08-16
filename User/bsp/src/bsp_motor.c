/*
*********************************************************************************************************
*
*	模块名称 : 电机驱动函数模块
*	文件名称 : bsp_motor.h
*	版    本 : V1.0
*	说    明 : 
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
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
//驱动motor的IO引脚
#define  MOTOR_PORT_CLK 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA)
#define  MOTOR_TIM        TIM3
#define  MOTOR_TIM_CLK    RCC_APB1Periph_TIM3
#define  MOTOR_Left_Pin1     GPIO_Pin_6
#define  MOTOR_Left_Pin2     GPIO_Pin_7
#define  MOTOR_Right_Pin1		 GPIO_Pin_0
#define  MOTOR_Right_Pin2    GPIO_Pin_1
#define  MOTOR_Left_PORT     GPIOA
#define  MOTOR_Right_PORT    GPIOB


//PWM输出控制
#define  MOTOR_Left_Out1			TIM3->CCR1
#define  MOTOR_Left_Out2			TIM3->CCR2
#define  MOTOR_Right_Out1			TIM3->CCR4
#define  MOTOR_Right_Out2			TIM3->CCR3
//PWM输出，duty=1250/2000 *100%
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
*	函 数 名: Motor_GPIO_Config
*	功能说明: 配置Motor复用输出PWM时用到的I/O
*						TIM3 CH1--PA6   MOTOR_Left_OUT1
*						TIM3 CH2--PA7   MOTOR_Left_OUT2
*						TIM3 CH3--PB0   MOTOR_Right_OUT1
*						TIM3 CH4--PB1	  MOTOR_Right_OUT1
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/

static void Motor_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* 设置motor用到的TIMx CLK 为 72MHZ */
  RCC_APB1PeriphClockCmd(MOTOR_TIM_CLK, ENABLE); 
  /* enable motor GPIO CLK */
  RCC_APB2PeriphClockCmd(MOTOR_PORT_CLK, ENABLE); 

  /*LEFT Motor GPIO Configuration: */
  GPIO_InitStructure.GPIO_Pin =  MOTOR_Left_Pin1 | MOTOR_Left_Pin2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR_Left_PORT, &GPIO_InitStructure);
	/*Right Motor GPIO Configuration: */
	GPIO_InitStructure.GPIO_Pin =  MOTOR_Right_Pin1 | MOTOR_Right_Pin2;
  GPIO_Init(MOTOR_Right_PORT, &GPIO_InitStructure);

}

 
/*********************************************************************************************************
*	函 数 名: TIM_Mode_Config
*	功能说明: 配置TIM4输出的PWM信号的模式，如周期、极性、占空比
*	形    参：TIMxCLK/CK_PSC --> TIMxCNT --> TIMx_ARR --> TIMxCNT 重新计数
*           TIMx_CCR(电平发生变化)
* 					信号周期=(TIMx_ARR +1 ) * 时钟周期   
* 					占空比=TIMx_CCR/(TIMx_ARR +1)
*	返 回 值: 
*********************************************************************************************************
*/
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM信号电平跳变值 */
 	u16 CCR_Val = MOTOR_PWM_MIN;     //信号周期为2K,初始占空比为5%   

/* ----------------------------------------------------------------------- 
    TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel3 duty cycle = (TIM2_CCR3/ TIM2_ARR+1)* 100% = 5%
    TIM2 Channel4 duty cycle = (TIM2_CCR4/ TIM2_ARR+1)* 100% = 5%
  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 1999;       //当定时器从0计数到1999，即为2000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：72M/72 = 10^6
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
  TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC1PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//使能自动重装

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
  TIM_OC2Init(MOTOR_TIM, &TIM_OCInitStructure);	  //使能通道2
  TIM_OC2PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//使能自动重装

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
  TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//使能自动重装

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
  TIM_OC4Init(MOTOR_TIM, &TIM_OCInitStructure);	//使能通道4
  TIM_OC4PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);//使能自动重装
	
	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);			 // 使能motor 用的TIM3重载寄存器ARR

  /* TIM3 enable counter */
  TIM_Cmd(MOTOR_TIM, ENABLE);                   //使能定时器3
}


/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************************************************
*	函 数 名: bsp_MotorInit
*	功能说明: 初始化定时器PWM输出
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void bsp_MotorInit(void)
{
	Motor_GPIO_Config();
	TIM_Mode_Config();	
}
/*********************************************************************************************************
*	函 数 名: Motor_1_Forward
*	功能说明: 电机1正转
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_1_Forward(void)
{
	   MOTOR_Left_Out1 = MOTOR_PWM_MID;
	   MOTOR_Left_Out2 = 0;
}
/*********************************************************************************************************
*	函 数 名: MOTOR_1_Reverse
*	功能说明: 电机1反转
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_1_Reverse(void)
{
	   MOTOR_Left_Out1 = 0;
	   MOTOR_Left_Out2 = MOTOR_PWM_MID;
}
/*********************************************************************************************************
*	函 数 名: MOTOR_1_Stop
*	功能说明: 电机1停止
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_1_Stop(void)
{
	   MOTOR_Left_Out1 = 0;
	   MOTOR_Left_Out2 = 0;
}
/*********************************************************************************************************
*	函 数 名: MOTOR_2_Forward
*	功能说明: 电机2正转
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_2_Forward(void)
{
	   MOTOR_Right_Out1 = MOTOR_PWM_MID;
	   MOTOR_Right_Out2 = 0;
}
/*********************************************************************************************************
*	函 数 名: MOTOR_2_Reverse
*	功能说明: 电机2反转
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_2_Reverse(void)
{
	   MOTOR_Right_Out1 = 0;
	   MOTOR_Right_Out2 = MOTOR_PWM_MID;
}
/*********************************************************************************************************
*	函 数 名: MOTOR_2_Stop
*	功能说明: 电机2停止
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void Motor_2_Stop(void)
{
	   MOTOR_Right_Out1 = 0;
	   MOTOR_Right_Out2 = 0;
}
/*********************************************************************************************************
*	函 数 名: Motor_Tuning
*	功能说明: 电机控制
*	形    参：
*	返 回 值: 
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
*	函 数 名: Car_Forward
*	功能说明: 小车前进
*	形    参：
*	返 回 值: 
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
*	函 数 名: Car_Backward
*	功能说明: 小车后退
*	形    参：
*	返 回 值: 
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
*	函 数 名: Car_Left
*	功能说明: 小车左转
*	形    参：
*	返 回 值: 
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
*	函 数 名: Car_Right
*	功能说明: 小车右转
*	形    参：
*	返 回 值: 
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
*	函 数 名: Car_Stop
*	功能说明: 小车停止
*	形    参：
*	返 回 值: 
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

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

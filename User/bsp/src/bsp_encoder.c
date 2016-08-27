/*
*********************************************************************************************************
*
*	模块名称 : 编码器驱动模块
*	文件名称 : stm32f10x_encoder.c
*	版    本 : V5.1
*	说    明 : This file contains the software implementation for the
*            encoder position(angle) and speed reading.Dont use the TIM->CNT to calculate motor displacement!
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "bsp_encoder.h"
#include "os.h"
#include "common.h"
/* Private define ------------------------------------------------------------*/
#define TRUE   1
#define FALSE  0
// Encoder Left connected to TIM2
#define ENCODER_LEFT_TIMER	   	TIM2  
#define ENCODER_LEFT_TIMER_IRQ 	TIM2_IRQn 
#define ENCODER_LEFT_TIMER_RCC  RCC_APB1Periph_TIM2
#define ENCODER_LEFT_PORT_RCC  	RCC_APB2Periph_GPIOA
#define ENCODER_LEFT_PORT  			GPIOA
#define ENCODER_LEFT_Pin   	  	(GPIO_Pin_0 | GPIO_Pin_1)
// Encoder Right connected to TIM4
#define ENCODER_RIGHT_TIMER	   	 TIM4  
#define ENCODER_RIGHT_TIMER_IRQ  TIM4_IRQn 
#define ENCODER_RIGHT_TIMER_RCC  RCC_APB1Periph_TIM4
#define ENCODER_RIGHT_PORT_RCC   RCC_APB2Periph_GPIOB
#define ENCODER_RIGHT_PORT  		 GPIOB
#define ENCODER_RIGHT_Pin   	   (GPIO_Pin_6 | GPIO_Pin_7)

#define TIMx_PRE_PRIORITY     ENCODER_PRE_PRIORITY
#define TIMx_SUB_PRIORITY     ENCODER_SUB_PRIORITY

#define ENCODER_MAX_NUM          2
#define ENCODER_PPR     				506//858   // number of pulses per revolution
#define SPEED_BUFFER_SIZE 			8

#define COUNTER_RESET   				(u16)0
#define ICx_FILTER      				(u8) 0 // 6<-> 670nsec

#define SPEED_SAMPLING_TIME  10 
#define SPEED_SAMPLING_FREQ (u16)(1000/SPEED_SAMPLING_TIME)

/* Private functions ---------------------------------------------------------*/
static float ENC_Get_Motor_Angle(u8  motor_id);
static s16 ENC_Calc_Motor_Speed(u8  motor_id);
static void ENC_Clear_Speed_Buffer(void);
static s16 ENC_Calc_Average_Speed(u8  motor_id);
/* Private variables ---------------------------------------------------------*/

static s16 hSpeed_Buffer[MOTOR_MAX_NUM][SPEED_BUFFER_SIZE];
static s16 hPrevious_angle[MOTOR_MAX_NUM] = {0};
static char bIs_First_Measurement[MOTOR_MAX_NUM];
static int distance_overflow_times[MOTOR_MAX_NUM];
/* Global variables ---------------------------------------------------------*/
_Encoder _encoder;
extern _Motor _motor[2];

/*******************************************************************************
* Function Name  : ENC_Data_Init
* Description    : Initialize the data-structrue of encoder 
*                  sensors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENC_Data_Init(void)
{
		_encoder.tim[MOTOR_LEFT] = ENCODER_LEFT_TIMER;
		_encoder.tim[MOTOR_RIGHT] = ENCODER_RIGHT_TIMER;
		_encoder.dis_count[MOTOR_LEFT] = 0;
		_encoder.dis_count[MOTOR_RIGHT] = 0;
		_encoder.rad_count[MOTOR_LEFT] = 0;
		_encoder.rad_count[MOTOR_RIGHT] = 0;
}
/*******************************************************************************
* Function Name  : bsp_ENCInit
* Description    : General Purpose Timer x set-up for encoder speed/position 
*                  sensors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void bsp_ENCInit(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
/* Encoder unit connected to TIM2, 4X mode */    
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Encoder Left GPIO Port and TIMx clock source enable */
  RCC_APB1PeriphClockCmd(ENCODER_LEFT_TIMER_RCC, ENABLE);
  RCC_APB2PeriphClockCmd(ENCODER_LEFT_PORT_RCC, ENABLE);
	/* Encoder Right GPIO Port and TIMx clock source enable */
  RCC_APB1PeriphClockCmd(ENCODER_RIGHT_TIMER_RCC, ENABLE);
  RCC_APB2PeriphClockCmd(ENCODER_RIGHT_PORT_RCC, ENABLE);
  
  /* Configure PA.0,PA.1 as left encoder input */
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCODER_LEFT_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ENCODER_LEFT_PORT, &GPIO_InitStructure);
	/* Configure PB.6,PB.7 as right encoder input */
  GPIO_InitStructure.GPIO_Pin = ENCODER_RIGHT_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ENCODER_RIGHT_PORT, &GPIO_InitStructure);
  
  /* Enable the Left Encoder's TIMx Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ENCODER_LEFT_TIMER_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/* Enable the Right Encoder's TIMx Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ENCODER_RIGHT_TIMER_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Timer configuration in Encoder mode */
//  TIM_DeInit(ENCODER_LEFT_TIMER);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = (4*ENCODER_PPR)-1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(ENCODER_LEFT_TIMER, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(ENCODER_LEFT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  TIM_ICInit(ENCODER_LEFT_TIMER, &TIM_ICInitStructure);
  TIM_ClearFlag(ENCODER_LEFT_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(ENCODER_LEFT_TIMER, TIM_IT_Update, ENABLE);
	
//	TIM_DeInit(ENCODER_RIGHT_TIMER);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = (4*ENCODER_PPR)-1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(ENCODER_RIGHT_TIMER, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(ENCODER_RIGHT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  TIM_ICInit(ENCODER_RIGHT_TIMER, &TIM_ICInitStructure);
  TIM_ClearFlag(ENCODER_RIGHT_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(ENCODER_RIGHT_TIMER, TIM_IT_Update, ENABLE);
	
  //Reset counter
  ENCODER_LEFT_TIMER->CNT = COUNTER_RESET;
  ENCODER_RIGHT_TIMER->CNT = COUNTER_RESET;
	
	ENC_Data_Init();
  ENC_Clear_Speed_Buffer();
  
  TIM_Cmd(ENCODER_LEFT_TIMER, ENABLE);  
	TIM_Cmd(ENCODER_RIGHT_TIMER, ENABLE);  
}

/*******************************************************************************
* Function Name  : ENC_Get_Motor_Angle
* Description    : Returns the absolute electrical Rotor angle 
* Input          : motor_id: 0 or 1
* Output         : None
* Return         : Rotor electrical angle: 0 ~360                 
*******************************************************************************/
static float ENC_Get_Motor_Angle(u8  motor_id)
{
  float temp;
  
  temp = ((float)_encoder.tim[motor_id]->CNT /(4*ENCODER_PPR)) * 360; 

  return  temp; 
}

/*******************************************************************************
* Function Name  : ENC_Clear_Speed_Buffer
* Description    : Clear speed buffer used for average speed calculation  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void ENC_Clear_Speed_Buffer(void)
{   
  u32 i,j;

	for(j = 0; j < ENCODER_MAX_NUM; j ++)
	{
		for (i=0;i<SPEED_BUFFER_SIZE;i++)
		{
			hSpeed_Buffer[j][i] = 0;
		}
		
		bIs_First_Measurement[j] = TRUE;
	}
  
}

/*******************************************************************************
* Function Name  : ENC_Calc_Motor_Speed
* Description    : Compute return latest speed measurement 
* Input          : motor_id  :0 or 1
* Output         : s16
* Return         : Return the speed in 0.1 Hz resolution.                    
*******************************************************************************/
static s16 ENC_Calc_Motor_Speed(u8  motor_id)
{   
  s32 wDelta_angle;
  u16 hEnc_Timer_Overflow_sample_one, hEnc_Timer_Overflow_sample_two;
  u16 hCurrent_angle_sample_one, hCurrent_angle_sample_two;
  signed long long temp;
  s16 haux;
	
  
  if (!bIs_First_Measurement[motor_id])
  {
    // 1st reading of overflow counter    
    hEnc_Timer_Overflow_sample_one = _encoder.rad_count[motor_id]; 
    // 1st reading of encoder timer counter
    hCurrent_angle_sample_one = _encoder.tim[motor_id]->CNT;
    // 2nd reading of overflow counter
    hEnc_Timer_Overflow_sample_two = _encoder.rad_count[motor_id];  
    // 2nd reading of encoder timer counter
    hCurrent_angle_sample_two = _encoder.tim[motor_id]->CNT;      

    // Reset rad_count and read the counter value for the next
    // measurement
    _encoder.rad_count[motor_id] = 0;
    haux = _encoder.tim[motor_id]->CNT;   
    
    if (_encoder.rad_count[motor_id] != 0) 
    {
      haux = _encoder.tim[motor_id]->CNT; 
      _encoder.rad_count[motor_id] = 0;            
    }
     
    if (hEnc_Timer_Overflow_sample_one != hEnc_Timer_Overflow_sample_two)
    { //Compare sample 1 & 2 and check if an overflow has been generated right 
      //after the reading of encoder timer. If yes, copy sample 2 result in 
      //sample 1 for next process 
      hCurrent_angle_sample_one = hCurrent_angle_sample_two;
      hEnc_Timer_Overflow_sample_one = hEnc_Timer_Overflow_sample_two;
    }
    
    if ( (_encoder.tim[motor_id]->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
    {// encoder timer down-counting
      wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle[motor_id] - 
                    (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
    }
    else  
    {//encoder timer up-counting
      wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle[motor_id] + 
                    (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
    }
    
    // speed computation as delta angle * 1/(speed sempling time)
    temp = (signed long long)(wDelta_angle * SPEED_SAMPLING_FREQ);
    temp *= 10;  // 0.1 Hz resolution
    temp /= (4*ENCODER_PPR);
        
  } //is first measurement, discard it
  else
  {
    bIs_First_Measurement[motor_id] = FALSE;
    temp = 0;
    _encoder.rad_count[motor_id] = 0;
    haux = _encoder.tim[motor_id]->CNT;       
    // Check if Encoder_Timer_Overflow is still zero. In case an overflow IT 
    // occured it resets overflow counter and wPWM_Counter_Angular_Velocity
    if (_encoder.rad_count[motor_id] != 0) 
    {
      haux = _encoder.tim[motor_id]->CNT; 
      _encoder.rad_count[motor_id] = 0;            
    }
  }
  
  hPrevious_angle[motor_id] = haux;  
 
  return((s16) temp);
}

/*******************************************************************************
* Function Name  : ENC_Calc_Average_Speed
* Description    : Compute smoothed motor speed based on last SPEED_BUFFER_SIZE
                   informations and store it variable  
* Input          : None
* Output         : s16
* Return         : Return rotor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
static s16 ENC_Calc_Average_Speed(u8  motor_id)
{   
		s32 wtemp;
		u32 i;
	
		wtemp = ENC_Calc_Motor_Speed(motor_id);
					
		/* Compute the average of the read speeds */  
		for(i = 1; i < SPEED_BUFFER_SIZE; i ++)
		{
			hSpeed_Buffer[motor_id][i-1] = hSpeed_Buffer[motor_id][i]; 
			
		}
		hSpeed_Buffer[motor_id][SPEED_BUFFER_SIZE-1] = wtemp;

		wtemp=0;

		for (i=0;i<SPEED_BUFFER_SIZE;i++)
		{
			wtemp += hSpeed_Buffer[motor_id][i];
		}
		wtemp /= SPEED_BUFFER_SIZE;
		
		return  ((s16)(wtemp));
}


/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
                   Encoder unit connected to TIM2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{ 
		CPU_SR_ALLOC();

		CPU_CRITICAL_ENTER();
		OSIntEnter();
	
		/* Clear the interrupt pending flag */
		TIM_ClearFlag(ENCODER_LEFT_TIMER, TIM_FLAG_Update);
		
		if(ENCODER_LEFT_TIMER->CNT == 0)
		{
				_encoder.dis_count[MOTOR_LEFT] ++;
		}
		else
		{
				_encoder.dis_count[MOTOR_LEFT] --;
		}

		_encoder.rad_count[MOTOR_LEFT] ++;
		
		CPU_CRITICAL_EXIT();
		OSIntExit();
}
/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
                   Encoder unit connected to TIM4
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{  
		CPU_SR_ALLOC();

		CPU_CRITICAL_ENTER();
		OSIntEnter();

		/* Clear the interrupt pending flag */
		TIM_ClearFlag(ENCODER_RIGHT_TIMER, TIM_FLAG_Update);
		
		if(ENCODER_RIGHT_TIMER->CNT == 0)
		{
				_encoder.dis_count[MOTOR_RIGHT] ++;
		}
		else
		{
				_encoder.dis_count[MOTOR_RIGHT] --;
		}
		_encoder.rad_count[MOTOR_RIGHT] ++;
		
		CPU_CRITICAL_EXIT();
		OSIntExit();
}
/*******************************************************************************
* Function Name  : GetMotorSpeedAndAngle
* Description    : Claculate each motor's speed and angle
*                  speed unit is cm/s
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CalcMotorSpeedAndAngle(void)
{

		u8   i;
	  for(i = 0; i< MOTOR_MAX_NUM; i++)
		{
			_motor[i].cur_speed = (float)ENC_Calc_Average_Speed(i) * MOTOR_DIAMETER * M_PI;			
			_motor[i].cur_angle = (float)ENC_Get_Motor_Angle(i);
		}

}
/*******************************************************************************
* Function Name  : CalcMotorMileage
* Description    : Claculate each motor's Mileages
*                  mileage unit is M
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CalcMotorMileage(void)
{
		u8   i;
	  int  overfloaw_counts = distance_overflow_times[i] * INT32_MAX;
	  for(i = 0; i< MOTOR_MAX_NUM; i++)
		{
				if(_encoder.dis_count[i] > INT32_MAX)
				{
					 distance_overflow_times[i] ++;
					 _encoder.dis_count[i] = 0;
				}
				else
				if(_encoder.dis_count[i] < INT32_MIN)
				{
					 distance_overflow_times[i] --;
					 _encoder.dis_count[MOTOR_LEFT] = 0;
				}
			_motor[i].distances = (_encoder.dis_count[i] + overfloaw_counts)* MOTOR_DIAMETER * M_PI *0.01;
														;			
		}
}
/*******************************************************************************
* Function Name  : DispEncoderData
* Description    : Display encoder count
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DispEncoderData(void)
{
		printf("LeftEncoderCNT:%d\n", _encoder.tim[MOTOR_LEFT]->CNT);
		printf("RightEncoderCNT:%d\n", _encoder.tim[MOTOR_RIGHT]->CNT);
		printf("LeftEncoderCount:%d\n", (int)_encoder.rad_count[MOTOR_LEFT]);
		printf("RightEncoderCount:%d\n", (int)_encoder.rad_count[MOTOR_RIGHT]);
		printf("LeftEncoderDistanceCount:%d\n",(int)_encoder.dis_count[MOTOR_LEFT]);
		printf("RightEncoderDistanceCount:%d\n",(int)_encoder.dis_count[MOTOR_RIGHT]);
}
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

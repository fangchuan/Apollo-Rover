/*
*********************************************************************************************************
*
*	模块名称 : RC遥控接收模块
*	文件名称 : RemoteControl.c
*	版    本 : V1.0
*	说    明 : 天地飞8通接收机，输出PWM波
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "stdint.h"
#include "stdio.h"
#include "common.h"
#include "os.h"
#include "RemoteControl.h"

/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/
#define  RECEIVER_TIM     	TIM1
#define  RECEIVER_TIM_RCC   RCC_APB2Periph_TIM1
#define  RECEIVER_PORT_RCC  RCC_APB2Periph_GPIOE
#define  RECEIVER_TIM_IRQ 	TIM1_CC_IRQn
#define  ReceiverIRQHandler TIM1_CC_IRQHandler
#define  RECEIVER_CH1_Pin   GPIO_Pin_9
#define  RECEIVER_CH2_Pin		GPIO_Pin_11
#define  RECEIVER_CH3_Pin		GPIO_Pin_13
#define  RECEIVER_CH4_Pin		GPIO_Pin_14
#define  RECEIVER_PORT      GPIOE
#define  RECEIVER_Pin_Remap GPIO_FullRemap_TIM1

#define  Channel1     1
#define  Channel2     2
#define  Channel3     3
#define  Channel4     4
//the base value of rc data
#define  RC_THROTTLE_BASE  1000
#define  RC_THROTTLE_MIN   0
#define  RC_THROTTLE_MAX   999

#define  RC_RPY_BASE           1500
#define  RC_RPY_DEADBAND       50
#define  RC_RPY_MAX            500

#define  MAX_YAW_RATE          180.0f/M_PI   //unit :deg/s
#define  MAX_RP_ANGLE          45
/*********************************************************************
*
*       Global var
*
**********************************************************************
*/
_RemoteControl  _rc;
/*********************************************************************
*
*       Static var
*
**********************************************************************
*/
static uint8_t CH1_CAPTURE_STA=1;  //边沿捕捉标志位，=1表示捕捉到了上升沿，=0表示捕捉到了下降沿
static uint8_t CH2_CAPTURE_STA=1;
static uint8_t CH3_CAPTURE_STA=1;
static uint8_t CH4_CAPTURE_STA=1;

static uint16_t CH1_Rise, CH1_Fall,
								CH2_Rise, CH2_Fall,
								CH3_Rise, CH3_Fall,
								CH4_Rise, CH4_Fall;
volatile static uint16_t TIM_Period;
//PWMInCh1:副翼  PWMInCh2:升降舵  PWMInCh3:油门  PWMInCh4:方向舵
volatile static uint16_t PWMInCh1=0, PWMInCh2=0, PWMInCh3=0, PWMInCh4=0;

static void ReceiverNvicConfiguration(void)
{ 
			NVIC_InitTypeDef NVIC_InitStructure;

			/* Enable the TIM2 gloabal Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = RECEIVER_TIM_IRQ;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RECEIVER_PRE_PRIORITY;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = RECEIVER_SUB_PRIORITY;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: ReceiverTIMInit
*	功能说明: 配置TIM1，用于捕获接收机输出的4个PWM信号，
*           输入捕获模式，不是PWM输入模式。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ReceiverTIMInit(void)
{	 
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			TIM_ICInitTypeDef  TIM_ICInitStructure;

			RCC_APB2PeriphClockCmd(RECEIVER_TIM_RCC, ENABLE);	//使能所用定时器时钟
			RCC_APB2PeriphClockCmd(RECEIVER_PORT_RCC, ENABLE); 
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//Start AFIO Clock
			//Remapping the TIM port and pin
			GPIO_PinRemapConfig(RECEIVER_Pin_Remap, ENABLE);
			GPIO_InitStructure.GPIO_Pin  = RECEIVER_CH1_Pin| RECEIVER_CH2_Pin | RECEIVER_CH3_Pin | RECEIVER_CH4_Pin;    
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //参考ST手册：配置成浮空输入，用于检测上升沿或者下降沿  
			GPIO_Init(RECEIVER_PORT, &GPIO_InitStructure);	
			
			//初始化定时器，对于我的接收机WFT07来说，接收机收到的是遥控发来的PPM信号
			//而接收机输出的是标准的PWM信号，频率50Hz，周期为20ms，脉冲宽度为1ms~2ms之间，中点为1.5ms	 
			TIM_TimeBaseStructure.TIM_Period = 0xffff - 1;     //设定计数器自动重装值 
			TIM_TimeBaseStructure.TIM_Prescaler = 72-1 ; 	//1Mhz，1us计一个数   
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分频
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
			TIM_TimeBaseInit(RECEIVER_TIM, &TIM_TimeBaseStructure); //初始化定时器参数
	 
			//初始化定时器输入捕获参数,PA0-副翼
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//先上升沿捕获
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 滤掉1us以下脉冲宽度的干扰
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//初始化定时器输入捕获参数,PA1-升降舵
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//先上升沿捕获
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 滤掉1us以下脉冲宽度的干扰
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//初始化定时器输入捕获参数,PA2-油门
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//先上升沿捕获
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 滤掉1us以下脉冲宽度的干扰
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//初始化定时器输入捕获参数,PA3-方向舵
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//先上升沿捕获
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 滤掉1us以下脉冲宽度的干扰
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC1,ENABLE);//允许CC1IE边沿捕获中断	 
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC2,ENABLE);//允许CC1IE边沿捕获中断	
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC3,ENABLE);//允许CC1IE边沿捕获中断	
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC4,ENABLE);//允许CC1IE边沿捕获中断	

			TIM_Cmd(RECEIVER_TIM, ENABLE ); 	//使能定时器
}
/*
*********************************************************************************************************
*	函 数 名: RcDataInit
*	功能说明: 初始化接收机数据.外部调用
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void RcDataInit(void)
{
	  _rc.ch1_val = 0;
		_rc.ch2_val = 0;
		_rc.ch3_val = 0;
		_rc.ch4_val = 0;
}
/*
*********************************************************************************************************
*	函 数 名: bsp_ReceiverInit
*	功能说明: 初始化接收机.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void  bsp_ReceiverInit(void)
{
			ReceiverNvicConfiguration();
	
			ReceiverTIMInit();
	
			RcDataInit();
}

/*
*********************************************************************************************************
*	函 数 名: ReceiverIRQHandler
*	功能说明: This function handles TIM Handler.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ReceiverIRQHandler(void)
{
		CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
	  OSIntEnter();
	  CPU_CRITICAL_EXIT();
   // 如果捕获到ch1副翼
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC1) != RESET)//ch1发生捕获事件
		{	
			TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC1);   //清除中断标志位

			if(CH1_CAPTURE_STA == 1)	//捕获到上升沿
			{ 
				CH1_Rise = TIM_GetCapture1(RECEIVER_TIM);		           //获取上升沿的数据	  
				CH1_CAPTURE_STA = 0;		                        //转换标志位为下降沿
				TIM_OC1PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//设置为下降沿捕获	  			    			  
			}
			else  						    //捕获到下降沿
			{
				CH1_Fall = TIM_GetCapture1(RECEIVER_TIM);      //获取下降沿的数据	  
				CH1_CAPTURE_STA = 1;		//转换标志位为上升沿
				if(CH1_Fall < CH1_Rise)  //情况1：表示高电平跨过了65535这个特殊值，此时状态为上升沿接近65535，并且下降沿超过了65535从0开始计算，Tim2有溢出
				{
						TIM_Period = 65535;
				}
				else  //情况2：表示正常情况，上升沿和下降沿都在0-65535之间，且下降沿数值>上升沿数值。
				{
						TIM_Period = 0;
				}	
				PWMInCh1 = CH1_Fall - CH1_Rise + TIM_Period;  //得到总的高电平时间，值域1000~2000
				TIM_OC1PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获		
			}		    
		}

			//如果捕获到ch2升降舵
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC2) != RESET)//ch2发生捕获事件
		{	
				TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC2);   //清除中断标志位

				if(CH2_CAPTURE_STA == 1)	//捕获到上升沿
				{ 
					CH2_Rise = TIM_GetCapture2(RECEIVER_TIM);		           //获取上升沿的数据	  
					CH2_CAPTURE_STA = 0;		                        //转换标志位为下降沿
					TIM_OC2PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//设置为下降沿捕获	  			    			  
				}
				else  						    //捕获到下降沿
				{
					CH2_Fall = TIM_GetCapture2(RECEIVER_TIM);      //获取下降沿的数据	  
					CH2_CAPTURE_STA = 1;		//转换标志位为上升沿
					if(CH2_Fall < CH2_Rise) 
					{
						TIM_Period = 65535;
					}
					else
					{
						TIM_Period = 0;
					}	
					PWMInCh2 = CH2_Fall - CH2_Rise + TIM_Period;  //得到总的高电平时间，值域1000~2000
					TIM_OC2PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获			
				}		    
		}			     	    					   

				//如果捕获到ch3油门
			if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC3) != RESET)//ch3发生捕获事件
			{	
					TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC3);   //清除中断标志位

					if(CH3_CAPTURE_STA == 1)	//捕获到上升沿
					{ 
						CH3_Rise = TIM_GetCapture3(RECEIVER_TIM);		           //获取上升沿的数据	  
						CH3_CAPTURE_STA = 0;		                        //转换标志位为下降沿
						TIM_OC3PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//设置为下降沿捕获	  			    			  
					}
					else  						    //捕获到下降沿
					{
						CH3_Fall = TIM_GetCapture3(RECEIVER_TIM);      //获取下降沿的数据	  
						CH3_CAPTURE_STA = 1;		//转换标志位为上升沿
						if(CH3_Fall < CH3_Rise)  
						{
							TIM_Period = 65535;
						}
						else  
						{
							TIM_Period = 0;
						}	
						PWMInCh3 = CH3_Fall - CH3_Rise + TIM_Period;  //得到总的高电平时间，值域1000~2000
						TIM_OC3PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获			
					}		    
			}

			//如果捕获到ch4方向舵
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC4) != RESET)//ch4发生捕获事件
		{	
				TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC4);   //清除中断标志位

				if(CH4_CAPTURE_STA == 1)	//捕获到上升沿
				{ 
					CH4_Rise = TIM_GetCapture4(RECEIVER_TIM);		           //获取上升沿的数据	  
					CH4_CAPTURE_STA = 0;		                        //转换标志位为下降沿
					TIM_OC4PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//设置为下降沿捕获	  			    			  
				}
				else  						    //捕获到下降沿
				{
					CH4_Fall = TIM_GetCapture4(RECEIVER_TIM);      //获取下降沿的数据	  
					CH4_CAPTURE_STA = 1;		//转换标志位为上升沿
					if(CH4_Fall < CH4_Rise)  
					{
						TIM_Period = 65535;
					}
					else 
					{
						TIM_Period = 0;
					}	
					PWMInCh4 = CH4_Fall - CH4_Rise + TIM_Period;  //得到总的高电平时间，值域1000~2000
					TIM_OC4PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获			
				}		    
		}		
		OSIntExit();
}
/*
*********************************************************************************************************
*	函 数 名: GetRCValue
*	功能说明: 获取4个通道的输入值,并把它们线性化到对应角度或角速度
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
void  GetRCValue(void)
{			//0~999
		_rc.ch3_val = constrain(PWMInCh3 - RC_THROTTLE_BASE, RC_THROTTLE_MIN, RC_THROTTLE_MAX);
		
//		_rc.ch1_val = MAX_RP_ANGLE * ScaleLinear((PWMInCh1 - RC_RPY_BASE), RC_RPY_MAX, RC_RPY_DEADBAND);
//		_rc.ch2_val = MAX_RP_ANGLE * ScaleLinear((PWMInCh2 - RC_RPY_BASE), RC_RPY_MAX, RC_RPY_DEADBAND);
	//yaw rate : °/s
		_rc.ch4_val = MAX_YAW_RATE * ScaleLinear((PWMInCh4 - RC_RPY_BASE), RC_RPY_MAX, RC_RPY_DEADBAND);
	 
}
/*******************************************************************************
* Function Name  : DispRcData
* Description    : Display receiver Data
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DispRcData(void)
{
		printf("Channel1:%d\n", (int)_rc.ch1_val);
		printf("Channel2:%d\n", (int)_rc.ch2_val);
		printf("Channel3:%d\n", _rc.ch3_val);
		printf("Channel4:%d\n", (int)_rc.ch4_val);

}	
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : RCң�ؽ���ģ��
*	�ļ����� : RemoteControl.c
*	��    �� : V1.0
*	˵    �� : ��ط�8ͨ���ջ������PWM��
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
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
static uint8_t CH1_CAPTURE_STA=1;  //���ز�׽��־λ��=1��ʾ��׽���������أ�=0��ʾ��׽�����½���
static uint8_t CH2_CAPTURE_STA=1;
static uint8_t CH3_CAPTURE_STA=1;
static uint8_t CH4_CAPTURE_STA=1;

static uint16_t CH1_Rise, CH1_Fall,
								CH2_Rise, CH2_Fall,
								CH3_Rise, CH3_Fall,
								CH4_Rise, CH4_Fall;
volatile static uint16_t TIM_Period;
//PWMInCh1:����  PWMInCh2:������  PWMInCh3:����  PWMInCh4:�����
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
*	�� �� ��: ReceiverTIMInit
*	����˵��: ����TIM1�����ڲ�����ջ������4��PWM�źţ�
*           ���벶��ģʽ������PWM����ģʽ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ReceiverTIMInit(void)
{	 
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			TIM_ICInitTypeDef  TIM_ICInitStructure;

			RCC_APB2PeriphClockCmd(RECEIVER_TIM_RCC, ENABLE);	//ʹ�����ö�ʱ��ʱ��
			RCC_APB2PeriphClockCmd(RECEIVER_PORT_RCC, ENABLE); 
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//Start AFIO Clock
			//Remapping the TIM port and pin
			GPIO_PinRemapConfig(RECEIVER_Pin_Remap, ENABLE);
			GPIO_InitStructure.GPIO_Pin  = RECEIVER_CH1_Pin| RECEIVER_CH2_Pin | RECEIVER_CH3_Pin | RECEIVER_CH4_Pin;    
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //�ο�ST�ֲ᣺���óɸ������룬���ڼ�������ػ����½���  
			GPIO_Init(RECEIVER_PORT, &GPIO_InitStructure);	
			
			//��ʼ����ʱ���������ҵĽ��ջ�WFT07��˵�����ջ��յ�����ң�ط�����PPM�ź�
			//�����ջ�������Ǳ�׼��PWM�źţ�Ƶ��50Hz������Ϊ20ms��������Ϊ1ms~2ms֮�䣬�е�Ϊ1.5ms	 
			TIM_TimeBaseStructure.TIM_Period = 0xffff - 1;     //�趨�������Զ���װֵ 
			TIM_TimeBaseStructure.TIM_Prescaler = 72-1 ; 	//1Mhz��1us��һ����   
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����Ƶ
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
			TIM_TimeBaseInit(RECEIVER_TIM, &TIM_TimeBaseStructure); //��ʼ����ʱ������
	 
			//��ʼ����ʱ�����벶�����,PA0-����
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�������ز���
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 �˵�1us���������ȵĸ���
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//��ʼ����ʱ�����벶�����,PA1-������
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�������ز���
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 �˵�1us���������ȵĸ���
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//��ʼ����ʱ�����벶�����,PA2-����
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�������ز���
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 �˵�1us���������ȵĸ���
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			//��ʼ����ʱ�����벶�����,PA3-�����
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�������ز���
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
			TIM_ICInitStructure.TIM_ICFilter = 0x0A;//IC1F=1001 �˵�1us���������ȵĸ���
			TIM_ICInit(RECEIVER_TIM, &TIM_ICInitStructure);
			
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC1,ENABLE);//����CC1IE���ز����ж�	 
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC2,ENABLE);//����CC1IE���ز����ж�	
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC3,ENABLE);//����CC1IE���ز����ж�	
			TIM_ITConfig(RECEIVER_TIM,TIM_IT_CC4,ENABLE);//����CC1IE���ز����ж�	

			TIM_Cmd(RECEIVER_TIM, ENABLE ); 	//ʹ�ܶ�ʱ��
}
/*
*********************************************************************************************************
*	�� �� ��: RcDataInit
*	����˵��: ��ʼ�����ջ�����.�ⲿ����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_ReceiverInit
*	����˵��: ��ʼ�����ջ�.
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: ReceiverIRQHandler
*	����˵��: This function handles TIM Handler.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ReceiverIRQHandler(void)
{
		CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
	  OSIntEnter();
	  CPU_CRITICAL_EXIT();
   // �������ch1����
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC1) != RESET)//ch1���������¼�
		{	
			TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC1);   //����жϱ�־λ

			if(CH1_CAPTURE_STA == 1)	//����������
			{ 
				CH1_Rise = TIM_GetCapture1(RECEIVER_TIM);		           //��ȡ�����ص�����	  
				CH1_CAPTURE_STA = 0;		                        //ת����־λΪ�½���
				TIM_OC1PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//����Ϊ�½��ز���	  			    			  
			}
			else  						    //�����½���
			{
				CH1_Fall = TIM_GetCapture1(RECEIVER_TIM);      //��ȡ�½��ص�����	  
				CH1_CAPTURE_STA = 1;		//ת����־λΪ������
				if(CH1_Fall < CH1_Rise)  //���1����ʾ�ߵ�ƽ�����65535�������ֵ����ʱ״̬Ϊ�����ؽӽ�65535�������½��س�����65535��0��ʼ���㣬Tim2�����
				{
						TIM_Period = 65535;
				}
				else  //���2����ʾ��������������غ��½��ض���0-65535֮�䣬���½�����ֵ>��������ֵ��
				{
						TIM_Period = 0;
				}	
				PWMInCh1 = CH1_Fall - CH1_Rise + TIM_Period;  //�õ��ܵĸߵ�ƽʱ�䣬ֵ��1000~2000
				TIM_OC1PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���		
			}		    
		}

			//�������ch2������
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC2) != RESET)//ch2���������¼�
		{	
				TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC2);   //����жϱ�־λ

				if(CH2_CAPTURE_STA == 1)	//����������
				{ 
					CH2_Rise = TIM_GetCapture2(RECEIVER_TIM);		           //��ȡ�����ص�����	  
					CH2_CAPTURE_STA = 0;		                        //ת����־λΪ�½���
					TIM_OC2PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//����Ϊ�½��ز���	  			    			  
				}
				else  						    //�����½���
				{
					CH2_Fall = TIM_GetCapture2(RECEIVER_TIM);      //��ȡ�½��ص�����	  
					CH2_CAPTURE_STA = 1;		//ת����־λΪ������
					if(CH2_Fall < CH2_Rise) 
					{
						TIM_Period = 65535;
					}
					else
					{
						TIM_Period = 0;
					}	
					PWMInCh2 = CH2_Fall - CH2_Rise + TIM_Period;  //�õ��ܵĸߵ�ƽʱ�䣬ֵ��1000~2000
					TIM_OC2PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���			
				}		    
		}			     	    					   

				//�������ch3����
			if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC3) != RESET)//ch3���������¼�
			{	
					TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC3);   //����жϱ�־λ

					if(CH3_CAPTURE_STA == 1)	//����������
					{ 
						CH3_Rise = TIM_GetCapture3(RECEIVER_TIM);		           //��ȡ�����ص�����	  
						CH3_CAPTURE_STA = 0;		                        //ת����־λΪ�½���
						TIM_OC3PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//����Ϊ�½��ز���	  			    			  
					}
					else  						    //�����½���
					{
						CH3_Fall = TIM_GetCapture3(RECEIVER_TIM);      //��ȡ�½��ص�����	  
						CH3_CAPTURE_STA = 1;		//ת����־λΪ������
						if(CH3_Fall < CH3_Rise)  
						{
							TIM_Period = 65535;
						}
						else  
						{
							TIM_Period = 0;
						}	
						PWMInCh3 = CH3_Fall - CH3_Rise + TIM_Period;  //�õ��ܵĸߵ�ƽʱ�䣬ֵ��1000~2000
						TIM_OC3PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���			
					}		    
			}

			//�������ch4�����
		if (TIM_GetITStatus(RECEIVER_TIM, TIM_IT_CC4) != RESET)//ch4���������¼�
		{	
				TIM_ClearITPendingBit(RECEIVER_TIM, TIM_IT_CC4);   //����жϱ�־λ

				if(CH4_CAPTURE_STA == 1)	//����������
				{ 
					CH4_Rise = TIM_GetCapture4(RECEIVER_TIM);		           //��ȡ�����ص�����	  
					CH4_CAPTURE_STA = 0;		                        //ת����־λΪ�½���
					TIM_OC4PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Falling);		//����Ϊ�½��ز���	  			    			  
				}
				else  						    //�����½���
				{
					CH4_Fall = TIM_GetCapture4(RECEIVER_TIM);      //��ȡ�½��ص�����	  
					CH4_CAPTURE_STA = 1;		//ת����־λΪ������
					if(CH4_Fall < CH4_Rise)  
					{
						TIM_Period = 65535;
					}
					else 
					{
						TIM_Period = 0;
					}	
					PWMInCh4 = CH4_Fall - CH4_Rise + TIM_Period;  //�õ��ܵĸߵ�ƽʱ�䣬ֵ��1000~2000
					TIM_OC4PolarityConfig(RECEIVER_TIM,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���			
				}		    
		}		
		OSIntExit();
}
/*
*********************************************************************************************************
*	�� �� ��: GetRCValue
*	����˵��: ��ȡ4��ͨ��������ֵ,�����������Ի�����Ӧ�ǶȻ���ٶ�
*	��    ��: 
*	�� �� ֵ:
*********************************************************************************************************
*/
void  GetRCValue(void)
{			//0~999
		_rc.ch3_val = constrain(PWMInCh3 - RC_THROTTLE_BASE, RC_THROTTLE_MIN, RC_THROTTLE_MAX);
		
//		_rc.ch1_val = MAX_RP_ANGLE * ScaleLinear((PWMInCh1 - RC_RPY_BASE), RC_RPY_MAX, RC_RPY_DEADBAND);
//		_rc.ch2_val = MAX_RP_ANGLE * ScaleLinear((PWMInCh2 - RC_RPY_BASE), RC_RPY_MAX, RC_RPY_DEADBAND);
	//yaw rate : ��/s
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
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : GPIO�����ⲿ�ж�ģ��
*	�ļ����� : bsp_exti.c
*	��    �� : V5.1
*	˵    �� : 
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "bsp_exti.h"
#include "motion_driver_9150.h"
#include "common.h"
#include "os.h"


#define  MPU9150_INT_Pin				GPIO_Pin_4
#define  MPU9150_INT_PORT       GPIOA
#define  MPU9150_INT_PORT_RCC   RCC_APB2Periph_GPIOA
#define  MPU9150_INT_PortSource GPIO_PortSourceGPIOA
#define  MPU9150_INT_PinSource  GPIO_PinSource4
#define  MPU9150_INT_EXTI_Line  EXTI_Line4
#define  MPU9150_INT_ISR        EXTI4_IRQn
#define  NVIC_MPU9150_PP        MPU9150_PRE_PRIORITY
#define  NVIC_MPU9150_SP        MPU9150_SUB_PRIORITY
/*
*********************************************************************************************************
*	�� �� ��: NVIC_Configuration
*	����˵��: ����Ƕ�������жϿ�����NVIC��
*	��    ��: 
*	�� �� ֵ: 
*********************************************************************************************************
*/
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
//  /* Configure one bit for preemption priority */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* �����ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = MPU9150_INT_ISR;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MPU9150_PP;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_MPU9150_SP;

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEXTI
*	����˵��: ������ӦGPIOΪ���жϿڣ��������ж����ȼ�
*	��    ��: 
*	�� �� ֵ: 
*********************************************************************************************************
*/
void bsp_InitEXTI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(MPU9150_INT_PORT_RCC | RCC_APB2Periph_AFIO,ENABLE);
												
	/* config the NVIC */
	NVIC_Configuration();

	/* EXTI line gpio config*/	
  GPIO_InitStructure.GPIO_Pin = MPU9150_INT_Pin;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // ��������
  GPIO_Init(MPU9150_INT_PORT, &GPIO_InitStructure);

	/* EXTI line mode config */
  GPIO_EXTILineConfig(MPU9150_INT_PortSource, MPU9150_INT_PinSource); 
  EXTI_InitStructure.EXTI_Line = MPU9150_INT_EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 

}
/*
*********************************************************************************************************
*	�� �� ��: EXTI0_IRQHandler
*	����˵��: PA4�����жϷ�����
*	��    ��: 
*	�� �� ֵ: 
*********************************************************************************************************
*/
void EXTI4_IRQHandler(void)
{
		CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
		OSIntEnter();
		CPU_CRITICAL_EXIT();
		if(EXTI_GetITStatus(MPU9150_INT_EXTI_Line) != RESET) //ȷ���Ƿ������EXTI Line�ж�
		{
				gyro_data_ready_cb();
				EXTI_ClearITPendingBit(MPU9150_INT_EXTI_Line);     //����жϱ�־λ
		}  
	
		OSIntExit();
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

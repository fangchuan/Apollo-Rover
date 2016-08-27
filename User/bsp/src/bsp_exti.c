/*
*********************************************************************************************************
*
*	模块名称 : GPIO引脚外部中断模块
*	文件名称 : bsp_exti.c
*	版    本 : V5.1
*	说    明 : 
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
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
*	函 数 名: NVIC_Configuration
*	功能说明: 配置嵌套向量中断控制器NVIC化
*	形    参: 
*	返 回 值: 
*********************************************************************************************************
*/
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
//  /* Configure one bit for preemption priority */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* 配置中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = MPU9150_INT_ISR;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MPU9150_PP;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_MPU9150_SP;

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitEXTI
*	功能说明: 配置相应GPIO为线中断口，并设置中断优先级
*	形    参: 
*	返 回 值: 
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入
  GPIO_Init(MPU9150_INT_PORT, &GPIO_InitStructure);

	/* EXTI line mode config */
  GPIO_EXTILineConfig(MPU9150_INT_PortSource, MPU9150_INT_PinSource); 
  EXTI_InitStructure.EXTI_Line = MPU9150_INT_EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 

}
/*
*********************************************************************************************************
*	函 数 名: EXTI0_IRQHandler
*	功能说明: PA4引脚中断服务函数
*	形    参: 
*	返 回 值: 
*********************************************************************************************************
*/
void EXTI4_IRQHandler(void)
{
		CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
		OSIntEnter();
		CPU_CRITICAL_EXIT();
		if(EXTI_GetITStatus(MPU9150_INT_EXTI_Line) != RESET) //确保是否产生了EXTI Line中断
		{
				gyro_data_ready_cb();
				EXTI_ClearITPendingBit(MPU9150_INT_EXTI_Line);     //清除中断标志位
		}  
	
		OSIntExit();
}

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

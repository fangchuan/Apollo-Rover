/*
*********************************************************************************************************
*
*	模块名称 : 编码器驱动模块
*	文件名称 : stm32f10x_encoder.h
*	版    本 : V5.1
*	说    明 : This file contains the software implementation for the
*            encoder position and speed reading.
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_ENCODER_H
#define __STM32F10x_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

typedef struct {
	      TIM_TypeDef* tim[2];		//两个编码器使用的定时器
				float rad_count[2];			//用于测速的两轮转数,每解算一次速度，该变量会清零
				int   dis_count[2];//用于测算行驶距离的两轮转数
}_Encoder;


/* Exported functions ------------------------------------------------------- */
extern void bsp_ENCInit(void);
extern void CalcMotorSpeedAndAngle(void);
extern void CalcMotorMileage(void);
extern void DispEncoderData(void);
#endif  /*__STM32F10x_ENCODER_H*/
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : ����������ģ��
*	�ļ����� : stm32f10x_encoder.h
*	��    �� : V5.1
*	˵    �� : This file contains the software implementation for the
*            encoder position and speed reading.
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_ENCODER_H
#define __STM32F10x_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

typedef struct {
	      TIM_TypeDef* tim[2];		//����������ʹ�õĶ�ʱ��
				float rad_count[2];			//���ڲ��ٵ�����ת��,ÿ����һ���ٶȣ��ñ���������
				int   dis_count[2];//���ڲ�����ʻ���������ת��
}_Encoder;


/* Exported functions ------------------------------------------------------- */
extern void bsp_ENCInit(void);
extern void CalcMotorSpeedAndAngle(void);
extern void CalcMotorMileage(void);
extern void DispEncoderData(void);
#endif  /*__STM32F10x_ENCODER_H*/
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

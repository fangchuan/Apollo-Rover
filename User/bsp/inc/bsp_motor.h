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
#ifndef  _BSP_MOTOR_H
#define  _BSP_MOTOR_H

#include "stm32f10x.h"




extern  void bsp_MotorInit(void);
extern  void Motor_1_Forward(void);
extern  void Motor_1_Reverse(void);
extern  void Motor_1_Stop(void);
extern  void Motor_2_Forward(void);
extern  void Motor_2_Reverse(void);
extern  void Motor_2_Stop(void);
extern void Car_Forward(void);
extern void Car_Backward(void);
extern void Car_Left(void);
extern void Car_Right(void);
extern void Car_Stop(void);

extern void Motor_Tuning(void* motor, uint8_t motor_id);
extern void DispMotorData(void);

#endif /*_BSP_MOTOR_H*/
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

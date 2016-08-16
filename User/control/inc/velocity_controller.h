/*
*********************************************************************************************************
*
*	ģ������ : ���ٿ���PIDģ��
*	�ļ����� : velocity_controller.h
*	��    �� : V1.0
*	˵    �� : PID-based attitude controller
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef _VELOCITY_CONTROLLER_H
#define _VELOCITY_CONTROLLER_H

#include "stdint.h"
#include "stdbool.h"

#define VELOCITY_UPDATE_FREQ  			100
#define VELOCITY_UPDATE_DT   				(float)(1.0/VELOCITY_UPDATE_FREQ)
#define VELOCITY_CONTROLLER_OUT_MAX   750
#define VELOCITY_CONTROLLER_OUT_MIN   -750

void  velocityControllerInit(void);
bool  velocityControllerTest(void);
int16_t VelocityCorrectPID(float VelocityActual, float VelocityDesired);


#endif
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

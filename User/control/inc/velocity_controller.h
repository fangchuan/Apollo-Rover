/*
*********************************************************************************************************
*
*	模块名称 : 车速控制PID模块
*	文件名称 : velocity_controller.h
*	版    本 : V1.0
*	说    明 : PID-based attitude controller
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
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
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

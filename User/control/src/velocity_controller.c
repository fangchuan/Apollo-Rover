/*
*********************************************************************************************************
*
*	模块名称 : 车速控制PID模块
*	文件名称 : celocity_controller.c
*	版    本 : V1.0
*	说    明 : Velocity controler using PID correctors
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "velocity_controller.h"
#include "pid.h"
#include "common.h"


PidObject  pidVelocity;
int16_t    velOutput;

extern _Motor _motor[MOTOR_MAX_NUM];

static bool isInit;
/*********************************************************************************************************
*	函 数 名: velocityControllerInit
*	功能说明: 速度控制器参数初始化;
*						初始化后就不能再初始化
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void  velocityControllerInit(void)
{
		if(isInit)
			return;

		pidInit(&pidVelocity, 0, PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD, VELOCITY_UPDATE_DT);
		pidSetIntegralLimit(&pidVelocity, PID_VELOCITY_INTEGRATION_LIMIT);

		isInit = true;
}
/*********************************************************************************************************
*	函 数 名: velocityControllerTest
*	功能说明: 检测控制器是否初始化
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
bool velocityControllerTest(void)
{
		return isInit;
}
/*********************************************************************************************************
*	函 数 名: VelocityCorrectPID
*	功能说明: 速度环PID控制器
*	形    参：实际速度、期望速度
*	返 回 值: 
*********************************************************************************************************
*/
int16_t VelocityCorrectPID(float VelocityActual, float VelocityDesired)
{
		float pid;
		pidSetDesired(&pidVelocity, VelocityDesired);
		pid = pidUpdate(&pidVelocity, VelocityActual, true);
		velOutput = constrain(pid, -VELOCITY_CONTROLLER_OUT_MAX, VELOCITY_CONTROLLER_OUT_MAX);
		return velOutput;
}

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	模块名称 : pid通用函数模块
*	文件名称 : pid.c
*	版    本 : V1.0
*	说    明 : PID控制器基础功能函数
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "pid.h"
#include "common.h"
#if  DEBUG_PID
#include "stdio.h"
#endif
/*********************************************************************************************************
*	函 数 名: pidInit
*	功能说明: PID参数初始化
*	形    参：pid结构体，期望值desired， 系数kp, 系数ki, 系数kd, 控制周期dt
*	返 回 值: 无
*********************************************************************************************************
*/
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
		pid->error     = 0;
		pid->prevError = 0;
		pid->integ     = 0;
		pid->deriv     = 0;
		pid->desired = desired;
		pid->kp = kp;
		pid->ki = ki;
		pid->kd = kd;
		pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
		pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
		pid->dt        = dt;
}
/*********************************************************************************************************
*	函 数 名: pidUpdate
*	功能说明: pid控制器更新函数
*	形    参：pid结构体, 测得值measured, 是否更新误差updateError
*	返 回 值: pid控制器输出值
*********************************************************************************************************
*/
float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output;

    if (updateError)
    {
        pid->error = pid->desired - measured;
				#if  DEBUG_PID
				printf("pid_error:%d\n", (int)pid->error);
				#endif
    }
		
    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }

    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;
		#if  DEBUG_PID
		printf("pid_outP:%d\n", (int)pid->outP);
		#endif
    output = pid->outP + pid->outI + pid->outD;
		#if  DEBUG_PID
		printf("pid_output:%d\n", (int)output);
		#endif
    pid->prevError = pid->error;

    return output;
}
/*********************************************************************************************************
*	函 数 名: pidSetIntegralLimit
*	功能说明: 设置积分上限值
*	形    参：pid结构体，  积分上限值limit
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetIntegralLimit(PidObject* pid, const float limit)
{
    pid->iLimit = limit;
}

/*********************************************************************************************************
*	函 数 名: pidSetIntegralLimitLow
*	功能说明: 设置积分下限值
*	形    参：pid结构体，  积分上限值limitLow
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) 
{
    pid->iLimitLow = limitLow;
}
/*********************************************************************************************************
*	函 数 名: pidReset
*	功能说明: 参数清零
*	形    参：pid结构体
*	返 回 值: 无
*********************************************************************************************************
*/
void pidReset(PidObject* pid)
{
		pid->error     = 0;
		pid->prevError = 0;
		pid->integ     = 0;
		pid->deriv     = 0;
}
/*********************************************************************************************************
*	函 数 名: pidSetError
*	功能说明: 设置控制器误差值
*	形    参：pid结构体, 误差值error
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetError(PidObject* pid, const float error)
{
		pid->error = error;
}
/*********************************************************************************************************
*	函 数 名: pidSetDesired
*	功能说明: 链表解析函数
*	形    参：pid结构体, 期望值desired
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetDesired(PidObject* pid, const float desired)
{
		pid->desired = desired;
}
/*********************************************************************************************************
*	函 数 名: pidGetDesired
*	功能说明: 获取期望值
*	形    参：pid结构体
*	返 回 值: pid.desired
*********************************************************************************************************
*/
float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}
/*********************************************************************************************************
*	函 数 名: pidIsActive
*	功能说明: 控制器是否激活
*	形    参：
*	返 回 值: bool
*********************************************************************************************************
*/
bool pidIsActive(PidObject* pid)
{
		bool isActive = true;

		if (pid->kp < 0.0001 && pid->ki < 0.0001 && pid->kd < 0.0001)
		{
			isActive = false;
		}

		return isActive;
}
/*********************************************************************************************************
*	函 数 名: pidSetKp
*	功能说明: 设置控制器kp系数
*	形    参：pid结构体，系数kp
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetKp(PidObject* pid, const float kp)
{
		pid->kp = kp;
}
/*********************************************************************************************************
*	函 数 名: pidSetKi
*	功能说明: 设置控制器ki系数
*	形    参：pid结构体，系数ki
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetKi(PidObject* pid, const float ki)
{
		pid->ki = ki;
}
/*********************************************************************************************************
*	函 数 名: pidSetKd
*	功能说明: 设置控制器kd系数
*	形    参：pid结构体，系数kd
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetKd(PidObject* pid, const float kd)
{
		pid->kd = kd;
}
/*********************************************************************************************************
*	函 数 名: pidSetDt
*	功能说明: 设置控制器控制周期dt
*	形    参：pid结构体，周期dt
*	返 回 值: 无
*********************************************************************************************************
*/
void pidSetDt(PidObject* pid, const float dt)
{
    pid->dt = dt;
}
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	模块名称 : Apollorobot控制模块
*	文件名称 : ApolloRobotRover.c
*	版    本 : V1.0
*	说    明 : 
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-08-18 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/  
#include "stdio.h"
#include "common.h"
#include "os.h"
#include "bsp_encoder.h"
#include "bsp_gps.h"
#include "bsp_motor.h"
#include "bsp_hmc5883l.h"
#include "bsp_led.h"
#include "motion_driver_9150.h"
#include "ahrs.h"
#include "pid.h"
#include "attitude_controller.h"
#include "velocity_controller.h"
#include "RemoteControl.h"
#include "ApolloRobotRover.h"
/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/
#define  ARROVER_LOOP_FRE         100
#define  ARROVER_LOOP_TIME        (float)0.01
#define  VELOCITY_CONTROL_FRE     33
#define  VELOCITY_CONTROL_COUNTS  (u8)ARROVER_LOOP_FRE/VELOCITY_CONTROL_FRE
#define  ATTITUDE_CONTROL_FRE     10
#define  ATTITUDE_CONTROL_COUNTS  1
#define  POSITION_CONTROL_FRE     1
#define  POSITION_CONTROL_COUNTS  ARROVER_LOOP_FRE/POSITION_CONTROL_FRE

/*********************************************************************
*
*       Global var
*
**********************************************************************
*/
extern _Motor _motor[MOTOR_MAX_NUM];
extern _RemoteControl  _rc;
extern GPS_T g_tGPS;
extern _Euler  _euler;

_Position _position;

static bool IsRoverInit = false;
/*********************************************************************************************************
*	函 数 名: ArRoverInit
*	功能说明: 控制初始化
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void ArRoverInit(void)
{
#if  AHRS_USE_DMP
	  int8_t  result;
#endif
	
		if(IsRoverInit)
			return ;
		
#if  AHRS_USE_DMP
	 result = dmp_init();
	 if(result)
	 {
		 printf("DMP initialize error!\n");
	 }	 
#endif
	 //Correct the hmc5883l for 10s
	  bsp_LedOn(1);
	  HMC5883L_Correct(10000);
	  bsp_LedOff(1);
	 
		attitudeControllerInit();
	
		velocityControllerInit();

		IsRoverInit = true;
}
/*********************************************************************************************************
*	函 数 名: TestArRoverInit
*	功能说明: 控制初始化
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
bool TestArRoverInit(void)
{
		return IsRoverInit;
}
/*********************************************************************************************************
*	函 数 名: UpdateGps
*	功能说明: 更新GPS数据，大于5颗星才信任此次数据
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void UpdateGps(void)
{		//trust the GPS data when the satellites inview more than 5
	  if(g_tGPS.ViewNumber >= 5 && g_tGPS.PositionOk)
			_position.isvalid = true;
		
		_position.lon = g_tGPS.JingDu_Du + gps_FenToDu(g_tGPS.JingDu_Fen )/1000000;
		_position.ew = g_tGPS.EW;
		_position.lat = g_tGPS.WeiDu_Du + gps_FenToDu(g_tGPS.WeiDu_Fen)/1000000;
		_position.ns = g_tGPS.NS;
		_position.altitude = g_tGPS.Altitude;
}
/*********************************************************************************************************
*	函 数 名: UpdateAttitude
*	功能说明: 更新姿态信息
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void UpdateAttitude(void)
{

#if  AHRS_USE_DMP
		dmp_update_euler();
#else
		ahrs_update_euler();
#endif
	
}
/*********************************************************************************************************
*	函 数 名: UpdateRC
*	功能说明: 更新遥控器控制信息, 每通道0--999
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void UpdateRC(void)
{


}
/*********************************************************************************************************
*	函 数 名: RunAttitudeController
*	功能说明: 运行姿态控制器
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void RunAttitudeController(void)
{
		float yaw_pid_out = 0;
		float target_yaw_rate = 0;
	
  	//Set desired yaw rate
		SetDesiredYawRate(_rc.ch4_val);
    target_yaw_rate = _rc.ch4_val * ARROVER_LOOP_TIME;
		//set desired angle
		SetDesiredAngle(_rc.ch1_val, _rc.ch2_val, target_yaw_rate);
	
		//run the attitude controller
		yaw_pid_out = YawStabilizer();
		_motor[MOTOR_LEFT].yaw_pid_out = -yaw_pid_out;
		_motor[MOTOR_RIGHT].yaw_pid_out = yaw_pid_out;
}
/*********************************************************************************************************
*	函 数 名: RunVelocityController
*	功能说明: 运行速度控制器
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void RunVelocityController(void)
{
	  static uint8_t  counts = 0;
	  float desired_speed = 0;
	 //0-250
		desired_speed = 0.25*_rc.ch3_val ;
		//set the desired speed
		SetMotorDesiredSpeed(&_motor[MOTOR_LEFT], desired_speed /*+ _motor[MOTOR_LEFT].yaw_pid_out*/);
	  SetMotorDesiredSpeed(&_motor[MOTOR_RIGHT], desired_speed /*+ _motor[MOTOR_LEFT].yaw_pid_out*/);
	
		//run the velocity controller
		if(counts == VELOCITY_CONTROL_COUNTS)
		{
			counts = 0;
			_motor[MOTOR_LEFT].vel_pid_out = VelocityController(MOTOR_LEFT, _motor[MOTOR_LEFT].cur_speed, _motor[MOTOR_LEFT].tar_speed);
			_motor[MOTOR_RIGHT].vel_pid_out = VelocityController(MOTOR_RIGHT, _motor[MOTOR_RIGHT].cur_speed, _motor[MOTOR_RIGHT].tar_speed);
		}
		counts ++;
}
/*********************************************************************************************************
*	函 数 名: UpdateController
*	功能说明: 更新姿态控制器、速度控制器、位置控制器
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void UpdateController(void)
{
		RunAttitudeController();

		RunVelocityController();		
}
/*********************************************************************************************************
*	函 数 名: ArRoverLoop
*	功能说明: 控制循环,应当被10ms执行一次
*	形    参：
*	返 回 值: 
*********************************************************************************************************
*/
void ArRoverLoop(void)
{
		CPU_SR_ALLOC();
	  
		UpdateGps();
	
		UpdateAttitude();
	
//		UpdateRC();
		CPU_CRITICAL_ENTER();
		//每10ms计算一次车轮的平均速度
		CalcMotorSpeedAndAngle();
	
		UpdateController();

		SetMotorsPWM();
		CPU_CRITICAL_EXIT();
}
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

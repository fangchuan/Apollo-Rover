/*
*********************************************************************************************************
*
*	ģ������ : Apollorobot����ģ��
*	�ļ����� : ApolloRobotRover.c
*	��    �� : V1.0
*	˵    �� : 
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-08-18 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/  
#include "common.h"
#include "bsp_encoder.h"
#include "bsp_gps.h"
#include "bsp_motor.h"
#include "motion_driver_9150.h"
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

_Position _position;

static bool IsRoverInit = false;
/*********************************************************************************************************
*	�� �� ��: ArRoverInit
*	����˵��: ���Ƴ�ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void ArRoverInit(void)
{
		if(IsRoverInit)
			return ;
		
//		attitudeControllerInit();
	
		velocityControllerInit();

		IsRoverInit = true;
}
/*********************************************************************************************************
*	�� �� ��: TestArRoverInit
*	����˵��: ���Ƴ�ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
bool TestArRoverInit(void)
{
		return IsRoverInit;
}
/*********************************************************************************************************
*	�� �� ��: UpdateGps
*	����˵��: ����GPS���ݣ�����5���ǲ����δ˴�����
*	��    �Σ�
*	�� �� ֵ: 
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
*	�� �� ��: UpdateAttitude
*	����˵��: ������̬��Ϣ
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void UpdateAttitude(void)
{

}
/*********************************************************************************************************
*	�� �� ��: UpdateRC
*	����˵��: ����ң����������Ϣ, ÿͨ��0--999
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void UpdateRC(void)
{
	  float desired_speed = 0;
	 //0--999
		GetRCValue();
	 //0-250
		desired_speed = 0.25*_rc.ch3_val;
		//set the desired speed
		SetMotorDesiredSpeed(&_motor[MOTOR_LEFT], desired_speed);
	  SetMotorDesiredSpeed(&_motor[MOTOR_RIGHT], desired_speed);
		//set desired angle
		SetDesiredAngle(_rc.ch1_val, _rc.ch2_val, _rc.ch3_val);
}
/*********************************************************************************************************
*	�� �� ��: UpdateController
*	����˵��: ������̬���������ٶȿ�������λ�ÿ�����
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void UpdateController(void)
{
	  static uint8_t  counts = 0;
		float yaw_pid_out = 0;
		
		//run the attitude controller
		yaw_pid_out = YawStabilizer();
		_motor[MOTOR_LEFT].yaw_pid_out = yaw_pid_out;
		_motor[MOTOR_RIGHT].yaw_pid_out = yaw_pid_out;
	
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
*	�� �� ��: ArRoverLoop
*	����˵��: ����ѭ��,Ӧ����10msִ��һ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void ArRoverLoop(void)
{
		UpdateGps();
	
		UpdateAttitude();
	
		UpdateRC();
	
		//ÿ10ms����һ�γ��ֵ�ƽ���ٶ�
		CalcMotorSpeedAndAngle();
	
		UpdateController();

		SetMotorsPWM();

}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

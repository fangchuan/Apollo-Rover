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
{
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
*	����˵��: ����ң����������Ϣ
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void UpdateRC(void)
{
		GetRCValue();
	
	
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

		//set the desired speed
		SetMotorDesiredSpeed(&_motor[MOTOR_LEFT], 150);
	  SetMotorDesiredSpeed(&_motor[MOTOR_RIGHT], 150);
		
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
*	����˵��: ����ѭ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void ArRoverLoop(void)
{
		UpdateGps();
	
		UpdateAttitude();
	
		//ÿ10ms����һ�γ��ֵ�ƽ���ٶ�
		CalcMotorSpeedAndAngle();
	
		UpdateController();

		SetMotorsPWM();

}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

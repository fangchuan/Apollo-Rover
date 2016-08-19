/*
*********************************************************************************************************
*
*	ģ������ : ���ٿ���PIDģ��
*	�ļ����� : celocity_controller.c
*	��    �� : V1.0
*	˵    �� : Velocity controler using PID correctors
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "common.h"
#include "pid.h"
#include "velocity_controller.h"



PidObject  pidVelocityLeftMotor;
PidObject  pidVelocityRightMotor;
int16_t    velOutput;


extern _Motor _motor[MOTOR_MAX_NUM];

static bool isInit = false;
/*********************************************************************************************************
*	�� �� ��: velocityControllerInit
*	����˵��: �ٶȿ�����������ʼ��;
*						��ʼ����Ͳ����ٳ�ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void  velocityControllerInit(void)
{
		if(isInit)
			return;

		pidInit(&pidVelocityLeftMotor, 0, PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD, VELOCITY_UPDATE_DT);
		pidSetIntegralLimit(&pidVelocityLeftMotor, PID_VELOCITY_INTEGRATION_LIMIT);
		
		pidInit(&pidVelocityRightMotor, 0, PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD, VELOCITY_UPDATE_DT);
		pidSetIntegralLimit(&pidVelocityRightMotor, PID_VELOCITY_INTEGRATION_LIMIT);

		isInit = true;
}
/*********************************************************************************************************
*	�� �� ��: velocityControllerTest
*	����˵��: ���������Ƿ��ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
bool velocityControllerTest(void)
{
		return isInit;
}
/*********************************************************************************************************
*	�� �� ��: VelocityCorrectPID
*	����˵��: �ٶȻ�PID������
*	��    �Σ�ʵ���ٶȡ������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
int16_t VelocityCorrectPID(PidObject* VelocityPid, const float VelocityActual, float VelocityDesired)
{
		float pid;
		pidSetDesired(VelocityPid, VelocityDesired);
		pid = pidUpdate(VelocityPid, VelocityActual, true);
		velOutput = constrain(pid, -VELOCITY_CONTROLLER_OUT_MAX, VELOCITY_CONTROLLER_OUT_MAX);

		return velOutput;
}
/*********************************************************************************************************
*	�� �� ��: VelocityCorrectPID
*	����˵��: �ٶȻ�PID������
*	��    �Σ�ʵ���ٶȡ������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
int16_t VelocityController(uint8_t motor_id, const float VelocityActual, float VelocityDesired)
{
		if(motor_id == MOTOR_LEFT)
		{
			return VelocityCorrectPID(&pidVelocityLeftMotor,VelocityActual, VelocityDesired);
		}
		else
		{
			return VelocityCorrectPID(&pidVelocityRightMotor,VelocityActual, VelocityDesired);
		}
}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

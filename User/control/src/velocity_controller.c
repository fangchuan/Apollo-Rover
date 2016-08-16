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
#include "velocity_controller.h"
#include "pid.h"
#include "common.h"


PidObject  pidVelocity;
int16_t    velOutput;

extern _Motor _motor[MOTOR_MAX_NUM];

static bool isInit;
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

		pidInit(&pidVelocity, 0, PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD, VELOCITY_UPDATE_DT);
		pidSetIntegralLimit(&pidVelocity, PID_VELOCITY_INTEGRATION_LIMIT);

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
int16_t VelocityCorrectPID(float VelocityActual, float VelocityDesired)
{
		float pid;
		pidSetDesired(&pidVelocity, VelocityDesired);
		pid = pidUpdate(&pidVelocity, VelocityActual, true);
		velOutput = constrain(pid, -VELOCITY_CONTROLLER_OUT_MAX, VELOCITY_CONTROLLER_OUT_MAX);
		return velOutput;
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

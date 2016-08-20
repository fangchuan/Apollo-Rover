/*
*********************************************************************************************************
*
*	ģ������ : ��̬����PIDģ��
*	�ļ����� : attitude_controller.c
*	��    �� : V1.0
*	˵    �� : Attitude controler using PID correctors
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/

#include "attitude_controller.h"
#include "pid.h"
#include "common.h"
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;


_Vector3f	rateDesired;
_Vector3f	angleDesired;

int16_t rollOutput;//����Щȫ�ֱ�����Ϊ�˷������ʱ�۲�
int16_t pitchOutput;
int16_t yawOutput;

extern _Euler _euler;


/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
static bool isInit;


/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************************************************
*	�� �� ��: attitudeControllerInit
*	����˵��: ��̬������������ʼ��, �ǶȻ������ٶȻ�;
*						��ʼ����Ͳ����ٳ�ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void attitudeControllerInit()
{
		if(isInit)
			return;

		pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
		pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
		pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);
		pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
		pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
		pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

		pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
		pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
		pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
		pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
		pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
		pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);

		isInit = true;
}
/*********************************************************************************************************
*	�� �� ��: attitudeControllerTest
*	����˵��: ���������Ƿ��ʼ��
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
bool attitudeControllerTest()
{
		return isInit;
}
/*********************************************************************************************************
*	�� �� ��: RollCorrectRatePID
*	����˵��: ���ٶȻ�PID������
*	��    �Σ�ʵ�ʽ��ٶȡ��������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void RollCorrectRatePID(const float rollRateActual, float rollRateDesired)
{
		float pid;
		pidSetDesired(&pidRollRate, rollRateDesired);
		pid = pidUpdate(&pidRollRate, rollRateActual, true);
		rollOutput = constrain(pid, -RATE_CONTROLLER_RP_OUT_MAX, RATE_CONTROLLER_RP_OUT_MAX);
}
/*********************************************************************************************************
*	�� �� ��: SetDesiredAngle
*	����˵��: ���������Ƕ�
*	��    �Σ�rollDesired���������,  pitchDesired���������� ,yawDesired���������
*	�� �� ֵ: 
*********************************************************************************************************
*/
void SetDesiredAngle(float rollDesired, float pitchDesired, float yawDesired)
{
		angleDesired.x = rollDesired;
	  angleDesired.y = pitchDesired;
	  angleDesired.z = yawDesired;
}
/*********************************************************************************************************
*	�� �� ��: PitchCorrectRatePID
*	����˵��: ���ٶȻ�PID������
*	��    �Σ�ʵ�ʽ��ٶȡ��������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void PitchCorrectRatePID(const float pitchRateActual, float pitchRateDesired)
{
		float pid;
		pidSetDesired(&pidPitchRate, pitchRateDesired);
		pid = pidUpdate(&pidPitchRate, pitchRateActual, true);
		pitchOutput = constrain(pid, -RATE_CONTROLLER_RP_OUT_MAX, RATE_CONTROLLER_RP_OUT_MAX);
}
/*********************************************************************************************************
*	�� �� ��: YawCorrectRatePID
*	����˵��: ���ٶȻ�PID������
*	��    �Σ�ʵ�ʽ��ٶȡ��������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void YawCorrectRatePID(const float yawRateActual, float yawRateDesired)
{
	  float pid;
		pidSetDesired(&pidYawRate, yawRateDesired);
		pid = pidUpdate(&pidYawRate, yawRateActual, true);
		yawOutput = constrain(pid, -RATE_CONTROLLER_YAW_OUT_MAX, RATE_CONTROLLER_YAW_OUT_MAX);
}
/*********************************************************************************************************
*	�� �� ��: RollCorrectAttitudePID
*	����˵��: �ǶȻ�PID������
*	��    �Σ�ʵ�ʽǶȡ������Ƕȡ�  �õ����������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void RollCorrectAttitudePID(const float eulerRollActual, float eulerRollDesired, float* rollRateDesired)
{	
		// Update PID for roll axis
		pidSetDesired(&pidRoll, eulerRollDesired);
		*rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);
	
}
/*********************************************************************************************************
*	�� �� ��: PitchCorrectAttitudePID
*	����˵��: �ǶȻ�PID������
*	��    �Σ�ʵ�ʽǶȡ������Ƕȡ�  �õ����������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void PitchCorrectAttitudePID(const float eulerPitchActual, float eulerPitchDesired, float* pitchRateDesired)
{

		// Update PID for pitch axis
		pidSetDesired(&pidPitch, eulerPitchDesired);
		*pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

}
/*********************************************************************************************************
*	�� �� ��: YawCorrectAttitudePID
*	����˵��: �ǶȻ�PID������
*	��    �Σ�ʵ�ʽǶȡ������Ƕȡ�  �õ����������ٶ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void YawCorrectAttitudePID(const float eulerYawActual, float eulerYawDesired, float* yawRateDesired)
{
		float yawError;
	
		// Update PID for yaw axis
		yawError = eulerYawDesired - eulerYawActual;
		if (yawError > 180.0)
			yawError -= 360.0;
		else if (yawError < -180.0)
			yawError += 360.0;
		pidSetError(&pidYaw, yawError);
		*yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
}
/*********************************************************************************************************
*	�� �� ��: attitudeControllerResetAllPID
*	����˵��: ����ǶȻ������ٶȻ�PID������
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void attitudeControllerResetAllPID(void)
{
		pidReset(&pidRoll);
		pidReset(&pidPitch);
		pidReset(&pidYaw);
		pidReset(&pidRollRate);
		pidReset(&pidPitchRate);
		pidReset(&pidYawRate);
}
/*********************************************************************************************************
*	�� �� ��: attitudeControllerGetActuatorOutput
*	����˵��: PID������ʵ�����
*	��    �Σ�*roll    *pitch   *yaw
*	�� �� ֵ: 
*********************************************************************************************************
*/
void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
		*roll = rollOutput;
		*pitch = pitchOutput;
		*yaw = yawOutput;
}

/*********************************************************************************************************
*	�� �� ��: RollStabilizer
*	����˵��: �����roll�Ŀ���
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
int16_t RollStabilizer(void)
{
		RollCorrectAttitudePID(_euler.roll, angleDesired.x, &rateDesired.x);	
		RollCorrectRatePID(_euler.rate[0], rateDesired.x);
		return rollOutput;
}
/*********************************************************************************************************
*	�� �� ��: PitchStabilizer
*	����˵��: ������pitch�Ŀ���
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
int16_t PitchStabilizer(void)
{
		PitchCorrectAttitudePID(_euler.pitch, angleDesired.y, &rateDesired.y);	
		PitchCorrectRatePID(_euler.rate[1], rateDesired.y);
		return pitchOutput;
}
/*********************************************************************************************************
*	�� �� ��: YawStabilizer
*	����˵��: ƫ����yaw�Ŀ���
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
int16_t YawStabilizer(void)
{
		YawCorrectAttitudePID(_euler.yaw, angleDesired.z, &rateDesired.z);	
		YawCorrectRatePID(_euler.rate[2], rateDesired.z);
		return yawOutput;
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

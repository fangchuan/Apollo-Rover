/*
*********************************************************************************************************
*
*	ģ������ : ��̬����PIDģ��
*	�ļ����� : attitude_controller.h
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

#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   						500
#define IMU_UPDATE_DT     						(float)(1.0/IMU_UPDATE_FREQ)
#define RATE_CONTROLLER_RP_OUT_MAX    4500
#define RATE_CONTROLLER_YAW_OUT_MAX   5000
	
/* Exported functions ------------------------------------------------------- */

extern void attitudeControllerInit(void);
extern bool attitudeControllerTest(void);
extern void SetDesiredAngle(float rollDesired, float pitchDesired, float yawDesired);
//void RollCorrectRatePID(float rollRateActual, float rollRateDesired);
//void PitchCorrectRatePID(float pitchRateActual, float pitchRateDesired);
//void YawCorrectRatePID(float yawRateActual, float yawRateDesired);
//void RollCorrectAttitudePID(float eulerRollActual, float eulerRollDesired, float* rollRateDesired);
//void PitchCorrectAttitudePID(float eulerPitchActual, float eulerPitchDesired, float* pitchRateDesired);
//void YawCorrectAttitudePID(float eulerYawActual, float eulerYawDesired, float* yawRateDesired);
extern int16_t RollStabilizer(void);
extern int16_t PitchStabilizer(void);
extern int16_t YawStabilizer(void);
extern void attitudeControllerResetAllPID(void);
extern void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);


#endif /* ATTITUDE_CONTROLLER_H_ */
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

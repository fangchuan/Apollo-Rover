/*
*********************************************************************************************************
*
*	ģ������ : ��̬����ģ��
*	�ļ����� : ahrs.h
*	��    �� : V1.0
*	˵    �� : ���������̬:Roll,Pitch,Yaw
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-08_05    fc   ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ�  www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef  __AHRS_H
#define  __AHRS_H


void get_compass_bias(void);
void compass_calibration(void);
void Init_MPU9150(void);
void init_quaternion(void);
void ahrs_update_euler(void);

#endif

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

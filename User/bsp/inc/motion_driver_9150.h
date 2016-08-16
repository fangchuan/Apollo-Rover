/*
*********************************************************************************************************
*
*	ģ������ : MPU9150  DMP����ģ��
*	�ļ����� : motion_driver_9150.h
*	��    �� : V5.1
*	˵    �� : 
*
*	Copyright (C), 2015-2016, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef  __MOTION_DRIVER_9150_H
#define  __MOTION_DRIVER_9150_H

#include "common.h"



struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
};

void gyro_data_ready_cb(void);;

extern int8_t dmp_init(void);
extern void dmp_update_euler(void);
extern void  DispEulerData(void);

#endif
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

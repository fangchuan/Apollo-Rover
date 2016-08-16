/*
*********************************************************************************************************
*
*	模块名称 : 姿态解算模块
*	文件名称 : ahrs.h
*	版    本 : V1.0
*	说    明 : 软件结算姿态:Roll,Pitch,Yaw
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-08_05    fc   正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技  www.apollorobot.cn
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

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

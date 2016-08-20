/*
*********************************************************************************************************
*
*	模块名称 : 日志输出模块
*	文件名称 : log.h
*	版    本 : V1.0
*	说    明 : 
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef __LOG_H
#define __LOG_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stm32f10x.h"

/* Exported functions ------------------------------------------------------- */
extern void  bsp_SDLogInit(void);
extern void  LogTest(void);
extern void  LogEncoderData(void* enc);
extern void  LogGpsData(void* gps);
extern void  LogMotorData(void);
extern void  LogRTCData(void);
extern void  LogMpu9150Data(void);
extern void  LogRcData(void);
#endif /*LOG_H*/
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

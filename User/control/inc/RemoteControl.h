/*
*********************************************************************************************************
*
*	模块名称 : RC遥控接收模块
*	文件名称 : RemoteControl.h
*	版    本 : V1.0
*	说    明 : 天地飞8通接收机，输出PWM波
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-07-21 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef  __REMOTE_CONTROL_H
#define  __REMOTE_CONTROL_H

#include "stm32f10x.h"

/********************Export Functions***********************************/

extern void  bsp_ReceiverInit(void);
extern void  RcDataInit(void);
extern void  GetRCValue(void);
extern void  DispRcData(void);
#endif
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

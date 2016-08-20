/*
*********************************************************************************************************
*
*	ģ������ : ��־���ģ��
*	�ļ����� : log.h
*	��    �� : V1.0
*	˵    �� : 
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
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
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

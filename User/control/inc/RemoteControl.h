/*
*********************************************************************************************************
*
*	ģ������ : RCң�ؽ���ģ��
*	�ļ����� : RemoteControl.h
*	��    �� : V1.0
*	˵    �� : ��ط�8ͨ���ջ������PWM��
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-07-21 ����  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
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
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	模块名称 : 三轴磁力计HMC5883L驱动模块
*	文件名称 : bsp_HMC5883L.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2012-10-12 armfly  ST固件库版本 V2.1.0
*
*	Copyright (C), 2015-2020, 阿波罗科技  www.apollorobot.cn
*
*********************************************************************************************************
*/

#ifndef _BSP_HMC5883L_H
#define _BSP_HMC5883L_H

#define HMC5883L_SLAVE_ADDRESS        0x1E/* 都采用7bit地址 */
#define HMC5883L_CONFIG_A_REGISTER    0x00
#define HMC5883L_CONFIG_B_REGISTER    0x01
#define HMC5883L_MODE_REGISTER        0x02
#define HMC5883L_X_OUT_REGISTER       0x03
#define HMC5883L_Y_OUT_REGISTER       0x07
#define HMC5883L_Z_OUT_REGISTER       0x05
#define HMC5883L_STATU_REGITER        0x09
#define HMC5883L_ID_REGISTER          0x0A

#define HMC5883L_CONFIG_A_DEFAULT     0x70
#define HMC5883L_CONFIG_B_DEFAULT     0x20
#define HMC5883L_MODE_DEFAULT         0x00
#define HMC5883L_ID_A_VALUE           0x48
#define HMC5883L_ID_B_VALUE           0x34
#define HMC5883L_ID_C_VALUE           0x33
//****************************************
// 定义HMC5883L内部地址
//****************************************
#define DATA_OUT_X		0x03

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;

	int16_t X_Min;
	int16_t Y_Min;
	int16_t Z_Min;

	int16_t X_Max;
	int16_t Y_Max;
	int16_t Z_Max;	

	float X_Offset;
	float Y_Offset;
	float Z_Offset;
	
	float X_Gain;
	float Y_Gain;
	float Z_Gain;
//	uint8_t	Status;
//	
//	uint8_t CfgRegA;
//	uint8_t CfgRegB;
//	uint8_t CfgRegC;
//	uint8_t ModeReg;
//	
//	uint8_t IDReg[3];	
}_Hmc5883l;


extern void bsp_InitHMC5883L(void);
extern void HMC5883L_GetRaw(int16_t *x,int16_t *y,int16_t *z);
extern void HMC5883L_Correct(uint32_t ms_counts);
extern void HMC5883_Check(void) ;
extern void DispHmc5883Data(void);
#endif

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/


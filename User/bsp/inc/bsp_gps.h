/*
*********************************************************************************************************
*
*	模块名称 : GPS定位模块驱动程序
*	文件名称 : bsp_gps.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2015-2020, 阿波罗科技  www.apollorobot.cn
*
*********************************************************************************************************
*/

#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stm32f10x.h"

typedef struct
{
	uint8_t UartOk;		/* 1表示串口数据接收正常，即可以收到GPS的命令字符串 */
	
	/* 从GGA 命令 : 时间、位置、定位类型  (位置信息可有 RMC命令获得) */

	/* 定位有效标志, 0:未定位 1:SPS模式，定位有效 2:差分，SPS模式，定位有效 3:PPS模式，定位有效 */
	uint8_t PositionOk;
	uint32_t Altitude;	/* 海拔高度，相对海平面多少米   x.x  */

	/* GLL：经度、纬度、UTC时间 */
	/* 样例数据：$GPGLL,3723.2475,N,12158.3416,W,161229.487,A*2C */


	/* GSA：GPS接收机操作模式，定位使用的卫星，DOP值 */
	/* 样例数据：$GPGSA,A,3,07,02,26,27,09,04,15, , , , , ,1.8,1.0,1.5*33 */
	uint8_t ModeAM;				/* M=手动（强制操作在2D或3D模式），A=自动 */
	uint8_t Mode2D3D;			/* 定位类型 1=未定位，2=2D定位，3=3D定位 */
	uint8_t SateID[12];			/* ID of 1st satellite used for fix */
	uint16_t PDOP;				/* 位置精度, 0.1米 */
	uint16_t HDOP;				/* 水平精度, 0.1米 */
	uint16_t VDOP;				/* 垂直精度, 0.1米 */

	/* GSV：可见GPS卫星信息、仰角、方位角、信噪比（SNR） */
	/*
		$GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
		$GPGSV,2,2,07,09,23,313,42,04,19,159,41,15,12,041,42*41
	*/
	uint8_t ViewNumber;			/* 可见卫星个数 */
	uint8_t Elevation[12];		/* 仰角 elevation in degrees  度,最大90°*/
	uint16_t Azimuth[12];		/* 方位角 azimuth in degrees to true  度,0-359°*/
	uint8_t SNR[12];			/* 载噪比（C/No）  信噪比？ ， dBHz   范围0到99，没有跟踪时为空 */

	/* RMC：时间、日期、位置	、速度 */
	/* 样例数据：$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10 */
	uint16_t WeiDu_Du;			/* 纬度，度 */
	uint32_t WeiDu_Fen;			/* 纬度，分. 232475；  小数点后4位, 表示 23.2475 分 */
	char     NS;				/* 纬度半球N（北半球）或S（南半球） */

	uint16_t JingDu_Du;			/* 经度，单位度 */
	uint32_t JingDu_Fen;		/* 经度，单位分 */
	char     EW;				/* 经度半球E（东经）或W（西经） */

	uint16_t Year;				/* 日期 120598 ddmmyy */
	uint8_t  Month;
	uint8_t  Day;
	uint8_t  Hour;				/* UTC 时间。 hhmmss.sss */
	uint8_t  Min;				/* 分 */
	uint8_t  Sec;				/* 秒 */
	uint16_t mSec;				/* 毫秒 */
	char   TimeOk;				/* A=UTC时间数据有效；V=数据无效 */

	/* VTG：地面速度信息 */
	/* 样例数据：$GPVTG,309.62,T, ,M,0.13,N,0.2,K*6E */
	uint16_t TrackDegTrue;		/* 以真北为参考基准的地面航向（000~359度，前面的0也将被传输） */
	uint16_t TrackDegMag;		/* 以磁北为参考基准的地面航向（000~359度，前面的0也将被传输） */
	uint32_t SpeedKnots;		/* 地面速率（000.0~999.9节，前面的0也将被传输） 1 knots = 1.852km/h*/
	uint32_t SpeedKM;			/* 地面速率（000.0~999.9节，前面的0也将被传输） */

}GPS_T;
//UBLOX NEO-6M 配置(清除,保存,加载等)结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG CFG ID:0X0906 (小端模式)
	u16 dlength;				//数据长度 12/13
	u32 clearmask;				//子区域清除掩码(1有效)
	u32 savemask;				//子区域保存掩码
	u32 loadmask;				//子区域加载掩码
	u8  devicemask; 		  	//目标器件选择掩码	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//校验CK_A 							 	 
	u8  ckb;			 		//校验CK_B							 	 
}_ublox_cfg_cfg; 

//UBLOX NEO-6M 消息设置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG MSG ID:0X0106 (小端模式)
	u16 dlength;				//数据长度 8
	u8  msgclass;				//消息类型(F0 代表NMEA消息格式)
	u8  msgid;					//消息 ID 
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC消输出设置    0,关闭;1,使能.
	u8  uart1set;				//UART1输出设置	   0,关闭;1,使能.
	u8  uart2set;				//UART2输出设置	   0,关闭;1,使能.
	u8  usbset;					//USB输出设置	   0,关闭;1,使能.
	u8  spiset;					//SPI输出设置	   0,关闭;1,使能.
	u8  ncset;					//未知输出设置	   默认为1即可.
 	u8  cka;			 		//校验CK_A 							 	 
	u8  ckb;			    	//校验CK_B							 	 
}_ublox_cfg_msg; 

//UBLOX NEO-6M UART端口设置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG PRT ID:0X0006 (小端模式)
	u16 dlength;				//数据长度 20
	u8  portid;					//端口号,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//保留,设置为0
	u16 txready;				//TX Ready引脚设置,默认为0
	u32 mode;					//串口工作模式设置,奇偶校验,停止位,字节长度等的设置.
 	u32 baudrate;				//波特率设置
 	u16 inprotomask;		 	//输入协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 outprotomask;		 	//输出协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 reserved4; 				//保留,设置为0
 	u16 reserved5; 				//保留,设置为0 
 	u8  cka;			 		//校验CK_A 							 	 
	u8  ckb;			    	//校验CK_B							 	 
}_ublox_cfg_prt; 

//UBLOX NEO-6M 时钟脉冲配置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG TP ID:0X0706 (小端模式)
	u16 dlength;				//数据长度
	u32 interval;				//时钟脉冲间隔,单位为us
	u32 length;				 	//脉冲宽度,单位为us
	signed char status;			//时钟脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.			  
	u8 timeref;			   		//参考时间:0,UTC时间;1,GPS时间;2,当地时间.
	u8 flags;					//时间脉冲设置标志
	u8 reserved;				//保留			  
 	signed short antdelay;	 	//天线延时
 	signed short rfdelay;		//RF延时
	signed int userdelay; 	 	//用户延时	
	u8 cka;						//校验CK_A 							 	 
	u8 ckb;						//校验CK_B							 	 
}_ublox_cfg_tp; 

//UBLOX NEO-6M 刷新速率配置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG RATE ID:0X0806 (小端模式)
	u16 dlength;				//数据长度
	u16 measrate;				//测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
	u16 navrate;				//导航速率（周期），固定为1
	u16 timeref;				//参考时间：0=UTC Time；1=GPS Time；
 	u8  cka;					//校验CK_A 							 	 
	u8  ckb;					//校验CK_B							 	 
}_ublox_cfg_rate; 

void bsp_InitGPS(void);
void gps_analyze(void);
uint32_t gps_FenToDu(uint32_t _fen);
uint16_t gps_FenToMiao(uint32_t _fen);
void DispGPSStatus(void);
extern GPS_T g_tGPS;

#endif

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

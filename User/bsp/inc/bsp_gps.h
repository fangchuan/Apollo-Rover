/*
*********************************************************************************************************
*
*	ģ������ : GPS��λģ����������
*	�ļ����� : bsp_gps.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2020, �����޿Ƽ�  www.apollorobot.cn
*
*********************************************************************************************************
*/

#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stm32f10x.h"

typedef struct
{
	uint8_t UartOk;		/* 1��ʾ�������ݽ����������������յ�GPS�������ַ��� */
	
	/* ��GGA ���� : ʱ�䡢λ�á���λ����  (λ����Ϣ���� RMC������) */

	/* ��λ��Ч��־, 0:δ��λ 1:SPSģʽ����λ��Ч 2:��֣�SPSģʽ����λ��Ч 3:PPSģʽ����λ��Ч */
	uint8_t PositionOk;
	uint32_t Altitude;	/* ���θ߶ȣ���Ժ�ƽ�������   x.x  */

	/* GLL�����ȡ�γ�ȡ�UTCʱ�� */
	/* �������ݣ�$GPGLL,3723.2475,N,12158.3416,W,161229.487,A*2C */


	/* GSA��GPS���ջ�����ģʽ����λʹ�õ����ǣ�DOPֵ */
	/* �������ݣ�$GPGSA,A,3,07,02,26,27,09,04,15, , , , , ,1.8,1.0,1.5*33 */
	uint8_t ModeAM;				/* M=�ֶ���ǿ�Ʋ�����2D��3Dģʽ����A=�Զ� */
	uint8_t Mode2D3D;			/* ��λ���� 1=δ��λ��2=2D��λ��3=3D��λ */
	uint8_t SateID[12];			/* ID of 1st satellite used for fix */
	uint16_t PDOP;				/* λ�þ���, 0.1�� */
	uint16_t HDOP;				/* ˮƽ����, 0.1�� */
	uint16_t VDOP;				/* ��ֱ����, 0.1�� */

	/* GSV���ɼ�GPS������Ϣ�����ǡ���λ�ǡ�����ȣ�SNR�� */
	/*
		$GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
		$GPGSV,2,2,07,09,23,313,42,04,19,159,41,15,12,041,42*41
	*/
	uint8_t ViewNumber;			/* �ɼ����Ǹ��� */
	uint8_t Elevation[12];		/* ���� elevation in degrees  ��,���90��*/
	uint16_t Azimuth[12];		/* ��λ�� azimuth in degrees to true  ��,0-359��*/
	uint8_t SNR[12];			/* ����ȣ�C/No��  ����ȣ� �� dBHz   ��Χ0��99��û�и���ʱΪ�� */

	/* RMC��ʱ�䡢���ڡ�λ��	���ٶ� */
	/* �������ݣ�$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10 */
	uint16_t WeiDu_Du;			/* γ�ȣ��� */
	uint32_t WeiDu_Fen;			/* γ�ȣ���. 232475��  С�����4λ, ��ʾ 23.2475 �� */
	char     NS;				/* γ�Ȱ���N�������򣩻�S���ϰ��� */

	uint16_t JingDu_Du;			/* ���ȣ���λ�� */
	uint32_t JingDu_Fen;		/* ���ȣ���λ�� */
	char     EW;				/* ���Ȱ���E����������W�������� */

	uint16_t Year;				/* ���� 120598 ddmmyy */
	uint8_t  Month;
	uint8_t  Day;
	uint8_t  Hour;				/* UTC ʱ�䡣 hhmmss.sss */
	uint8_t  Min;				/* �� */
	uint8_t  Sec;				/* �� */
	uint16_t mSec;				/* ���� */
	char   TimeOk;				/* A=UTCʱ��������Ч��V=������Ч */

	/* VTG�������ٶ���Ϣ */
	/* �������ݣ�$GPVTG,309.62,T, ,M,0.13,N,0.2,K*6E */
	uint16_t TrackDegTrue;		/* ���汱Ϊ�ο���׼�ĵ��溽��000~359�ȣ�ǰ���0Ҳ�������䣩 */
	uint16_t TrackDegMag;		/* �Դű�Ϊ�ο���׼�ĵ��溽��000~359�ȣ�ǰ���0Ҳ�������䣩 */
	uint32_t SpeedKnots;		/* �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩 1 knots = 1.852km/h*/
	uint32_t SpeedKM;			/* �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩 */

}GPS_T;
//UBLOX NEO-6M ����(���,����,���ص�)�ṹ��
__packed typedef struct  
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG CFG ID:0X0906 (С��ģʽ)
	u16 dlength;				//���ݳ��� 12/13
	u32 clearmask;				//�������������(1��Ч)
	u32 savemask;				//�����򱣴�����
	u32 loadmask;				//�������������
	u8  devicemask; 		  	//Ŀ������ѡ������	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//У��CK_A 							 	 
	u8  ckb;			 		//У��CK_B							 	 
}_ublox_cfg_cfg; 

//UBLOX NEO-6M ��Ϣ���ýṹ��
__packed typedef struct  
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG MSG ID:0X0106 (С��ģʽ)
	u16 dlength;				//���ݳ��� 8
	u8  msgclass;				//��Ϣ����(F0 ����NMEA��Ϣ��ʽ)
	u8  msgid;					//��Ϣ ID 
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC���������    0,�ر�;1,ʹ��.
	u8  uart1set;				//UART1�������	   0,�ر�;1,ʹ��.
	u8  uart2set;				//UART2�������	   0,�ر�;1,ʹ��.
	u8  usbset;					//USB�������	   0,�ر�;1,ʹ��.
	u8  spiset;					//SPI�������	   0,�ر�;1,ʹ��.
	u8  ncset;					//δ֪�������	   Ĭ��Ϊ1����.
 	u8  cka;			 		//У��CK_A 							 	 
	u8  ckb;			    	//У��CK_B							 	 
}_ublox_cfg_msg; 

//UBLOX NEO-6M UART�˿����ýṹ��
__packed typedef struct  
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG PRT ID:0X0006 (С��ģʽ)
	u16 dlength;				//���ݳ��� 20
	u8  portid;					//�˿ں�,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//����,����Ϊ0
	u16 txready;				//TX Ready��������,Ĭ��Ϊ0
	u32 mode;					//���ڹ���ģʽ����,��żУ��,ֹͣλ,�ֽڳ��ȵȵ�����.
 	u32 baudrate;				//����������
 	u16 inprotomask;		 	//����Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 outprotomask;		 	//���Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 reserved4; 				//����,����Ϊ0
 	u16 reserved5; 				//����,����Ϊ0 
 	u8  cka;			 		//У��CK_A 							 	 
	u8  ckb;			    	//У��CK_B							 	 
}_ublox_cfg_prt; 

//UBLOX NEO-6M ʱ���������ýṹ��
__packed typedef struct  
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG TP ID:0X0706 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u32 interval;				//ʱ��������,��λΪus
	u32 length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.			  
	u8 timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	u8 flags;					//ʱ���������ñ�־
	u8 reserved;				//����			  
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ	
	u8 cka;						//У��CK_A 							 	 
	u8 ckb;						//У��CK_B							 	 
}_ublox_cfg_tp; 

//UBLOX NEO-6M ˢ���������ýṹ��
__packed typedef struct  
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG RATE ID:0X0806 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u16 measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
	u16 navrate;				//�������ʣ����ڣ����̶�Ϊ1
	u16 timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
 	u8  cka;					//У��CK_A 							 	 
	u8  ckb;					//У��CK_B							 	 
}_ublox_cfg_rate; 

void bsp_InitGPS(void);
void gps_analyze(void);
uint32_t gps_FenToDu(uint32_t _fen);
uint16_t gps_FenToMiao(uint32_t _fen);
void DispGPSStatus(void);
extern GPS_T g_tGPS;

#endif

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

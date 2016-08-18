/*
*********************************************************************************************************
*
*	ģ������ : ��־���ģ��
*	�ļ����� : log.c
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
#include "log.h"
#include "common.h"
#include "ff.h"
#include "diskio.h"
#include "bsp_sdio_sd.h"
#include "bsp_gps.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_cpu_rtc.h"
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
FATFS 					fs;
FRESULT log_result;
FIL 			log_file;
UINT 				log_bw;
UINT    		log_br;
extern  _Encoder _encoder;
extern  _Motor   _motor[MOTOR_MAX_NUM];
extern  GPS_T 			g_tGPS;
extern  RTC_T g_tRTC;
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/

static  char test_text[] = "APOLLO ROVER Log Test!\r\n";
static  char text_buffer[50] = {0};
static uint32_t  log_rows = 0;
/*********************************************************************
*
*       Public Function
*
**********************************************************************
*/
/*
*********************************************************************************************************
*	�� �� ��: bsp_SDLogInit
*	����˵��: SD��־��ʼ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SDLogInit(void)
{
	/* SD�����ļ�ϵͳ */
	log_result = f_mount(&fs, FS_VOLUME_SD, 1);
	
	if( log_result == FR_NO_FILESYSTEM )//����ô���û�б���ʽ��ΪFatFS�����ʽ����
  {
        log_result = f_mkfs(FS_VOLUME_SD, 0, 4096); //��ʽ����ʽΪFDISK,����������4096Ϊÿ���صĴ�С
        log_result = f_mount(&fs, FS_VOLUME_SD, 0);
        log_result = f_mount(&fs, FS_VOLUME_SD, 1);
  }
	//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	if ( log_result == FR_OK )
	{
			log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			if(log_result != FR_OK)
				return ;
				
	}
		f_close(&log_file);
}
/*
*********************************************************************************************************
*	�� �� ��: LogTest
*	����˵��: �����־�Ĳ���
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void  LogTest(void)
{
		//test the SD Filesystem read or write
		log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if ( log_result == FR_OK )
		{
				log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			  if(log_result != FR_OK)
					return ;
				
		}
		f_close(&log_file);
}
/*
*********************************************************************************************************
*	�� �� ��: LogEncoderData
*	����˵��: ������������ݵ���־
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void LogEncoderData(void* enc)
{
	  _Encoder *_encoder = enc;
		sprintf(text_buffer, "Encoder Data: Left %d Right %d \n",
													_encoder->dis_count[MOTOR_LEFT],
													_encoder->dis_count[MOTOR_RIGHT]  );
		log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (log_result != FR_OK)
			return ;

		log_result = f_lseek(&log_file, log_rows * sizeof(text_buffer));
		log_rows ++;
		if ( log_result == FR_OK )
		{
				log_result = f_write(&log_file, text_buffer, sizeof(text_buffer), &log_bw);
			  if(log_result != FR_OK)
					return ;
				
		}
		f_close(&log_file);
}
/*
*********************************************************************************************************
*	�� �� ��: LogGpsData
*	����˵��: ���GPS���ݵ���־
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void LogGpsData(void* gps)
{
	  GPS_T * g_tGPS = gps;
		sprintf(text_buffer, "GPS Data: Lon %d %c Lat %d %c\n", 
													g_tGPS->JingDu_Du, g_tGPS->EW, 
													g_tGPS->WeiDu_Du, g_tGPS->NS   );
		log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if(log_result != FR_OK)
			return ;
		
		log_result = f_lseek(&log_file, log_rows * sizeof(text_buffer));
		log_rows ++;
		if ( log_result == FR_OK )
		{
				log_result = f_write(&log_file, text_buffer, sizeof(text_buffer), &log_bw);
			  if(log_result != FR_OK)
					return ;
				
		}
		f_close(&log_file);
}
/*
*********************************************************************************************************
*	�� �� ��: LogMotorData
*	����˵��: ���������ݵ���־
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void LogMotorData(void)
{
		sprintf(text_buffer, "Motor Speed : Left %d  Right %d \r\n", 
													(int)_motor[MOTOR_LEFT].cur_speed, 
													(int)_motor[MOTOR_RIGHT].cur_speed    );
		log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if(log_result != FR_OK)
			return ;
		
		log_result = f_lseek(&log_file, log_rows * sizeof(text_buffer));
		log_rows ++;
		if ( log_result == FR_OK )
		{
				log_result = f_write(&log_file, text_buffer, sizeof(text_buffer), &log_bw);
			  if(log_result != FR_OK)
					return ;
				
		}
		f_close(&log_file);
}
/*
*********************************************************************************************************
*	�� �� ��: LogRTCData
*	����˵��: ���ʵʱʱ�����ݵ���־
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void LogRTCData(void)
{
		RTC_ReadClock();
	
		sprintf(text_buffer, "RTC Data : %d - %d - %d - %d - %d - %d\r\n", 
													g_tRTC.Year, g_tRTC.Mon, g_tRTC.Day,
													g_tRTC.Hour, g_tRTC.Min, g_tRTC.Sec);
		log_result = f_open(&log_file, APOLLOROBOT_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if(log_result != FR_OK)
			return ;
		
		log_result = f_lseek(&log_file, log_rows * sizeof(text_buffer));
		log_rows ++;
		if ( log_result == FR_OK )
		{
				log_result = f_write(&log_file, text_buffer, sizeof(text_buffer), &log_bw);
			  if(log_result != FR_OK)
					return ;
				
		}
		f_close(&log_file);
}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

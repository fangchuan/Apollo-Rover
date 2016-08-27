/*
*********************************************************************************************************
*
*	模块名称 : 日志输出模块
*	文件名称 : log.c
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
#include "log.h"
#include "common.h"
#include "ff.h"
#include "diskio.h"
#include "bsp_sdio_sd.h"
#include "bsp_gps.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_cpu_rtc.h"
#include "RemoteControl.h"

#define  APOLLOROBOT_ENCODER_LOG  "log_encoder.txt"
#define  APOLLOROBOT_MPU9150_LOG  "log_mpu9150.txt"
#define  APOLLOROBOT_RTC_LOG      "log_rtc.txt"
#define  APOLLOROBOT_MOTOR_LOG    "log_motor.txt"
#define  APOLLOROBOT_RC_LOG       "log_rc.txt"
#define  APOLLOROBOT_GPS_LOG      "log_gps.txt"
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
extern  _Euler   _euler;
extern  _RemoteControl _rc;
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
*	函 数 名: bsp_SDLogInit
*	功能说明: SD日志初始化
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SDLogInit(void)
{
	/* SD挂载文件系统 */
	log_result = f_mount(&fs, FS_VOLUME_SD, 1);
	
	if( log_result == FR_NO_FILESYSTEM )//如果该磁盘没有被格式化为FatFS，则格式化它
  {
        log_result = f_mkfs(FS_VOLUME_SD, 0, 4096); //格式化方式为FDISK,建立分区表，4096为每个簇的大小
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
	
		//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_ENCODER_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	if ( log_result == FR_OK )
	{
			log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			if(log_result != FR_OK)
				return ;
				
	}
		f_close(&log_file);
	
		//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_MOTOR_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	if ( log_result == FR_OK )
	{
			log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			if(log_result != FR_OK)
				return ;
				
	}
		f_close(&log_file);
	
		//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_RC_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	if ( log_result == FR_OK )
	{
			log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			if(log_result != FR_OK)
				return ;
				
	}
		f_close(&log_file);
	
		//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_GPS_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	if ( log_result == FR_OK )
	{
			log_result = f_write(&log_file, test_text, sizeof(test_text), &log_bw);
			if(log_result != FR_OK)
				return ;
				
	}
		f_close(&log_file);
	
			//overwrite the file if  it is exist
	log_result = f_open(&log_file, APOLLOROBOT_MPU9150_LOG, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
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
*	函 数 名: LogTest
*	功能说明: 输出日志的测试
*	形    参:  无
*	返 回 值: 无
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
*	函 数 名: LogEncoderData
*	功能说明: 输出编码器数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogEncoderData(void* enc)
{
	  _Encoder *_encoder = enc;
	  memset(text_buffer, 0, sizeof(text_buffer));
		sprintf(text_buffer, "Encoder Data: Left %d Right %d \n",
													_encoder->dis_count[MOTOR_LEFT],
													_encoder->dis_count[MOTOR_RIGHT]  );
		log_result = f_open(&log_file, APOLLOROBOT_ENCODER_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
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
*	函 数 名: LogGpsData
*	功能说明: 输出GPS数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogGpsData(void* gps)
{
	  GPS_T * g_tGPS = gps;
	  memset(text_buffer, 0, sizeof(text_buffer));
		sprintf(text_buffer, "GPS Data: Lon %d %c Lat %d %c\n", 
													g_tGPS->JingDu_Du, g_tGPS->EW, 
													g_tGPS->WeiDu_Du, g_tGPS->NS   );
		log_result = f_open(&log_file, APOLLOROBOT_GPS_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
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
*	函 数 名: LogMotorData
*	功能说明: 输出电机数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogMotorData(void)
{
	  memset(text_buffer, 0, sizeof(text_buffer));
		sprintf(text_buffer, "Motor Speed : Left %d  Right %d \r\n", 
													(int)_motor[MOTOR_LEFT].cur_speed, 
													(int)_motor[MOTOR_RIGHT].cur_speed    );
		log_result = f_open(&log_file, APOLLOROBOT_MOTOR_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
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
*	函 数 名: LogRTCData
*	功能说明: 输出实时时钟数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogRTCData(void)
{
		RTC_ReadClock();
	  memset(text_buffer, 0, sizeof(text_buffer));
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
/*
*********************************************************************************************************
*	函 数 名: LogMpu9150Data
*	功能说明: 输出9150数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogMpu9150Data(void)
{
	  memset(text_buffer, 0, sizeof(text_buffer));
		sprintf(text_buffer, "Roll: %d  Pitch: %d  Yaw: %d \r\n", 
												(int)_euler.roll, (int)_euler.pitch, (int)_euler.yaw);
		log_result = f_open(&log_file, APOLLOROBOT_MPU9150_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
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
*	函 数 名: LogRcData
*	功能说明: 输出遥控器数据到日志
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void LogRcData(void)
{
	   GetRCValue();
		memset(text_buffer, 0, sizeof(text_buffer));
		sprintf(text_buffer, "Channel1: %d   Channel2: %d   Channel3: %d   Channel4: %d\r\n", 
												(int)_rc.ch1_val, (int)_rc.ch2_val, _rc.ch3_val, (int)_rc.ch4_val);
		log_result = f_open(&log_file, APOLLOROBOT_RC_LOG, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
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
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

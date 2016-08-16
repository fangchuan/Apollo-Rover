#include "log.h"
#include "common.h"
#include "ff.h"
#include "bsp_sdio_sd.h"
#include "bsp_gps.h"
#include "bsp_encoder.h"

/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
FRESULT log_result;
FIL 		log_file;
UINT 		log_bw;
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
static  char test_text[] = "APOLLO ROVER Log Test!";
/*********************************************************************
*
*       Public Function
*
**********************************************************************
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

void LogEncoderData(void* enc)
{
		
}

void LogGpsData(void* gps)
{
		
}

void LogMotorData(void* motor)
{
		
}
/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	˵    �� : ���ǵײ�����ģ�����е�h�ļ��Ļ����ļ��� Ӧ�ó���ֻ�� #include bsp.h ���ɣ�
*			  ����Ҫ#include ÿ��ģ��� h �ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

/* ʹ����Դ�ļ���ʹ��uCOS-II�ĺ���, �����Դ�ļ���Ҫ��ָBSP�����ļ� */
#define uCOS_EN       1

#if uCOS_EN == 1    
	#include "os.h"   

	#define  ENABLE_INT()      OS_CRITICAL_EXIT()     /* ʹ��ȫ���ж� */
	#define  DISABLE_INT()     OS_CRITICAL_ENTER()    /* ��ֹȫ���ж� */
#else
	/* ����ȫ���жϵĺ� */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */
#endif


#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/*
	EXTI9_5_IRQHandler ���жϷ�������ɢ�ڼ��������� bsp�ļ��С�
	��Ҫ���ϵ� stm32f4xx_it.c �С�

	���������б�ʾEXTI9_5_IRQHandler��ں������зŵ� stm32f4xx_it.c��
*/
#define EXTI9_5_ISR_MOVE_OUT

#define DEBUG_GPS_TO_COM1	/* ��ӡGPS���ݵ�����1 */

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
#include "bsp_uart_fifo.h"
#include "bsp_led.h"
//#include "bsp_timer.h"
//#include "bsp_key.h"
//#include "bsp_msg.h"
//#include "bsp_beep.h"
#include "bsp_tim_pwm.h"
#include "bsp_cpu_flash.h"
#include "bsp_sdio_sd.h"
#include "bsp_i2c_gpio.h"
#include "bsp_eeprom_24xx.h"
//#include "bsp_si4730.h"
//#include "bsp_hmc5883l.h"
//#include "bsp_mpu6050.h"
//#include "bsp_bh1750.h"
//#include "bsp_bmp180.h"
//#include "bsp_wm8978.h"
//#include "bsp_gt811.h"
#include "bsp_fsmc_sram.h"
#include "bsp_nand_flash.h"
//#include "bsp_nor_flash.h"
//#include "LCD_RA8875.h"
//#include "LCD_SPFD5420.h"
//#include "LCD_ILI9488.h"
//#include "bsp_ra8875_port.h"
//#include "bsp_tft_lcd.h"
//#include "bsp_touch.h"
//#include "bsp_oled.h"
//#include "bsp_sim800.h"
//#include "bsp_ra8875_flash.h"
#include "bsp_spi_bus.h"
//#include "bsp_spi_flash.h"
//#include "bsp_tm7705.h"
//#include "bsp_vs1053b.h"
//#include "bsp_tsc2046.h"
//#include "bsp_ds18b20.h"
//#include "bsp_dac8501.h"
//#include "bsp_dht11.h"
//#include "bsp_ir_decode.h"
//#include "bsp_ps2.h"
//#include "bsp_modbus.h"
//#include "bsp_rs485_led.h"
#include "bsp_user_lib.h"
//#include "bsp_dac8501.h"
//#include "bsp_dac8562.h"
//#include "bsp_esp8266.h"
//#include "bsp_step_moto.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_gps.h"
#include "bsp_exti.h"
#include "bsp_bmp.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motion_driver_9150.h"
#include "diskio.h"

/* �ṩ������C�ļ����õĺ��� */
void bsp_DelayMS(uint32_t _ulDelayTime);
void bsp_GetMs(unsigned long  *timestamp);
void bsp_DelayUS(uint32_t _ulDelayTime);
void bsp_Init(void);
void bsp_Idle(void);
void BSP_Tick_Init (void);

#endif

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

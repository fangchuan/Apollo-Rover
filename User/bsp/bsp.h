/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	说    明 : 这是底层驱动模块所有的h文件的汇总文件。 应用程序只需 #include bsp.h 即可，
*			  不需要#include 每个模块的 h 文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

/* 使能在源文件中使用uCOS-II的函数, 这里的源文件主要是指BSP驱动文件 */
#define uCOS_EN       1

#if uCOS_EN == 1    
	#include "os.h"   

	#define  ENABLE_INT()      OS_CRITICAL_EXIT()     /* 使能全局中断 */
	#define  DISABLE_INT()     OS_CRITICAL_ENTER()    /* 禁止全局中断 */
#else
	/* 开关全局中断的宏 */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */
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


/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */
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
#include "bsp_cpu_rtc.h"
#include "bsp_hmc5883l.h"
#include "bsp_mpu6050.h"
//#include "bsp_si4730.h"
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

/* 提供给其他C文件调用的函数 */
void bsp_DelayMS(uint32_t _ulDelayTime);
void bsp_GetMs(unsigned long  *timestamp);
void bsp_DelayUS(uint32_t _ulDelayTime);
void bsp_Init(void);
void bsp_Idle(void);
void BSP_Tick_Init (void);

#endif

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

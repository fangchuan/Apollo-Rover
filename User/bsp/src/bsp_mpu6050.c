/*
*********************************************************************************************************
*
*	模块名称 : 三轴陀螺仪MPU-6050驱动模块
*	文件名称 : bsp_mpu6050.c
*	版    本 : V1.0
*	说    明 : 实现MPU-6050的读写操作。
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技  www.apollorobot.cn
*
*********************************************************************************************************
*/

/*
	应用说明：访问MPU-6050前，请先调用一次 bsp_InitI2C()函数配置好I2C相关的GPIO.
*/

#include "bsp.h"




/*********************************************************************
*
*       Static Data
*
**********************************************************************
*/
static uint8_t buffer[14];

static int16_t  MPU6050_FIFO[6][11];
static int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
static int16_t MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz,
							 MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz;

/*
*********************************************************************************************************
*	函 数 名: MPU6050_newValues
*	功能说明: 将新的ADC数据更新到 FIFO数组，进行滤波处理
*	形    参: 
*	返 回 值: 无
*********************************************************************************************************
*/
static void  MPU6050_NewValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{

	unsigned char i ;
  int32_t sum=0;
  for(i=1;i<10;i++){	//FIFO 操作
      MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
      MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
      MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
      MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
      MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
      MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
      }
      MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
      MPU6050_FIFO[1][9]=ay;
      MPU6050_FIFO[2][9]=az;
      MPU6050_FIFO[3][9]=gx;
      MPU6050_FIFO[4][9]=gy;
      MPU6050_FIFO[5][9]=gz;

      sum=0;
      for(i=0;i<10;i++){	//求当前数组的合，再取平均值
          sum+=MPU6050_FIFO[0][i];
          }
          MPU6050_FIFO[0][10]=sum/10;

      sum=0;
      for(i=0;i<10;i++){
      sum+=MPU6050_FIFO[1][i];
          }
      MPU6050_FIFO[1][10]=sum/10;

      sum=0;
      for(i=0;i<10;i++){
      sum+=MPU6050_FIFO[2][i];
          }
      MPU6050_FIFO[2][10]=sum/10;

      sum=0;
      for(i=0;i<10;i++){
      sum+=MPU6050_FIFO[3][i];
      }
      MPU6050_FIFO[3][10]=sum/10;

      sum=0;
      for(i=0;i<10;i++){
          sum+=MPU6050_FIFO[4][i];
       }
      MPU6050_FIFO[4][10]=sum/10;

      sum=0;
      for(i=0;i<10;i++){
      sum+=MPU6050_FIFO[5][i];
      }
      MPU6050_FIFO[5][10]=sum/10;
}
/*
*********************************************************************************************************
*	函 数 名: MPU6050_InitGyro_Offset
*	功能说明: 读取 MPU6050的陀螺仪偏置
*           此时模块应该被静止放置。以测试静止时的陀螺仪输出
*	形    参: 
*	返 回 值: 无
*********************************************************************************************************
*/
static void MPU6050_InitGyro_Offset(void)
{
		unsigned char i;
		int16_t temp[6];
		int32_t	tempgx=0,tempgy=0,tempgz=0;
		int32_t	tempax=0,tempay=0,tempaz=0;
		Gx_offset=0;
		Gy_offset=0;
		Gz_offset=0;
		//Initialize the fifo
		for(i=0;i<50;i++){
				bsp_DelayUS(100);
				MPU6050_GetMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		}
		for(i=0;i<100;i++){
			bsp_DelayUS(200);
			MPU6050_GetMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
			tempax+= temp[0];
			tempay+= temp[1];
			tempaz+= temp[2];
			tempgx+= temp[3];
			tempgy+= temp[4];
			tempgz+= temp[5];
		}

		Gx_offset=tempgx/100;
		Gy_offset=tempgy/100;
		Gz_offset=tempgz/100;
//		tempax/=100;
//		tempay/=100;
//		tempaz/=100;
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
static void MPU6050_setClockSource(uint8_t source){
    i2cWriteBits(MPU6050_SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
static void MPU6050_setFullScaleGyroRange(uint8_t range) {
    i2cWriteBits(MPU6050_SLAVE_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
static void MPU6050_setFullScaleAccelRange(uint8_t range) {
    i2cWriteBits(MPU6050_SLAVE_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				     enabled =1   睡觉
			       enabled =0   工作
*******************************************************************************/
static void MPU6050_setSleepEnabled(uint8_t enabled) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
static void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
static void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
static void MPU6050_reset(void) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}
/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
static void MPU6050_getDeviceID(uint8_t *buffer) {

    i2cread(MPU6050_SLAVE_ADDR, MPU6050_RA_WHO_AM_I, 1, buffer);
    
}
/*
*********************************************************************************************************
*	函 数 名: bsp_MPU6050Initialize
*	功能说明: 
*	形    参: 
*	返 回 值: 
*********************************************************************************************************
*/
void bsp_MPU6050Init(void) {
	
		MPU6050_reset();
	  bsp_DelayMS(50);
	  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
	  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	  MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
													 
		MPU6050_InitGyro_Offset();
	
	  if(MPU6050_testConnection())
			printf("MPU6050 Initialize Success.");
		else
			printf("MPU6050 Initialize Error!!!");
}
/*
*********************************************************************************************************
*	函 数 名: MPU6050_testConnection
*	功能说明: 检测MPU6050 是否已经连接
*	形    参: 
*	返 回 值: 
*********************************************************************************************************
*/

uint8_t MPU6050_testConnection(void) {
	 uint8_t  id;
	 
	 MPU6050_getDeviceID(&id);
   if(id == 0x68)  //0b01101000;
		return 1;
 	else
		return 0;
}
/*
*********************************************************************************************************
*	函 数 名: MPU6050_is_DRY
*	功能说明: 检查 MPU6050的中断引脚，测试是否完成转换
*	形    参: 
*	返 回 值: 返回 1  转换完成
*               0 数据寄存器还没有更新
*********************************************************************************************************
*/
uint8_t MPU6050_is_DRY(void)
{
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}
/*
*********************************************************************************************************
*	函 数 名: MPU6050_GetMotion6
*	功能说明: 读取 MPU6050的当前测量值
*	形    参: 
*	返 回 值: 无
*********************************************************************************************************
*/
void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
  
		i2cread(MPU6050_SLAVE_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    MPU6050_Lastax=(((int16_t)buffer[0]) << 8) | buffer[1];
    MPU6050_Lastay=(((int16_t)buffer[2]) << 8) | buffer[3];
    MPU6050_Lastaz=(((int16_t)buffer[4]) << 8) | buffer[5];
		//跳过温度ADC
    MPU6050_Lastgx=(((int16_t)buffer[8]) << 8) | buffer[9];
    MPU6050_Lastgy=(((int16_t)buffer[10]) << 8) | buffer[11];
    MPU6050_Lastgz=(((int16_t)buffer[12]) << 8) | buffer[13];
//		MPU6050_NewValues(MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz,
//											MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz);
//		
//		*ax  =MPU6050_FIFO[0][10];
//		*ay  =MPU6050_FIFO[1][10];
//		*az = MPU6050_FIFO[2][10];
//		*gx  =MPU6050_FIFO[3][10]-Gx_offset;
//		*gy = MPU6050_FIFO[4][10]-Gy_offset;
//		*gz = MPU6050_FIFO[5][10]-Gz_offset;
		*ax = MPU6050_Lastax;
	  *ay = MPU6050_Lastay;
	  *az = MPU6050_Lastaz;
		*gx = MPU6050_Lastgx -Gx_offset;
		*gy = MPU6050_Lastgy -Gy_offset;
		*gz = MPU6050_Lastgz -Gz_offset;

}
/*
*********************************************************************************************************
*	函 数 名: MPU6050_GetlastMotion6
*	功能说明: 读取 MPU6050的最新测量值
*	形    参: 
*	返 回 值: 无
*********************************************************************************************************
*/
void MPU6050_GetlastMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
		*ax  =MPU6050_FIFO[0][10];
		*ay  =MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  =MPU6050_FIFO[3][10]-Gx_offset;
		*gy = MPU6050_FIFO[4][10]-Gy_offset;
		*gz = MPU6050_FIFO[5][10]-Gz_offset;
}


/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

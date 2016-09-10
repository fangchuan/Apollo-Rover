/*
*********************************************************************************************************
*
*	ģ������ : ����������MPU-6050����ģ��
*	�ļ����� : bsp_mpu6050.c
*	��    �� : V1.0
*	˵    �� : ʵ��MPU-6050�Ķ�д������
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ�  www.apollorobot.cn
*
*********************************************************************************************************
*/

/*
	Ӧ��˵��������MPU-6050ǰ�����ȵ���һ�� bsp_InitI2C()�������ú�I2C��ص�GPIO.
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
*	�� �� ��: MPU6050_newValues
*	����˵��: ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void  MPU6050_NewValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{

	unsigned char i ;
  int32_t sum=0;
  for(i=1;i<10;i++){	//FIFO ����
      MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
      MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
      MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
      MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
      MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
      MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
      }
      MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
      MPU6050_FIFO[1][9]=ay;
      MPU6050_FIFO[2][9]=az;
      MPU6050_FIFO[3][9]=gx;
      MPU6050_FIFO[4][9]=gy;
      MPU6050_FIFO[5][9]=gz;

      sum=0;
      for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
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
*	�� �� ��: MPU6050_InitGyro_Offset
*	����˵��: ��ȡ MPU6050��������ƫ��
*           ��ʱģ��Ӧ�ñ���ֹ���á��Բ��Ծ�ֹʱ�����������
*	��    ��: 
*	�� �� ֵ: ��
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
static void MPU6050_setFullScaleAccelRange(uint8_t range) {
    i2cWriteBits(MPU6050_SLAVE_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				     enabled =1   ˯��
			       enabled =0   ����
*******************************************************************************/
static void MPU6050_setSleepEnabled(uint8_t enabled) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
static void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    i2cWriteBit(MPU6050_SLAVE_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
static void MPU6050_getDeviceID(uint8_t *buffer) {

    i2cread(MPU6050_SLAVE_ADDR, MPU6050_RA_WHO_AM_I, 1, buffer);
    
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_MPU6050Initialize
*	����˵��: 
*	��    ��: 
*	�� �� ֵ: 
*********************************************************************************************************
*/
void bsp_MPU6050Init(void) {
	
		MPU6050_reset();
	  bsp_DelayMS(50);
	  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
    MPU6050_setSleepEnabled(0); //���빤��״̬
	  MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	  MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
													 
		MPU6050_InitGyro_Offset();
	
	  if(MPU6050_testConnection())
			printf("MPU6050 Initialize Success.");
		else
			printf("MPU6050 Initialize Error!!!");
}
/*
*********************************************************************************************************
*	�� �� ��: MPU6050_testConnection
*	����˵��: ���MPU6050 �Ƿ��Ѿ�����
*	��    ��: 
*	�� �� ֵ: 
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
*	�� �� ��: MPU6050_is_DRY
*	����˵��: ��� MPU6050���ж����ţ������Ƿ����ת��
*	��    ��: 
*	�� �� ֵ: ���� 1  ת�����
*               0 ���ݼĴ�����û�и���
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
*	�� �� ��: MPU6050_GetMotion6
*	����˵��: ��ȡ MPU6050�ĵ�ǰ����ֵ
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
  
		i2cread(MPU6050_SLAVE_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    MPU6050_Lastax=(((int16_t)buffer[0]) << 8) | buffer[1];
    MPU6050_Lastay=(((int16_t)buffer[2]) << 8) | buffer[3];
    MPU6050_Lastaz=(((int16_t)buffer[4]) << 8) | buffer[5];
		//�����¶�ADC
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
*	�� �� ��: MPU6050_GetlastMotion6
*	����˵��: ��ȡ MPU6050�����²���ֵ
*	��    ��: 
*	�� �� ֵ: ��
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


/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

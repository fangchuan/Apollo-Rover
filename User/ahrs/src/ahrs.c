/*
*********************************************************************************************************
*
*	模块名称 : 姿态解算模块
*	文件名称 : ahrs.c
*	版    本 : V1.0
*	说    明 : 软件结算姿态:Roll,Pitch,Yaw
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-08_05    fc   正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技  www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "inv_mpu.h"
#include "common.h"
#include "ahrs.h"
#include "bsp_i2c_gpio.h"

/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/
#define MPU9150_ADDR           0x68
#define POWER_MANAGE_1         0x6B
#define ACCEL_FILTER_98HZ			 98
#define ACCEL_XOUT_H		    	 0x3B
#define GYRO_XOUT_H		         0x43
#define BYPASS_ENABLE_CFG 		 0x37
#define USER_CTRL              0x6A
#define COMPASS_ADDR           0x0C
#define COMPASS_CNTL           0x0A
#define COMPASS_ST1            0x02
#define	COMPASS_HXL            0x03

#define MPU_SAMPLE_FREQ        1000

#define  Accel_Xout_Offset  -130
#define  Accel_Yout_Offset  96
#define  Accel_Zout_Offset  460

#define  Kp          2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define  Ki          0.005f   //integral gain governs rate of convergence of gyroscope biases
#define	 halfT 				0.005f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度
/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/
uint8_t  IsQuaternionInit = 0;
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
float maxMagX = 0;
float minMagX = 0;
float maxMagY = 0;
float minMagY = 0;
float maxMagZ = 0;
float minMagZ = 0;
float MXgain = 0;
float MYgain = 0;
float MZgain = 0;

float MXoffset = 0;
float MYoffset = 0;
float MZoffset = 0;

float q0 = 1.0, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

extern _Euler _euler;


/*
*********************************************************************************************************
*	函 数 名: delayus
*	功能说明: us 级别延时，不是很精确
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void delay_us(volatile uint32_t cnt)
{
    uint16_t i;
    for(i = 0;i<cnt;i++)
    {
        uint8_t us = 12; /* 设置值为12，大约延1微秒 */    
        while (us--)     /* 延1微秒	*/
        {
            ;   
        }
    }
}
/*
*********************************************************************************************************
*	函 数 名: delayms
*	功能说明: ms级别延时，不是很精确
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void delay_ms(volatile uint32_t cnt)
{
    uint16_t i;
    for(i = 0;i<cnt;i++)
    {
       delay_us(1000);
    }
}
/*
*********************************************************************************************************
*	函 数 名: read_magnetometer
*	功能说明: 读取compass数据，用于compass校准	
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
extern  long mag_sens_adj_val[3];

void read_magnetometer(void)
{
  signed short int mag[3];
  unsigned char tmp[7], data_write[1];
  
  tmp[6]=0x00;
  data_write[0]=0x01;
  i2cread(COMPASS_ADDR, COMPASS_ST1, 1, tmp+6);
  if(tmp[6] == 1)
  {
    i2cread(COMPASS_ADDR, COMPASS_HXL, 6, tmp);
		mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
    mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
    mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

		mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //灵敏度调整
    mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
    mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;

		init_mx =(float)mag[1];		//转换坐标轴				
    init_my =(float)mag[0];
    init_mz =(float)-mag[2];
		
		i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write);	 //开启compass：single measurement mode
  }  
}
/*
*********************************************************************************************************
*	函 数 名: get_compass_bias
*	功能说明: 得到mag的Xmax、Xmin、Ymax、Ymin、Zmax、Zmin	
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void get_compass_bias(void)
{
  read_magnetometer();

  if(init_mx > maxMagX)
		maxMagX = init_mx;
  if(init_mx < minMagX)
		minMagX = init_mx;

  if(init_my > maxMagY)
		maxMagY = init_my;
  if(init_my < minMagY)
		minMagY = init_my;

  if(init_mz > maxMagZ)
		maxMagZ = init_mz;
  if(init_mz < minMagZ)
		minMagZ = init_mz;
//  printf("maxMagX=%f, minMagX=%f, maxMagY=%f, minMagY=%f, maxMagZ=%f, minMagZ=%f \n\r", 
//          maxMagX, minMagX, maxMagY, minMagY, maxMagZ, minMagZ);  
}
/*
*********************************************************************************************************
*	函 数 名: get_compass_bias
*	功能说明: 校准compass
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void compass_calibration(void)
{ //将有最大响应的轴的增益设为1
  if(((maxMagX - minMagX) >= (maxMagY - minMagY)) && ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
  {
    MXgain = 1.0;
		MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
		MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);
		MXoffset = -0.5 * (maxMagX + minMagX);
		MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
		MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);	 
  }
  if(((maxMagY - minMagY) > (maxMagX - minMagX)) && ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
  {
    MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
		MYgain = 1.0;
		MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
		MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
		MYoffset = -0.5 * (maxMagY + minMagY);
		MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);    
  }
  if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) && ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
  {
    MXgain = (maxMagZ - minMagZ) / (maxMagX - minMagX);
		MYgain = (maxMagZ - minMagZ) / (maxMagY - minMagY);
		MZgain = 1.0;
		MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
		MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
		MZoffset = -0.5 * (maxMagZ + minMagZ);    
  }
  printf("MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f \n\r", 
          MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);         
}
/*
*********************************************************************************************************
*	函 数 名: Init_Magnetometer
*	功能说明: 读取compass数据，在初始化mpu9150后先读几次mag的数据，因为前几次读取的mag数据有错误
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
int Init_Magnetometer(void)
{
		unsigned char data_write[3];
		signed short int mag[3];
		unsigned char tmp[7];
		uint8_t i=6;
		
		data_write[0]=0x02;       
		data_write[1]=0x00;
		data_write[2]=0x01;
		
		i2cwrite(MPU9150_ADDR, BYPASS_ENABLE_CFG, 1, data_write);	 //开启bypass
		delay_ms(10);                     
		i2cwrite(MPU9150_ADDR, USER_CTRL, 1, data_write+1);	 //关闭MPU9150的I2C_MASTER模式，必须要有这句
		delay_ms(10);
		i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write+2);	 //开启compass：single measurement mode

		
		tmp[6]=0x00;
		data_write[0]=0x01;
		for(;i>0;i--)
		{
			i2cread(COMPASS_ADDR, COMPASS_ST1, 1, tmp+6);
			if(tmp[6] == 1)
			{
				i2cread(COMPASS_ADDR, COMPASS_HXL, 6, tmp);
				mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
				mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
				mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

				mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //灵敏度调整
				mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
				mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;

				i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write + 2);	 //开启compass：single measurement mode
			} 
		}
		return 0;  
}
/*
*********************************************************************************************************
*	函 数 名: Init_MPU9150
*	功能说明: 读取compass数据，在初始化mpu9150后先读几次mag的数据，因为前几次读取的mag数据有错误
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void Init_MPU9150(void)
{
  int result;
  unsigned char data_write[1];

  result = mpu_init((void*)0);
  if(!result)
  {
	  printf("\r\n mpu initialization complete ......\n\r ");

	  //开启加速度计、陀螺仪、磁力计，使用电子罗盘，要加入INV_XYZ_COMPASS
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
	  {
	  	 printf("\r\n mpu_set_sensor complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_sensor error ......\n\r");
			 while(1);
	  }

	  //设置MPU9150的时钟源为GX的PLL
	  data_write[0]=0x01;				//GX_PLL:0x01
	  if(!i2cwrite(MPU9150_ADDR, POWER_MANAGE_1, 1, data_write))
	  {
	  	 printf("\r\n set_mpu9150_ClockSource complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n set_mpu9150_ClockSource error ......\n\r");
			 while(1);
	  }

	  //设置陀螺仪测量范：+-500度/s
	  if(!mpu_set_gyro_fsr(2000))
	  {
	  	 printf("\r\n mpu_set_gyro_fsr complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_gyro_fsr error ......\n\r");
			 while(1);
	  }

	  //设置加速度计测量范围：+-4G
	  if(!mpu_set_accel_fsr(2))
	  {
	  	 printf("\r\n mpu_set_accel_fsr complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_accel_fsr error ......\n\r");
			 while(1);
	  }

	  //设置加速度计的低通滤波器，防震动，由于朗宇X2212 kv980电机电压11.1v下转速为120r/s，因此DLPF选98hz，后面可考虑加权平均滤波
	  if(!mpu_set_lpf(98))
	  {
	  	 printf("\r\n mpu_set_lpf complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_lpf error ......\n\r");
			 while(1);
	  }

	  //设置采样率1kHz
	  if(!mpu_set_sample_rate(MPU_SAMPLE_FREQ))
	  {
	  	 printf("\r\n mpu_set_sample_rate complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_sample_rate error ......\n\r");
			 while(1);
	  }

	  //先读几次mag数据，因为前几次mag数据有错误，芯片bug
	  if(!Init_Magnetometer())
	  {
	  	 printf("\r\n Init_Magnetometer complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n Init_Magnetometer error ......\n\r");
			 while(1);
	  }
  }
  else
  {
      printf("\r\n mpu initialization error......\n\r ");
			while(1);
  }

}


/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
		float init_Yaw, init_Pitch, init_Roll;
		signed short int accel[3], mag[3];
		unsigned char tmp[7], data_write[7];

		if(!i2cread(MPU9150_ADDR, ACCEL_XOUT_H, 6, data_write))
    {
			accel[0]=((((signed short int)data_write[0])<<8) | data_write[1]) + Accel_Xout_Offset;
			accel[1]=((((signed short int)data_write[2])<<8) | data_write[3]) + Accel_Yout_Offset;
			accel[2]=((((signed short int)data_write[4])<<8) | data_write[5]) + Accel_Zout_Offset;
	  	    
			init_ax=((float)accel[0]) * ACCEL_2_SACLE_PARAMETER;	   //单位转化成重力加速度的单位：g
			init_ay=((float)accel[1]) * ACCEL_2_SACLE_PARAMETER;
      init_az=((float)accel[2]) * ACCEL_2_SACLE_PARAMETER;

		}
      tmp[6]=0x00;
      data_write[6]=0x01;
			i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write+6);	 //开启compass：single measurement mode
			delay_ms(10);  //wait data ready
      i2cread(COMPASS_ADDR, COMPASS_ST1, 1, tmp+6);
      if(tmp[6] == 1)
      {
        i2cread(COMPASS_ADDR, COMPASS_HXL, 6, tmp);
				mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
        mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
        mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

				mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //灵敏度调整
        mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
        mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;

				init_mx =(float)mag[1] * MXgain + MXoffset;		//转换坐标轴				
        init_my =(float)mag[0] * MYgain + MYoffset;
        init_mz =(float)-mag[2] * MZgain + MZoffset;
				//开启compass：single measurement mode
				i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write+6);	 
			}
			//根据三角函数关系算出初始欧拉角，注意偏航角的计算要考虑倾角补偿
			init_Roll  =  atan2(init_ay, init_az);
			init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
			init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));      			//atan2(mx, my);
			
			q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
			q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
			q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
			q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

  
			printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f, q0=%f, q1=%f, q2=%f, q3=%f", 
               init_Yaw, init_Pitch, init_Roll, q0, q1, q2, q3);
}
/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
		float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
		static float tempq0 = 0, tempq1 = 0, tempq2 = 0, tempq3 = 0;

		if(IsQuaternionInit == 0)
		{
				init_quaternion();
			  IsQuaternionInit = 1;
		}
		//auxiliary variables to reduce number of repeated operations，
		q0q0 = q0*q0;
		q0q1 = q0*q1;
		q0q2 = q0*q2;
		q0q3 = q0*q3;
		q1q1 = q1*q1;
		q1q2 = q1*q2;
		q1q3 = q1*q3;
		q2q2 = q2*q2;   
		q2q3 = q2*q3;
		q3q3 = q3*q3;          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
        //normalise the measurements
			if(!(ax == 0 && ay == 0 && az == 0))
			{
					norm = invSqrt(ax*ax + ay*ay + az*az);       
					ax = ax * norm;
					ay = ay * norm;
					az = az * norm;
				
				if(!(mx == 0 && my == 0 && mz == 0))
				{
					norm = invSqrt(mx*mx + my*my + mz*mz);          
					mx = mx * norm;
					my = my * norm;
					mz = mz * norm;         
        
					/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
					//compute reference direction of flux
					hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
					hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
					hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

					/*计算地理坐标系下的磁场矢量bxyz（参考值）。
					因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
					但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
					我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
					磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
					因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。*/
					bx = sqrtf((hx*hx) + (hy*hy));
					bz = hz;        
					/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
					因为by=0，所以所有涉及到by的部分都被省略了。
					类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
					你可以看看两个公式：wxyz的公式，把bx换成gx（0），把bz换成gz（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
          wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
          wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
          wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
				}
					// estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
					vx = 2*(q1q3 - q0q2);
					vy = 2*(q0q1 + q2q3);
					vz = q0q0 - q1q1 - q2q2 + q3q3;
           
					//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
					// error is sum of cross product between reference direction of fields and direction measured by sensors
					ex = (ay*vz - az*vy) + (my*wz - mz*wy);
					ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
					ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
		          
					// integral error scaled integral gain
					if(Ki > 0){
					exInt = exInt + ex*Ki*halfT;
					eyInt = eyInt + ey*Ki*halfT;
					ezInt = ezInt + ez*Ki*halfT;
					}
					else
					{
						exInt = 0;
						eyInt = 0;
						ezInt = 0;
					}
					// adjusted gyroscope measurements
					gx = gx + Kp*ex + exInt;
					gy = gy + Kp*ey + eyInt;
					gz = gz + Kp*ez + ezInt;


					// integrate quaternion rate and normalise，四元数更新算法
					tempq0 = (-q1*gx - q2*gy - q3*gz)*halfT;
					tempq1 = (q0*gx + q2*gz - q3*gy)*halfT;
					tempq2 = (q0*gy - q1*gz + q3*gx)*halfT;
					tempq3 = (q0*gz + q1*gy - q2*gx)*halfT; 
			}
        q0 = q0 + tempq0;
        q1 = q1 + tempq1;
        q2 = q2 + tempq2;
        q3 = q3 + tempq3; 
        
        // normalise quaternion
        norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 * norm;       //w
        q1 = q1 * norm;       //x
        q2 = q2 * norm;       //y
        q3 = q3 * norm;       //z

		/*最初的由四元数计算出Pitch  Roll  Yaw
		Roll=arctan2(2wx+2yz, 1-2xx-2yy);
		Pitch=arcsin(2wy-2zx);
		Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
		1=q0*q0+q1*q1+q2*q2+q3*q3;*/
		_euler.pitch = asin(-2*q1*q3 + 2*q0*q2) * RAD_TO_DEGREE; //俯仰角，绕y轴转动	 
		_euler.roll  = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * RAD_TO_DEGREE; //滚动角，绕x轴转动
		_euler.yaw   = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * RAD_TO_DEGREE;  //偏航角，绕z轴转动

    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", _euler.yaw, _euler.pitch, _euler.roll);
 
}


/*******************************************************************************
* Function Name  : get_mpu9150_data
* Description    : 读取mpu9150的加速度计 陀螺仪 磁力计数据并做校准和滤波.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void get_mpu9150_data(void)
{
   signed short int gyro[3], accel[3], mag[3]; 
   unsigned char tmp[7], data_write[14];

   if(!i2cread(MPU9150_ADDR, ACCEL_XOUT_H, 14, data_write))
   {
			accel[0]=(((signed short int)data_write[0])<<8) | data_write[1];
			accel[1]=(((signed short int)data_write[2])<<8) | data_write[3];
			accel[2]=(((signed short int)data_write[4])<<8) | data_write[5];
			gyro[0] =(((signed short int)data_write[8])<<8) | data_write[9];
			gyro[1] =(((signed short int)data_write[10])<<8) | data_write[11];
			gyro[2] =(((signed short int)data_write[12])<<8) | data_write[13];
			
			init_ax=(float)(accel[0] + Accel_Xout_Offset);   	  
			init_ay=(float)(accel[1] + Accel_Yout_Offset);
			init_az=(float)(accel[2] + Accel_Zout_Offset);  		 
			init_gx=(float)gyro[0] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;    //单位转化成：弧度/s
			init_gy=(float)gyro[1] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;
			init_gz=(float)gyro[2] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;
		
			_euler.rate[0] = init_gx;
			_euler.rate[1] = init_gy;
			_euler.rate[2] = init_gz;
		 
      tmp[6]=0x00;
      data_write[6]=0x01;
      i2cread(COMPASS_ADDR, COMPASS_ST1, 1, tmp+6);
      if(tmp[6] == 1)
      {
        i2cread(COMPASS_ADDR, COMPASS_HXL, 6, tmp);//读取compass
				mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
        mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
        mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

				mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //灵敏度调整
        mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
        mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;
				//修正magnetometer
				init_mx =(float)mag[1] * MXgain + MXoffset;		//转换坐标轴				
        init_my =(float)mag[0] * MYgain + MYoffset;
        init_mz =(float)-mag[2] * MZgain + MZoffset;
//				//进行x y轴的校准，未对z轴进行校准，参考ST的校准方法 
//         init_mx =(float)1.046632*mag[0]-1.569948;						
//         init_my =(float)mag[1]-8;
//         init_mz =(float)mag[2];
				i2cwrite(COMPASS_ADDR, COMPASS_CNTL, 1, data_write+6);	 //开启compass：single measurement mode
	  }

   }
}
/*******************************************************************************
* Function Name  : Get_Attitude
* Description    : 更新姿态
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ahrs_update_euler(void)
{
	 get_mpu9150_data();
   AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
}



/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

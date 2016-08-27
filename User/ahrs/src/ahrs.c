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
#include "common.h"
#include "bsp_mpu6050.h"
#include "bsp_hmc5883l.h"
#include "ahrs.h"
#include "os.h"
/*********************************************************************
*
*       Macro define
*
**********************************************************************
*/


#define  Kp          10.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define  Ki          0.25f   //integral gain governs rate of convergence of gyroscope biases
float    halfT = 0.005;
///#define	 halfT 			 0.005f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度
/*********************************************************************
*
*       Global Data
*
**********************************************************************
*/
static uint8_t  IsQuaternionInit = 0;
static float    init_ax, init_ay, init_az,
					      init_gx, init_gy, init_gz,
					      init_mx, init_my, init_mz;

float q0 = 1.0, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

extern _Euler _euler;

/*
*********************************************************************************************************
*	函 数 名: GetMotion9
*	功能说明: 获取九轴原始数据
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void GetMotion9(void)
{
	     int16_t buffer[9];
	
	     MPU6050_GetMotion6(&buffer[0], &buffer[1], &buffer[2], &buffer[3], &buffer[4], &buffer[5]);
	     HMC5883L_GetRaw(&buffer[6], &buffer[7], &buffer[8]);
	
	     init_ax = (float)buffer[0] * ACCEL_2_SACLE_PARAMETER;
	     init_ay = (float)buffer[1] * ACCEL_2_SACLE_PARAMETER;
	     init_az = (float)buffer[2] * ACCEL_2_SACLE_PARAMETER;
	     init_gx = (float)buffer[3] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;
	     init_gy = (float)buffer[4] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;
	     init_gz = (float)buffer[5] * GYRO_2000_SCALE_PARAMETER * DEGREE_TO_RAD;
	     init_mx = (float)buffer[6] * MAG_GAIN_230_PARAMETER;
	     init_my = (float)buffer[7] * MAG_GAIN_230_PARAMETER;
	     init_mz = (float)buffer[8] * MAG_GAIN_230_PARAMETER;
	
	     _euler.acc[0] = init_ax;
	     _euler.acc[1] = init_ay;
	     _euler.acc[2] = init_az;
	     _euler.rate[0] = init_gx;
	     _euler.rate[1] = init_gy;
	     _euler.rate[2] = init_gz;
}
/*
*********************************************************************************************************
*	函 数 名: init_quaternion
*	功能说明: 初始化四元数q0 q1 q2 q3.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void init_quaternion(void)
{ 

		float init_Yaw, init_Pitch, init_Roll;
		

	
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
		static uint32_t last_timestamp = 0, now_timestamp = 0;
	
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

			//------------ 读取解算时间 ---------------
//		now_timestamp = CPU_TS32_to_uSec(OS_TS_GET());  	  //读取时间
//		if(now_timestamp < last_timestamp)  //定时器溢出过了。
//		{ 
//			halfT =  ((float)(now_timestamp + (0xffffffff- last_timestamp)) / 2000000.0f);	
//		}
//		else
//		{
//			halfT =  ((float)(now_timestamp - last_timestamp) / 2000000.0f);
//		}
//		last_timestamp = now_timestamp;	 //更新时间
//		printf("halfT:%d\n",(int)halfT*1000);
		
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

 
}

/*******************************************************************************
* Function Name  : ahrs_update_euler
* Description    : 更新姿态
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ahrs_update_euler(void)
{
	 CPU_SR_ALLOC();
	 GetMotion9();
	 
	 CPU_CRITICAL_ENTER();
   AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
	 CPU_CRITICAL_EXIT();
}

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

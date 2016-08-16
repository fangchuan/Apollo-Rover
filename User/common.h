/*
*********************************************************************************************************
*
*	模块名称 : 通用模块头文件
*	文件名称 : common.h
*	版    本 : V5.1
*	说    明 : 
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef  _COMMON_H
#define  _COMMON_H
/*********************************************************************
*
*       Macro Define
*
**********************************************************************
*/
//project macro define  
#define  MOTION_DRIVER_TARGET_STM32

#define  AHRS_USE_DMP       1
#define  AHRS_USE_SOFTWARE  0

#if      AHRS_USE_DMP
		#define  MPU9150
#else
		#define  MPU6050
		#define  AK89xx_BYPASS
		#define  AK8975_SECONDARY
#endif

#define  ROVER_FRAME  			1
#define  QUAD_FRAME         2
#define  FRAME_CONFIG       ROVER_FRAME

#define  APOLLOROBOT_LOG    "log.txt"

//#define  USE_EXTERN_SRAM

//常用宏定义
#define DEGREE_TO_RAD        (float)0.017453292519943295769236907684886
#define RAD_TO_DEGREE        (float)57.295779513082320876798154814105
#define min(a,b)             ((a)<(b)?(a):(b))
#define max(a,b)             ((a)>(b)?(a):(b))
#define constrain(amt,low,high)   ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define radians(deg)         ((deg)*DEGREE_TO_RAD)
#define degrees(rad)         ((rad)*RAD_TO_DEGREE)
#define M_PI                 3.14159265358979323846
#define q30                  1073741824.0f					//2^30
#define CONSTANTS_ONE_G			 9.80665f		/* m/s^2		*/
#define  MOTOR_DIAMETER  		 (float)5.0f //cm
#define  MOTOR_RADIUS		 			MOTOR_DIAMETER/2

//定义mpu9150不同测量范围的刻度因子
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f
#define GYRO_250_SCALE_PARAMETER   (float)1/Gyro_250_Scale_Factor
#define GYRO_500_SCALE_PARAMETER   (float)1/Gyro_500_Scale_Factor
#define GYRO_1000_SCALE_PARAMETER  (float)1/Gyro_1000_Scale_Factor
#define GYRO_2000_SCALE_PARAMETER  (float)1/Gyro_2000_Scale_Factor
#define ACCEL_2_SACLE_PARAMETER    (float)1/Accel_2_Scale_Factor
#define ACCEL_4_SACLE_PARAMETER    (float)1/Accel_4_Scale_Factor
#define ACCEL_8_SACLE_PARAMETER    (float)1/Accel_8_Scale_Factor
#define ACCEL_16_SACLE_PARAMETER    (float)1/Accel_16_Scale_Factor
	
//
#define  ENCODER_PRE_PRIORITY  	0
#define  ENCODER_SUB_PRIORITY  	0
#define  GPS_UART_PRE_PRIORITY 	0
#define  GPS_UART_SUB_PRIORITY 	1
#define  MPU9150_PRE_PRIORITY		1
#define  MPU9150_SUB_PRIORITY		0
#define  SD_SDIO_PRE_PRIORITY   1
#define  SD_SDIO_SUB_PRIORITY   1
#define  SD_DMA2_PRE_PRIORITY   1
#define  SD_DMA2_SUB_PRIORITY   2
//
#define  MOTOR_MAX_NUM   2
#define  MOTOR_LEFT      0
#define  MOTOR_RIGHT     1

/*********************************************************************
*
*       Data Structrue 
*
**********************************************************************
*/
typedef struct  _Vector3f{
				float x;
				float y;
				float z;
}_Vector3f;

typedef struct _Matrix3f{
				_Vector3f  a;
				_Vector3f  b;
				_Vector3f  c;
}_Matrix3f;

typedef struct _Euler{
				float roll;
				float pitch;
				float yaw;
#if     AHRS_USE_DMP
	      short gyro[3];
	      long quat[4];
	      short accel[3];
	      short sensors;
	      unsigned char more;
#endif
				float rate[3];//units  rad
				float acc[3];
}_Euler;

typedef struct _Position{
				float lon;
				float lat;
				float height;
}_Position;

typedef struct _Motor{
				float cur_angle;
				float tar_angle;
				float cur_speed;
				float tar_speed;
				float distances;
				float out;
	
}_Motor;

typedef struct _Car{
				float cur_distance;
				float tar_distance;
				float cur_velocity;
				float tar_velocity;
				float accel;
				float cur_yaw;
				float tar_yaw;
	      _Position pos;
	
}_car;

/*********************************************************************
*
*       Public Function 
*
**********************************************************************
*/
float invSqrt(float x) ;

#endif
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

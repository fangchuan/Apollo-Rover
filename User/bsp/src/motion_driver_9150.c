/*
*********************************************************************************************************
*
*	ģ������ : MPU9150  DMP����ģ��
*	�ļ����� : motion_driver_9150.c
*	��    �� : V5.1
*	˵    �� : 
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "common.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bsp_exti.h"
#include "motion_driver_9150.h"
#include "os.h"
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
_Euler  _euler;

/*********************************************************************
*
*       Macro Define
*
**********************************************************************
*/

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)


/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static struct hal_s hal = {0};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {1, 0, 0,
                                           0,1, 0,
                                           0, 0, 1};


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: dmpsafe_asin
*	����˵��: �����Һ���
*	��    ��: ����ֵ
*	�� �� ֵ: �ǶȵĻ���ֵ
*********************************************************************************************************
*/
static float dmpsafe_asin(float v)
{
				if(isnan(v)){
					return 0.0;
				}
				if (v >= 1.0) {
					return M_PI/2;
				}
				if (v <= -1.0) {
					return -M_PI/2;
				}
				return asin(v);
}
/*
*********************************************************************************************************
*	�� �� ��: quaterion_to_euler
*	����˵��: ��Ԫ��תΪŷ����,  Z-Y-X˳��
*	��    ��: ��Ԫ��
*	�� �� ֵ: 
*********************************************************************************************************
*/
static void quaterion_to_euler(long *data)
{  
     double q[4];
     q[0] = (data[0])*1.0f/q30;
     q[1] = (data[1])*1.0f/q30;
     q[2] = (data[2])*1.0f/q30;
     q[3] = (data[3])*1.0f/q30;
     
     _euler.roll = (atan2(2.0*(q[0]*q[1] + q[2]*q[3]),1-2*(q[1]*q[1] + q[2]*q[2])))* RAD_TO_DEGREE;
	 // we let safe_asin() handle the singularities near 90/-90 in pitch
     _euler.pitch = dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* RAD_TO_DEGREE;

     _euler.yaw = (atan2(2.0*(q[0]*q[3] + q[1]*q[2]),1-2*(q[2]*q[2] + q[3]*q[3])))* RAD_TO_DEGREE;
}
/*
*********************************************************************************************************
*	�� �� ��: dmp_init
*	����˵��: DMP��ʼ��
*	��    ��: 
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����-1��ʾδд�ɹ�
*********************************************************************************************************
*/
int8_t dmp_init(void)
{
    int result;
    struct int_param_s int_param;
    int_param.cb = gyro_data_ready_cb;//�ص�������Ӧ����DMP_INT���ŵ��жϴ������е���
    int_param.init_isr=bsp_InitEXTI;	//DMP_INT���ŵ��жϳ�ʼ������
    result = mpu_init(&int_param);
    if (result)
    {
        printf("mpu init fail!!!!\n");
        return -1;
    }

    /* Wake up all sensors. */
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
    {
         printf("mpu_set_sensors fail!!!!\n");
         return -1;
    }
	 /* Push both gyro and accel data into the FIFO. */
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    {
         printf("mpu_configure_fifo fail!!!!\n");
         return -1;
    }
    if (mpu_set_sample_rate(DEFAULT_MPU_HZ))
    {     
         printf("mpu_set_sample_rate fail!!!!\n");
         return -1;
    }

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;

    if (dmp_load_motion_driver_firmware())
    {
         printf("dmp_load_motion_driver_firmware fail!!!!\n");
         return -1;
    }
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
    {
				 printf("dmp_set_orientation fail!!!!\n");
         return -1;
    }
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    if (dmp_enable_feature(hal.dmp_features))
    {
				 printf("dmp_enable_feature fail!!!!\n");
         return -1;
    }
    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ))
    {
	       printf("dmp_set_fifo_rate fail!!!!\n");
         return -1;
    }
    if (mpu_set_dmp_state(1))
    {
	       printf("mpu_set_dmp_state fail!!!!\n");
         return -1;
    }
    hal.dmp_on = 1;
    if (dmp_set_interrupt_mode(DMP_INT_CONTINUOUS))
    {
	       printf("dmp_set_interrupt_mode fail!!!!\n");
         return -1;
    }
    printf("mpu initialize success!\r\n");
    return 0;
}
/*
*********************************************************************************************************
*	�� �� ��: dmp_update_euler
*	����˵��: DMP�жϸ���ŷ����(N->B)
*           ������ִ�����ڴ��470us
*	��    ��: 
*	�� �� ֵ:
*********************************************************************************************************
*/
#if   AHRS_USE_DMP
void dmp_update_euler(void)
{
		CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
	  
    if(hal.new_gyro && hal.dmp_on)
    {
			 unsigned long sensor_timestamp;
       dmp_read_fifo(_euler.gyro, _euler.accel, _euler.quat, &sensor_timestamp, &_euler.sensors, &_euler.more);
//			 sensor_timestamp  = OS_TS_GET();
	
       quaterion_to_euler(_euler.quat);
       _euler.rate[0] =  radians((float)_euler.gyro[0]*GYRO_2000_SCALE_PARAMETER);//X����ٶ�
       _euler.rate[1] =  radians((float)_euler.gyro[1]*GYRO_2000_SCALE_PARAMETER);//Y����ٶ�
       _euler.rate[2] =  radians((float)_euler.gyro[2]*GYRO_2000_SCALE_PARAMETER);//Z����ٶ�
       
			 _euler.acc[0] = (float)_euler.accel[0] * ACCEL_2_SACLE_PARAMETER;//X����ٶ�
			 _euler.acc[1] = (float)_euler.accel[1] * ACCEL_2_SACLE_PARAMETER;//Y����ٶ�
			 _euler.acc[2] = (float)_euler.accel[2] * ACCEL_2_SACLE_PARAMETER;//Z����ٶ�
			
			 if (!_euler.more)
				 hal.new_gyro = 0;
			 
//			 printf("DMP time:%lld\n", CPU_TS32_to_uSec(OS_TS_GET() - sensor_timestamp));
			}         
		CPU_CRITICAL_EXIT();
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: DispEulerData
*	����˵��: ��ӡŷ���ǡ����ٶȡ����ٶ���Ϣ(DEBUG USE)
*	��    ��: 
*	�� �� ֵ:
*********************************************************************************************************
*/
void  DispEulerData(void)
{
	
		printf("Roll: %d\n", (int)_euler.roll);
		printf("Pitch: %d\n",(int)_euler.pitch);
		printf("Yaw:  %d\n", (int)_euler.yaw);
		
		printf("Gyro X:  %d\n", (int)(_euler.rate[0]*RAD_TO_DEGREE));
		printf("Gyro Y:  %d\n", (int)(_euler.rate[1]*RAD_TO_DEGREE));
		printf("Gyro Z:  %d\n", (int)(_euler.rate[2]*RAD_TO_DEGREE));
	
//		printf("Accel X:  %d\n", (int)(_euler.acc[0]*CONSTANTS_ONE_G));
//		printf("Accel Y:  %d\n", (int)(_euler.acc[1]*CONSTANTS_ONE_G));
//		printf("Accel Z:  %d\n", (int)(_euler.acc[2]*CONSTANTS_ONE_G));

}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

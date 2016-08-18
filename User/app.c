/*
*********************************************************************************************************
*
*	ģ������ : Ӧ��ģ��
*	�ļ����� : app.c
*	��    �� : V1.0
*	˵    �� : �û�����(����)������ִ�С�����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#include  <includes.h>

/*
*********************************************************************************************************
*                                       ��̬ȫ�ֱ���
*********************************************************************************************************
*/                                                          
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB   AppTaskGPSTCB;
static  CPU_STK  AppTaskGPSStk[APP_CFG_TASK_GPS_STK_SIZE];

static  OS_TCB   AppTaskCOMTCB;
static  CPU_STK  AppTaskCOMStk[APP_CFG_TASK_COM_STK_SIZE];

static  OS_TCB   AppTaskUserIFTCB;
static  CPU_STK  AppTaskUserIFStk[APP_CFG_TASK_USER_IF_STK_SIZE];

static  OS_TCB   AppTaskSensorTCB;
static  CPU_STK  AppTaskSensorStk[APP_CFG_TASK_SENSOR_STK_SIZE];

static  OS_TCB   AppTaskMotorTCB;
static  CPU_STK  AppTaskMotorStk[APP_CFG_TASK_MOTOR_STK_SIZE];

/*
*********************************************************************************************************
*                                      ��������
*********************************************************************************************************
*/
static  void   AppTaskStart          (void     *p_arg);
static  void   AppTaskCreate         (void);
static  void   AppTaskUserIF         (void     *p_arg);
static  void   AppTaskCOM						 (void 	   *p_arg);
static  void   AppTaskGPS						 (void 	   *p_arg);
static  void   AppTaskSensorUpdate	 (void 	   *p_arg);
static  void 	 AppTaskMotorControl   (void *p_arg);
static  void   DispTaskInfo          (void);
static  void   AppObjCreate          (void);
static  void   App_Printf (CPU_CHAR *format, ...);

/*
*********************************************************************************************************
*                                      ϵͳ���
*********************************************************************************************************
*/
static  OS_SEM   SEM_MUTEX;	   //���ڻ���
static  OS_SEM   SEM_SYNCH;	   //����ͬ��
static OS_TMR    Timer_500ms;
static OS_TMR    Timer_1000ms;
/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
extern _Motor _motor[MOTOR_MAX_NUM];
/*********************************************************************
*
*       Static  code
*
**********************************************************************
*/
static void _cbOfTmr_500(OS_TMR *p_tmr, void *p_arg)
{
    (void)p_arg;

		bsp_LedToggle(1);
		bsp_LedToggle(2);

}
static void _cbOfTmr_1000(OS_TMR *p_tmr, void *p_arg)
{
    (void)p_arg;

//		LogEncoderData();
//		LogGpsData();
		LogMotorData();
		LogRTCData();

}
/*
*********************************************************************************************************
*	�� �� ��: AppObjCreate
*	����˵��: ��������ͨѶ
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static  void  AppObjCreate (void)
{
	OS_ERR  err;
	/* ���������ź��� */
	BSP_OS_SemCreate(&SEM_MUTEX, 1,	(CPU_CHAR *)"SEM_MUTEX");
	
	/* ����ͬ���ź��� */ 
  BSP_OS_SemCreate(&SEM_SYNCH, 0, (CPU_CHAR *)"SEM_SYNCH");
	
	//������ʱ��  OS_CFG_TMR_TASK_RATE_HZ = 100HZ
  OSTmrCreate ((OS_TMR              *)&Timer_500ms,
              (CPU_CHAR            *)"MyTimer 500ms",
              (OS_TICK              )0,                  //��һ����ʱ����Ϊ0ms��
              (OS_TICK              )50,                  //��ʱ������500ms
              (OS_OPT               )OS_OPT_TMR_PERIODIC,//ģʽ����Ϊ�ظ�ģʽ
              (OS_TMR_CALLBACK_PTR  )_cbOfTmr_500,          //�ص�����
              (void                *)0,                  //��������Ϊ0
              (OS_ERR              *)err);
	OSTmrCreate ((OS_TMR              *)&Timer_1000ms,
              (CPU_CHAR            *)"MyTimer 1000ms",
              (OS_TICK              )0,                  //��һ����ʱ����Ϊ0ms��
              (OS_TICK              )100,                  //��ʱ������1000ms
              (OS_OPT               )OS_OPT_TMR_PERIODIC,//ģʽ����Ϊ�ظ�ģʽ
              (OS_TMR_CALLBACK_PTR  )_cbOfTmr_1000,          //�ص�����
              (void                *)0,                  //��������Ϊ0
              (OS_ERR              *)err);
							
	OSTmrStart(&Timer_500ms, (OS_ERR *)err);
	OSTmrStart(&Timer_1000ms, (OS_ERR *)err);
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static  void  AppTaskCreate (void)
{
	OS_ERR      err;
	

	/**************����COM����*********************/
	OSTaskCreate((OS_TCB       *)&AppTaskCOMTCB,            
                 (CPU_CHAR     *)"App Task COM",
                 (OS_TASK_PTR   )AppTaskCOM, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_COM_PRIO,
                 (CPU_STK      *)&AppTaskCOMStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_COM_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_COM_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
	
	/**************����USER IF����*********************/
	OSTaskCreate((OS_TCB       *)&AppTaskUserIFTCB,             
                 (CPU_CHAR     *)"App Task UserIF",
                 (OS_TASK_PTR   )AppTaskUserIF, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_USER_IF_PRIO,
                 (CPU_STK      *)&AppTaskUserIFStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_USER_IF_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_USER_IF_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
	/**************����GPS Update����*********************/
	OSTaskCreate((OS_TCB       *)&AppTaskGPSTCB,             
                 (CPU_CHAR     *)"App Task GPS",
                 (OS_TASK_PTR   )AppTaskGPS, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_GPS_PRIO,
                 (CPU_STK      *)&AppTaskGPSStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_GPS_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_GPS_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
	/**************����Sensor Update����*********************/
	OSTaskCreate((OS_TCB       *)&AppTaskSensorTCB,             
                 (CPU_CHAR     *)"App Task SensorUpdate",
                 (OS_TASK_PTR   )AppTaskSensorUpdate, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_SENSOR_PRIO,
                 (CPU_STK      *)&AppTaskSensorStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_SENSOR_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_SENSOR_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
	/**************����Sensor Update����*********************/
	OSTaskCreate((OS_TCB       *)&AppTaskMotorTCB,             
                 (CPU_CHAR     *)"App Task MotorControl",
                 (OS_TASK_PTR   )AppTaskMotorControl, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_MOTOR_PRIO,
                 (CPU_STK      *)&AppTaskMotorStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_MOTOR_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_MOTOR_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )2,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
								 
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskStart
*	����˵��: ����һ�����������ڶ�����ϵͳ�����󣬱����ʼ���δ������(��BSP_Init��ʵ��)
*             
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 2
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{
		OS_ERR      err;

		(void)p_arg;
		
		CPU_Init();
		bsp_Init();
		BSP_Tick_Init();                      

#if OS_CFG_STAT_TASK_EN > 0u
     OSStatTaskCPUUsageInit(&err);   
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
    
		/* �������� */
    AppTaskCreate(); 
		AppObjCreate();

	  /*Delete task*/
    OSTaskDel(&AppTaskStartTCB, &err);
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskCom
*	����˵��: ��ӡGPS���ݡ���̬����������(DEBUG��)
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 6
*********************************************************************************************************
*/
static void AppTaskCOM(void *p_arg)
{	
	(void)p_arg;
	 
	while(1)
	{
//		DispGPSStatus();
//		DispEulerData();
//		DispEncoderData();
		DispMotorData();
		BSP_OS_TimeDlyMs(100);
	} 						  	 	       											   
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskUserIF
*	����˵��: ��������ֵ
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 5
*********************************************************************************************************
*/
static void AppTaskUserIF(void *p_arg)
{

	(void)p_arg;	               /* ������������� */

	while (1) 
	{   		
    BSP_OS_TimeDlyMs(1000);	     
	}
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskGPS
*	����˵��: ����GPS���ݰ���ֻ������γ�Ⱥͺ�����Ϣ���ٶ���Ϣ��׼ȷ���Բ������ٶȰ�
*						���Ƕ�λ��������5�Ų����δ˴�����
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 4
*********************************************************************************************************
*/
static void AppTaskGPS(void *p_arg)
{	
	(void)p_arg;
	 
	while(1)
	{
		gps_analyze();
		BSP_OS_TimeDlyMs(1000);
	} 						  	 	       											   
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskSensorUpdate
*	����˵��: MPU9150��̬����
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 3
*********************************************************************************************************
*/
static void AppTaskSensorUpdate(void *p_arg)
{	
	 int8_t result;
	
	 CPU_SR_ALLOC();
	 (void)p_arg;

#if  AHRS_USE_DMP
	 result = dmp_init();
	 if(result)
	 {
		 printf("DMP initialize error!\n");
	 }
#else
	 Init_MPU9150();
//	 get_compass_bias();
//	 compass_calibration();
#endif
	 
	while(1)
	{
		CPU_CRITICAL_ENTER();
#if  AHRS_USE_DMP
		dmp_update_euler();
#else
		ahrs_update_euler();
#endif
		//ÿ10ms����һ�γ��ֵ�ƽ���ٶ�
		CalcMotorSpeedAndAngle();
		CPU_CRITICAL_EXIT();
		BSP_OS_TimeDlyMs(10);
	} 						  	 	       											   
}
/*
*********************************************************************************************************
*	�� �� ��: AppTaskMotorControl
*	����˵��: ������������봫��������������ͬ���ȼ�
*						
*	��    ��: p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ��: 3
*********************************************************************************************************
*/
static void AppTaskMotorControl(void *p_arg)
{	
	(void)p_arg;
	 
		Motor_1_Forward();
		Motor_2_Forward();
	while(1)
	{

		BSP_OS_TimeDlyMs(10);
	} 						  	 	       											   
}
/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
    OS_ERR  err;                                         
    
	/* ��ʼ��uC/OS-III �ں� */
    OSInit(&err);  


	/* ����һ����������Ҳ���������񣩡���������ᴴ�����е�Ӧ�ó������� */
	OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,  /* ������ƿ��ַ */           
                 (CPU_CHAR     *)"App Task Start",  /* ������ */
                 (OS_TASK_PTR   )AppTaskStart,      /* ������������ַ */
                 (void         *)0,                 /* ���ݸ�����Ĳ��� */
                 (OS_PRIO       )APP_CFG_TASK_START_PRIO, /* �������ȼ� */
                 (CPU_STK      *)&AppTaskStartStk[0],     /* ��ջ����ַ */
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE / 10, /* ��ջ������������ʾ��10%��Ϊ����� */
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,  /* ��ջ�ռ��С */
                 (OS_MSG_QTY    )0,  /* ������֧�ֽ��ܵ������Ϣ�� */
                 (OS_TICK       )0,  /* ����ʱ��Ƭ */
                 (void         *)0,  /* ��ջ�ռ��С */  
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
		
				 /*  �������£�
					OS_TASK_OPT_STK_CHK      ʹ�ܼ������ջ��ͳ������ջ���õĺ�δ�õ�
					OS_TASK_OPT_STK_CLR      �ڴ�������ʱ����������ջ
					OS_TASK_OPT_SAVE_FP      ���CPU�и���Ĵ��������������л�ʱ���渡��Ĵ���������
				 */  
                 (OS_ERR       *)&err);

	/* ����������ϵͳ������Ȩ����uC/OS-III */
    OSStart(&err);                                             
    
    (void)&err;
    
    return (0);
}

/*
*********************************************************************************************************
*	�� �� ��: App_Printf
*	����˵��: �̰߳�ȫ��printf��ʽ		  			  
*	��    ��: ͬprintf�Ĳ�����
*             ��C�У����޷��г����ݺ���������ʵ�ε����ͺ���Ŀʱ,������ʡ�Ժ�ָ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void  App_Printf(CPU_CHAR *format, ...)
{
    CPU_CHAR  buf_str[80 + 1];
    va_list   v_args;
    OS_ERR    os_err;


    va_start(v_args, format);
   (void)vsnprintf((char       *)&buf_str[0],
                   (size_t      ) sizeof(buf_str),
                   (char const *) format,
                                  v_args);
    va_end(v_args);

    OSSemPend((OS_SEM  *)&SEM_MUTEX,
              (OS_TICK  )0u,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&os_err);

    printf("%s", buf_str);

   (void)OSSemPost((OS_SEM  *)&SEM_MUTEX,
                   (OS_OPT   )OS_OPT_POST_1,
                   (OS_ERR  *)&os_err);

}

/*
*********************************************************************************************************
*	�� �� ��: DispTaskInfo
*	����˵��: ��uCOS-III������Ϣͨ�����ڴ�ӡ����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DispTaskInfo(void)
{
	OS_TCB      *p_tcb;	        /* ����һ��������ƿ�ָ��, TCB = TASK CONTROL BLOCK */
	float CPU = 0.0f;
	CPU_SR_ALLOC();

	  CPU_CRITICAL_ENTER();
    p_tcb = OSTaskDbgListPtr;
    CPU_CRITICAL_EXIT();
	
	/* ��ӡ���� */
	App_Printf("===============================================================\r\n");
	App_Printf(" ���ȼ� ʹ��ջ ʣ��ջ �ٷֱ� ������   ������\r\n");
	App_Printf("  Prio   Used  Free   Per    CPU     Taskname\r\n");

	/* ����������ƿ��б�(TCB list)����ӡ���е���������ȼ������� */
	while (p_tcb != (OS_TCB *)0) 
	{
		CPU = (float)p_tcb->CPUUsage / 100;
		App_Printf("   %2d  %5d  %5d   %02d%%   %5.2f%%   %s\r\n", 
		p_tcb->Prio, 
		p_tcb->StkUsed, 
		p_tcb->StkFree, 
		(p_tcb->StkUsed * 100) / (p_tcb->StkUsed + p_tcb->StkFree),
		CPU,
		p_tcb->NamePtr);		
	 	
		CPU_CRITICAL_ENTER();
        p_tcb = p_tcb->DbgNextPtr;
        CPU_CRITICAL_EXIT();
	}
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	模块名称 : 应用模块
*	文件名称 : app.c
*	版    本 : V1.0
*	说    明 : 用户进程(任务)创建、执行、管理
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#include  <includes.h>

/*
*********************************************************************************************************
*                                       静态全局变量
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
*                                      函数声明
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
*                                      系统组件
*********************************************************************************************************
*/
static  OS_SEM   SEM_MUTEX;	   //用于互斥
static  OS_SEM   SEM_SYNCH;	   //用于同步
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
*	函 数 名: AppObjCreate
*	功能说明: 创建任务通讯
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
*********************************************************************************************************
*/
static  void  AppObjCreate (void)
{
	OS_ERR  err;
	/* 创建互斥信号量 */
	BSP_OS_SemCreate(&SEM_MUTEX, 1,	(CPU_CHAR *)"SEM_MUTEX");
	
	/* 创建同步信号量 */ 
  BSP_OS_SemCreate(&SEM_SYNCH, 0, (CPU_CHAR *)"SEM_SYNCH");
	
	//创建定时器  OS_CFG_TMR_TASK_RATE_HZ = 100HZ
  OSTmrCreate ((OS_TMR              *)&Timer_500ms,
              (CPU_CHAR            *)"MyTimer 500ms",
              (OS_TICK              )0,                  //第一次延时设置为0ms，
              (OS_TICK              )50,                  //定时器周期500ms
              (OS_OPT               )OS_OPT_TMR_PERIODIC,//模式设置为重复模式
              (OS_TMR_CALLBACK_PTR  )_cbOfTmr_500,          //回调函数
              (void                *)0,                  //参数设置为0
              (OS_ERR              *)err);
	OSTmrCreate ((OS_TMR              *)&Timer_1000ms,
              (CPU_CHAR            *)"MyTimer 1000ms",
              (OS_TICK              )0,                  //第一次延时设置为0ms，
              (OS_TICK              )100,                  //定时器周期1000ms
              (OS_OPT               )OS_OPT_TMR_PERIODIC,//模式设置为重复模式
              (OS_TMR_CALLBACK_PTR  )_cbOfTmr_1000,          //回调函数
              (void                *)0,                  //参数设置为0
              (OS_ERR              *)err);
							
	OSTmrStart(&Timer_500ms, (OS_ERR *)err);
	OSTmrStart(&Timer_1000ms, (OS_ERR *)err);
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
*********************************************************************************************************
*/
static  void  AppTaskCreate (void)
{
	OS_ERR      err;
	

	/**************创建COM任务*********************/
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
	
	/**************创建USER IF任务*********************/
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
	/**************创建GPS Update任务*********************/
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
	/**************创建Sensor Update任务*********************/
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
	/**************创建Sensor Update任务*********************/
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
*	函 数 名: AppTaskStart
*	功能说明: 这是一个启动任务，在多任务系统启动后，必须初始化滴答计数器(在BSP_Init中实现)
*             
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 2
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
    
		/* 创建任务 */
    AppTaskCreate(); 
		AppObjCreate();

	  /*Delete task*/
    OSTaskDel(&AppTaskStartTCB, &err);
}
/*
*********************************************************************************************************
*	函 数 名: AppTaskCom
*	功能说明: 打印GPS数据、姿态传感器数据(DEBUG用)
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 6
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
*	函 数 名: AppTaskUserIF
*	功能说明: 处理按键键值
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 5
*********************************************************************************************************
*/
static void AppTaskUserIF(void *p_arg)
{

	(void)p_arg;	               /* 避免编译器报警 */

	while (1) 
	{   		
    BSP_OS_TimeDlyMs(1000);	     
	}
}
/*
*********************************************************************************************************
*	函 数 名: AppTaskGPS
*	功能说明: 解析GPS数据包，只解析经纬度和海拔信息，速度信息不准确所以不解析速度包
*						卫星定位颗数大于5颗才信任此次数据
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 4
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
*	函 数 名: AppTaskSensorUpdate
*	功能说明: MPU9150姿态结算
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 3
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
		//每10ms计算一次车轮的平均速度
		CalcMotorSpeedAndAngle();
		CPU_CRITICAL_EXIT();
		BSP_OS_TimeDlyMs(10);
	} 						  	 	       											   
}
/*
*********************************************************************************************************
*	函 数 名: AppTaskMotorControl
*	功能说明: 电机控制任务，与传感器更新任务相同优先级
*						
*	形    参: p_arg 是在创建该任务时传递的形参
*	返 回 值: 无
	优 先 级: 3
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
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
    OS_ERR  err;                                         
    
	/* 初始化uC/OS-III 内核 */
    OSInit(&err);  


	/* 创建一个启动任务（也就是主任务）。启动任务会创建所有的应用程序任务 */
	OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,  /* 任务控制块地址 */           
                 (CPU_CHAR     *)"App Task Start",  /* 任务名 */
                 (OS_TASK_PTR   )AppTaskStart,      /* 启动任务函数地址 */
                 (void         *)0,                 /* 传递给任务的参数 */
                 (OS_PRIO       )APP_CFG_TASK_START_PRIO, /* 任务优先级 */
                 (CPU_STK      *)&AppTaskStartStk[0],     /* 堆栈基地址 */
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE / 10, /* 堆栈监测区，这里表示后10%作为监测区 */
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,  /* 堆栈空间大小 */
                 (OS_MSG_QTY    )0,  /* 本任务支持接受的最大消息数 */
                 (OS_TICK       )0,  /* 设置时间片 */
                 (void         *)0,  /* 堆栈空间大小 */  
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
		
				 /*  定义如下：
					OS_TASK_OPT_STK_CHK      使能检测任务栈，统计任务栈已用的和未用的
					OS_TASK_OPT_STK_CLR      在创建任务时，清零任务栈
					OS_TASK_OPT_SAVE_FP      如果CPU有浮点寄存器，则在任务切换时保存浮点寄存器的内容
				 */  
                 (OS_ERR       *)&err);

	/* 启动多任务系统，控制权交给uC/OS-III */
    OSStart(&err);                                             
    
    (void)&err;
    
    return (0);
}

/*
*********************************************************************************************************
*	函 数 名: App_Printf
*	功能说明: 线程安全的printf方式		  			  
*	形    参: 同printf的参数。
*             在C中，当无法列出传递函数的所有实参的类型和数目时,可以用省略号指定参数表
*	返 回 值: 无
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
*	函 数 名: DispTaskInfo
*	功能说明: 将uCOS-III任务信息通过串口打印出来
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void DispTaskInfo(void)
{
	OS_TCB      *p_tcb;	        /* 定义一个任务控制块指针, TCB = TASK CONTROL BLOCK */
	float CPU = 0.0f;
	CPU_SR_ALLOC();

	  CPU_CRITICAL_ENTER();
    p_tcb = OSTaskDbgListPtr;
    CPU_CRITICAL_EXIT();
	
	/* 打印标题 */
	App_Printf("===============================================================\r\n");
	App_Printf(" 优先级 使用栈 剩余栈 百分比 利用率   任务名\r\n");
	App_Printf("  Prio   Used  Free   Per    CPU     Taskname\r\n");

	/* 遍历任务控制块列表(TCB list)，打印所有的任务的优先级和名称 */
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

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

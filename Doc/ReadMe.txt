修改记录 :
版本号    日期         作者            说明
V1.0    2016-08-02      fc        1. 添加bsp_encoder,bsp_motor,bsp_gps,bsp_exti,但没在bsp_init()里初始化
                   	              2. 添加pid
                      	          3. 修改软件定时器优先级10, RATE为100HZ
                           	      4. 创建10ms定时器任务来算车轮平均速度

        2016-08-03                1.想配置GPS模块输出速率，但是这个模块上并没EEPROM，可能行不通
                                  2.更改GPS_Init,不用bsp_uuasrt_fifo.c中的缓存

		2016-08-04                1.完善PID文件，增加了几个全局变量为了方便观察中间值
                                         
        2016-08-07                1.只解析GPRMC、GPGGA数据包
								  2.打印观察DMP数据，发现YAW不会发生初始漂移，是否可以用DMP代替需要校准磁力计的AHRS?
                                  3.明天移植SD
                 
		2016-08-10				  1.SD Fatfs(0.11)移植成功，出现FR_DISK_ERR是因为安福来程序里有个SD卡检测引脚.之前那个8G卡是坏的
								  2.GPS模块要定位到5克星以上，根据分的小数点后三位来确定偏移量(以实际调试为准)
								  3.添加bmp.c  powermanage.c.明天移植
								  
		2016-08-11				  1.bq24057是锂电池充电管理芯片，用不着。。。
								  2.驱动电机的引脚与SPI引脚冲突，暂时不用SPI
								  3.测速还是有问题，速度不对
		
		2016-08-12				  1.测速问题解决，因为有两个电机，软件解算车轮速度时有些变量要分开！！！
								  2.输出的车速是放大10倍的，单位  转/S
								  3.编码器encoder结构体中增加用于统计两轮转数的变量,用于解算行车距离
								  4.明天写SD_Log.c
								  
		2016-08-13                1.要测量满电状态下的最高转速，速度控制需要限制输出.如果还是不超过2rad/s考虑换电机
								  2.小车向前行驶时一个车轮是正转一个车轮是反转，这个要仔细考虑
								  
		2016-08-16                1.增加RTC模块
								  2.可以互换2号电机的S1 S2线，这样测得的速度就是正的了
								  
		2016-08-18                1.编码器测速子程序不能和dmp_uodate()放在一个任务里，会影响测速，具体原因还没想明白
		                          2.编码器测速正常
								  3.RTC晶振启动不起来，不知是否晶振的原因
								  4.VelocityPid工作不正常，修改SetMotorsPWM(),使之以目标速度的方向为依据来控制电机转向
								    虽然这样写会影响PID控制器的工作过程，但是为了加入RC Throttle只能这样办了
								  5.明天应当看看HANDFREE的控制是如何做的
								  
		2016-08-19                1.修改SetMotorsPWM()
		                          2.速度控制周期改为30ms，只用一个Kp效果良好
								  3.增加RemoteControl.c 使用TIM1重映射
								  
		2016-08-20                1.遥控器初始设置完成， CH5-SwitchC  CH7-SwitchD  CH8-SwitchE
		                          2.接收机暂时接收两个通道：CH3  CH4
								  3.换上1:46  100rpm的电机，感觉转动更平滑点，以后就用这个电机？
								  4.60RPM: Velocity_Max=220cm/s
									100RPM:Velocity_Max=268cm/s
								  5.电机油门应该直接对应到速度上，通过RC_CH3来给电机速度赋值
								  6.重新调节速度PID，基本上能跟上遥控器输入，但是在输入速度<40时电机震荡较大
								  7.控制器的D起反向作用是因为 P 仍然震荡
								  8.线性化接收机输入值到角度，yaw线性化为角速度
								  9._Motor增加on_off，输入速度低于40时直接关掉电机
								  
	   2016-08-22                 1.所用全局变量多的进程一定要做临界区保护!!!
                                  2.yaw 还是会漂。。。两个小时漂24°
                                  3.姿态数据仍然会有奇异点，是否中断优先级的问题？
                                    可将关中断方法换成调度器上锁来保护dmp_update(),再测算一下dmp_update时间来判断是否真的是因为关中断时间太长影响RC的	
  
       2016-08-23                 1.dmp_read_fifo时间太长，影响RC接收,dmp_update不能关中断
                                  2._euler数据仍然会被干扰
								  
	   2016-08-26                 1.IIC总线上的传感器都只能使用7bit地址
	                              2.放弃MPU9150DMP方法，MPU6050+HMC5883L软件结算偏航正确，且ahrs_update_euler()关中断不影响RC接收
								  3.YAW控制采用速率控制与角度控制相结合的方案

	   2016-08-31                 1.HMC5883L存在软磁偏移，一周偏航角不规则
							
	   2016-09-08                 1.八字法校准后仍然存在软磁偏移，而且在空中走完八字后，角速度输出也出现问题
	                              2.将磁力计增益改为660/guass
								  3.大幅度运动会导致角速度数据出错, 滑动FIFO有问题

	   2016-09-10                 1.水平面校准yaw没问题，但是Roll Pitch倾角补偿没做处理
	                              2.另一个解决方案就是IMU解算，然后yaw再单独倾角补偿结算
								  3.采用manual和automatic两种模式控制，manual则遥控器输出是yaw_target_rate,automatic则是自动确定下一航点方向


APOLLOROBOTROVER引脚分配:			
	Motor_Left: PA6-TIM3_CH1          Encoder_Left: PA0-TIM2_CH1
				PA7-TIM3_CH2		  Encoder_Left: PA1-TIM2_CH2
	Motor_Right:PB0-TIM3_CH3		  Encoder_Right:PB6-TIM4_CH1
				PB1-TIM3_CH4		  Encoder_Right:PB7-TIM4_CH2
				
	串口(调试打印): PA9-UART1_TX
				   PA10-UART_RX
				   
	GPS:  PA2-UART2_TX
		  PA3-UART2_RX
		  
	MPU9150: SCL-PC6
			 SDA-PC7
			 INT-PA4
			 
	接收机:  CH1-PE9
	         CH2-PE11
			 CH3-PE13
			 CH4-PE14
/*
*********************************************************************************************************
*
*	模块名称 : 三轴磁力计HMC5883L驱动模块
*	文件名称 : bsp_HMC5883L.c
*	版    本 : V1.0
*	说    明 : 实现HMC5883L的读写操作。
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

/*
	应用说明:访问HMC5883L前，请先调用一次 bsp_InitI2C()函数配置好I2C相关的GPIO.
*/

#include "bsp.h"

int16_t  HMC5883_FIFO[3][11]; //磁力计滑动滤波窗口



/*
*********************************************************************************************************
*	函 数 名: HMC5883L_FIFOInit
*	功能说明: 连续读取50次数据，以初始化FIFO数组
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void HMC5883L_FIFOInit(void)
{
	    int16_t temp[3];
			unsigned char i;
			for(i=0;i<50;i++){
			HMC5883L_GetRaw(&temp[0],&temp[1],&temp[2]);
			bsp_DelayUS(200);  //延时再读取数据
			}
}

/*
*********************************************************************************************************
*	函 数 名: HMC5883L_NewValues
*	功能说明: 更新一组数据到FIFO数组
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void  HMC5883L_NewValues(int16_t x,int16_t y,int16_t z){
		unsigned char i ;
		int32_t sum=0;

		for(i=1;i<10;i++){
			HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
			HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
			HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
		}

		HMC5883_FIFO[0][9]=x;
		HMC5883_FIFO[1][9]=y;
		HMC5883_FIFO[2][9]=z;

		sum=0;
		for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
				sum+=HMC5883_FIFO[0][i];
		}
		HMC5883_FIFO[0][10]=sum/10;	//将平均值更新

		sum=0;
		for(i=0;i<10;i++){
				sum+=HMC5883_FIFO[1][i];
		}
		HMC5883_FIFO[1][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
				sum+=HMC5883_FIFO[2][i];
		}
		HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues
 
/*
*********************************************************************************************************
*	函 数 名: HMC5883L_SetGain
*	功能说明: 设置 5883L的增益
*	形    参: 目标增益 0-7:
*	返 回 值: 无
*********************************************************************************************************
*/
static void HMC5883L_SetGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7)
		return;
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, gain << 5);
}

/*
*********************************************************************************************************
*	函 数 名: HMC5883L_SetMode
*	功能说明: 设置 5883L的工作模式
*	形    参: 模式值:
*                 0--->连续测量模式
*                 1--->单次测量模式
*	返 回 值: 无
*********************************************************************************************************
*/
static void HMC5883L_SetMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, mode);
  bsp_DelayUS(100);
}
/*
*********************************************************************************************************
*	函 数 名: HMC5883L_SetDOR
*	功能说明: 设置 5883L的 数据输出速率
*           0 -> 0.75Hz  |   1 -> 1.5Hz
*           2 -> 3Hz     |   3 -> 7.5Hz
*           4 -> 15Hz    |   5 -> 30Hz
*           6 -> 75Hz  
*	形    参: Data Output Rate
*	返 回 值: 无
*********************************************************************************************************
*/
static void HMC5883L_SetDOR(unsigned char DOR) {
  if (DOR>6) 
		return ;
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER,DOR<<2);
}

/*
*********************************************************************************************************
*	函 数 名: HMC5883L_GetID
*	功能说明: 读取芯片的ID
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void HMC5883L_GetID(uint8_t* buffer) 
{
	i2cread(HMC5883L_SLAVE_ADDRESS, HMC5883L_ID_REGISTER, 3, buffer);

}  

/*
*********************************************************************************************************
*	函 数 名: HMC5883_Check
*	功能说明: 检测HMC5883是否已连接
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void HMC5883_Check(void) 
{
	  uint8_t ID[3];
		HMC5883L_GetID(ID);
	
		//HMC的固定ID号为三个字节，16进制表示分别为48,34,33
		if(ID[0] == HMC5883L_ID_A_VALUE && ID[1] == HMC5883L_ID_B_VALUE && ID[2] == HMC5883L_ID_C_VALUE)
					printf("HMC5883L check success...\r\n");
		else 
					printf("HMC5883L not found...\r\n");
  
}   
/*
*********************************************************************************************************
*	函 数 名: bsp_InitHMC5883L
*	功能说明: 初始化HMC5883L
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitHMC5883L(void)
{
	
	/* 设置Mode寄存器 */
	#if 1
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER, HMC5883L_CONFIG_A_DEFAULT);//Default Output Rate	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, HMC5883L_CONFIG_B_DEFAULT);//±1.3GUASS	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, HMC5883L_MODE_DEFAULT);	//连续测量模式
	#else	/* 自校准模式 */
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER, HMC5883L_MODE_DEFAULT + 2);	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, HMC5883L_CONFIG_B_DEFAULT);	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, HMC5883L_MODE_DEFAULT);		
	#endif		
	
	  HMC5883L_SetMode(0);//连续测量模式
		HMC5883L_SetDOR(6);  //75hz 更新率
    HMC5883L_SetGain(7); //±8.1guass  gain=230
		HMC5883_Check(); //检测HMC5883是否存在

		HMC5883L_FIFOInit();//fifo初始化

}
/**************************实现函数********************************************
*函数原型:	  HMC58X3_getRaw
*功　　能:	  写HMC5883L的寄存器
输入参数：   reg  寄存器地址
			      val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC5883L_GetRaw(int16_t *x,int16_t *y,int16_t *z) 
{
   unsigned char vbuff[6] = {0};
	 
   i2cread(HMC5883L_SLAVE_ADDRESS, HMC5883L_X_OUT_REGISTER, 6, vbuff);
   HMC5883L_NewValues(((int16_t)vbuff[0] << 8)|vbuff[1], ((int16_t)vbuff[4] << 8)|vbuff[5], ((int16_t)vbuff[2] << 8)|vbuff[3]);
	 
   *x = HMC5883_FIFO[0][10] ;
   *y = HMC5883_FIFO[1][10] ;
   *z = HMC5883_FIFO[2][10] ;
}

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/


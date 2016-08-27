/*
*********************************************************************************************************
*
*	模块名称 : I2C总线驱动模块
*	文件名称 : bsp_i2c_gpio.c
*	版    本 : V1.0
*	说    明 : 用gpio模拟i2c总线, 适用于STM32F4系列CPU。该模块不包括应用层命令帧，仅包括I2C总线基本操作函数。
*
*	修改记录 :
*		版本号  日期       作者    说明
*		V1.0    2016-03-30 方川  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/


#include "bsp.h"

/*
 		PB6/I2C1_SCL
 		PB7/I2C1_SDA
*/

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */

#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */

#define PORT_I2C_SCL	GPIOC			/* GPIO端口 */
#define PIN_I2C_SCL		GPIO_Pin_6		/* GPIO引脚 */

#define PORT_I2C_SDA	GPIOC			/* GPIO端口 */
#define PIN_I2C_SDA		GPIO_Pin_7		/* GPIO引脚 */

#define I2C_SCL_PIN		GPIO_Pin_6			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* 连接到SDA数据线的GPIO */

/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  PORT_I2C_SCL->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  PORT_I2C_SCL->BRR = I2C_SCL_PIN				/* SCL = 0 */

#define I2C_SDA_1()  PORT_I2C_SDA->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  PORT_I2C_SDA->BRR = I2C_SDA_PIN				/* SDA = 0 */

#define I2C_SDA_READ()  ((PORT_I2C_SDA->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#define I2C_SCL_READ()  ((PORT_I2C_SCL->IDR & I2C_SCL_PIN) != 0)	/* 读SCL口线状态 */



/*********************************************************************
*
*      Static Function
*
**********************************************************************
*/
/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < 10; i++);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
	{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参:  _ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参:  无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参:  无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
int8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*********************************************************************
*
*      Public Function
*
**********************************************************************
*/
/*
*********************************************************************************************************
*	函 数 名: bsp_InitI2C
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	/* 开漏输出模式 */
	
	GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL;
	GPIO_Init(PORT_I2C_SCL, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
	GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}
/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参:  _Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* 发送启动信号 */

		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

		i2c_Stop();			/* 发送停止信号 */

		return ucAck;
	}
	return 1;	/* I2C总线异常 */
}

/*
*********************************************************************************************************
*	函 数 名: i2cwrite
*	功能说明: 向指定地址的指定寄存器写一定长度的数据
*	形    参:  addr：设备的I2C总线地址，   reg:寄存器地址   len:数据长度   *data数据指针
*	返 回 值: 返回值 0 表示正确， 返回-1表示未写成功
*********************************************************************************************************
*/
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    
		i2c_Start();
    i2c_SendByte(addr << 1 | I2C_WR);
    if (i2c_WaitAck()) 
		{
        i2c_Stop();
        return -1;
    }
    i2c_SendByte(reg);
    i2c_WaitAck();
    for (i = 0; i < len; i++) {
        i2c_SendByte(data[i]);
        if (i2c_WaitAck()) {
            i2c_Stop();
            return -1;
        }
    }
    i2c_Stop();
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: i2cread
*	功能说明: 从指定地址的指定寄存器读一定长度的数据
*	形    参:  addr：设备的I2C总线地址，   reg:寄存器地址   len:数据长度   *buf数据指针
*	返 回 值: 返回值 0 表示正确， 返回-1表示未写成功
*********************************************************************************************************
*/
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    i2c_Start();
    i2c_SendByte(addr << 1 | I2C_WR);
    if (i2c_WaitAck()) {
        i2c_Stop();
        return -1;
    }
    i2c_SendByte(reg);
    i2c_WaitAck();
    i2c_Start();
    i2c_SendByte(addr << 1 | I2C_RD);
    i2c_WaitAck();
    while (len) {
        *buf = i2c_ReadByte();
        if (len == 1)
            i2c_NAck();
        else
            i2c_Ack();
        buf++;
        len--;
    }
    i2c_Stop();
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: i2cWriteOneByte
*	功能说明: 向指定地址的指定寄存器写一字节的数据
*	形    参:  addr：设备的I2C总线地址，   reg:寄存器地址   len:数据长度   data数据
*	返 回 值: 返回值 0 表示正确， 返回-1表示未写成功
*********************************************************************************************************
*/
int8_t i2cWriteOneByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_Start();
    i2c_SendByte(addr << 1 | I2C_WR);
    if (i2c_WaitAck()) {
        i2c_Stop();
        return -1;
    }
    i2c_SendByte(reg);
    i2c_WaitAck();
    i2c_SendByte(data);
    i2c_WaitAck();
    i2c_Stop();
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: i2cWriteBits
*	功能说明: 读 修改 写 指定设备 指定寄存器一个字节 中的多个位
*	形    参: dev  目标设备地址
*       		reg	   寄存器地址
*						bitStart  目标字节的起始位
*						length   位长度
*						data    存放改变目标字节位的值
*	返 回 值: 返回值 0 表示正确， 返回-1表示未写成功
*********************************************************************************************************
*/

int8_t i2cWriteBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (!i2cread(addr, reg, 1, &b)) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return i2cWriteOneByte(addr, reg, b);
    } else {
        return -1;
    }
}
/*
*********************************************************************************************************
*	函 数 名: i2cwriteBit
*	功能说明: 读 修改 写 指定设备 指定寄存器一个字节 中的1个位
*	形    参: dev  目标设备地址
*       		reg	   寄存器地址
						bitNum  要修改目标字节的bitNum位
						data  为0 时，目标位将被清0 否则将被置位
*	返 回 值: 返回值 0 表示正确， 返回-1表示未写成功
*********************************************************************************************************
*/
int8_t  i2cWriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    
    i2cread(addr, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return i2cWriteOneByte(addr, reg, b);
}
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

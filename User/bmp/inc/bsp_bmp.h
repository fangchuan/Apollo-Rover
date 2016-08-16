/*
*********************************************************************************************************
*
*	模块名称 : bmp图片驱动程序
*	文件名称 : bsp_bmp.h
*	版    本 : V1.1
*	说    明 : 解析BMP(24bit)图片格式，实现存储和读取
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2016-08-11  fc  正式发布
*
*	Copyright (C), 2015-2020, 阿波罗科技 www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef __BSP_BMP_H
#define	__BSP_BMP_H

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>



typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef long LONG;


//位图文件头信息结构定义
//其中不包含文件类型信息（由于结构体的内存结构决定，要是加了的话将不能正确读取文件信息）
typedef struct tagBITMAPFILEHEADER 
{   
    DWORD bfSize; 															//文件大小																				4
    WORD bfReserved1; 													//保留字，不考虑		 															2
    WORD bfReserved2; 													//保留字，同上			 															2
    DWORD bfOffBits; 														//实际位图数据的偏移字节数，即前三个部分长度之和	  4
} BITMAPFILEHEADER,tagBITMAPFILEHEADER;


typedef struct tagBITMAPINFOHEADER
{
    DWORD biSize; 														//指定此结构体的长度，为40		 												4
    LONG biWidth; 														//位图宽											 												4
    LONG biHeight; 														//位图高											 												4
    WORD biPlanes; 														//平面数，为1								 												2
    WORD biBitCount; 													//采用颜色位数，可以是1，2，4，8，16，24新的可以是32	  2
    DWORD biCompression; 											//压缩方式，可以是0，1，2，其中0表示不压缩						  4
    DWORD biSizeImage; 												//实际位图数据占用的字节数														4
    LONG biXPelsPerMeter; 										//X方向分辨率																				4
    LONG biYPelsPerMeter; 										//Y方向分辨率																				4
    DWORD biClrUsed; 													//使用的颜色数，如果为0，则表示默认值(2^颜色位数)			4
    DWORD biClrImportant; 										//重要颜色数，如果为0，则表示所有颜色都是重要的				4
} BITMAPINFOHEADER,tagBITMAPINFOHEADER;


typedef struct tagRGBQUAD 
{
    BYTE rgbBlue; 													//该颜色的蓝色分量
    BYTE rgbGreen; 													//该颜色的绿色分量
    BYTE rgbRed; 														//该颜色的红色分量
    BYTE rgbReserved;											 	//保留值
} RGBQUAD,tagRGBQUAD;


typedef struct RGB_PIXEL
{      //像素的数据类型
    unsigned char   rgbBlue;
    unsigned char   rgbGreen;
    unsigned char   rgbRed;
}RGB_PIXEL;





void Lcd_show_bmp(unsigned short int x, unsigned short int y,unsigned char *pic_name);
int Screen_shot(unsigned short int x,\
                unsigned short int y,\
                unsigned short int Width,\
                unsigned short int Height,\
                unsigned char *filename);


#endif /* __BSP_BMP_H */
/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

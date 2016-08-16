/*
*********************************************************************************************************
*
*	ģ������ : bmpͼƬ��������
*	�ļ����� : bsp_bmp.h
*	��    �� : V1.1
*	˵    �� : ����BMP(24bit)ͼƬ��ʽ��ʵ�ִ洢�Ͷ�ȡ
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2016-08-11  fc  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
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


//λͼ�ļ�ͷ��Ϣ�ṹ����
//���в������ļ�������Ϣ�����ڽṹ����ڴ�ṹ������Ҫ�Ǽ��˵Ļ���������ȷ��ȡ�ļ���Ϣ��
typedef struct tagBITMAPFILEHEADER 
{   
    DWORD bfSize; 															//�ļ���С																				4
    WORD bfReserved1; 													//�����֣�������		 															2
    WORD bfReserved2; 													//�����֣�ͬ��			 															2
    DWORD bfOffBits; 														//ʵ��λͼ���ݵ�ƫ���ֽ�������ǰ�������ֳ���֮��	  4
} BITMAPFILEHEADER,tagBITMAPFILEHEADER;


typedef struct tagBITMAPINFOHEADER
{
    DWORD biSize; 														//ָ���˽ṹ��ĳ��ȣ�Ϊ40		 												4
    LONG biWidth; 														//λͼ��											 												4
    LONG biHeight; 														//λͼ��											 												4
    WORD biPlanes; 														//ƽ������Ϊ1								 												2
    WORD biBitCount; 													//������ɫλ����������1��2��4��8��16��24�µĿ�����32	  2
    DWORD biCompression; 											//ѹ����ʽ��������0��1��2������0��ʾ��ѹ��						  4
    DWORD biSizeImage; 												//ʵ��λͼ����ռ�õ��ֽ���														4
    LONG biXPelsPerMeter; 										//X����ֱ���																				4
    LONG biYPelsPerMeter; 										//Y����ֱ���																				4
    DWORD biClrUsed; 													//ʹ�õ���ɫ�������Ϊ0�����ʾĬ��ֵ(2^��ɫλ��)			4
    DWORD biClrImportant; 										//��Ҫ��ɫ�������Ϊ0�����ʾ������ɫ������Ҫ��				4
} BITMAPINFOHEADER,tagBITMAPINFOHEADER;


typedef struct tagRGBQUAD 
{
    BYTE rgbBlue; 													//����ɫ����ɫ����
    BYTE rgbGreen; 													//����ɫ����ɫ����
    BYTE rgbRed; 														//����ɫ�ĺ�ɫ����
    BYTE rgbReserved;											 	//����ֵ
} RGBQUAD,tagRGBQUAD;


typedef struct RGB_PIXEL
{      //���ص���������
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
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

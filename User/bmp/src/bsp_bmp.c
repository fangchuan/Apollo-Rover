/*
*********************************************************************************************************
*
*	模块名称 : bmp图片驱动程序
*	文件名称 : bsp_bmp.c
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

#include "bsp_bmp.h"
#include "ff.h"

#define BITMAP_FILE 				0x4d42 //bmp文件类型：bm
#define PIX_BYTES           3  		 //每个像素所占字节数
#define HEAD_INFO_OFFSETS   54		 //图片头部信息到像素数据的偏移

#define WIDTHBYTES(bits) 		(((bits)+31)/32*4)	//对于24位真彩色 每一行的像素宽度必须是4的倍数  否则补0补齐	
#define RGB24TORGB16(R,G,B) ((unsigned short int)((((R)>>3)<<11) | (((G)>>2)<<5)	| ((B)>>3)))
#define GETR_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )RGB565) >>11)<<3))		  			//返回8位 R
#define GETG_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )(RGB565 & 0x7ff)) >>5)<<2)) 	//返回8位 G
#define GETB_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )(RGB565 & 0x1f))<<3)))       	//返回8位 B
#pragma diag_suppress 870 	//使编译器支持多字节字符,否则会有invalid multibyte character sequence警告
/* 如果不需要打印bmp相关的提示信息,将printf注释掉即可
 * 如要用printf()，需将串口驱动文件包含进来
 */
#define BMP_DEBUG_PRINTF(FORMAT,...)  //printf(FORMAT,##__VA_ARGS__)


/*********************************************************************
*
*       Global various
*
**********************************************************************
*/

BYTE pColorData[960];                   /* 显示BMP时用，一行真彩色数据缓存 320 * 3 = 960 */
BYTE pColorDatapic[960];                /* 截图时用的，一行真彩色数据缓存 320 * 3 = 960 */

FIL bmpfsrc, bmpfdst; 
FRESULT bmpres;
/*
*********************************************************************************************************
*	函 数 名: showBmpHead
*	功能说明: 打印BMP文件的头信息，用于调试 
*						
*	形    参: pBmpHead位图文件头结构体
*	返 回 值: 无
*********************************************************************************************************
*/
static void showBmpHead(BITMAPFILEHEADER* pBmpHead)
{
    BMP_DEBUG_PRINTF("位图文件头:\r\n");
    BMP_DEBUG_PRINTF("文件大小:%d\r\n",(*pBmpHead).bfSize);
    BMP_DEBUG_PRINTF("保留字:%d\r\n",(*pBmpHead).bfReserved1);
    BMP_DEBUG_PRINTF("保留字:%d\r\n",(*pBmpHead).bfReserved2);
    BMP_DEBUG_PRINTF("实际位图数据的偏移字节数:%d\r\n",(*pBmpHead).bfOffBits);
		BMP_DEBUG_PRINTF("\r\n");	
}
/*
*********************************************************************************************************
*	函 数 名: showBmpInforHead
*	功能说明: 打印BMP文件的头信息，用于调试 
*						
*	形    参: pBmpInforHead位图信息头结构体
*	返 回 值: 无
*********************************************************************************************************
*/
static void showBmpInforHead(tagBITMAPINFOHEADER* pBmpInforHead)
{
    BMP_DEBUG_PRINTF("位图信息头:\r\n");
    BMP_DEBUG_PRINTF("结构体的长度:%d\r\n",(*pBmpInforHead).biSize);
    BMP_DEBUG_PRINTF("位图宽:%d\r\n",(*pBmpInforHead).biWidth);
    BMP_DEBUG_PRINTF("位图高:%d\r\n",(*pBmpInforHead).biHeight);
    BMP_DEBUG_PRINTF("biPlanes平面数:%d\r\n",(*pBmpInforHead).biPlanes);
    BMP_DEBUG_PRINTF("biBitCount采用颜色位数:%d\r\n",(*pBmpInforHead).biBitCount);
    BMP_DEBUG_PRINTF("压缩方式:%d\r\n",(*pBmpInforHead).biCompression);
    BMP_DEBUG_PRINTF("biSizeImage实际位图数据占用的字节数:%d\r\n",(*pBmpInforHead).biSizeImage);
    BMP_DEBUG_PRINTF("X方向分辨率:%d\r\n",(*pBmpInforHead).biXPelsPerMeter);
    BMP_DEBUG_PRINTF("Y方向分辨率:%d\r\n",(*pBmpInforHead).biYPelsPerMeter);
    BMP_DEBUG_PRINTF("使用的颜色数:%d\r\n",(*pBmpInforHead).biClrUsed);
    BMP_DEBUG_PRINTF("重要颜色数:%d\r\n",(*pBmpInforHead).biClrImportant);
		BMP_DEBUG_PRINTF("\r\n");
}

/*
*********************************************************************************************************
*	函 数 名: Lcd_show_bmp
*	功能说明: 显示bmp图片, 24位真彩色 
*						图片大小不能超过320*240
*	形    参: X, Y,pic_name
*	返 回 值: 无
*********************************************************************************************************
*/
void Lcd_show_bmp(unsigned short int x, unsigned short int y,unsigned char *pic_name)
{
	int i, j, k;
	int width, height, l_width;

	BYTE red,green,blue;
	BITMAPFILEHEADER bitHead;
	BITMAPINFOHEADER bitInfoHead;
	WORD fileType;

	unsigned int read_num;
	unsigned char tmp_name[20];
	sprintf((char*)tmp_name,"0:%s",pic_name);

	bmpres = f_open( &bmpfsrc , (char *)tmp_name, FA_OPEN_EXISTING | FA_READ);	

	if(bmpres == FR_OK)
	{
		BMP_DEBUG_PRINTF("Open file success\r\n");

		/* 读取位图文件头信息  两个字节*/         
		f_read(&bmpfsrc,&fileType,sizeof(WORD),&read_num);     

		/* 判断是不是bmp文件 */
		if(fileType != BITMAP_FILE)
		{
			BMP_DEBUG_PRINTF("file is not .bmp file!\r\n");
			return;
		}
		else
		{
			BMP_DEBUG_PRINTF("Ok this is .bmp file\r\n");	
		}        

		/* 读取BMP文件头信息*/
		f_read(&bmpfsrc,&bitHead,sizeof(tagBITMAPFILEHEADER),&read_num);        
		showBmpHead(&bitHead);

		/* 读取位图信息头信息 */
		f_read(&bmpfsrc,&bitInfoHead,sizeof(BITMAPINFOHEADER),&read_num);        
		showBmpInforHead(&bitInfoHead);
	}    
	else
	{
		BMP_DEBUG_PRINTF("file open fail!\r\n");
		return;
	}    

	width = bitInfoHead.biWidth;
	height = bitInfoHead.biHeight;

	/* 计算位图的实际宽度并确保它为4的倍数	*/
	l_width = WIDTHBYTES(width* bitInfoHead.biBitCount);		    

	if(l_width > 960)
	{
		BMP_DEBUG_PRINTF("\n SORRY, PIC IS TOO BIG (<=320)\n");
		return;
	}
	
	/* 判断是否是24bit真彩色图 */
	if(bitInfoHead.biBitCount >= 24)
	{
		for(i=0;i< height; i++)
		{	  
			/* 读取一行bmp的数据到数组pColorData里面 */	
			f_read(&bmpfsrc, pColorData, l_width/2, &read_num);
			f_read(&bmpfsrc, pColorData+l_width/2, l_width/2, &read_num);

			for(j=0; j<width; j++) 											   //一行有效信息
			{
				k = j*3;																	 //一行中第K个像素的起点
				red = pColorData[k+2];
				green = pColorData[k+1];
				blue = 	pColorData[k];
//				LCD_WR_Data(RGB24TORGB16(red,green,blue)); //写入LCD-GRAM
			}            
		}        	 																					     
	}    
	else 
	{        
		BMP_DEBUG_PRINTF("SORRY, THIS PIC IS NOT A 24BITS REAL COLOR");
		return ;
	}
	f_close(&bmpfsrc);
	
}

/**********************************************************
 * 函数名：Screen_shot
 * 描述  ：截取LCD指定位置  指定宽高的像素 保存为24位真彩色bmp格式图片
 * 输入  : 	x								---水平位置 
 *					y								---竖直位置  
 *					Width						---水平宽度   
 *					Height					---竖直高度  	
 *					filename				---文件名
 * 输出  ：	0 		---成功
 *  				-1 		---失败
 *	    		8			---文件已存在
 * 举例  ：Screen_shot(0, 0, 320, 240, "/myScreen");-----全屏截图
 * 注意  ：x范围[0,319]  y范围[0,239]  Width范围[0,320-x]  Height范围[0,240-y]
 *					如果文件已存在,将直接返回	
 **************************************************************/ 
/*
 * bmp文件头有54个字节，其中前14个字节是文件头信息，后40个字节是位图信息头信息
 * bmp文件头之后就是具体的像素信息
 * 0x42 0x4d :bm
 * 54        :实际位图数据的偏移字节数
 * 40        :位图信息头结构体的长度
 * 1         :平面数为1
 * 24        :24bit真彩色
 */
int Screen_shot(unsigned short int x, unsigned short int y, unsigned short int Width,
                unsigned short int Height,  unsigned char *filename)
{
		/* bmp  文件头 54个字节 */
		unsigned char header[54] ={	0x42, 0x4d, 0, 0, 0, 0, 
																0, 0, 0, 0, 54, 0, 
																0, 0, 40,0, 0, 0, 
																0, 0, 0, 0, 0, 0, 
																0, 0, 1, 0, 24, 0, 
																0, 0, 0, 0, 0, 0, 
																0, 0, 0, 0, 0, 
																0, 0, 0, 0, 0,
																0, 0, 0, 0, 0, 
																0, 0, 0};
		
		int i, j;
		long file_size, width, height;
		unsigned char r,g,b;	
		unsigned char tmp_name[30];
		unsigned int mybw;
		unsigned int read_data;
		char kk[4]={0,0,0,0};
		
		/* 宽*高 +补充的字节 + 头部信息 */
		file_size = (long)Width * (long)Height * PIX_BYTES + Height*(Width%4) + HEAD_INFO_OFFSETS;		

		/* 文件大小 4个字节 */
		header[2] = (unsigned char)(file_size &0x000000ff);
		header[3] = (file_size >> 8) & 0x000000ff;
		header[4] = (file_size >> 16) & 0x000000ff;
		header[5] = (file_size >> 24) & 0x000000ff;
		
		/* 位图宽 4个字节 */
		width=Width;	
		header[18] = width & 0x000000ff;
		header[19] = (width >> 8) &0x000000ff;
		header[20] = (width >> 16) &0x000000ff;
		header[21] = (width >> 24) &0x000000ff;
		
		/* 位图高 4个字节 */
		height = Height;
		header[22] = height &0x000000ff;
		header[23] = (height >> 8) &0x000000ff;
		header[24] = (height >> 16) &0x000000ff;
		header[25] = (height >> 24) &0x000000ff;
		
		/* 将filename 按照一定的格式拷贝到 tmp_name */
		sprintf((char*)tmp_name,"0:%s.bmp",filename);
    
    bmpres = f_open( &bmpfsrc , (char*)tmp_name,  FA_OPEN_ALWAYS | FA_WRITE);
		if ( bmpres == FR_OK )
		{    
			/* 将预先定义好的bmp头部信息写进文件里面 */
			bmpres = f_write(&bmpfsrc, header,sizeof(unsigned char)*54, &mybw);		
			
			/* 下面是将指定窗口的数据读出来写到文件里面去 */
			for(i=0; i<Height; i++)					
			{
				if( !(Width%4) )				/* 刚好是4字节对齐 */
				{
						for(j=0; j<Width; j++)  
						{					
//							read_data = LCD_GetPoint(y+j, x+i);				
							
							r =  GETR_FROM_RGB16(read_data);
							g =  GETG_FROM_RGB16(read_data);
							b =  GETB_FROM_RGB16(read_data);

							bmpres = f_write(&bmpfsrc, &b,sizeof(unsigned char), &mybw);
							bmpres = f_write(&bmpfsrc, &g,sizeof(unsigned char), &mybw);
							bmpres = f_write(&bmpfsrc, &r,sizeof(unsigned char), &mybw);
						}

				}
				else
				{
					for(j=0;j<Width;j++)
					{					
//						read_data = LCD_GetPoint(y+j, x+i);
						
						r =  GETR_FROM_RGB16(read_data);
						g =  GETG_FROM_RGB16(read_data);
						b =  GETB_FROM_RGB16(read_data);

						bmpres = f_write(&bmpfsrc, &b,sizeof(unsigned char), &mybw);
						bmpres = f_write(&bmpfsrc, &g,sizeof(unsigned char), &mybw);
						bmpres = f_write(&bmpfsrc, &r,sizeof(unsigned char), &mybw);
					}
					/* 不是4字节对齐则需要补齐 */	
					bmpres = f_write(&bmpfsrc, kk,sizeof(unsigned char)*(Width%4), &mybw);
				}	
			}/* 截屏完毕 */

			f_close(&bmpfsrc); 
			return 0;
		}
		else if ( bmpres == FR_EXIST )  //如果文件已经存在
		{
			return FR_EXIST;	 					//8
		}
		else/* 截屏失败 */
		{
			return -1;
		}    
}

/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

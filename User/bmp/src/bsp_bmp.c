/*
*********************************************************************************************************
*
*	ģ������ : bmpͼƬ��������
*	�ļ����� : bsp_bmp.c
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

#include "bsp_bmp.h"
#include "ff.h"

#define BITMAP_FILE 				0x4d42 //bmp�ļ����ͣ�bm
#define PIX_BYTES           3  		 //ÿ��������ռ�ֽ���
#define HEAD_INFO_OFFSETS   54		 //ͼƬͷ����Ϣ���������ݵ�ƫ��

#define WIDTHBYTES(bits) 		(((bits)+31)/32*4)	//����24λ���ɫ ÿһ�е����ؿ�ȱ�����4�ı���  ����0����	
#define RGB24TORGB16(R,G,B) ((unsigned short int)((((R)>>3)<<11) | (((G)>>2)<<5)	| ((B)>>3)))
#define GETR_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )RGB565) >>11)<<3))		  			//����8λ R
#define GETG_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )(RGB565 & 0x7ff)) >>5)<<2)) 	//����8λ G
#define GETB_FROM_RGB16(RGB565)  ((unsigned char)(( ((unsigned short int )(RGB565 & 0x1f))<<3)))       	//����8λ B
#pragma diag_suppress 870 	//ʹ������֧�ֶ��ֽ��ַ�,�������invalid multibyte character sequence����
/* �������Ҫ��ӡbmp��ص���ʾ��Ϣ,��printfע�͵�����
 * ��Ҫ��printf()���轫���������ļ���������
 */
#define BMP_DEBUG_PRINTF(FORMAT,...)  //printf(FORMAT,##__VA_ARGS__)


/*********************************************************************
*
*       Global various
*
**********************************************************************
*/

BYTE pColorData[960];                   /* ��ʾBMPʱ�ã�һ�����ɫ���ݻ��� 320 * 3 = 960 */
BYTE pColorDatapic[960];                /* ��ͼʱ�õģ�һ�����ɫ���ݻ��� 320 * 3 = 960 */

FIL bmpfsrc, bmpfdst; 
FRESULT bmpres;
/*
*********************************************************************************************************
*	�� �� ��: showBmpHead
*	����˵��: ��ӡBMP�ļ���ͷ��Ϣ�����ڵ��� 
*						
*	��    ��: pBmpHeadλͼ�ļ�ͷ�ṹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void showBmpHead(BITMAPFILEHEADER* pBmpHead)
{
    BMP_DEBUG_PRINTF("λͼ�ļ�ͷ:\r\n");
    BMP_DEBUG_PRINTF("�ļ���С:%d\r\n",(*pBmpHead).bfSize);
    BMP_DEBUG_PRINTF("������:%d\r\n",(*pBmpHead).bfReserved1);
    BMP_DEBUG_PRINTF("������:%d\r\n",(*pBmpHead).bfReserved2);
    BMP_DEBUG_PRINTF("ʵ��λͼ���ݵ�ƫ���ֽ���:%d\r\n",(*pBmpHead).bfOffBits);
		BMP_DEBUG_PRINTF("\r\n");	
}
/*
*********************************************************************************************************
*	�� �� ��: showBmpInforHead
*	����˵��: ��ӡBMP�ļ���ͷ��Ϣ�����ڵ��� 
*						
*	��    ��: pBmpInforHeadλͼ��Ϣͷ�ṹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void showBmpInforHead(tagBITMAPINFOHEADER* pBmpInforHead)
{
    BMP_DEBUG_PRINTF("λͼ��Ϣͷ:\r\n");
    BMP_DEBUG_PRINTF("�ṹ��ĳ���:%d\r\n",(*pBmpInforHead).biSize);
    BMP_DEBUG_PRINTF("λͼ��:%d\r\n",(*pBmpInforHead).biWidth);
    BMP_DEBUG_PRINTF("λͼ��:%d\r\n",(*pBmpInforHead).biHeight);
    BMP_DEBUG_PRINTF("biPlanesƽ����:%d\r\n",(*pBmpInforHead).biPlanes);
    BMP_DEBUG_PRINTF("biBitCount������ɫλ��:%d\r\n",(*pBmpInforHead).biBitCount);
    BMP_DEBUG_PRINTF("ѹ����ʽ:%d\r\n",(*pBmpInforHead).biCompression);
    BMP_DEBUG_PRINTF("biSizeImageʵ��λͼ����ռ�õ��ֽ���:%d\r\n",(*pBmpInforHead).biSizeImage);
    BMP_DEBUG_PRINTF("X����ֱ���:%d\r\n",(*pBmpInforHead).biXPelsPerMeter);
    BMP_DEBUG_PRINTF("Y����ֱ���:%d\r\n",(*pBmpInforHead).biYPelsPerMeter);
    BMP_DEBUG_PRINTF("ʹ�õ���ɫ��:%d\r\n",(*pBmpInforHead).biClrUsed);
    BMP_DEBUG_PRINTF("��Ҫ��ɫ��:%d\r\n",(*pBmpInforHead).biClrImportant);
		BMP_DEBUG_PRINTF("\r\n");
}

/*
*********************************************************************************************************
*	�� �� ��: Lcd_show_bmp
*	����˵��: ��ʾbmpͼƬ, 24λ���ɫ 
*						ͼƬ��С���ܳ���320*240
*	��    ��: X, Y,pic_name
*	�� �� ֵ: ��
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

		/* ��ȡλͼ�ļ�ͷ��Ϣ  �����ֽ�*/         
		f_read(&bmpfsrc,&fileType,sizeof(WORD),&read_num);     

		/* �ж��ǲ���bmp�ļ� */
		if(fileType != BITMAP_FILE)
		{
			BMP_DEBUG_PRINTF("file is not .bmp file!\r\n");
			return;
		}
		else
		{
			BMP_DEBUG_PRINTF("Ok this is .bmp file\r\n");	
		}        

		/* ��ȡBMP�ļ�ͷ��Ϣ*/
		f_read(&bmpfsrc,&bitHead,sizeof(tagBITMAPFILEHEADER),&read_num);        
		showBmpHead(&bitHead);

		/* ��ȡλͼ��Ϣͷ��Ϣ */
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

	/* ����λͼ��ʵ�ʿ�Ȳ�ȷ����Ϊ4�ı���	*/
	l_width = WIDTHBYTES(width* bitInfoHead.biBitCount);		    

	if(l_width > 960)
	{
		BMP_DEBUG_PRINTF("\n SORRY, PIC IS TOO BIG (<=320)\n");
		return;
	}
	
	/* �ж��Ƿ���24bit���ɫͼ */
	if(bitInfoHead.biBitCount >= 24)
	{
		for(i=0;i< height; i++)
		{	  
			/* ��ȡһ��bmp�����ݵ�����pColorData���� */	
			f_read(&bmpfsrc, pColorData, l_width/2, &read_num);
			f_read(&bmpfsrc, pColorData+l_width/2, l_width/2, &read_num);

			for(j=0; j<width; j++) 											   //һ����Ч��Ϣ
			{
				k = j*3;																	 //һ���е�K�����ص����
				red = pColorData[k+2];
				green = pColorData[k+1];
				blue = 	pColorData[k];
//				LCD_WR_Data(RGB24TORGB16(red,green,blue)); //д��LCD-GRAM
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
 * ��������Screen_shot
 * ����  ����ȡLCDָ��λ��  ָ����ߵ����� ����Ϊ24λ���ɫbmp��ʽͼƬ
 * ����  : 	x								---ˮƽλ�� 
 *					y								---��ֱλ��  
 *					Width						---ˮƽ���   
 *					Height					---��ֱ�߶�  	
 *					filename				---�ļ���
 * ���  ��	0 		---�ɹ�
 *  				-1 		---ʧ��
 *	    		8			---�ļ��Ѵ���
 * ����  ��Screen_shot(0, 0, 320, 240, "/myScreen");-----ȫ����ͼ
 * ע��  ��x��Χ[0,319]  y��Χ[0,239]  Width��Χ[0,320-x]  Height��Χ[0,240-y]
 *					����ļ��Ѵ���,��ֱ�ӷ���	
 **************************************************************/ 
/*
 * bmp�ļ�ͷ��54���ֽڣ�����ǰ14���ֽ����ļ�ͷ��Ϣ����40���ֽ���λͼ��Ϣͷ��Ϣ
 * bmp�ļ�ͷ֮����Ǿ����������Ϣ
 * 0x42 0x4d :bm
 * 54        :ʵ��λͼ���ݵ�ƫ���ֽ���
 * 40        :λͼ��Ϣͷ�ṹ��ĳ���
 * 1         :ƽ����Ϊ1
 * 24        :24bit���ɫ
 */
int Screen_shot(unsigned short int x, unsigned short int y, unsigned short int Width,
                unsigned short int Height,  unsigned char *filename)
{
		/* bmp  �ļ�ͷ 54���ֽ� */
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
		
		/* ��*�� +������ֽ� + ͷ����Ϣ */
		file_size = (long)Width * (long)Height * PIX_BYTES + Height*(Width%4) + HEAD_INFO_OFFSETS;		

		/* �ļ���С 4���ֽ� */
		header[2] = (unsigned char)(file_size &0x000000ff);
		header[3] = (file_size >> 8) & 0x000000ff;
		header[4] = (file_size >> 16) & 0x000000ff;
		header[5] = (file_size >> 24) & 0x000000ff;
		
		/* λͼ�� 4���ֽ� */
		width=Width;	
		header[18] = width & 0x000000ff;
		header[19] = (width >> 8) &0x000000ff;
		header[20] = (width >> 16) &0x000000ff;
		header[21] = (width >> 24) &0x000000ff;
		
		/* λͼ�� 4���ֽ� */
		height = Height;
		header[22] = height &0x000000ff;
		header[23] = (height >> 8) &0x000000ff;
		header[24] = (height >> 16) &0x000000ff;
		header[25] = (height >> 24) &0x000000ff;
		
		/* ��filename ����һ���ĸ�ʽ������ tmp_name */
		sprintf((char*)tmp_name,"0:%s.bmp",filename);
    
    bmpres = f_open( &bmpfsrc , (char*)tmp_name,  FA_OPEN_ALWAYS | FA_WRITE);
		if ( bmpres == FR_OK )
		{    
			/* ��Ԥ�ȶ���õ�bmpͷ����Ϣд���ļ����� */
			bmpres = f_write(&bmpfsrc, header,sizeof(unsigned char)*54, &mybw);		
			
			/* �����ǽ�ָ�����ڵ����ݶ�����д���ļ�����ȥ */
			for(i=0; i<Height; i++)					
			{
				if( !(Width%4) )				/* �պ���4�ֽڶ��� */
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
					/* ����4�ֽڶ�������Ҫ���� */	
					bmpres = f_write(&bmpfsrc, kk,sizeof(unsigned char)*(Width%4), &mybw);
				}	
			}/* ������� */

			f_close(&bmpfsrc); 
			return 0;
		}
		else if ( bmpres == FR_EXIST )  //����ļ��Ѿ�����
		{
			return FR_EXIST;	 					//8
		}
		else/* ����ʧ�� */
		{
			return -1;
		}    
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : �ڴ����ģ��
*	�ļ����� : malloc.h
*	��    �� : V1.0
*	˵    �� : 
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#ifndef __MALLOC_H
#define __MALLOC_H
#include "stm32f10x.h"

 
#ifndef NULL
#define NULL 0
#endif

//���������ڴ��
#define SRAMIN	 0		//�ڲ��ڴ��
#define SRAMEX   1		//�ⲿ�ڴ�� 

#ifndef USE_EXTERN_SRAM
	#define SRAMBANK 	1	//����֧�ֵ�SRAM����.	
#else
	#define SRAMBANK  2
#endif


//mem1�ڴ�����趨.mem1��ȫ�����ڲ�SRAM����.
#define MEM1_BLOCK_SIZE					32  	  						//�ڴ���СΪ32�ֽ�
#define MEM1_MAX_SIZE						15*1024  						//�������ڴ� 15K
#define MEM1_ALLOC_TABLE_SIZE		MEM1_MAX_SIZE/MEM1_BLOCK_SIZE 	//�ڲ��ڴ���С:480bytes


//mem2�ڴ�����趨.mem2���ڴ�ش����ⲿSRAM����
#define MEM2_BLOCK_SIZE					32  	  						//�ڴ���СΪ32�ֽ�
#define MEM2_MAX_SIZE						500 *1024  						//�������ڴ�500K
#define MEM2_ALLOC_TABLE_SIZE		MEM2_MAX_SIZE/MEM2_BLOCK_SIZE 	//�ⲿ�ڴ���С��16000bytes 
 
//�ڴ���������
struct _m_mallco_dev
{
	void (*init)(u8);							//��ʼ��
	u8 (*perused)(u8);		  	    //�ڴ�ʹ����
	u8 	*membase[SRAMBANK];				//�ڴ�� ����SRAMBANK��������ڴ�
	u16 *memmap[SRAMBANK]; 				//�ڴ����״̬��
	u8  memrdy[SRAMBANK]; 				//�ڴ�����Ƿ����
};
extern struct _m_mallco_dev mallco_dev;	 //��mallco.c���涨��
u8 my_mem_perused(u8 memx);				//����ڴ�ʹ����(��/�ڲ�����) 
void my_mem_init(u8 memx);				//�ڴ�����ʼ������(��/�ڲ�����)

////////////////////////////////////////////////////////////////////////////////
//�û����ú���
void myfree(u8 memx,void *ptr);  			//�ڴ��ͷ�(�ⲿ����)
void *mymalloc(u8 memx,u32 size);			//�ڴ����(�ⲿ����)
void *myrealloc(u8 memx,void *ptr,u32 size);//���·����ڴ�(�ⲿ����)
#endif

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

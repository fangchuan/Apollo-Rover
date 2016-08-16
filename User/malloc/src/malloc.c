/*
*********************************************************************************************************
*
*	ģ������ : �ڴ����ģ��
*	�ļ����� : malloc.c
*	��    �� : V1.0
*	˵    �� : 
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/
#include "malloc.h"	    


//�ڴ��(32�ֽڶ���)
__align(32) u8 mem1base[MEM1_MAX_SIZE];//�ڲ�SRAM�ڴ��
#ifdef  USE_EXTERN_SRAM
__align(32) u8 mem2base[MEM2_MAX_SIZE] __attribute__((at(0X68000000)));	//�ⲿSRAM�ڴ��
#endif

//�ڴ�����
u16 mem1mapbase[MEM1_ALLOC_TABLE_SIZE];	//�ڲ�SRAM�ڴ��MAP
#ifdef  USE_EXTERN_SRAM
u16 mem2mapbase[MEM2_ALLOC_TABLE_SIZE] __attribute__((at(0X68000000+MEM2_MAX_SIZE)));	//�ⲿSRAM�ڴ��MAP
#endif

//�ڴ�������	   
const u32 memtblsize[SRAMBANK]={	//�ڴ���С
					MEM1_ALLOC_TABLE_SIZE,
	#ifdef  USE_EXTERN_SRAM
					MEM2_ALLOC_TABLE_SIZE
	#endif
};			
const u32 memblksize[SRAMBANK]={	//�ڴ�ֿ��С
					MEM1_BLOCK_SIZE,
	#ifdef  USE_EXTERN_SRAM
					MEM2_BLOCK_SIZE
	#endif
};						
const u32 memsize[SRAMBANK]={			//�ڴ��ܴ�С
					MEM1_MAX_SIZE,
	#ifdef  USE_EXTERN_SRAM
					MEM2_MAX_SIZE
	#endif
};								
/*
*********************************************************************************************************
*                                      ��������
*********************************************************************************************************
*/
static void mymemset(void *s,u8 c,u32 count);	//�����ڴ�
static void mymemcpy(void *des,void *src,u32 n);//�����ڴ�     
static u32 my_mem_malloc(u8 memx,u32 size);	//�ڴ����(�ڲ�����)
static u8 my_mem_free(u8 memx,u32 offset);		//�ڴ��ͷ�(�ڲ�����)


//�ڴ���������
struct _m_mallco_dev mallco_dev=
{
	my_mem_init,					//�ڴ��ʼ��
	my_mem_perused,				//�ڴ�ʹ����
	mem1base,							//�ڴ��
#ifdef USE_EXTERN_SRAM
	mem2base,
#endif
	mem1mapbase,					//�ڴ����״̬��
#ifdef USE_EXTERN_SRAM
	mem2mapbase,
#endif
	0,			  		 				//�ڴ����δ����
#ifdef USE_EXTERN_SRAM
	0,
#endif
};


/*
*********************************************************************************************************
*	�� �� ��: mymemcpy
*	����˵��: �����ڴ�
*	��    ��: *des:Ŀ�ĵ�ַ; *src:Դ��ַ; n:��Ҫ���Ƶ��ڴ泤��(�ֽ�Ϊ��λ)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void mymemcpy(void *des,void *src,u32 n)  
{  
    u8 *xdes=des;
		u8 *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  
/*
*********************************************************************************************************
*	�� �� ��: mymemcpy
*	����˵��: �����ڴ�
*	��    ��: *s:�ڴ��׵�ַ; c :Ҫ���õ�ֵ; count:��Ҫ���õ��ڴ��С(�ֽ�Ϊ��λ)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void mymemset(void *s,u8 c,u32 count)  
{  
    u8 *xs = s;  
    while(count--)*xs++=c;  
}	 
/*
*********************************************************************************************************
*	�� �� ��: my_mem_init
*	����˵��: �ڴ�����ʼ��
*	��    ��: memx:�����ڴ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void my_mem_init(u8 memx)  
{  
    mymemset(mallco_dev.memmap[memx], 0,memtblsize[memx]*2);//�ڴ�״̬����������  
	  mymemset(mallco_dev.membase[memx], 0,memsize[memx]);	//�ڴ��������������  
	  mallco_dev.memrdy[memx]=1;								//�ڴ�����ʼ��OK  
} 
/*
*********************************************************************************************************
*	�� �� ��: my_mem_perused
*	����˵��: ��ȡ�ڴ�ʹ����
*	��    ��: memx:�����ڴ��
*	�� �� ֵ: ����ֵ:ʹ����(0~100)
*********************************************************************************************************
*/
uint8_t my_mem_perused(u8 memx)  
{  
    u32 used=0;  
    u32 i;  
    for(i=0;i<memtblsize[memx];i++)  
    {  
        if(mallco_dev.memmap[memx][i])used++; 
    } 
    return (used*100)/(memtblsize[memx]);  
} 
/*
*********************************************************************************************************
*	�� �� ��: my_mem_malloc
*	����˵��: �ڴ����(�ڲ�����)
*	��    ��: memx:�����ڴ��; size:Ҫ������ڴ��С(�ֽ�)
*	�� �� ֵ: ����ֵ:0XFFFFFFFF,�������;����,�ڴ�ƫ�Ƶ�ַ 
*********************************************************************************************************
*/
static u32 my_mem_malloc(u8 memx,u32 size)  
{  
    signed long offset=0;  
    u32 nmemb;	//��Ҫ���ڴ����  
		u32 cmemb=0;//�������ڴ����
    u32 i;  
    if(!mallco_dev.memrdy[memx])
			mallco_dev.init(memx);//δ��ʼ��,��ִ�г�ʼ�� 
    if(size==0)
			return 0XFFFFFFFF;//����Ҫ����
    nmemb=size/memblksize[memx];  	//��ȡ��Ҫ����������ڴ����
    if(size%memblksize[memx])
			nmemb++;  
    for(offset=memtblsize[memx]-1;offset>=0;offset--)//���������ڴ������  
    {     
			if(!mallco_dev.memmap[memx][offset])
				cmemb++;								//�������ڴ��������
			else
				cmemb=0;								//�����ڴ������
			if(cmemb==nmemb)					//�ҵ�������nmemb�����ڴ��
			{
				for(i=0;i<nmemb;i++)  	//��ע�ڴ��ǿ� 
				{  
						mallco_dev.memmap[memx][offset+i]=nmemb;  
				}  
				return (offset*memblksize[memx]);//����ƫ�Ƶ�ַ  
			}
    }  
    return 0XFFFFFFFF;//δ�ҵ����Ϸ����������ڴ��  
} 
/*
*********************************************************************************************************
*	�� �� ��: my_mem_free
*	����˵��: �ͷ��ڴ�(�ڲ�����)
*	��    ��: memx:�����ڴ��; offset:�ڴ��ַƫ��
*	�� �� ֵ: ����ֵ:0,�ͷųɹ�;1,�ͷ�ʧ��; 2,ƫ�Ƶ�ַ����
*********************************************************************************************************
*/
static u8 my_mem_free(u8 memx,u32 offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[memx])//δ��ʼ��,��ִ�г�ʼ��
		{
				mallco_dev.init(memx);    
        return 1;//δ��ʼ��  
    }  
    if(offset<memsize[memx])//ƫ�����ڴ����. 
    {  
        int index=offset/memblksize[memx];			//ƫ�������ڴ�����  
        int nmemb=mallco_dev.memmap[memx][index];	//�ڴ������
        for(i=0;i<nmemb;i++)  						//�ڴ������
        {  
            mallco_dev.memmap[memx][index+i]=0;  
        }  
        return 0;  
    }
		else 
			return 2;//ƫ�Ƴ�����.  
}  
/*
*********************************************************************************************************
*	�� �� ��: myfree
*	����˵��: �ͷ��ڴ�(�ⲿ����)
*	��    ��: memx:�����ڴ��; ptr:�ڴ��׵�ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void myfree(u8 memx,void *ptr)  
{  
	u32 offset;   
	if(ptr==NULL)
		return;//��ַΪ0. 
	
 	offset=(u32)ptr-(u32)mallco_dev.membase[memx];  
	
  my_mem_free(memx,offset);	//�ͷ��ڴ�  
	
}  
/*
*********************************************************************************************************
*	�� �� ��: mymalloc
*	����˵��: �����ڴ�(�ⲿ����)
*	��    ��: memx:�����ڴ��; size:�ڴ��С(�ֽ�)
*	�� �� ֵ: ���䵽���ڴ��׵�ַ.
*********************************************************************************************************
*/
void *mymalloc(u8 memx,u32 size)  
{  
    u32 offset;   
		offset=my_mem_malloc(memx,size); 
	
    if(offset==0XFFFFFFFF)
			return NULL;  
    else
			return (void*)((u32)mallco_dev.membase[memx]+offset);  
}
/*
*********************************************************************************************************
*	�� �� ��: myrealloc
*	����˵��: ���·����ڴ�(�ⲿ����)
*	��    ��: memx:�����ڴ��; *ptr:���ڴ��׵�ַ; size:�ڴ��С(�ֽ�)
*	�� �� ֵ: �·��䵽���ڴ��׵�ַ.
*********************************************************************************************************
*/
void *myrealloc(u8 memx,void *ptr,u32 size)  
{  
    u32 offset;    
    offset=my_mem_malloc(memx,size);   	
    if(offset==0XFFFFFFFF)
			return NULL;     
    else  
    {  									   
				mymemcpy((void*)((u32)mallco_dev.membase[memx]+offset),ptr,size);	//�������ڴ����ݵ����ڴ�   
        myfree(memx,ptr);  											  		//�ͷž��ڴ�
        return (void*)((u32)mallco_dev.membase[memx]+offset);  				//�������ڴ��׵�ַ
    }  
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/


#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "common.h"

/*
*********************************************************************************************************
*	�� �� ��: invSqrt
*	����˵��: ���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float invSqrt(float x) 
{
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
}






/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

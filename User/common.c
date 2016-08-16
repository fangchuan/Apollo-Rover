#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "common.h"

/*
*********************************************************************************************************
*	函 数 名: invSqrt
*	功能说明: 快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 
*	形    参: 无
*	返 回 值: 无
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






/***************************** 阿波罗科技 www.apollorobot.cn (END OF FILE) *********************************/

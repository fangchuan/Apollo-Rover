/*
*********************************************************************************************************
*
*	ģ������ : LEDָʾ������ģ��
*	�ļ����� : bsp_led.c
*	��    �� : V1.0
*	˵    �� : ����LEDָʾ��
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*
*	Copyright (C), 2015-2020, �����޿Ƽ� www.apollorobot.cn
*
*********************************************************************************************************
*/

#include "bsp.h"



/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG)

#define GPIO_PORT_LED1  GPIOD
#define GPIO_PIN_LED1		GPIO_Pin_13

#define GPIO_PORT_LED2  GPIOG
#define GPIO_PIN_LED2		GPIO_Pin_14


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitLed
*	����˵��: ����LEDָʾ����ص�GPIO,  �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitLed(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_LED, ENABLE);


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED1;
	GPIO_Init(GPIO_PORT_LED1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED2;
	GPIO_Init(GPIO_PORT_LED2, &GPIO_InitStructure);
	
	bsp_LedOff(1);
	bsp_LedOff(2);
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_LedOn
*	����˵��: ����ָ����LEDָʾ�ơ�
*	��    ��:  _no : ָʾ����ţ���Χ 1 - 4
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_LedOn(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_PORT_LED1->BSRR = GPIO_PIN_LED1;
	}
	else if (_no == 1)
	{
		GPIO_PORT_LED2->BSRR = GPIO_PIN_LED2;
	}

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_LedOff
*	����˵��: Ϩ��ָ����LEDָʾ�ơ�
*	��    ��:  _no : ָʾ����ţ���Χ 1 - 4
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_LedOff(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_PORT_LED1->BRR = GPIO_PIN_LED1;
	}
	else if (_no == 1)
	{
		GPIO_PORT_LED2->BRR = GPIO_PIN_LED2;
	}

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_LedToggle
*	����˵��: ��תָ����LEDָʾ�ơ�
*	��    ��:  _no : ָʾ����ţ���Χ 1 - 4
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_LedToggle(uint8_t _no)
{
	if (_no == 1)
	{
		GPIO_PORT_LED1->ODR ^= GPIO_PIN_LED1;
	}
	else if (_no == 2)
	{
		GPIO_PORT_LED2->ODR ^= GPIO_PIN_LED2;
	}

}
/*
*********************************************************************************************************
*	�� �� ��: bsp_IsLedOn
*	����˵��: �ж�LEDָʾ���Ƿ��Ѿ�������
*	��    ��:  _no : ָʾ����ţ���Χ 1 - 4
*	�� �� ֵ: 1��ʾ�Ѿ�������0��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_IsLedOn(uint8_t _no)
{
	if (_no == 1)
	{
		if ((GPIO_PORT_LED1->ODR & GPIO_PIN_LED1) == 1)
		{
			return 1;
		}
		return 0;
	}
	else if (_no == 2)
	{
		if ((GPIO_PORT_LED2->ODR & GPIO_PIN_LED2) == 1)
		{
			return 1;
		}
		return 0;
	}

	return 0;
}

/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : ���������HMC5883L����ģ��
*	�ļ����� : bsp_HMC5883L.c
*	��    �� : V1.0
*	˵    �� : ʵ��HMC5883L�Ķ�д������
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

/*
	Ӧ��˵��:����HMC5883Lǰ�����ȵ���һ�� bsp_InitI2C()�������ú�I2C��ص�GPIO.
*/

#include "bsp.h"

int16_t  HMC5883_FIFO[3][11]; //�����ƻ����˲�����
static uint8_t   isCorrectted = 0;
static _Hmc5883l _hmc5883l;

/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_FIFOInit
*	����˵��: ������ȡ50�����ݣ��Գ�ʼ��FIFO����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void HMC5883L_FIFOInit(void)
{
	    int16_t temp[3];
			unsigned char i;
			for(i=0;i<50;i++){
			HMC5883L_GetRaw(&temp[0],&temp[1],&temp[2]);
			bsp_DelayUS(200);  //��ʱ�ٶ�ȡ����
			}
}

/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_NewValues
*	����˵��: ����һ�����ݵ�FIFO����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void  HMC5883L_NewValues(int16_t x,int16_t y,int16_t z){
		unsigned char i ;
		int32_t sum=0;

		for(i=1;i<10;i++){
			HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
			HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
			HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
		}

		HMC5883_FIFO[0][9]=x;
		HMC5883_FIFO[1][9]=y;
		HMC5883_FIFO[2][9]=z;

		sum=0;
		for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
				sum+=HMC5883_FIFO[0][i];
		}
		HMC5883_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

		sum=0;
		for(i=0;i<10;i++){
				sum+=HMC5883_FIFO[1][i];
		}
		HMC5883_FIFO[1][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
				sum+=HMC5883_FIFO[2][i];
		}
		HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues
 
/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_SetGain
*	����˵��: ���� 5883L������
*	��    ��: Ŀ������ 0-7:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void HMC5883L_SetGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7)
		return;
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, gain << 5);
}

/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_SetMode
*	����˵��: ���� 5883L�Ĺ���ģʽ
*	��    ��: ģʽֵ:
*                 0--->��������ģʽ
*                 1--->���β���ģʽ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void HMC5883L_SetMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, mode);
  bsp_DelayUS(100);
}
/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_SetDOR
*	����˵��: ���� 5883L�� �����������
*           0 -> 0.75Hz  |   1 -> 1.5Hz
*           2 -> 3Hz     |   3 -> 7.5Hz
*           4 -> 15Hz    |   5 -> 30Hz
*           6 -> 75Hz  
*	��    ��: Data Output Rate
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void HMC5883L_SetDOR(unsigned char DOR) {
  if (DOR>6) 
		return ;
  i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER,DOR<<2);
}

/*
*********************************************************************************************************
*	�� �� ��: HMC5883L_GetID
*	����˵��: ��ȡоƬ��ID
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void HMC5883L_GetID(uint8_t* buffer) 
{
	i2cread(HMC5883L_SLAVE_ADDRESS, HMC5883L_ID_REGISTER, 3, buffer);

}  

/*
*********************************************************************************************************
*	�� �� ��: HMC5883_Check
*	����˵��: ���HMC5883�Ƿ�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HMC5883_Check(void) 
{
	  uint8_t ID[3];
		HMC5883L_GetID(ID);
	
		//HMC�Ĺ̶�ID��Ϊ�����ֽڣ�16���Ʊ�ʾ�ֱ�Ϊ48,34,33
		if(ID[0] == HMC5883L_ID_A_VALUE && ID[1] == HMC5883L_ID_B_VALUE && ID[2] == HMC5883L_ID_C_VALUE)
					printf("HMC5883L check success...\r\n");
		else 
					printf("HMC5883L not found...\r\n");
  
}   
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitHMC5883L
*	����˵��: ��ʼ��HMC5883L
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitHMC5883L(void)
{
	
	/* ����Mode�Ĵ��� */
	#if 1
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER, HMC5883L_CONFIG_A_DEFAULT);//Default Output Rate	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, HMC5883L_CONFIG_B_DEFAULT);//��1.3GUASS	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, HMC5883L_MODE_DEFAULT);	//��������ģʽ
	#else	/* ��У׼ģʽ */
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_A_REGISTER, HMC5883L_CONFIG_A_DEFAULT + 1);	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_CONFIG_B_REGISTER, HMC5883L_CONFIG_B_DEFAULT);	
		i2cWriteOneByte(HMC5883L_SLAVE_ADDRESS, HMC5883L_MODE_REGISTER, HMC5883L_MODE_DEFAULT);		
	#endif		
	
	  HMC5883L_SetMode(0);//��������ģʽ
		HMC5883L_SetDOR(6);  //75hz ������
    HMC5883L_SetGain(3); //��2.5guass  gain=660
		HMC5883_Check(); //���HMC5883�Ƿ����

		HMC5883L_FIFOInit();//fifo��ʼ��

}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	  HMC58X3_getRaw
*��������:	  дHMC5883L�ļĴ���
���������   reg  �Ĵ�����ַ
			      val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC5883L_GetRaw(int16_t *x,int16_t *y,int16_t *z) 
{
   unsigned char vbuff[6] = {0};
	 int16_t  magx,magy, magz;
	 
   i2cread(HMC5883L_SLAVE_ADDRESS, HMC5883L_X_OUT_REGISTER, 6, vbuff);
//   HMC5883L_NewValues(((int16_t)vbuff[0] << 8)|vbuff[1], ((int16_t)vbuff[4] << 8)|vbuff[5], ((int16_t)vbuff[2] << 8)|vbuff[3]);
	 magx = ((int16_t)vbuff[0] << 8)| vbuff[1];
	 magy = ((int16_t)vbuff[4] << 8)| vbuff[5];
	 magz = ((int16_t)vbuff[2] << 8)| vbuff[3];
	 
	 if(isCorrectted){
		 *x = magx * _hmc5883l.X_Gain + _hmc5883l.X_Offset;
		 *y = magy * _hmc5883l.Y_Gain + _hmc5883l.Y_Offset;
		 *z = magz * _hmc5883l.Z_Gain + _hmc5883l.Z_Offset;
	 }
	 else{
//	   *x = HMC5883_FIFO[0][10];
//		 *y = HMC5883_FIFO[1][10];
//		 *z = HMC5883_FIFO[2][10];
		   *x = magx;
		   *y = magy;
		   *z = magz;
			 
	 }
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	  HMC5883L_Correct
*��������:	  У׼HMC5883L�������ϵ����ƫ��
���������   ms_counts
���������  ��
*******************************************************************************/
void HMC5883L_Correct(uint32_t ms_counts)
{
	   uint8_t vbuff[6];
	   if(isCorrectted)
			 return ;
		 
		 do{
			  i2cread(HMC5883L_SLAVE_ADDRESS, HMC5883L_X_OUT_REGISTER, 6, vbuff);
			  _hmc5883l.X = (int16_t)vbuff[0] <<8 | vbuff[1];
			  _hmc5883l.Y = (int16_t)vbuff[4] <<8 | vbuff[5];
			  _hmc5883l.Z = (int16_t)vbuff[2] <<8 | vbuff[3];;
			 
			  if(_hmc5883l.X > _hmc5883l.X_Max){
					_hmc5883l.X_Max = _hmc5883l.X;
				}
				else{
					if(_hmc5883l.X < _hmc5883l.X_Min)
						_hmc5883l.X_Min = _hmc5883l.X;
				}
				
				if(_hmc5883l.Y > _hmc5883l.Y_Max){
					_hmc5883l.Y_Max = _hmc5883l.Y;
				}
				else{
					if(_hmc5883l.Y < _hmc5883l.Y_Min)
						_hmc5883l.Y_Min = _hmc5883l.Y;
				}
				
			 if(_hmc5883l.Y > _hmc5883l.Y_Max){
					_hmc5883l.Y_Max = _hmc5883l.Y;
				}
				else{
					if(_hmc5883l.Z < _hmc5883l.Z_Min)
						_hmc5883l.Z_Min = _hmc5883l.Z;
				}
				
				bsp_DelayMS(1);
		 }while(ms_counts --);
		 
		 isCorrectted = 1;
		 
 //���������Ӧ�����������Ϊ1
  if(((_hmc5883l.X_Max - _hmc5883l.X_Min) >= (_hmc5883l.Y_Max - _hmc5883l.Y_Min)) && 
		 ((_hmc5883l.X_Max - _hmc5883l.X_Min) >= (_hmc5883l.Z_Max - _hmc5883l.Z_Min)))
  {
    _hmc5883l.X_Gain = 1.0;
		_hmc5883l.Y_Gain = (float)(_hmc5883l.X_Max - _hmc5883l.X_Min) / (_hmc5883l.Y_Max - _hmc5883l.Y_Min);
		_hmc5883l.Z_Gain = (float)(_hmc5883l.X_Max - _hmc5883l.X_Min) / (_hmc5883l.Z_Max - _hmc5883l.Z_Min);
		_hmc5883l.X_Offset = -0.5 * (_hmc5883l.X_Max + _hmc5883l.X_Min);
		_hmc5883l.Y_Offset = -0.5 * _hmc5883l.Y_Gain * (_hmc5883l.Y_Max + _hmc5883l.Y_Min);
		_hmc5883l.Z_Offset = -0.5 * _hmc5883l.Z_Gain * (_hmc5883l.Z_Max + _hmc5883l.Z_Min);	 
  }
	if(((_hmc5883l.Y_Max - _hmc5883l.Y_Min) >= (_hmc5883l.X_Max - _hmc5883l.X_Min)) && 
		 ((_hmc5883l.Y_Max - _hmc5883l.Y_Min) >= (_hmc5883l.Z_Max - _hmc5883l.Z_Min)))
  {
    _hmc5883l.Y_Gain = 1.0;
		_hmc5883l.X_Gain = (float)(_hmc5883l.Y_Max - _hmc5883l.Y_Min) / (_hmc5883l.X_Max - _hmc5883l.X_Min);
		_hmc5883l.Z_Gain = (float)(_hmc5883l.Y_Max - _hmc5883l.Y_Min) / (_hmc5883l.Z_Max - _hmc5883l.Z_Min);
		_hmc5883l.Y_Offset = -0.5 * (_hmc5883l.Y_Max + _hmc5883l.Y_Min);
		_hmc5883l.X_Offset = -0.5 * _hmc5883l.X_Gain * (_hmc5883l.X_Max + _hmc5883l.X_Min);
		_hmc5883l.Z_Offset = -0.5 * _hmc5883l.Z_Gain * (_hmc5883l.Z_Max + _hmc5883l.Z_Min);	 
  }
	if(((_hmc5883l.Z_Max - _hmc5883l.Z_Min) >= (_hmc5883l.Y_Max - _hmc5883l.Y_Min)) && 
		 ((_hmc5883l.Z_Max - _hmc5883l.Z_Min) >= (_hmc5883l.X_Max - _hmc5883l.X_Min)))
  {
    _hmc5883l.Z_Gain = 1.0;
		_hmc5883l.Y_Gain = (float)(_hmc5883l.Z_Max - _hmc5883l.Z_Min) / (_hmc5883l.Y_Max - _hmc5883l.Y_Min);
		_hmc5883l.Z_Gain = (float)(_hmc5883l.Z_Max - _hmc5883l.Z_Min) / (_hmc5883l.X_Max - _hmc5883l.X_Min);
		_hmc5883l.Z_Offset = -0.5 * (_hmc5883l.Z_Max + _hmc5883l.Z_Min);
		_hmc5883l.Y_Offset = -0.5 * _hmc5883l.Y_Gain * (_hmc5883l.Y_Max + _hmc5883l.Y_Min);
		_hmc5883l.X_Offset = -0.5 * _hmc5883l.X_Gain * (_hmc5883l.X_Max + _hmc5883l.X_Min);	 
  }
		 
}
/*********************************************************************************************************
*	�� �� ��: DispHmc5883Data
*	����˵��: ��ӡ������У׼����
*	��    �Σ�
*	�� �� ֵ: 
*********************************************************************************************************
*/
void DispHmc5883Data(void)
{
		printf("X_Offset:%f\n", _hmc5883l.X_Offset);
		printf("Y_Offset:%f\n", _hmc5883l.Y_Offset);
		printf("Z_Offset:%f\n", _hmc5883l.Z_Offset);
		printf("X_Gain:%f\n", _hmc5883l.X_Gain);
		printf("Y_Gain:%f\n", _hmc5883l.Y_Gain);
		printf("Z_Gain:%f\n", _hmc5883l.Z_Gain);
}
/***************************** �����޿Ƽ� www.apollorobot.cn (END OF FILE) *********************************/


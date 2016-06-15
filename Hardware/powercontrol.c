/********************************************************************************
	*	@file		powercontrol.c
	*	@author	Jeff
	*	@date		2015/5/26
	*	@brief	��Դ���ƹܽ�����
	*					
	*					�������			PA5-POWERCTRL0(��ѹ)
	*												PA6-POWERCTRL1(BT)
	*												PB2-CTRL(�ػ�)
	*												PC15-ACCTRL(����)
	*******************************************************************************/
#include "stm32f37x.h"
#include "powercontrol.h"
#include "nvic_systick.h"

void PowerControl_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);//ʹ��PA,PC�˿�ʱ��
	
//	/*PA5-POWERCTRL0(��ѹ),PA6-POWERCTRL1(BT)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOA,GPIO_Pin_5);//��ѹ��Դ��
////	GPIO_ResetBits(GPIOA,GPIO_Pin_5);//��ѹ��Դ��
//	GPIO_SetBits(GPIOA,GPIO_Pin_6);//BT��Դ��
//	GPIO_ResetBits(GPIOA,GPIO_Pin_6);//BT��Դ��
	
//	/*PB2-CTRL(�ػ�)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
////	GPIO_SetBits(GPIOB,GPIO_Pin_2);//�ػ�
//	GPIO_ResetBits(GPIOB,GPIO_Pin_2);//������Դ�������ǹػ�

///*PA4-CTRL(�ػ�)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//20160601 ����PA4 DAC1  ǯͷ��������������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
////	GPIO_SetBits(GPIOA,GPIO_Pin_4);//�ػ�
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);//������Դ�������ǹػ�
	
	/*PC15-ACCTRL(����)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOC,GPIO_Pin_15);//������Դ��
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);//������Դ��
}

void Power_On_Voltage(void)//��ѹ��Դ��
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}
void Power_Off_Voltage(void)//��ѹ��Դ��
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}
void Power_On_Current(void)//������Դ��
{
	GPIO_SetBits(GPIOC,GPIO_Pin_15);
}
void Power_Off_Current(void)//������Դ��
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}
void Power_On_Bt(void)//BT��Դ��
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}
void Power_Off_Bt(void)//BT��Դ��
{
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
}
void Power_On_VDD(void)//������Դ�������ǹػ�
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
void Power_Off_VDD(void)//�ػ���CTRL=1��CP1�����أ���O~1������͵�ƽ���ٽ�CTRL���㡣
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	delay_us(100);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
/************************END*****************************/

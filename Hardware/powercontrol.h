#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H
#include "stm32f37x.h"

void Power_On_Voltage(void);//��ѹ��Դ��
void Power_Off_Voltage(void);//��ѹ��Դ��
void Power_On_Current(void);//������Դ��
void Power_Off_Current(void);//������Դ��
void Power_On_Bt(void);//BT��Դ��
void Power_Off_Bt(void);//BT��Դ��
void Power_On_VDD(void);//������Դ�������ǹػ�
void Power_Off_VDD(void);//�ػ���CTRL=1��CP1�����أ���O1�Σ�����͵�ƽ���ٽ�CTRL���㡣

void PowerControl_Init(void);

#endif

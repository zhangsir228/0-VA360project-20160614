#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f37x.h"
#include "stmdataprocess.h"

extern uint8_t Is_Cal_Mode ;//2016-07-22 Lea ����ȫ��У׼ģʽ��־  Ϊ�˽�У׼���ܲ��඼���ɵ�����������

typedef struct{
	///
	float curr_now;
	
}defSysValue;


void display_dta0660(void);
#endif

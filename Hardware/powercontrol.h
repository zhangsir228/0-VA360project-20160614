#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H
#include "stm32f37x.h"

void Power_On_Voltage(void);//电压电源开
void Power_Off_Voltage(void);//电压电源关
void Power_On_Current(void);//电流电源开
void Power_Off_Current(void);//电流电源关
void Power_On_Bt(void);//BT电源开
void Power_Off_Bt(void);//BT电源关
void Power_On_VDD(void);//整机电源正常，非关机
void Power_Off_VDD(void);//关机，CTRL=1，CP1上升沿，（O1拔）输出低电平。再将CTRL置零。

void PowerControl_Init(void);

#endif

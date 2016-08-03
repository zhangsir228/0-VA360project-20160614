#ifndef __TIMER_H
#define __TIMER_H	
#include "stm32f37x.h"


extern u16 powertimer;
extern u8 powertimer_1s_flag;
extern u8 powertimer_0_5s_flag;
extern u8 timer_1s_flag_for_Standby;

void TIM12_Config_1s(void);
void TIM14_Config_0_5s(void);

void TIM13_Config_1s_Standby(void);




#endif /* TIMER_H */


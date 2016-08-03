#ifndef __NVIC_SYSTICK_H
#define __NVIC_SYSTICK_H

#include "stm32f37x.h"

void NVIC_Configuration(void);

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif /* __NVIC_SYSTICK_H */

/**************************************************end file**************************************************/

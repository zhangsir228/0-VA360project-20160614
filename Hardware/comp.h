#ifndef __COMP_H
#define	__COMP_H

#include "stm32f37x.h"
extern u16 TIM5CH4_CAPTURE_STA;//输入捕获状态
extern u32 TIM5CH4_CAPTURE_VAL;//输入捕获值
void TIM5_2_Compare_Capture_Init(void);


#endif /* __COMP_H */

/**************************************************end file**************************************************/

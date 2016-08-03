#ifndef __BUZZER_LED_H
#define	__BUZZER_LED_H

#include "stm32f37x.h"
extern u8 longshort;

void BUZZER_LED_Init(void);

//BUZZER
void BUZZER_Open(u8 flag);
void BUZZER_Close(void);
void BUZZER_Toggle(void);

//LED
void LED_Open(void);
void LED_Close(void);
void LED_Toggle(void);
#endif /* __BUZZER_LED_H */

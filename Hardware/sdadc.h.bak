#ifndef __SDADC_H
#define __SDADC_H	
#include "stm32f37x.h"

#define POT_SDADC_VREF       SDADC_VREF_VREFINT2/*!< The reference voltage is forced internally to 1.8V VREFINT */
#define POT_SDADC_GAIN       SDADC_Gain_1   /* Internal gain 1 is seleted: 
                                               SDADC_GAIN must be updated according to
                                               POT_SDADC_GAIN */
#define SDADC_GAIN           (uint32_t) 1  /* SDADC internal gain is set to 1: update this define
                                              according to POT_SDADC_GAIN */
//#define SDADC_RESOL          (uint32_t) 65535 /* 2e16 - 1 */
#define SDADC_INIT_TIMEOUT   30 /* ~ about two SDADC clock cycles after INIT is set */
#define SDADC_CAL_TIMEOUT    4*30720 /*  ~5.12 ms at 6 MHz  in a single calibration sequence */
#define SDADC_VREFERENCE           (float) 3.6//1.8  /* SDADC reference is set to 1.22V */

#define SDADC_CAL_COEF 			(float)SDADC_VREFERENCE / (2 * SDADC_GAIN * 32767)
#define SDADC_CAL_COEF2			SDADC_CAL_COEF* SDADC_CAL_COEF

extern uint16_t SAMPLING_FREQ;
extern int32_t InjectedConvData[];
extern uint16_t DMA2_MEM_LEN; 

u8 SDADC1_Config(void);
void TIM19_Config(void);

#endif

/**************************************************end file**************************************************/

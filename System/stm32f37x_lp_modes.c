/**
  ******************************************************************************
  * @file    PWR/PWR_CurrentConsumption/stm32f37x_lp_modes.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2012
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the STM32F37x Low Power Modes:
  *           - Sleep Mode
  *           - STOP mode with RTC
  *           - STANDBY mode without RTC
  *           - STANDBY mode with RTC
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x_lp_modes.h"
#include "ht1621.h"
#include "powercontrol.h"

/** @addtogroup STM32F37x_StdPeriph_Examples
  * @{
  */

/** @addtogroup PWR_CurrentConsumption
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  This function configures the system to enter Standby mode.
  *         STANDBY Mode
  *         ============
  *           
  *           
  *           
  * @param  None
  * @retval None
  */
void StandbyMode_Measure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//  /* Enable WKUP pin 2 */
//  PWR_WakeUpPinCmd(PWR_WakeUpPin_1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	
	
	Power_Off_Voltage();//PA5=1
	Power_Off_Current();//PC15=0
	
	TIM_Cmd(TIM3, DISABLE);
	DMA_Cmd(DMA2_Channel3, DISABLE);
	USART_Cmd(USART1, DISABLE);
	USART_Cmd(USART1, DISABLE);
	
	TIM_Cmd(TIM5, DISABLE);
	TIM_Cmd(TIM12, DISABLE);
	TIM_Cmd(TIM14, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	
	TIM_Cmd(TIM13, DISABLE);
	
	TIM_Cmd(TIM19, DISABLE);
	SDADC_Cmd(SDADC1, DISABLE);
	SDADC_Cmd(SDADC2, DISABLE);
	COMP_Cmd(COMP_Selection_COMP1, DISABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM19, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDADC1 | RCC_APB2Periph_SDADC2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC1, DISABLE);
	
	
	
	
	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
                        RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOF , ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Disable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
                         RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOF, DISABLE);
	
	
//	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
  PWR_EnterSTANDBYMode();
  
  /* Infinite loop */
  while (1)
  {
  }
}





/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

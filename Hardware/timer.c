/********************************************************************************
	*	@file		timer.c
	*	@author	Jeff
	*	@date		2015/7/27
	*	@brief	
	*					
	*					
	*	
	*					ʹ�����裺TIM12(1,1),TIM13(3,3),TIM14(1,2)
	*
	*					
	*					
	*******************************************************************************/

#include "stm32f37x.h"
#include "timer.h"

u16 powertimer;
u8 powertimer_1s_flag;
u8 powertimer_0_5s_flag;
u8 timer_1s_flag_for_Standby;
/**************************************************************** 
*	��������: TIM12_Config_1s 
*	��    ��: ��ʱ��12����1s�����ж�           
*	��    ��: �� 
*	�� �� ֵ: ��  
*****************************************************************/
void TIM12_Config_1s(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM12 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	
  /* Time base configuration */
					/*Ƶ�ʼ��㣺f = 72,000,000/(71999+1)/(999+1) = 1Hz������1s */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(1999+1)-1;//35999
  TIM_TimeBaseStructure.TIM_Prescaler = 1999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM12 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM12_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//	/* TIM12 enable counter */
//	TIM_Cmd(TIM12, ENABLE);
}

/**************************************************************** 
*	��������: TIM13_Config_1s 
*	��    ��: ��ʱ��13����1s�����ж�           
*	��    ��: �� 
*	�� �� ֵ: ��  
*****************************************************************/
void TIM13_Config_1s_Standby(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM13 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	
  /* Time base configuration */
					/*Ƶ�ʼ��㣺f = 72,000,000/(71999+1)/(999+1) = 1Hz������1s */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(1999+1)-1;//35999
  TIM_TimeBaseStructure.TIM_Prescaler = 1999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM13 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM13_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	/* TIM13 enable counter */
	TIM_Cmd(TIM13, ENABLE);
}

/**************************************************************** 
*	��������: TIM14_Config_0_5s 
*	��    ��: ��ʱ��14����0.5s�����ж�           
*	��    ��: �� 
*	�� �� ֵ: ��  
*****************************************************************/
void TIM14_Config_0_5s(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM13 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	
  /* Time base configuration */
					/*Ƶ�ʼ��㣺f = 72,000,000/(35999+1)/(999+1) = 2Hz������0.5s */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(999+1)/2-1;//35999
  TIM_TimeBaseStructure.TIM_Prescaler = 999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM14 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//	/* TIM14 enable counter */
//	TIM_Cmd(TIM14, ENABLE);
}











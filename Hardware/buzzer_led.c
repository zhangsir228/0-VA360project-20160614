/********************************************************************************
	*	@file		buzzer_led.c
	*	@author	Jeff
	*	@date		2015/3/19
	*	@brief	蜂鸣器和LED配置
	*					
	*
	*					使用外设：TIM4(1,3)
	*
	*					输出引脚			蜂鸣器		PC13
	*												LED				PC14
	*					
	*
	*******************************************************************************/
#include "buzzer_led.h"
#include "nvic_systick.h"

u8 longshort;
/**************************************************************** 
*	函数名称: TIM4_Config 
*	功    能: 定时器4产生10ms更新中断           
*	参    数: 无 
*	返 回 值: 无  
*****************************************************************/
static void TIM4_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
  /* Time base configuration */
					/*频率计算：f = 72,000,000/(99+1)/(359+1) = 2kHz*/
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(99+1)/3000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 99;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM4 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//	/* TIM4 enable counter */
//	TIM_Cmd(TIM4, ENABLE);
}

void BUZZER_LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	
	TIM4_Config();
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14);
}





//BUZZER
void BUZZER_Open(u8 flag)
{
	longshort=flag;
	TIM_Cmd(TIM4, ENABLE);
}

void BUZZER_Close(void)
{
	TIM_Cmd(TIM4, DISABLE);
	GPIO_ResetBits(GPIOC , GPIO_Pin_13);
}

void BUZZER_Toggle(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_13 , (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))));
}
//LED
void LED_Open(void)
{
	GPIO_SetBits(GPIOC , GPIO_Pin_14);
}

void LED_Close(void)
{
	GPIO_ResetBits(GPIOC , GPIO_Pin_14);
}

void LED_Toggle(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_14 , (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14))));
}
//************************END*****************************//

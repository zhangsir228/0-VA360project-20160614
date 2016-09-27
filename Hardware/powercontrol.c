/********************************************************************************
	*	@file	powercontrol.c
	*	@author	lea
	*	@date	2018/8/15
	*	@brief	电源控制管脚配置
	*					
	*			输出引脚			PA5-POWERCTRL0(外围)
	*							PA6-POWERCTRL1(BT)							
	*							PC15-ACCTRL(电流)
	*******************************************************************************/
#include "stm32f37x.h"
#include "powercontrol.h"
#include "nvic_systick.h"

void PowerControl_Init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	NVIC_InitTypeDef   	NVIC_InitStructure;
	EXTI_InitTypeDef 	EXTI_InitStructure;
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);//使能PA,PC端口时钟
	
	
//	/*PA5-POWERCTRL0(外围电源),PA6-POWERCTRL1(BT)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//IO口速度为50MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_5);//外围电源开
	//GPIO_ResetBits(GPIOA,GPIO_Pin_5);//外围电源关
	GPIO_SetBits(GPIOA,GPIO_Pin_6);//BT电源关
	//GPIO_ResetBits(GPIOA,GPIO_Pin_6);//BT电源开
	
	/*PC15-ACCTRL(电流)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_15);//电流电源开
	//GPIO_ResetBits(GPIOC,GPIO_Pin_15);//电流电源关


}



void Power_On_Voltage(void)//外围电源开
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}
void Power_Off_Voltage(void)//外围电源关
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}
void Power_On_Current(void)//电流电源开
{
	GPIO_SetBits(GPIOC,GPIO_Pin_15);
}
void Power_Off_Current(void)//电流电源关
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}
void Power_On_Bt(void)//BT电源开
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}
void Power_Off_Bt(void)//BT电源关
{
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
}
















/************************END*****************************/

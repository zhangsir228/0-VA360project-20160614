/********************************************************************************
	*	@file		powercontrol.c
	*	@author	Jeff
	*	@date		2015/5/26
	*	@brief	电源控制管脚配置
	*					
	*					输出引脚			PA5-POWERCTRL0(电压)
	*												PA6-POWERCTRL1(BT)
	*												PB2-CTRL(关机)
	*												PC15-ACCTRL(电流)
	*******************************************************************************/
#include "stm32f37x.h"
#include "powercontrol.h"
#include "nvic_systick.h"

void PowerControl_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);//使能PA,PC端口时钟
	
//	/*PA5-POWERCTRL0(电压),PA6-POWERCTRL1(BT)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOA,GPIO_Pin_5);//电压电源关
////	GPIO_ResetBits(GPIOA,GPIO_Pin_5);//电压电源开
//	GPIO_SetBits(GPIOA,GPIO_Pin_6);//BT电源关
//	GPIO_ResetBits(GPIOA,GPIO_Pin_6);//BT电源开
	
//	/*PB2-CTRL(关机)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
////	GPIO_SetBits(GPIOB,GPIO_Pin_2);//关机
//	GPIO_ResetBits(GPIOB,GPIO_Pin_2);//整机电源正常，非关机

///*PA4-CTRL(关机)*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//20160601 用于PA4 DAC1  钳头霍尔传感器驱动
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
////	GPIO_SetBits(GPIOA,GPIO_Pin_4);//关机
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);//整机电源正常，非关机
	
	/*PC15-ACCTRL(电流)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOC,GPIO_Pin_15);//电流电源开
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);//电流电源关
}

void Power_On_Voltage(void)//电压电源开
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}
void Power_Off_Voltage(void)//电压电源关
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
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
void Power_On_VDD(void)//整机电源正常，非关机
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
void Power_Off_VDD(void)//关机，CTRL=1，CP1上升沿，（O~1）输出低电平。再将CTRL置零。
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	delay_us(100);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
/************************END*****************************/

/****************************************************************************************
	*	@file		main.c
	*	@author	Jeff
	*	@date		2014/8/21
	*	@brief	本程序实现功能：
	*					1）通过SDADC1和SDADC2同步采集（+DMA+TIM19/采样频率为5120Hz）1024个差分数据，
	*					得到2路测量值——电压和电流，用来后续计算**基本量+FFT**；
	*					2）通过COMP1和COMP2分别得到2路相电压、相电流的基频；
	*					3）串口发送/接收，波特率为初始化函数的形参；
	*					4）外部中断按键操作，按键功能未定义，目前只是简单的功能；
	*
	*		其他：5）液晶TFT成功配置，可配合使用，供调试；
	*					6）使用TIM3产生PWM波，供测试COMP使用；
	*					7）DAC输出电压，供测试SDADC；
	*					8）开发板上LED；
	*
	**具体使用的引脚见相应的配置文件，有具体注释**
	*					
	***************************************************************************************/
#include "stm32f37x.h"
#include "nvic_systick.h"
#include <math.h>
#include "matrixkey.h"
#include "ht1621.h"
#include "userVAinteraction.h"
#include "buzzer_led.h"
#include "uart.h"
#include "sdadc.h"
#include "comp.h"
#include "dta0660.h"
#include "stmdataprocess.h"
#include "timer.h"
#include "powercontrol.h"
#include "stm32f37x_lp_modes.h"
#include "comm.h"
#include "flash_data.h"
#include "dac.h"

//uint8_t BuffDisplay[16];

//20160420 lea 优化程序逻辑：	1.将采样点数提升至1024+2^8(1024+256=1280),
//															不做硬件上的锅过零点判断，软件上判断。仍然只取中间的1024个数据点。
//														2.新增通信
//
//
//20160616 lea 新增AC出发超时计量功能。： AC档由输入电压过零点通过比较器触发输入捕捉中断，使用睡眠计数变量，如果一个采样触发周期超过某个值
//																			则立即启动一次SDADC采样。能保证在无输入型号或输入信号不满足要求时都能进行大致的测量，不至于保持零
//																			值或上次的测量值不动，还能解决从某些特殊功能档回到正常测量过程中错过触发信号而导致不能正确开始的情况。
//
//20160725 lea 尝试修改主频降低功耗->24M
//
//20160801 	新增 mV_V档，电阻挡软件校准。


uint8_t Is_Cal_Mode = 0;//2016-07-22 Lea 定义全局校准模式标志  为了将校准功能步奏都集成到按键操作上
uint8_t OnOffState = 0; //开关机状态 0：关机休眠状态   1：开机运行状态

u16 count_for_Standby=0;
defSysValue SysValue ;

void RCC_Conf(void)
{
	SystemInit();
}


void start_configration(void)
{
	uint8_t sdadc_cali_flag=0;//SDADC配置成功与否标志，0-成功，1-INITRDY未置位，2-EOCAL未置位
	
	RCC_Conf();
	
	
	Init_flash();//初始化一些参数
		
	delay_init();//延时函数初始化，可以使用delay_ms(u16 nms),delay_us(u32 nus)
	NVIC_Configuration();//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	HT1621_Init();
	lcd_clr();
	lcd_full_test();
	Init_Keyboard_Interrupt();//内含TIM3-按键10ms检测中断、EXTI9_5-按键外部中断
	BUZZER_LED_Init();		//TIM4驱动
	uart_init(9600);//内含USART2与上位机通信串口中断9600
	delay_ms(20);
	sdadc_cali_flag=SDADC1_Config();//采样配置，内含DMA2-SDADC数据DMA中断
	TIM19_Config();//采样触发、采样频率配置
	TIM5_2_Compare_Capture_Init();//COMP比较器、定时器配置，用于基频捕获，内含TIM5比较器捕获中断
	
	DTA0660_Init();//内含Usart1-DTA0660通信串口中断，用于单片机与DTA通信
	TIM12_Config_1s();//1s定时器，内含TIM12-1s中断：秒表显示闪烁
	TIM14_Config_0_5s();//0.5s定时器，内含TIM14-0.5s中断：相序测量闪烁

	PowerControl_Init(); //总电源，蓝牙电源，钳头供电电源；换挡间隙的外部中断
	
	
	
	DAC_Config();//新增DAC输出驱动钳头
	Dac1_Set_Vol(1400);
	
	TIM13_Config_1s_Standby();//内含1s中断，计数1800次为30min，进入Standby模式
	
	//2016-07-25 增加内部校准  启动时的按键检测  确定是否进入校准模式
	SoftKeyValue = OnceSoftKey();//无消抖的获取按键值  ScanKey();//
	if(SoftKeyValue==KEY_VALUE_5)//启动检测到func按键 置位校准标志位
	{
		SoftKeyValue = KEY_NULL;		
		Is_Cal_Mode = 1;
		
		lcd_clr();
		lcd_show_Cal(1);delay_ms(2000);//显示CAL进入校准状态
	}
	
	//旋转开关位置预检测
	EXTI->IMR &= ~Keyboard_EXTI_Line;//屏蔽来自线上的中断请求，防止在定时器扫描矩阵键盘时进入此中断
	TIM_Cmd(TIM3, ENABLE);//打开定时器3，在定时器作用下开始扫描矩阵键盘
}


int main(void)
{
//	u8 t;
//	u8 len;	
//	u16 times=0;
//	float num;
	start_configration();
			
	//WriteEeprom();
	//Self_Calibration();//在电阻档自检线路和放大器
	
//	//液晶全亮
//	Send(SetAddr,0x3B,0x00,0x8B);//设置LCD地址
//	CheckReceive(SetAddr);//等待60ms，DTA从接收到发送完需要时间
//	for(t=0;t<8;t++)
//	{
//		Send(WriteFwd,0xFF,0xFF+WriteFwd);
//		CheckReceive(WriteFwd);
//	}
	
	while(1)
	{
		if(timer_1s_flag_for_Standby==1)//用于休眠计时
		{
			timer_1s_flag_for_Standby=0;
			count_for_Standby++;//休眠计时+1s;              //OFF档
			if((count_for_Standby==5000)||(RotaryKeyValue==KEY_VALUE_11))//5min-300,10min-600,15min-900,20min-1200,25min-1500,30min-1800
			{//长时间无操作或者 打到关机档 则进入休眠状态
				
				StandbyMode_Measure();//进入休眠
				count_for_Standby = 0;			
			}						
		}
		if((RotaryKeyChanged_flag == 1) || (SoftKeyChanged_flag == 1))//有按键
		{
			if(RotaryKeyValue==KEY_VALUE_11){}//OFF 档位
							
			count_for_Standby=0;//休眠计时清零
			manipulate();
		}
		
		DataProcessing();//根据相应功能档配置DTA0660,并做当前档位的数据处理。  
		display();//液晶显示
		
		Communication_Service();//蓝牙串口服务函数	
	}
	return 0;
}




/**************************************************end file**************************************************/


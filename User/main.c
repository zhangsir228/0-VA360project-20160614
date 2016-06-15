/****************************************************************************************
	*	@file		main.c
	*	@author	Jeff
	*	@date		2014/8/21
	*	@brief	������ʵ�ֹ��ܣ�
	*					1��ͨ��SDADC1��SDADC2ͬ���ɼ���+DMA+TIM19/����Ƶ��Ϊ5120Hz��1024��������ݣ�
	*					�õ�2·����ֵ������ѹ�͵�����������������**������+FFT**��
	*					2��ͨ��COMP1��COMP2�ֱ�õ�2·���ѹ��������Ļ�Ƶ��
	*					3�����ڷ���/���գ�������Ϊ��ʼ���������βΣ�
	*					4���ⲿ�жϰ�����������������δ���壬Ŀǰֻ�Ǽ򵥵Ĺ��ܣ�
	*
	*		������5��Һ��TFT�ɹ����ã������ʹ�ã������ԣ�
	*					6��ʹ��TIM3����PWM����������COMPʹ�ã�
	*					7��DAC�����ѹ��������SDADC��
	*					8����������LED��
	*
	**����ʹ�õ����ż���Ӧ�������ļ����о���ע��**
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

//20160420 lea �Ż������߼���	1.����������������1024+2^8(1024+256=1280),
//															����Ӳ���ϵĹ�������жϣ��������жϡ���Ȼֻȡ�м��1024�����ݵ㡣
//														2.����ͨ��
//
//
//
//


////���ڶ��У�����������										
//float Cal_A_tab[15][2]={
//	{1501.6f,	1500},
//	{1598.3f,	1600},
//	{1698.9f,	1700},
//	{1743.5f,	1750},
//	{1790.3f,	1800},
//	{1881.0f,	1900},
//	{1976.0f,	2000},
//	{2063.2f,	2100},
//	{2150.0f,	2200},
//	{2229.5f,	2300},
//	{2303.0f,	2400},
//	{2367.3f,	2500},
//	{2429.6f,	2600},
//	{2450.9f,	2650},
//		
//};
////���ڶ��У�����������										
//float Cal_A_gain[15]={
//	1501.6f,	//1500,
//	1598.3f,	//1600,
//	1698.9f,	//1700,
//	1743.5f,	//1750,
//	1790.3f,	//1800,
//	1881.0f,	//1900,
//	1976.0f,	//2000,
//	2063.2f,	//2100,
//	2150.0f,	//2200,
//	2229.5f,	//2300,
//	2303.0f,	//2400,
//	2367.3f,	//2500,
//	2429.6f,	//2600,
//	2450.9f,	//2650,
//		
//};


defSysValue SysValue ;

void start_configration(void)
{
	uint8_t sdadc_cali_flag=0;//SDADC���óɹ�����־��0-�ɹ���1-INITRDYδ��λ��2-EOCALδ��λ
	
	Init_flash();//��ʼ��һЩ����
		
	delay_init();//��ʱ������ʼ��������ʹ��delay_ms(u16 nms),delay_us(u32 nus)
	NVIC_Configuration();//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	HT1621_Init();
	lcd_clr();
	lcd_full_test();
	Init_Keyboard_Interrupt();//�ں�TIM3-����10ms����жϡ�EXTI9_5-�����ⲿ�ж�
	BUZZER_LED_Init();		//TIM4����
	uart_init(115200);//�ں�USART2����λ��ͨ�Ŵ����ж�9600
	delay_ms(500);
	sdadc_cali_flag=SDADC1_Config();//�������ã��ں�DMA2-SDADC����DMA�ж�
	TIM19_Config();//��������������Ƶ������
	TIM5_2_Compare_Capture_Init();//COMP�Ƚ�������ʱ�����ã����ڻ�Ƶ�����ں�TIM5�Ƚ��������ж�
	DTA0660_Init();//�ں�Usart1-DTA0660ͨ�Ŵ����жϣ����ڵ�Ƭ����DTAͨ��
	TIM12_Config_1s();//1s��ʱ�����ں�TIM12-1s�жϣ������ʾ��˸
	TIM14_Config_0_5s();//0.5s��ʱ�����ں�TIM14-0.5s�жϣ����������˸

	PowerControl_Init();
	
	DAC_Config();//����DAC�������ǯͷ
	Dac1_Set_Vol(1800);
	
	TIM13_Config_1s_Standby();//�ں�1s�жϣ�����1800��Ϊ30min������Standbyģʽ
}


int main(void)
{
//	u8 t;
//	u8 len;	
//	u16 times=0;
//	float num;
	u16 count_for_Standby=0;
	start_configration();
	
	
	//��ת����λ��Ԥ���
	EXTI->IMR &= ~Keyboard_EXTI_Line;//�����������ϵ��ж����󣬷�ֹ�ڶ�ʱ��ɨ��������ʱ������ж�
	TIM_Cmd(TIM3, ENABLE);//�򿪶�ʱ��3���ڶ�ʱ�������¿�ʼɨ��������
	
//	WriteEeprom();
//	Self_Calibration();//�ڵ��赵�Լ���·�ͷŴ���

	
//	//Һ��ȫ��
//	Send(SetAddr,0x3B,0x00,0x8B);//����LCD��ַ
//	CheckReceive(SetAddr);//�ȴ�60ms��DTA�ӽ��յ���������Ҫʱ��
//	for(t=0;t<8;t++)
//	{
//		Send(WriteFwd,0xFF,0xFF+WriteFwd);
//		CheckReceive(WriteFwd);
//	}
	
	while(1)
	{
		if(timer_1s_flag_for_Standby==1)//�������߼�ʱ
		{
			timer_1s_flag_for_Standby=0;
			count_for_Standby++;//���߼�ʱ+1s;
			if(count_for_Standby==5400)//5min-300,10min-600,15min-900,20min-1200,25min-1500,30min-1800
			{
				lcd_clr();
				StandbyMode_Measure();//��������
//				Power_Off_VDD();
			}
		}
		if((RotaryKeyChanged_flag == 1) || (SoftKeyChanged_flag == 1))//�а���
		{
			count_for_Standby=0;//���߼�ʱ����
			manipulate();
		}
		
		DataProcessing();//������Ӧ���ܵ�����DTA0660,������ǰ��λ�����ݴ�����  
		display();//Һ����ʾ
		
		Communication_Service();//�������ڷ�����
	}
	return 0;
}


/**************************************************end file**************************************************/

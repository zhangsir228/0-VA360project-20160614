/********************************************************************************
	*	@file		matrixkey.c
	*	@author	Jeff
	*	@date		2014/11/1
	*	@brief	矩阵键盘配置。
	*					矩阵键盘有3列4行，配置为：3列——推挽输出，4行——下拉输入。
	*					实现方法：对于stm32而言，没有按键按下时，列输出高电平，行为下拉输入，
	*										当有按键按下，行线检测到上升沿中断触发信号，在外部中断处理函数中
	*										关闭外部中断（因为定时器扫描矩阵键盘时，采用逐列扫描——分别
	*										另每列为高电平检测，会在这期间产生外部中断，所以关掉，当定时器扫描
	*										完成后再打开），并打开定时器中断。定时器每10ms扫描一次，扫描得到
	*										按键的短按释放、长按信号，以供操作。
	*					
	*					使用外设：TIM3(0,0),EXTI9_5_IRQn(2,0)
	*
	*					列线					PF6/7/PB5(从左至右)
	*					行线					PB6/7/8/9(从上至下)
	*					
	*										——|——|——|——
	*										——|——|——|——
	*										——|——|——|——
	*										——|——|——|——
	*					
	*******************************************************************************/
#include "stm32f37x.h"
#include "matrixkey.h"


/* 定义按键处理状态 */
#define        KEY_STATE_INIT           0
#define        KEY_STATE_WOBBLE         1
#define        KEY_STATE_PRESS          2
#define        KEY_STATE_LONG           3
//#define        KEY_STATE_CONTINUE      	4
#define        KEY_STATE_RELEASE        5
    
//#define        KEY_CONTINUE_PERIOD      50        /* 500ms */

boolean RotaryKeyChanged_flag = 0;
boolean SoftKeyChanged_flag = 0;
boolean ShortKey_flag,LongKey_flag;

u8 KeyValue;
u8 RotaryKeyValue,RotaryKeyValue_before=KEY_NULL;
u8 SoftKeyValue,SoftKeyValue_before=KEY_NULL;
u8 oncekey = KEY_NULL;


/*1、2、3列输出低*/
static void KEY_LINE_write_low_all(void)
{
//	GPIO_Write(GPIOF, GPIOF->ODR &= (~0x00C0));
//	GPIO_Write(GPIOB, GPIOB->ODR &= (~0x0020));
	GPIO_ResetBits(Keyboard_Line_Port_1,Keyboard_Line_1);
	GPIO_ResetBits(Keyboard_Line_Port_2,Keyboard_Line_2);
	GPIO_ResetBits(Keyboard_Line_Port_3,Keyboard_Line_3);
}
/*1、2、3列输出高*/
void KEY_LINE_write_high_all(void)
{
//	GPIO_Write(GPIOF, GPIOF->ODR |= 0x00C0);
//	GPIO_Write(GPIOB, GPIOB->ODR |= 0x0020);
	GPIO_SetBits(Keyboard_Line_Port_1,Keyboard_Line_1);
	GPIO_SetBits(Keyboard_Line_Port_2,Keyboard_Line_2);
	GPIO_SetBits(Keyboard_Line_Port_3,Keyboard_Line_3);
}
/*指定(0/1/2)列输出高*/
static void KEY_LINE_write_high(uint8_t i)
{
	KEY_LINE_write_low_all();
	if(i == 0)
		GPIO_SetBits(Keyboard_Line_Port_1 ,Keyboard_Line_1);
	else if(i == 1)
		GPIO_SetBits(Keyboard_Line_Port_2 ,Keyboard_Line_2);
	else if(i == 2)
		GPIO_SetBits(Keyboard_Line_Port_3 ,Keyboard_Line_3);
}
/*读取所有行的值(PB6-PB9)，并输出为0x0F(低四位)格式
*注意：###如果行对应的IO口改变，此处需改变###
*/
static u8 KEY_ROW_read(void)
{
	return ((GPIO_ReadInputData(Keyboard_Row_Port)>>6)&0x0F);
}

/**************************************************************** 
*	函数名称: TIM3_Config 
*	功    能: 定时器3产生10ms更新中断           
*	参    数: 无 
*	返 回 值: 无  
*****************************************************************/
static void TIM3_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  /* Time base configuration */
					/*频率计算：f = 72,000,000/(99+1)/(7199+1) = 100Hz，周期10ms */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(99+1)/100-1;//7199
  TIM_TimeBaseStructure.TIM_Prescaler = 99;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, ENABLE);
}

/**************************************************************** 
*	函数名称: Init_Keyboard_Interrupt 
*	功    能: 键盘初始化为中断扫描模式            
*						初始化键盘需要的IO，Line1-Line3设为输出高           
*						Row1-Row4 接下拉电阻，使能上升沿中断 
*	参    数: 无 
*	返回值  : 无  
*****************************************************************/
void Init_Keyboard_Interrupt(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* Enable GPIOB/F clock */
  RCC_AHBPeriphClockCmd(Keyboard_Line_1_CLK | Keyboard_Line_2_CLK | Keyboard_Line_3_CLK | Keyboard_Row_CLK, ENABLE);
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	TIM3_Config();
	EXTI_DeInit();
	
	//Line1,Line2:列PF6,PF7       
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Keyboard_Line_Port_1 ,&GPIO_InitStructure);
	GPIO_SetBits(Keyboard_Line_Port_1 ,Keyboard_Line_1);
	
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_2;
	GPIO_Init(Keyboard_Line_Port_2 ,&GPIO_InitStructure);	
	GPIO_SetBits(Keyboard_Line_Port_2 ,Keyboard_Line_2);
	
	//Line3:列PB5
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_3;
	GPIO_Init(Keyboard_Line_Port_3 ,&GPIO_InitStructure);	
	GPIO_SetBits(Keyboard_Line_Port_3 ,Keyboard_Line_3);
	
	//Row1-Row4:行PB6-PB9，设置为下拉输入，用来接收上升沿中断
	GPIO_InitStructure.GPIO_Pin = Keyboard_Row;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉输入
	GPIO_Init(Keyboard_Row_Port ,&GPIO_InitStructure);
	
	/* Enable and set EXTI9_5 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI6 Line to PB6 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
	//PB6:上升沿触发
	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	//PB7
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
	EXTI_Init(&EXTI_InitStructure);
	//PB8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);
	//PB9
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
	EXTI_Init(&EXTI_InitStructure);
	
/*********************************************************************************/
	//20160728Lea 新增外部中断PB2 用于在任意档位休眠后能够实现换挡唤醒。
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉输入
	GPIO_Init(GPIOB ,&GPIO_InitStructure);
	
		
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_TS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI2 Line to PB2 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	//PB2:上升沿触发
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
}



/**************************************************************************************************************
* 函数名称: OnceSoftKey
* 功能说明: 进行一次键盘按键值的扫描获取
* 输    入: 无
* 输    出: u8	扫描到的按键值
* 注意事项: 1、编码0x0F(KEY_NULL)表示没有按键被按下
*						2、本函数扫描按键没有进行消抖，下面的ScanKey()函数在本函数的基础上进行了基于状态机机制的机械消抖操作
****************************************************************************************************************/
u8 OnceSoftKey(void)
{
	u8 i = 0;
	u8 keytemp=KEY_NULL;
	
	KEY_LINE_write_high_all();
	if(KEY_ROW_read() == 0)
	{
//		KEY_LINE_write_high_all();
		return KEY_NULL;         		//没有按键按下，直接返回
	}
	
	for(i = 0;i < 3;i++)//按列搜索，从左到右，列：0、1、2.
	{
		KEY_LINE_write_low_all();
		KEY_LINE_write_high(i);
		oncekey = (KEY_ROW_read() & 0x03);//只取数据的后两位
		switch(oncekey)
		{
			case (1 & 0x0F)://PB6按下
				keytemp = i;
				break;
			case (2 & 0x0F)://PB7按下
				keytemp = 3 + i;
				break;
			default:
				keytemp = 0x0F;
				break;
		}
	
		if(keytemp < 0x06)
		{
			return keytemp;
		}
	}
	
	return KEY_NULL;
}

u8 OnceRotaryKey(void)
{
	u8 i = 0;
	u8 keytemp=KEY_NULL;
	
	KEY_LINE_write_high_all();
	if(KEY_ROW_read() == 0)
	{
//		KEY_LINE_write_high_all();
		return KEY_NULL;         		//没有按键按下，直接返回
	}
	
	for(i = 0;i < 3;i++)//按列搜索，从左到右，列：0、1、2.
	{
		KEY_LINE_write_low_all();
		KEY_LINE_write_high(i);
		oncekey = (KEY_ROW_read() & 0x0C);//只取数据的倒数4-3两位
		switch(oncekey)
		{
			case (4 & 0x0F)://PB8按下
				keytemp = 6 + i;
				break;
			case (8 & 0x0F)://PB9按下
			{
				//if(i<2)
					keytemp = 9 + i;
			}
				break;
			default:
				keytemp = 0x0F;break;
		}
	
		if((keytemp <= 0x0B) && (keytemp >= 0x06))
		{
			return keytemp;
		}
	}
	
	return KEY_NULL;
}
/***************************************************************************** 
* 函数名称: ScanKey
* 功能说明: 扫描获取按键值及其状态
* 输    入: 无
* 输    出: u8	按键值及其状态
*						高四位：按键状态
*						低四位：按键值
* 注意事项: 0、可能输出的值：KEY_NULL(0x0F)、1001####（短按释放响应）、0100####（长按及时响应）、0101####（长按释放—没有用）
*															0x0#(无用的返回值)//见355行注释（1月28日）：使用注释后就没有无用的返回值了。
*															也可以不用改，因为在调用ScanKey后，会判断返回值的状态和键值。
*						1、编码：短按——1001####，需要释放按钮识别
*						2、编码：长按——0100####，马上执行
******************************************************************************/
u8 ScanKey(void)
{
	static u8 KeyState = KEY_STATE_INIT;
	static u8 KeyTimeCounter = 0;
	static u8 LastKey = KEY_NULL;
	u8 KeyTemp = KEY_NULL;//每次进入此函数，更新KeyTemp = 0x0F
	
	KeyTemp = OnceSoftKey();//检测按键，KeyTemp的值为：0000####（0x00-0x0B,0x0F）
//	KEY_LINE_write_high_all();
	switch(KeyState)
	{
		case KEY_STATE_INIT://0
		{
			if(KEY_NULL != (KeyTemp))//有按键按下
			{
				KeyState = KEY_STATE_WOBBLE;//进入按键消抖状态
			}
		}
		break;
		
		case KEY_STATE_WOBBLE://1
		{
			KeyState = KEY_STATE_PRESS;//进入按键按下状态
		}
		break;
		
		case KEY_STATE_PRESS://2
		{
			if(KEY_NULL != (KeyTemp))//有按键按下
			{
				KeyTemp |= KEY_DOWN;//保存按键按下标志(0x80)，KeyTemp的8位数为： 1000####
				LastKey = KeyTemp;//保存键值和短按标志，以便在释放状态返回键值
				KeyState = KEY_STATE_LONG;//进入按键长按状态
			}
			else
			{
				KeyState = KEY_STATE_INIT;//如果按键未按下，认为是抖动，返回初始状态
			}
		}
		break;
		
		case KEY_STATE_LONG://3 
		{
			
			if(KEY_NULL != (KeyTemp))//有按键按下
			{
				if(++KeyTimeCounter > KEY_LONG_PERIOD)//按键时间计数大于长按阈值
				{
					KeyTimeCounter = 0;
					KeyTemp |= KEY_LONG;//保存按键长按标志(0x40)，KeyTemp的8位数为：0100####
					LastKey = KeyTemp;//保存键值和长按标志，以便在释放状态返回键值
//					KeyState = KEY_STATE_CONTINUE;//进入按键连发状态（意思是一直长按着，比长按时间长）
					//KeyState = KEY_STATE_RELEASE;//20160830 Lea 长按计数达到后就等待下一次跳到释放处理部分  然后中断中就会执行超时操作
				}
			}
			else
			{
				KeyTimeCounter = 0;
				KeyState = KEY_STATE_RELEASE;//在长按时间到来前，若检测到按键松开，进入按键释放状态（0x10)——判断为短按
			}																//或者在长按之后检测到按键松开
		}
		break;
		
		/*2014-10-25-19:01,在结合万用表的的功能时注释掉一开始编写的含有连发状态的程序*/
//		case KEY_STATE_CONTINUE://4
//		{
//			if(KEY_NULL != (KeyTemp))//有按键按下
//			{
//				if(++KeyTimeCounter > KEY_CONTINUE_PERIOD)
//				{
//					KeyTimeCounter = 0;
//					KeyTemp |= KEY_CONTINUE;//保存按键连发标志(0x20)，KeyTemp的8位数为：0010####
//					LastKey = KeyTemp;//保存键值和连发标志，以便在释放状态返回键值
//				}
//			}
//			else
//			{
//				KeyTimeCounter = 0;
//				KeyState = KEY_STATE_RELEASE;//连发时间到来前（——判断为长按），或者已经有过连发状态（——判断为连发），若检测到按键松开，进入按键释放状态
//			}
//		}
//		break;
		
		case KEY_STATE_RELEASE://5
		{
			LastKey |= KEY_UP;//按键释放标志(0x10)，此标志只有在已经存在短按、长按或连发标志的情况下出现
			KeyTemp = LastKey;//KeyTemp的8位数为：短按释放——1001####，长按释放——0101####，连发释放——0011####
			KeyState = KEY_STATE_INIT;
			return KeyTemp;//短按需要松手才能识别//
		}
		break;
		
		default:break;
	}
	
	if(KeyTemp < 0x80 && ((KeyTemp&0xF0) != 0))//1月28日：（先做个记号）可以修改为if((KeyTemp < 80) && ((KeyTemp&0xF0) != 0))
		return KeyTemp; //长按和连发必须马上执行//长按——0100####，连发——0010####
	else
		return KEY_NULL;
}

/**************************************************end file**************************************************/

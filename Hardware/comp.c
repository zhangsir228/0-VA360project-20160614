/********************************************************************************
	*	@file		comp.c
	*	@author	Jeff
	*	@date		2014/8/21
	*	@brief	V_PHASE、A_PHASE过零比较后，将比较后的方波输出给定时器TIM5、TIM2，进
	*					行输入捕获，得到V_PHASE、A_PHASE的基频。
	*					
	*	
	*					使用外设：TIM5(1,0)
	*
	*					COMP1				PA1				V_PHASE
	*					COMP2				PA3				A_PHASE
	*	修改：20160615  COM1反向输入端改为SGND 做过零检测。用于正常AC测量和相序测量模式下触发起始点的确定
	*									com2同上，用于浪涌电流的检测起始触发功能。
	*******************************************************************************/
	
	
#include "comp.h"

u16 TIM5CH4_CAPTURE_STA;//捕获状态（0x4000-捕获到A相，0x8000-捕获到另一相）+计时器溢出累计次数
u32 TIM5CH4_CAPTURE_VAL;//输入捕获值

uint32_t CCR4_Val = 20011;//10005;//TIM58 周期定时脉冲个数

static void COMP_Config(void)
{
  COMP_InitTypeDef	COMP_InitStructure;
  GPIO_InitTypeDef	GPIO_InitStructure;
  
  /* GPIOA Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PA1/3: PA1/3 are used as COMP1 and COMP2 non inveting input */
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3;//新增comp2作为浪涌电流的触发
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//设置PA6复用方式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_8);    
	
	
  /* COMP Peripheral clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* COMP1 Init: the higher threshold is set to VREFINT ~ 1.22V
     but can be changed to other available possibilities */
  COMP_StructInit(&COMP_InitStructure);
  COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_IO;//COMP_InvertingInput_1_2VREFINT;//COMP_InvertingInput_VREFINT;//COMP_InvertingInput_1_4VREFINT;//COMP_InvertingInput_DAC1OUT1;
	//COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_DAC1OUT1;
  COMP_InitStructure.COMP_Output = COMP_Output_TIM5IC4 ;//| COMP_Output_None;
	COMP_InitStructure.COMP_OutputPol=COMP_OutputPol_NonInverted;//COMP_OutputPol_Inverted;
  COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed;//COMP_Mode_HighSpeed;//
	COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_High;//COMP_Hysteresis_No;//
  COMP_Init(COMP_Selection_COMP1, &COMP_InitStructure);
	
	
	COMP_StructInit(&COMP_InitStructure);
  COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_1_4VREFINT;//COMP_InvertingInput_VREFINT;//COMP_InvertingInput_1_4VREFINT;//COMP_InvertingInput_DAC1OUT1;;
  COMP_InitStructure.COMP_Output = COMP_Output_TIM2IC4 ;//| COMP_Output_None;
	COMP_InitStructure.COMP_OutputPol=COMP_OutputPol_NonInverted;
  COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed ;
	COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_High;
	COMP_Init(COMP_Selection_COMP2, &COMP_InitStructure);

////  /* Disable Window mode */
////  COMP_WindowCmd(DISABLE);

  /* Enable COMP1: the higher threshold is set to VREFINT ~ 1.22 V */
  COMP_Cmd(COMP_Selection_COMP1, ENABLE);
	//COMP_Cmd(COMP_Selection_COMP2, ENABLE);
	
}


void TIM5_2_Compare_Capture_Init(void)
{
	int32_t Fpahse=(SystemCoreClock/(3600*50))-1;


	GPIO_InitTypeDef	GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	
	COMP_Config();	//比较器的配置
  /* TIM clock enable */
  RCC_APB1PeriphClockCmd(/*RCC_APB1Periph_TIM2 | */RCC_APB1Periph_TIM5, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	/* Time base configuration */
					/*计数频率计算：f = 72,000,000/(71+1) = 1MHz*/
	TIM_TimeBaseStructure.TIM_Period =  0xffffffff;//65535;//71//SystemCoreClock/71;//   	
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/(1000000))-1;//Fpahse;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	//TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; 
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
///*****/
//	/*TIM2 PA3测试输出比较*/
//	/* GPIOA Configuration: TIM2 CH4 (PA3) */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;// | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//	GPIO_Init(GPIOA, &GPIO_InitStructure); 
//     
//	/* Connect TIM Channels to AF1 */
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);



//	 /* Output Compare Toggle Mode configuration: Channel1 */
//   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//   TIM_OC4Init(TIM2, &TIM_OCInitStructure);
// 
//   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

//	 /* TIM enable counter */
//   //TIM_Cmd(TIM2, ENABLE);//测试 在捕捉到A相上升沿后打开，输出一个脉冲对比待测的相位差
// 
//   /* TIM IT enable */
////   TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);

//	
///*****/
	
/////*****/
////	/*测试输出比较*/
////	/* GPIOA Configuration: TIM3 CH1 (PA6) and TIM3 CH2 (PA7) */
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;// | GPIO_Pin_7;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
////	GPIO_Init(GPIOA, &GPIO_InitStructure); 
////     
////	/* Connect TIM Channels to AF1 */
////	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);



////	 /* Output Compare Toggle Mode configuration: Channel1 */
////   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
////   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////   TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
////   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
////   TIM_OC4Init(TIM5, &TIM_OCInitStructure);
//// 
////   TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Disable);

////	 /* TIM enable counter */
////   TIM_Cmd(TIM5, ENABLE);
//// 
////   /* TIM IT enable */
////   TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

////	
/////*****/
	/* TIM Channel4 Input capture Mode configuration */
  TIM_ICStructInit(&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);

	/* Enable the CC4 Interrupt Request */
  TIM_ITConfig(TIM5, /*TIM_IT_Update|*/TIM_IT_CC4, ENABLE);//20160603lea不使能tim溢出中断，比较器中断后再读计数器值
//	TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_CC4, ENABLE);
	
  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);//8-20注释，把使能关掉，在main.c中有按键按下才打开
//	TIM_Cmd(TIM2, ENABLE);
	
  /* Reset the flags */
  TIM5->SR = 0;
//	TIM2->SR = 0;
}

/**************************************************end file**************************************************/


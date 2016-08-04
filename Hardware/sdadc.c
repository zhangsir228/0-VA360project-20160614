/********************************************************************************
	*	@file		sdadc.c
	*	@author	Jeff
	*	@date		2014/8/21
	*	@brief	SDADC配置。SDADC1 synchronous with SDADC2 + TIM19 + DMA2_Channel3。
	*					TIM19确定采样频率，SDADC1和SDADC2同步采集，DMA传输数据并触发中断。
	*					SDADC使用差分模式采集，采样时间由TIM19的SAMPLING_FREQ决定，采到的周期
	*					数为1024/(SAMPLING_FREQ/基频)。采样频率10240Hz对应(基频/10)个周期,
	*					5120Hz对应(基频/5)个周期。
	*					采集1024个32位数据，高16位为SDADC2的，低16位为SDADC1的。
	*
	*					使用外设：TIM19，SDADC1/2，DMA(0,1)
	*
	*					SDADC1_AIN8P				PE8				改为(2015_3_18)：	SDADC2_AIN8P				PE8	
	*					SDADC1_AIN8M				PE9													SDADC2_AIN8M				PE9	
	*					SDADC2_AIN4P				PE11												SDADC1_AIN6P				PB0
	*					SDADC2_AIN4M				PE12												SDADC1_AIN6M				PB1
	*******************************************************************************/

#include "sdadc.h"
#include "nvic_systick.h"
#include "stmdataprocess.h"

/* __IO uint32_t JDATA12R;     !< SDADC1 and SDADC2 injected data register,        Address offset: 0x70 */ 
#define SDADC1_DR_Address     ((uint32_t)0x40016070) 
//const u16 length=1024;
const u16 length=1280;//1024+256=1280
u16 DMA2_MEM_LEN;//保存DMA每次数据传送的长度

//s32 InjectedConvData[length]; //换到联合体中 节省RAM

u16 SAMPLING_FREQ=5120;//12800;//SDADC的采样频率Hz，FFT计算时要用到


/**
  * @brief  Configure timer TIM19: It is used as trigger for SDADC conversion
  * @param  None
  * @retval None
  */
void TIM19_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure;
  
  /* Enable TIM19 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM19, ENABLE);
  
	TIM_DeInit(TIM19);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(15+1)/SAMPLING_FREQ-1;//自动装载值72M/100/SAMPLING_FREQ,相当于得到定时器的频率为SAMPLING_FREQ
  TIM_TimeBaseStructure.TIM_Prescaler = 15;															 //预分频CK_CNT=fCK_PSC/(99+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM19, &TIM_TimeBaseStructure);
  
  /* Channel2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
  TIM_OCInitStructure.TIM_Pulse = SystemCoreClock/100/SAMPLING_FREQ/2;	//保证从0向上计数时，在TIM_Period/2处产生一个上升沿,保证在定时器频率下，
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        			//每个周期触发SDADC一次，触发频率为SAMPLING_FREQ
  TIM_OC2Init(TIM19, &TIM_OCInitStructure);
	
  /* TIM19 enable counter */
  //TIM_Cmd(TIM19, ENABLE);
}


/**
  * @brief  Configure SDADC1 and SDADC2
  * @param  None
  * @retval None
  */
u8 SDADC1_Config(void)
{
  SDADC_AINStructTypeDef SDADC_AINStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	uint32_t SDADCTimeout = 0;
	
	/* Enable DMA2 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
     
  /* SDADC APB2 interface clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDADC1 | RCC_APB2Periph_SDADC2, ENABLE);
  
  /* PWR APB1 interface clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* Enable SDADC analog interface */
  PWR_SDADCAnalogCmd(PWR_SDADCAnalog_1, ENABLE);
	PWR_SDADCAnalogCmd(PWR_SDADCAnalog_2, ENABLE);
//	PWR_SDADCAnalogCmd(PWR_SDADCAnalog_3, ENABLE);
  
  /* Set the SDADC divider: The SDADC should run @6MHz */
  /* If Sysclk is 72MHz, SDADC divider should be 12 */
  RCC_SDADCCLKConfig(RCC_SDADCCLK_SYSCLK_Div12);
	
	/* GPIOE/B Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOA, ENABLE);
	
	/**************************************************************///新增 比较器的引脚可能会影响SDADC的测量 这里将其配置为输入
  /* Configure PA1/3: PA1/3 are used as COMP1 and COMP2 non inveting input */
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/****************************************************************/
	
	
  /* SDADC channel pin configuration: PE8,PE9,PB0,PB1*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	DMA_StructInit(&DMA_InitStructure);
	/* Config the DMA2 channel 3 */
  DMA_DeInit(DMA2_Channel3);
  DMA2_MEM_LEN=length;//DMA传输数据长度等于定义的数组长度
  DMA_InitStructure.DMA_PeripheralBaseAddr  = SDADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr      = (uint32_t)RAMsave.K4_tab.InjectedConvData;
  DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize          = DMA2_MEM_LEN;
  DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M                 = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel3, &DMA_InitStructure);
	
	/* Enable DMA2 Channel3 Transfer Complete interrupt */	//SDADC1对应DMA2_Channel3
  DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);
  
  /* Enable DMA2 Channel3 */
  DMA_Cmd(DMA2_Channel3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 
		
	SDADC_DeInit(SDADC1);
	SDADC_DeInit(SDADC2);
  /* Selects the reference voltage: The reference voltage selection is available
     only in SDADC1 and therefore to select the VREF for SDADC2/SDADC3, SDADC1
     clock must be already enabled */
     //SDADC_VREFSelect(SDADC_VREF_VDDA);//SDADC_VREF_Ext
	
	//SDADC_VREFSelect(SDADC_VREF_Ext);//现在使用外部参考  修改需将外部电路断开
	SDADC_VREFSelect(SDADC_VREF_VREFINT2);//SDADC_VREF_Ext
  /* Insert delay equal to ~10 ms (4 ms required) */
	
  delay_ms(50);
  
  /* ENABLE SDADC */
  SDADC_Cmd(SDADC1, ENABLE);
	SDADC_Cmd(SDADC2, ENABLE);
  
  /* Enter initialization mode */
  SDADC_InitModeCmd(SDADC1, ENABLE);
	SDADC_InitModeCmd(SDADC2, ENABLE);
	
	SDADCTimeout = SDADC_INIT_TIMEOUT;
  /* wait for INITRDY flag to be set */
  while((SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_INITRDY) == RESET) && (--SDADCTimeout != 0));
	if(SDADCTimeout == 0)
  {
    /* INITRDY flag can not set */
    return 1;
  }
	
	SDADCTimeout = SDADC_INIT_TIMEOUT;
  /* wait for INITRDY flag to be set */
  while((SDADC_GetFlagStatus(SDADC2, SDADC_FLAG_INITRDY) == RESET) && (--SDADCTimeout != 0));
	if(SDADCTimeout == 0)
  {
    /* INITRDY flag can not set */
    return 1;
  }
	
  SDADC_AINStructure.SDADC_InputMode = SDADC_InputMode_Diff;
  SDADC_AINStructure.SDADC_Gain = SDADC_Gain_1;
  SDADC_AINStructure.SDADC_CommonMode = SDADC_CommonMode_VSSA;
  SDADC_AINStructure.SDADC_Offset = 0;
  SDADC_AINInit(SDADC1, SDADC_Conf_0, &SDADC_AINStructure);
	//SDADC_AINInit(SDADC2, SDADC_Conf_0, &SDADC_AINStructure);
  /*****************///单独修改电流通道增益
	SDADC_AINStructure.SDADC_Gain = SDADC_Gain_2;
	SDADC_AINInit(SDADC2, SDADC_Conf_1, &SDADC_AINStructure);
  /********************/
  /* select SDADC channel to use conf0 */
	SDADC_ChannelConfig(SDADC1, SDADC_Channel_6, SDADC_Conf_0);
	SDADC_ChannelConfig(SDADC2, SDADC_Channel_8, SDADC_Conf_1);
	
	/* select SDADC Channel */
	SDADC_InjectedChannelSelect(SDADC1, SDADC_Channel_6 );
	SDADC_InjectedChannelSelect(SDADC2, SDADC_Channel_8 );
	
  /* Select an external trigger */	/*TIM19_OC2*/
 	SDADC_ExternalTrigInjectedConvConfig(SDADC1, SDADC_ExternalTrigInjecConv_T19_CC2);
	
  /* Select rising edge */
	SDADC_ExternalTrigInjectedConvEdgeConfig(SDADC1, SDADC_ExternalTrigInjecConvEdge_Rising);
	
  
	/*JSYNC: Launch a injected conversion synchronously with SDADC1
	0: Do not launch injected conversion synchronously with SDADC1
	1: Launch an injected conversion in this SDADC at the same moment that an injected conversion is
	launched in SDADC1
	This bit can be modified only when INITRDY=1 (SDADC_ISR) or ADON=0 (SDADC_CR2).*/
	SDADC_InjectedSynchroSDADC1(SDADC2,ENABLE);
	
	/* Enable DMA transfer for injected conversions */
  SDADC_DMAConfig(SDADC1, SDADC_DMATransfer_Injected, ENABLE);
	
	SDADC_FastConversionCmd(SDADC1,ENABLE);
	SDADC_FastConversionCmd(SDADC2,ENABLE);
	
//	/*CALIBCNT[1:0]:This bit can be modified only when INITRDY=1 (SDADC_ISR) or ADON=0 (SDADC_CR2).*/
//	/* configure calibration to be performed on conf0 */
//	SDADC_CalibrationSequenceConfig(SDADC1, SDADC_CalibrationSequence_1);
//	/* configure calibration to be performed on conf0 */
//  SDADC_CalibrationSequenceConfig(SDADC2, SDADC_CalibrationSequence_1);
//	
  /* Exit initialization mode */
  SDADC_InitModeCmd(SDADC1, DISABLE);
	SDADC_InitModeCmd(SDADC2, DISABLE);
	
//	/****************************校准SDADC1*********************/
//  /* start SDADC1 Calibration */
//  SDADC_StartCalibration(SDADC1);
//	/* Set calibration timeout: 5.12 ms at 6 MHz in a single calibration sequence */
//  SDADCTimeout = SDADC_CAL_TIMEOUT;
//  /* wait for SDADC1 Calibration process to end */
//  while(SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_EOCAL) == RESET && (--SDADCTimeout != 0));
//	if(SDADCTimeout == 0)
//  {
//    /* EOCAL flag can not set */
//    return 2;
//  } 
//	
//	/****************************校准SDADC2**********************/	
//  /* start SDADC2 Calibration */
//  SDADC_StartCalibration(SDADC2);
//	/* Set calibration timeout: 5.12 ms at 6 MHz in a single calibration sequence */
//  SDADCTimeout = SDADC_CAL_TIMEOUT;
//  /* wait for SDADC2 Calibration process to end */
//  while(SDADC_GetFlagStatus(SDADC2, SDADC_FLAG_EOCAL) == RESET && (--SDADCTimeout != 0));
//	if(SDADCTimeout == 0)
//  {
//    /* EOCAL flag can not set */
//    return 2;
//  }	
	
	return 0;
}

//用于修改SDADC2 前置放大倍数
//conf0 是不放大  con1 是两倍
void SDADC2_Gain(uint32_t SDADC2_Gain)
{
	TIM_Cmd(TIM19, DISABLE);
	TIM_SetCounter(TIM19,0);
	
	SDADC_InitModeCmd(SDADC2, ENABLE);
	SDADC_FastConversionCmd(SDADC2,DISABLE);
	
	SDADC_InjectedSynchroSDADC1(SDADC2,DISABLE);				
	
	delay_ms(50);
	SDADC_ChannelConfig(SDADC2, SDADC_Channel_8, SDADC2_Gain);//修改增益
	
	SDADC_InjectedSynchroSDADC1(SDADC2,ENABLE);
	SDADC_FastConversionCmd(SDADC2,ENABLE);
	
	DMA_ClearITPendingBit(DMA2_IT_GL3);//清空DMA中断			
	
	SDADC_InitModeCmd(SDADC2, DISABLE);
	
	TIM_Cmd(TIM19, ENABLE);
}

/**************************************************end file**************************************************/

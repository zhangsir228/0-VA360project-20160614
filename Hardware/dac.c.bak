/********************************************************************************
	*	@file		dac.c
	*	@author	Jeff
	*	@date		2014/8/23
	*	@brief	DAC配置
	*					
	*					使用外设：DAC1
	*
	*					输出引脚			PA4-DAC1_OUT1
	*
	*					
	*
	*******************************************************************************/
#include "dac.h"

void DAC_Config(void)
{
  DAC_InitTypeDef    DAC_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Configure PA.04 (DAC1_OUT1) in analog mode -------------------------*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC1, ENABLE);
  
  /* DAC1 channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);
  
  /* Enable DAC1 Channel1 */
  DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);
	DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 0);
}

//设置通道1输出电压
//vol:0~3300,代表0~3.3V  //3.613 
void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.613;//3.3;
	DAC_SetChannel1Data(DAC1, DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}

/************************END*****************************/

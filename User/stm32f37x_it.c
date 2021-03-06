/**
  ******************************************************************************
  * @file    Project/STM32F37x_StdPeriph_Templates/stm32f37x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x_it.h"
#include "main.h"
#include "nvic_systick.h"
#include "dta0660.h"
#include "matrixkey.h"
#include "timer.h"
#include "ht1621.h"
#include "comp.h"
#include "buzzer_led.h"
#include "userVAinteraction.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//蓝牙串口2 相关参数。
uint8_t Rx2Buffer[128]={0};
uint8_t	Rx2Counter=0;
uint8_t	Flag_Rx2End=0;
int8_t  Rx2Reset=0;


//uint32_t IC3ReadValue1 = 0, IC3ReadValue2 = 0;
uint32_t ReadValue = 0,ReadValue_before=0, ReadValue1 = 0,ReadValue_before1=0;
uint32_t CaptureNumber = 0, CaptureNumber1 = 0;
uint32_t Capture = 0, Capture1 = 0;

float Frequency[12] = {0}, Frequency1[12] = {0};
float TIM2Freq = 0, TIM5Freq = 0;

uint8_t coe[12]={1,2,3,4,5,6,7,8,9,10,11,12};
uint8_t coe_sum = 1+2+3+4+5+6+7+8+9+10+11+12;
//uint8_t coe[12]={1,1,1,1,1,1,1,1,1,1,1,1};


uint32_t  capture=0,capstats=0,TIM2_cap1=0,TIM2_cap2=0,TIM2_cap3=0,TIM5_cap1=0,TIM5_cap2=0,TIM5_cap3=0;
extern  uint32_t CCR4_Val;//TIM58 周期定时脉冲个数

extern uint8_t VadENA;

u8 dta_temp=0,cs_temp=0;

u8 rdy;
u8 iii,jjj;

uint32_t SDADC1testread=0;
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/**
  * @brief  This function handles DMA2_Channel3 global interrupt request.
	* @brief	TIM定时SDADC采集，DMA传输中断-得到传输完成标志
  * @param  None
  * @retval None
  */
void DMA2_Channel3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_IT_TC3))
	{
		collect_finished = 1;
		TIM_Cmd(TIM19, DISABLE);//采集到一列数据，关闭定时器，无采样频率触发，待处理完数据后再打开此定时器
		DMA_ClearITPendingBit(DMA2_IT_GL3);
//		DMA_Cmd(DMA2_Channel3, DISABLE);  //关闭DMA2 所指示的通道
	}
}


/**************************************************************** 
*	函数名称: 
*	功    能: 2015-4-28新编中断服务函数。
*						2016-06-08 lea比较其开始后捕获第二次中断为相位起始和结束标志
*						当A相电压大于100V，在开启了比较器、输入捕获之后，捕获到A相经过比较器之后的上升沿，则定时器开始计数。
						关闭比较器，使无比较输出，即无捕获输入。
*						这时，同时需要判断再次检测到的电压要大于100V，再开启捕获功能，记录下再次捕获到的另一相的上升沿时的计数值。
*						供相序判断。
*	参    数: 无 
*	返 回 值: 无  
*****************************************************************/
void TIM5_IRQHandler(void)
{
	if(phasestatus != state0)//相序测量状态下比较用于触发相序的时间间隔测量
	{
		if((TIM5CH4_CAPTURE_STA&0x8000)==0)//还未成功捕获A相与B相orC相之间的时间
		{
//			if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
//			{
//				if(TIM5CH4_CAPTURE_STA&0x4000)//已经捕获到A相的上升沿了
//				{
//					if((TIM5CH4_CAPTURE_STA&0x3FFF)==0x3FFF)//两次捕获A相与B相或C相的时间太长了，0x3F=4.194304s（1MHz计数0xFFFF次，再累加0x3F次）
//					{
//						TIM5CH4_CAPTURE_STA|=0x8000;//标记成功捕获了一次
//						TIM5CH4_CAPTURE_VAL=0xFFFF;
//	//					COMP_Cmd(COMP_Selection_COMP1, DISABLE);//关闭比较器
//					}
//					else TIM5CH4_CAPTURE_STA++;
//				}
//			}
			if(TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)//捕获4发生捕获事件
			{
				if(TIM5CH4_CAPTURE_STA&0x4000)		//已经捕获到一次，再次捕获到一个上升沿 		
				{				
					/**************测试两次读取判断第二相线频率脉冲计数个数*/
					//20160607lea 两相线上的起始脉冲均从第二个开始，因为刚打开比较器时若此时输入脉冲处于正半周则会立即触发输入捕捉中断
					//所以从第二个脉冲开始判定。相序测量功能 相位差的测量基本正确。
					if(capstats==0)
					{						
						capstats=1;
					}
					else if(capstats==1)
					{
						TIM2_cap1 = TIM2->CNT;
						TIM5_cap1 = TIM_GetCapture4(TIM5);
						capstats=2;
					}
					else
					{
						TIM5CH4_CAPTURE_VAL = TIM_GetCapture4(TIM5);
						TIM5_cap2 = TIM5CH4_CAPTURE_VAL;
						TIM_Cmd(TIM5, DISABLE);
						
						TIM2_cap2 = TIM2->CNT;//TIM_GetCapture4(TIM2);
						TIM_Cmd(TIM2, DISABLE);
						TIM5CH4_CAPTURE_STA|=0x8000;		//标记成功捕获到B或C相的上升沿
						COMP_Cmd(COMP_Selection_COMP1, DISABLE);//关闭比较器
		
						TIM_Cmd(TIM14, DISABLE);//0.5s定时关闭
						
						TIM5_cap3=TIM5_cap2-TIM5_cap1;
						//一个周期的计数个数 用于对总计数个数取余后求相位差，确定第二相相序
						CCR4_Val = TIM5_cap3;
						
						TIM2_cap3=TIM2_cap2-TIM2_cap1;
						capstats=0;
						//BUZZER_Open(0);//蜂鸣一声						
					}
					/****************/
				}
				else  								//已经检测到A相电压大于100V，开始比较器和定时器后,第一次捕获上升沿
				{
					if(capstats==0)
					{
						capstats=1;
					}
					else
					{
						TIM5CH4_CAPTURE_STA=0;			//清空，此标志除了用来0x4000,0x8000做A、B/C两次的标记外，还用来定时器溢出计时
						TIM5CH4_CAPTURE_VAL=0;
						TIM_SetCounter(TIM5,0);
						TIM_SetCounter(TIM2,0);
						TIM5CH4_CAPTURE_STA|=0x4000;		//标记捕获到了A相经过比较器之后的上升沿
						
						TIM_Cmd(TIM5, ENABLE);//开启定时器计数
						
						TIM_Cmd(TIM2, ENABLE);//测试 在捕捉到A相上升沿后打开，输出一个脉冲对比待测的相位差
	 
						COMP_Cmd(COMP_Selection_COMP1, DISABLE);//关闭比较器，使无比较输出，即无捕获输入。等待下次检测到100V以上电压，再开启
						BUZZER_Open(0);//蜂鸣一声
						capstats=0;
					}
					
				}
			}
		}
	}
	else//正常模式下，比较器用于过零点触发交流的采样
	{
		if(TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET) 
		{
			/* Clear TIM2 Capture compare interrupt pending bit */
			TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);			
			
			if(VadENA!=1)
			{
				VadENA=1;
				TIM_Cmd(TIM19, ENABLE);
				//printf("*converting!*\r\n");
			}
				
		}

	}
		

	TIM_ClearITPendingBit(TIM5, TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
}
/**
  * @brief  This function handles TIM2 global interrupt request.
	* @brief	COMP2比较，TIM2输入捕获中断-得到基波频率并输出
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{ 
		
//	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
//   {
//     TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
//     capture = TIM_GetCapture4(TIM2);
//     TIM_SetCompare4(TIM2, capture + CCR4_Val);
//   }
}



/*DTA0660串口接收中断*/
void USART1_IRQHandler(void)
{
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE   错误
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  

		dta_temp = USART_ReceiveData(USART1); //取出来扔掉
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE     : Noise Error interrupt
		USART_ClearFlag(USART1, USART_FLAG_NE);
	}


	if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
	{//同   @arg USART_IT_FE     : Framing Error interrupt
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE     : Parity Error interrupt
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
	  /* Read one byte from the receive data register */
		dta_temp= USART_ReceiveData(USART1)&0xFF;
    RxBuffer[RxCounter++] = dta_temp;

		if(dta_temp==dta_cs)//判断一下校验和CS：收到的数dta_temp与前面的和相等？是则再检查一下RDY电压，为低则返回dta_receive_flag= 1；为高则继续等待校验和到来。
		{
			if(Read_RDY() == RESET)//RDY引脚电压判断：DTA0660接收到有效的命令后会将RDY信号置为高电平（无效命令
			{							//或校验出错则维持原电平状态），待返回数据发送完毕或状态准备好之后会将RDY
				dta_receive_flag= 1;	//信号重新置为低电平。
				dta_cs=0;
				RxCounter=0;
			}
			else
				dta_cs+= dta_temp;
		}
		else
			dta_cs+= dta_temp;//只要收到数据，就计算当前值与之前所有数的和。
	}
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}
//蓝牙串口中断，可用于调试
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
  {		
    /* Read one byte from the receive data register */
    Rx2Buffer[Rx2Counter++] = USART_ReceiveData(USART2);
		Rx2Reset=200;
		if((Rx2Buffer[Rx2Counter-1]==0x0A)&&(Rx2Buffer[Rx2Counter-2]==0x0D))//一帧数据
		{
			Flag_Rx2End=1;
		}

  }
	  USART_ClearFlag(USART2,USART_FLAG_RXNE);  
}


/*矩阵键盘按键中断*/
void EXTI9_5_IRQHandler(void)	//有按键中断，在EXTI中断中打开定时器中断，每个十秒扫描按键
{ 
  EXTI_ClearITPendingBit(Keyboard_EXTI_Row1);//清除外部中断标志
	EXTI_ClearITPendingBit(Keyboard_EXTI_Row2);
	EXTI_ClearITPendingBit(Keyboard_EXTI_Row3);
	EXTI_ClearITPendingBit(Keyboard_EXTI_Row4);
	
	EXTI->IMR &= ~Keyboard_EXTI_Line;//屏蔽来自线上的中断请求，防止在定时器扫描矩阵键盘时进入此中断
	
	TIM_Cmd(TIM3, ENABLE);//打开定时器3，在定时器作用下开始扫描矩阵键盘
}

/*矩阵键盘10ms扫描中断*/
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		/*下面是定时器中断到来该做的程序*/
		
		//每次按键扫描都先扫一次（真的一次Once）旋转开关，判断是否与之前的旋转开关值相同，
		//若不同，则等待5个TIM3定时时间，再次扫一次旋转开关，
		if(iii == 0)
			KeyValue = OnceRotaryKey()&0x0F;
		/*旋转开关处理程序*/
		if(KeyValue != RotaryKeyValue_before)//有扫到旋转开关按键值与之前不同，应是真有开关变化
		{
			iii++;
			if(iii > 5)
			{
				u8 temp=KEY_NULL;
				temp = OnceRotaryKey()&0x0F;
				if(temp == KeyValue)
				{
					if(KeyValue >= KEY_VALUE_6)//0x06->KEY_VALUE_6,KeyValue有0x46,0x47,0x48,0x49,0x4A
					{
						RotaryKeyChanged_flag = 1;
						RotaryKeyValue = (KeyValue&0x0F);//只保留KeyValue中的按键值6-10
						RotaryKeyValue_before = RotaryKeyValue;//RotaryKeyValue_before只在前面if(KeyValue != RotaryKeyValue_before)比较用到。
					}
				}
				iii=0;
				KEY_LINE_write_high_all();
				KeyValue = KEY_NULL;//按键值、状态清零
				TIM_Cmd(TIM3, DISABLE);//同时关闭定时器
				EXTI->IMR |= Keyboard_EXTI_Line;//同时开放来自线上的中断请求
			}
		}
		/*软按钮处理程序，及开关抖动处理程序*/
		else//若扫到旋转开关按键值与之前相同，可能是旋转开关抖动或有软按键，则扫软按键，若没有软按键，也将TIM3关闭，打开外部中断
		{
			jjj++;
			KeyValue = ScanKey();
			/*开关抖动处理程序*/
			if(((KeyValue&0x0F) == KEY_NULL) && jjj > 100+6)//表示无软按键,100表示长按检测次数  （长按键检测阀值为100 所以这里的超时应该大于100）
			{
				jjj=0;
				KEY_LINE_write_high_all();
				KeyValue = KEY_NULL;//按键值、状态清零
				TIM_Cmd(TIM3, DISABLE);//同时关闭定时器
				EXTI->IMR |= Keyboard_EXTI_Line;//同时开放来自线上的中断请求 6789
			}
			/*软按钮处理程序*/
			if((KeyValue&0xF0) != 0x00 && (KeyValue&0x0F) < KEY_VALUE_6)//有按键也有状态（短按、长按），其中0x0F->KEY_NULL
			{
				jjj=0;
				
				SoftKeyChanged_flag = 1;
				SoftKeyValue_before = SoftKeyValue;
				SoftKeyValue = (KeyValue&0x0F);//只保留KeyValue中的按键值0-5
				
				if((KeyValue&0xF0)==0x90)//0x90=0b10010000
					ShortKey_flag = 1;
				else if((KeyValue&0xF0)==KEY_LONG)
					LongKey_flag = 1;
				
				KEY_LINE_write_high_all();
				KeyValue = KEY_NULL;//按键值、状态清零
				TIM_Cmd(TIM3, DISABLE);//同时关闭定时器
				EXTI->IMR |= Keyboard_EXTI_Line;//同时开放来自线上的中断请求
			}
		}
  }
}


/*1s定时*/
void TIM12_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
		powertimer++;
		powertimer_1s_flag=1;
	}
}

/*0.5s定时*/
void TIM14_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
		powertimer_0_5s_flag=1;
	}
}

void TIM13_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM13, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
		timer_1s_flag_for_Standby=1;
	}
}


void TIM4_IRQHandler(void)
{
	static u16 buzzer_counter;
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		Rx2Reset--;if(Rx2Reset<=0){Rx2Reset=0;Rx2Counter=0;}//用于无通信时重置接收计数器
		if(longshort==0)
		{
			if(buzzer_counter<500)
			{
				BUZZER_Toggle();
				buzzer_counter++;
			}
			else
			{
				BUZZER_Close();
				buzzer_counter=0;
			}
		}
		else if(longshort==1)
		{
			BUZZER_Toggle();
		}
	}
}

/**
  * @brief  This function handles SDADC1 interrupt request.
  * @param  None
  * @retval : None
  */
void SDADC1_IRQHandler(void)
{
  uint32_t ChannelIndex;

  if(SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_JEOC) != RESET)
  {
    /* Get the converted value */
    SDADC1testread = SDADC_GetInjectedConversionValue(SDADC1, &ChannelIndex);
  }
}
/******************************************************************************/
/*                 STM32F37x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f37x.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/********************************************************************************
	*	@file		uart.c
	*	@author	Jeff
	*	@date		2015/3/27
	*	@brief	重定向pritf函数，支持串口发送。
	*					UART配置，定义接收中断、接收协议。
	*					数据协议为：接收到的最后两个数据为0x0d(\r -> 回车),0x0a(\n -> 换行)。
	*	
	*					使用外设：USART2(0,3)
	*
	*					USART2_TX				PB3
	*					USART2_RX				PB4
	*******************************************************************************/


#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
    USART_SendData(USART2,(uint8_t)ch);   
	return ch;
}
#endif 


#if EN_USART2_RX   //如果使能了接收
//串口2中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
//初始化IO 串口1 
//baudrate:波特率
void uart_init(u32 baudrate)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE );
					
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_7);        
	/*
	*  USART2_TX -> PB3 , USART2_RX -> PB4
	*/                                
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);        
	
	USART_DeInit(USART2);  //复位串口2
	
	USART_InitStructure.USART_BaudRate = baudrate;//设置串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//设置数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//设置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//设置效验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//设置流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//设置工作模式
	USART_Init(USART2, &USART_InitStructure); //配置入结构体

#if EN_USART2_RX		  //如果使能了接收  
  //Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接收中断
#endif
	USART_Cmd(USART2, ENABLE);//使能串口1
}



//void USART2_IRQHandler(void)                	//串口1接收中断服务程序
//{
//	u8 Res;
//	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
//		OSIntEnter();    
//	#endif
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		Res =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据

//		if((USART_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(Res!=0x0a)USART_RX_STA=0;//上一个数据是0x0d,判断这帧数据。不是0x0a,则接收错误,重新开始
//				else USART_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))
//						USART_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}   		 
//	} 
//	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
//		OSIntExit();  											 
//	#endif
//} 
#endif	

/**************************************************end file**************************************************/

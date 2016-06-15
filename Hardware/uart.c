/********************************************************************************
	*	@file		uart.c
	*	@author	Jeff
	*	@date		2015/3/27
	*	@brief	�ض���pritf������֧�ִ��ڷ��͡�
	*					UART���ã���������жϡ�����Э�顣
	*					����Э��Ϊ�����յ��������������Ϊ0x0d(\r -> �س�),0x0a(\n -> ����)��
	*	
	*					ʹ�����裺USART2(0,3)
	*
	*					USART2_TX				PB3
	*					USART2_RX				PB4
	*******************************************************************************/


#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
    USART_SendData(USART2,(uint8_t)ch);   
	return ch;
}
#endif 


#if EN_USART2_RX   //���ʹ���˽���
//����2�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
//��ʼ��IO ����1 
//baudrate:������
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
	
	USART_DeInit(USART2);  //��λ����2
	
	USART_InitStructure.USART_BaudRate = baudrate;//���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����Ч��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//���ù���ģʽ
	USART_Init(USART2, &USART_InitStructure); //������ṹ��

#if EN_USART2_RX		  //���ʹ���˽���  
  //Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//���������ж�
#endif
	USART_Cmd(USART2, ENABLE);//ʹ�ܴ���1
}



//void USART2_IRQHandler(void)                	//����1�����жϷ������
//{
//	u8 Res;
//	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
//		OSIntEnter();    
//	#endif
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{
//		Res =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������

//		if((USART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//			{
//				if(Res!=0x0a)USART_RX_STA=0;//��һ��������0x0d,�ж���֡���ݡ�����0x0a,����մ���,���¿�ʼ
//				else USART_RX_STA|=0x8000;	//��������� 
//			}
//			else //��û�յ�0X0D
//			{	
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))
//						USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//				}		 
//			}
//		}   		 
//	} 
//	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
//		OSIntExit();  											 
//	#endif
//} 
#endif	

/**************************************************end file**************************************************/

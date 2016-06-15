#ifndef __UART_H
#define	__UART_H

#include "stm32f37x.h"
#include <stdio.h>

//*******************Jeff��д��������*********************//
#define USART_REC_LEN				200  				//�����������ֽ��� 200
#define EN_USART2_RX				1						//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u16 USART_RX_STA;         				//����״̬���
void uart_init(u32 baudrate);
#endif /* __UART_H */

/**************************************************end file**************************************************/

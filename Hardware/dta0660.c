/********************************************************************************
	*	@file		dta0660.c
	*	@author	Jeff
	*	@date		2014/10/5
	*	@brief	DTA0660���á�	
	*					
	*					ʹ�����裺USART1(0,2)
	*
	*					CS				PA11
	*					RDY				PA12
	*					RX				USART1_TX				PA9
	*					TX				USART1_RX				PA10
	*******************************************************************************/


#include "dta0660.h"
#include "stdarg.h"
#include "nvic_systick.h"
#include <math.h>
#include "uart.h"
#include "ht1621.h"
#include "nvic_systick.h"
#include "buzzer_led.h"

const u8 eeprom_init[]={
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x52,0x00,0xFA,0x00,0x00,0xBE,0x03,
0x70,0x17,0x38,0x18,0x44,0x02,0x6E,0x4B,0x64,0x3C,0x3C,0x3C,0x0A,0x40,0xFF,0xFF,
0x99,0x99,0x00,0x80,0x64,0x00,0x96,0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,
0x4E,0x02,0x09,0x4E,0x02,0x09,0x77,0xFD,0x0A,0x9A,0x19,0x0A,0x00,0x00,0x0A,0x00,
0x00,0x01,0x00,0x01,0x00,0x07,0x98,0x00,0x64,0x00,0x64,0x00,0x64,0x00,0x00,0x00,
0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,
0x00,0x80,0x00,0x83,0x01,0x00,0x6D,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x80,0x00,0x80,0x00,0x80,0xE0,0x7C,0x18,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0xFC,0x1C,0x01,0x98,0x18,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x0D,0x00,0x02,0x10,0x0D,0x00,0x03,0x20,0x20,0x00,0x03,0x20,0x20,0x00,0x03,0x10,
0x41,0x00,0x03,0x08,0x41,0x00,0x03,0x05,0x41,0x00,0x03,0x05,0x0D,0x00,0x02,0x20,
0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x5A,0xC6,0xCE,0x0F,0x0F,0x00,0x00,0x00
};


uint8_t RxCounter = 0;

uint8_t RxBuffer[RXBUFFERSIZE]={0};

//u16 i=0,j=0;

u8 dta_cs=0;

boolean dta_receive_flag,check_flag;

//u8 eeprom[256]={0};

u32 receive_data=0;
float receive_f=0,receive_f1=0;
u8 funcnum=0;
//u8 rangenum=0;

/****************************************************************************
* ��    �ƣ�DTA_Pins_Config(void)
* ��    �ܣ�DTA0660ͨ�ſ�������CS��RDY��ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵    ������DTA�ϵ��CSΪ�ߵ�ƽ���ɴ���״̬����ʱ��CS��Ϊ�͵�ƽ�����к���ͨ��
												�����ɴ˽�CS����Ϊ�������
												RDYΪ�͵�ƽ��ͨ��ʱ��DTA���յ���Ч�����RDY��Ϊ�ߵ�ƽ�����������������Ϊ�͵�ƽ
												�����ɴ˽�RDY����Ϊ��������
* ���÷�����DTA_Pins_Config()
****************************************************************************/
static void DTA_Pins_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(DTA_CLK_CS | DTA_CLK_RDY,ENABLE);
	
	//CS
	GPIO_InitStructure.GPIO_Pin = DTA_Pin_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
  GPIO_Init(DTA_PORT_CS, &GPIO_InitStructure);
	
	GPIO_SetBits(DTA_PORT_CS,DTA_Pin_CS);
	
	//RDY
	GPIO_InitStructure.GPIO_Pin = DTA_Pin_RDY;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(DTA_PORT_RDY, &GPIO_InitStructure);
}

/****************************************************************************
* ��    �ƣ�UART1_init(void)
* ��    �ܣ�DTA0660ͨ�Ŵ��ڳ�ʼ��
* ��ڲ�����baudarate      ������
* ���ڲ�������
* ˵    ����
* ���÷�����UART1_init()
****************************************************************************/
static void UART1_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_7);        
	/*
	*  USART1_TX -> PA9 , USART1_RX -> PA10
	*/                                
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_DeInit(USART1);  //��λ����1
	
	USART_InitStructure.USART_BaudRate = 2400;									//���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//��������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;							//����Ч��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//���ù���ģʽ
	USART_Init(USART1, &USART_InitStructure); 											//������ṹ��

  //Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00 ;			//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;						//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);																	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
   
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
	
	USART_ITConfig(USART1, USART_IT_PE, ENABLE);    //����PE��������ж�Bit 8PEIE: PE interrupt enable
  //CR2 ����ERR�ж�
  USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	USART_Cmd(USART1, ENABLE);//ʹ�ܴ���1
}

/****************************************************************************
* ��    �ƣ�DTA0660_Init(void)
* ��    �ܣ�DTA0660��ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����DTA0660_Init()
****************************************************************************/
void DTA0660_Init(void)
{
	u16 len;	
	u16 times=0;
	DTA_Pins_Config();
	UART1_init();
	
	ClrCS//��CS��Ϊ�͵�ƽ
	while(Read_RDY() != RESET);//�ȴ�RDYΪ�͵�ƽ
	delay_ms(1000);/*delay_ms(1000);*///2015-3-29���ȴ�DTA���ϵ��ʼ�����ݷ�������������ɺ������ݽ��ջ���
}

/****************************************************************************
* ��    �ƣ�Check_Calibration(void)
* ��    �ܣ�DTA0660�ļ���У׼
* ��ڲ�������
* ���ڲ�������
* ˵    ����һ����У��ʱʹ��
* ���÷�����Check_Calibration()
****************************************************************************/
void Check_Calibration(void)
{
	if(dta_receive_flag==1)//��STM32û����DTA0660����״̬�£����յ�������CSУ׼����RetOK��RetErr�����
	{
		dta_receive_flag=0;
		
		if(RxBuffer[0]==RetOK)
		{
			if(RxBuffer[1]==0x01)
			{
				printf("�ϵ��ʼ�����\r\n");
				lcd_show_num(1111,1,4);
				delay_ms(1000);
			}
			else if(RxBuffer[1]==0x02)	printf("�Լ�1���\r\n");
			else if(RxBuffer[1]==0x03)	printf("�Լ�2���\r\n");
			else	printf("����RetOK");
		}
		else if(RxBuffer[0]==RetErr)
		{
			if(RxBuffer[1]==0x01)	printf("EEPROM����\r\n");
			else if(RxBuffer[1]==0x02)	printf("EEPROMδ��ʼ��\r\n");
			else if(RxBuffer[1]==0x03)	printf("EEPROMд�����\r\n");
			else if(RxBuffer[1]==0x04)	printf("У�����ݳ�������Χ\r\n");
			else printf("����RetErr");
		}
		printf("\r\n");
	}
}

/****************************************************************************
* ��    �ƣ�Send(u8 data,...)
* ��    �ܣ����ƶ���DTA0660���Ϳ�������
* ��ڲ�����data			���͵����ݣ���������Ϊ���ֽڣ�
* ���ڲ�������
* ˵    �������з�������ͷ��ز�����ĩβһ���ֽ�����Ϊǰ�����������ݵ��ۼӺ͡�
						���趨��ַ����Ϊ��50H+21H+02H+73H(CS) ��50HΪ�����֣�21H+02HΪ��ַ��73HΪ50H+21H+02H���ۼӺͣ�
						��/д�ĵ�ַΪ2�ֽ�AddrL+AddrH�����ֽ���ǰ���ֽ��ں󣬵�ַ����Χ��
						000H~17FH��IC�ڲ�RAM,����03BH~042H��ӦLCD BUFFER����200H~2FFH����ӦEEPROM��ַ000H~0FFH����
* ���÷�����Send(SetAddr,0x3B,0x00,0x8B);//����LCD��ַ
****************************************************************************/
void Send(u8 data,...)
{
	u8 cs=0;
	va_list arg_ptr;
	u8 nArgValue = data;
	va_start(arg_ptr,data);//�Թ̶������ĵ�ַΪ���ȷ����ε��ڴ���ʼ��ַ��
	
	ClrCS
	
	do
	{
		cs+= nArgValue;//�����ֹ����ǰ����ֵ���з���ֵ�ĺ�
		
		USART_SendData(USART1,nArgValue);//�����������ֵ
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
		nArgValue= va_arg(arg_ptr,u8); 		//�õ���һ���ɱ������ֵ
		
	}while(nArgValue !=  cs);//ֹͣ��������ֹ����ǰ����ֵ���з���ֵ�ĺ�  == ��һ���ɱ������ֵ
	
	USART_SendData(USART1,nArgValue);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	
	va_end(arg_ptr);
	
	RxCounter=0;//����������󣬼��̽�RxCounter���㣬ʹRxBuffer�����ͷд���������
}

/****************************************************************************
* ��    �ƣ�CheckReceive
* ��    �ܣ��ȴ�DTA0660�������ݣ�У�����ֽں�ĩ�ֽ�
* ��ڲ�����- command�����ݷ��͵Ŀ�������ȷ��
* ���ڲ�����- 1��У��ɹ���־
						- 0��У��ʧ�ܱ�־
* ˵    ����
* ���÷�����check_flag= CheckReceive(SetRangeAuto)��У��DTA0660�趨�Զ����ɹ����
****************************************************************************/
boolean CheckReceive(u8 command)
{
	u8 j;
	u16 i;
	for(j=0;j<30;j++)//ʹ������for�������ʱ���ڼ�����ݽ�����ɱ�־�����յ����򷵻سɹ���1�������򷵻�ʧ�ܣ�0����
	{
		for(i=0;i<10000;i++)
		{
			if(dta_receive_flag==1)//���յ�������CSУ׼����
			{
//				printf("\r\nj=%d,i=%d",j,i);
				dta_receive_flag=0;
				if(RxBuffer[0]==command)	
					return 1;//���յ����������ֽ�Ϊ���������ַ�����
			}
		}
	}
	//2015-3-29�����У������ʧ�ܣ����ڴ˵ȴ����Ա�֤DTA����������ϡ�
	//���������DTA�ڷ���δ������ݣ��ڻ���´�Ҫ������ݣ��������޷�У�顣
	while(Read_RDY() != RESET);//�ȴ�RDYΪ�͵�ƽ
	dta_receive_flag=0;
	RxCounter=0;
	dta_cs=0;//2015-3-29��֮ǰһֱû�а����������룬���¼�ʹ���·������ݣ���������ԭ�����ۼӺ��ڼ��㣬��dta_cs����Ҫ���յ�ÿ����䣬������ȷ���Ҫ����ġ�
	return 0;
}

/****************************************************************************
* ��    �ƣ�ReadDTAValue
* ��    �ܣ���ȡDTA0660�Ĳ�������
* ��ڲ�����- command��Ҫ��ȡ��������
* ���ڲ�����- DTA0660�Ĳ�������
* ˵    ������ѡ��ڲ�����ReadADC��ReadRMS_DC��ReadResult��ReadFreq��ReadDuty��ReadRange��ReadStatus
* ���÷�����receive_f= ReadDTAValue(ReadResult);
****************************************************************************/
float ReadDTAValue(u8 command)
{
	float temp=0;
	Send(command,command);//�趨��Ҫ��ȡ������
	check_flag= CheckReceive(command);
	if(check_flag==1)
	{
		check_flag= 0;
		//��ȡADCֵ: 05EH+AdcL+AdcM+AdcH+CS
		if(command == ReadADC)																										
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16;
			temp=	receive_data;
		}
		//��ȡ5bytesRMS��3bytes DCֵ: 05FH+Rms0~Rms4+Dc0~2+CS
		else if(command == ReadRMS_DC)																						
		{
			
		}
		//��ȡ�����Ľ����060H + DataL + DataM + DataH + RangeNum + CS
		else if(command == ReadResult)																						
		{
			float positive_negative= 1.0f;
			receive_data = (RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16) & 0xFFFFFF;
			
			if((receive_data>>23)==0x01)//�ж������Ƿ�Ϊ���������ǣ�������ű�־λ-1������þ���ֵ
			{
				positive_negative= -1.0;
				receive_data=(~receive_data+1) & 0xFFFFFF;
			}
			
			if(funcnum==DCmV)//δ��֤��10��6�����
			{
				temp= (float)receive_data/ 4922.0f* 5* positive_negative;
				if(RxBuffer[4]==0);
				else if(RxBuffer[4]==1)
					temp= temp* 10;
				else if(RxBuffer[4]==2)
					temp= temp* 100;
				else if(RxBuffer[4]==3)
					temp= temp* 1000;
			}
			else if(funcnum==ACmV)//δ��֤��10��6�����
			{
				temp= (float)receive_data/ 4922.0f* 5* positive_negative;
				if(RxBuffer[4]==0);
				else if(RxBuffer[4]==1)
					temp= temp* 10;
				else if(RxBuffer[4]==2)
					temp= temp* 100;
				else if(RxBuffer[4]==3)
					temp= temp* 1000;
			}
			else if(funcnum==DCV)//����֤
			{//2015-8-30��ɾ�������
				temp= (float)receive_data* positive_negative;
			}
			else if(funcnum==ACV)//����֤
			{//2015-7-7��ɾ�������
				temp= (float)receive_data* positive_negative;
			}
			
			else if(funcnum==DCVmV)//����֤
			{//2015-7-7��ɾ�������
				temp= (float)receive_data* positive_negative;
			}
			else if(funcnum==ACVmV)//����֤
			{
				temp= (float)receive_data* positive_negative;/// 1501.0f* 1.5f
				if(RxBuffer[4]==0);
				else if(RxBuffer[4]==1);
				else if(RxBuffer[4]==2)
					temp= temp* 10;
				else if(RxBuffer[4]==3)
					temp= temp* 100;
				else if(RxBuffer[4]==4)
					temp= temp* 1000;
			}
			else if(funcnum==Ohm)//����֤10-4��������ݵ�λ��ŷķ
			{
				temp= receive_data;
//				if(RxBuffer[4]==0)
//					temp= temp/ 10;
//				else if(RxBuffer[4]==1);
//				else if(RxBuffer[4]==2)
//					temp= temp* 10;
//				else if(RxBuffer[4]==3)
//					temp= temp* 100;
//				else if(RxBuffer[4]==4)
//					temp= temp* 1000;
//				else if(RxBuffer[4]==5)
//					temp= temp* 10000;
			}
			else if(funcnum==Cont)//����֤10-4��������ݵ�λ��ŷķ
			{
				temp= receive_data;
				if(RxBuffer[4]==0)
					temp= temp/ 10;
			}
			else if(funcnum==Diode)//����֤10-4��������ݵ�λ��mV
			{//2015-3-29:�޸�Ϊ�����λV
				temp= receive_data/1000.0f;
			}
			else if(funcnum==Cap)//����֤10-4��������ݵ�λ��nF(3λС��)/uF(3λС��)/uF(2λС��)/mF(3λС��)
			{//�޸�2015-3-28���������λͳһΪnF
				temp= receive_data;
				if(RxBuffer[4]==0)//nF
					temp= temp/ 1000;
				else if(RxBuffer[4]==1)//uF
					temp= temp;
				else if(RxBuffer[4]==2)//uF
					temp= temp*10;
				else if(RxBuffer[4]==3)//mF
					temp= temp* 1000;
			}
			else if(funcnum==Temp)//����֤10-4��������ݵ�λ�����϶�
			{
				temp= receive_data;
			}
			
		}
		//��ȡƵ��ֵ��061H+Freq0~Freq4+CS����λΪ0.001HZ
		else if(command == ReadFreq)
		{
			receive_data = (u32)RxBuffer[1] | ((u32)RxBuffer[2])<<8 | ((u32)RxBuffer[3])<<16 | ((u32)RxBuffer[4])<<24;
			temp= (float)receive_data/ 1000.f+ (float)RxBuffer[5]/ 1000.f* pow(2,32);
		}
		//��ȡռ�ձ�ֵ��062H+DutyL+DutyH+CS����λΪ0.01%
		else if(command == ReadDuty)
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8;
			temp= (float)receive_data/ 100;
		}
		//��ȡ��λֵ��063H+RangeNum+CS
		else if(command == ReadRange)																							
			temp= RxBuffer[1];
		//��ȡϵͳ״̬��067H+Status0~5+CS
		else if(command == ReadStatus)																						
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16;
			temp= 0;
		}
	}
	else
	{
		temp= 0;
//		printf("\r\n*��Ч�����У�����*\r\n");
		lcd_show_Erro(1);//������DTAͨ�ų�����Һ������ʾ�����ȴ�1s.
		delay_ms(1000);
	}	
	return temp;
}

/****************************************************************************
* ��    �ƣ�WriteEeprom
* ��    �ܣ���д����EEPROM
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����WriteEeprom();
****************************************************************************/
void WriteEeprom(void)
{
	u16 i=0;//20160312���޸ġ�����LDC��ʾ������FDH.Bit7=0ʱ������֮ǰһֱ��u8��
	
	printf("*************Write to eeprom*************\r\n");
	
	Send(SetAddr,0x00,0x02,0x52);
	check_flag= CheckReceive(SetAddr);
	if(check_flag==1)
	{
		check_flag=0;
		for(i=0;i<=255;i++)
		{
			Send(WriteFwd,eeprom_init[i],eeprom_init[i]+WriteFwd);
			check_flag= CheckReceive(WriteFwd);
			if(check_flag==1)
			{
				printf("%x-%x	",0x200+i,eeprom_init[i]);
			}
			if((i&0x0F)==0x0F)
				printf("\r\n");
		}
	}
	else
	{
		printf("*δ�ҵ�EEPROM��У�����*\r\n");
	}
}

/****************************************************************************
* ��    �ƣ�ReadEeprom
* ��    �ܣ���ȡ����EEPROM
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����ReadEeprom();
****************************************************************************/
void ReadEeprom(void)
{
	u16 i=0;
	//printf("*************Read from eeprom*************\r\n");
	
	Send(SetAddr,0x00,0x02,0x52);
	check_flag= CheckReceive(SetAddr);
	if(check_flag==1)
	{
		check_flag=0;
		for(i=0;i<=255;i++)
		{
			Send(ReadFwd,ReadFwd);
			check_flag= CheckReceive(ReadFwd);
			if(check_flag==1)
			{
//				eeprom[i]=RxBuffer[1];														//����10��5�գ�eeprom[]�������ò�ƿ��Բ�Ҫ��ֱ���ϱ�������//2015��3��18��ע��
				printf("%02x-%02x	|",0x200+i,RxBuffer[1]);//eeprom[i]);
			}
			else
			{
				dta_cs=0;
				dta_receive_flag=0;
				printf("%x-ER	",0x200+i);
			}
			if((i&0x0F)==0x0F)
				printf("\r\n");
		}
	}
	else
		printf("������--��λEEPROM����----ERROR! \r\n");		
}

/****************************************************************************
* ��    �ƣ�On_CalMode
* ��    �ܣ�����У��
* ��ڲ�������
* ���ڲ�����- 1:�ɹ�
						- 0:ʧ��
* ˵    ����
* ���÷�����On_CalMode();
****************************************************************************/
static u8 On_CalMode(void)
{
	Send(SetCalMode,0x55,SetCalMode+0x55);
	check_flag= CheckReceive(SetCalMode);
	if(check_flag==1)
	{
		check_flag=0;
		printf("\r\n����У��:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
		printf("\r\n����У��ģʽ\r\n");
	}
	return 1;
}

/****************************************************************************
* ��    �ƣ�On_CalMode
* ��    �ܣ��ر�У��
* ��ڲ�������
* ���ڲ�����- 1:�ɹ�
						- 0:ʧ��
* ˵    ����
* ���÷�����On_CalMode();
****************************************************************************/
static u8 Off_CalMode(void)
{
	Send(SetCalMode,0x00,SetCalMode);
	check_flag= CheckReceive(SetCalMode);
	if(check_flag==1)
	{
		check_flag=0;
		printf("\r\n�ر�У��:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
		printf("\r\n�ر�У��ģʽ\r\n");
	}
	return 1;
}

/****************************************************************************
* ��    �ƣ�Self_Calibration
* ��    �ܣ�������У�飬�Լ�1-��·���Լ�2-�Ŵ���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����Self_Calibration();
****************************************************************************/
void Self_Calibration(void)
{
//		printf("------------------�ϵ��ʼ���ȴ���������-----------------------\r\n");
//		dta_receive_flag=0;
//		while(dta_receive_flag==0)
//		{
//			printf("1���������ϵ��ʼ���У��ȴ����ؽ������������\r\n");
//			delay_ms(1000);
//		}
//		Check_Calibration();//��ʼ��֮��δ��������֮ǰ���鿴DTA0660���ص�״̬
		
		printf("------------------����У��ģʽ----------------------\r\n");		
		printf("------------------��ʼ�Լ�1-----------------------\r\n");
		BUZZER_Open(0);
		On_CalMode();//����У��ģʽ
		//�Լ�1
		Send(CalSelfTest1,CalSelfTest1);
		while(dta_receive_flag==0)
		{
			printf("�Լ�1����1�����ѷ����Լ�1����ȴ��ظ��С�����\r\n");
		}
		check_flag= CheckReceive(CalSelfTest1);	
		if(check_flag==1)
		{
			check_flag=0;
			printf("�Լ�1����2�����Լ�1��·����(���͵�)�������\r\n");
			printf("%x + %x  \r\n",RxBuffer[0],RxBuffer[1]);
			BUZZER_Open(0);
		}
		else
			printf("�Լ�1����2�����Լ�1��·����(���͵�)����ʧ��\r\n");
		while(dta_receive_flag==0)
		{
			printf("�Լ�1����3���������Լ���·�У��ȴ����ؽ���������������е�ã�64s������������\r\n");
			delay_ms(1000);
		}
		Check_Calibration();
		BUZZER_Open(0);
		
		//printf("------------------��ʼ�Լ�2-----------------------\r\n");
		On_CalMode();//����У��ģʽ
		//�Լ�2
		Send(CalSelfTest2,CalSelfTest2);
		while(dta_receive_flag==0)
		{
			printf("�Լ�2����1�����ѷ����Լ�2����ȴ��ظ��С�����\r\n");
		}
		check_flag= CheckReceive(CalSelfTest2);
		
		if(check_flag==1)
		{
			check_flag=0;
			printf("�Լ�2����2�����Լ�2�Ŵ�������(���͵�)�������\r\n");
			printf("%x + %x  \r\n",RxBuffer[0],RxBuffer[1]);
			BUZZER_Open(0);
		}
		else
			printf("�Լ�2����2�����Լ�2�Ŵ�������(���͵�)����ʧ��\r\n");
		while(dta_receive_flag==0)
		{
			printf("�Լ�2����3���������Լ�Ŵ����У��ȴ����ؽ������������\r\n");
			delay_ms(1000);
		}
		Check_Calibration();
		BUZZER_Open(0);
}
/****************************************************************************
* ��    �ƣ�PowerOnSelfCheck
* ��    �ܣ��ϵ���У��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����PowerOnSelfCheck();
****************************************************************************/
void PowerOnSelfCheck(void)
{
//	printf("------------------�ϵ��ʼ���ȴ���������-----------------------\r\n");
//	dta_receive_flag=0;
	while(dta_receive_flag==0)
	{
//		printf("1���������ϵ��ʼ���У��ȴ����ؽ������������\r\n");
//		delay_ms(1000);
	}
	Check_Calibration();//��ʼ��֮��δ��������֮ǰ���鿴DTA0660���ص�״̬
}

void display_dta0660(void)
{
	printf("\r\n\%x		1	2	3	4	5	6	7	8	9	10			��λ	Hex	Dec	����ֵ",funcnum);
	printf("\r\n---------------------------------------------------------------------------------------------------------------------------");
	ReadDTAValue(ReadADC);
	printf("\r\nReadADC(5):	%x	%x	%x	%x	%x*	%x	%x	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);

	ReadDTAValue(ReadRMS_DC);
	printf("\r\nReadRMS_DC(10):	%x	%x	%x	%x	%x	%x	%x	%x	%x	%x*	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);

	receive_f= ReadDTAValue(ReadResult);
	printf("\r\nReadResult(6):	%x	%x	%x	%x	%x	%x*	%x	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);
	printf("ReadResult:	%x	%x	%d	%2.5f",RxBuffer[4],receive_data,receive_data,receive_f);
	
	receive_f= ReadDTAValue(ReadFreq);
	printf("\r\nReadFreq(7):	%x	%x	%x	%x	%x	%x	%x*	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);
	printf("ReadFreq:		%x	%d	%2.5f",receive_data,receive_data,receive_f);
	
	receive_f= ReadDTAValue(ReadDuty);
	printf("\r\nReadDuty(4):	%x	%x	%x	%x*	%x	%x	%x	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);
	printf("ReadDuty:		%x	%d	%2.5f",receive_data,receive_data,receive_f);
	
	receive_f= ReadDTAValue(ReadRange);
	printf("\r\nReadRange(3):	%x	%x	%x*	%x	%x	%x	%x	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);
	printf("ReadRange:				%2.5f",receive_f);
	
	ReadDTAValue(ReadStatus);
	printf("\r\nReadStatus(8):	%x	%x	%x	%x	%x	%x	%x	%x*	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2],RxBuffer[3],RxBuffer[4],RxBuffer[5],RxBuffer[6],RxBuffer[7],RxBuffer[8],RxBuffer[9]);

	
	printf("\r\n---------------------------------------------------------------------------------------------------------------------------\r\n");
}

void FunctionSet(u8 cmd)
{
	//�趨����
	funcnum= cmd;
	Send(SetFuncX,funcnum,SetFuncX+ funcnum);
	check_flag= CheckReceive(SetFuncX);
	if(check_flag==1)
	{
		check_flag=0;
//		printf("\r\n%x + %x + %x  ",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
	}
}

void RangeSet(u8 cmd)
{
	u8 rangenum=0;
	rangenum=cmd;
	Send(SetRangeX,rangenum,SetRangeX+ rangenum);
	check_flag= CheckReceive(SetRangeX);
	if(check_flag==1)
		check_flag=0;
}
/**************************************************end file**************************************************/

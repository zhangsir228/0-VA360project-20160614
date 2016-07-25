/********************************************************************************
	*	@file		dta0660.c
	*	@author	Jeff
	*	@date		2014/10/5
	*	@brief	DTA0660配置。	
	*					
	*					使用外设：USART1(0,2)
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
* 名    称：DTA_Pins_Config(void)
* 功    能：DTA0660通信控制引脚CS、RDY初始化
* 入口参数：无
* 出口参数：无
* 说    明：给DTA上电后，CS为高电平；由待机状态开机时将CS置为低电平，进行后续通信
												——由此将CS设置为推挽输出
												RDY为低电平；通信时，DTA接收到有效命令，将RDY置为高电平，返回数据完毕重置为低电平
												——由此将RDY设置为下拉输入
* 调用方法：DTA_Pins_Config()
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
* 名    称：UART1_init(void)
* 功    能：DTA0660通信串口初始化
* 入口参数：baudarate      波特率
* 出口参数：无
* 说    明：
* 调用方法：UART1_init()
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
	
	USART_DeInit(USART1);  //复位串口1
	
	USART_InitStructure.USART_BaudRate = 2400;									//设置串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//设置数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//设置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;							//设置效验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//设置流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//设置工作模式
	USART_Init(USART1, &USART_InitStructure); 											//配置入结构体

  //Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00 ;			//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;						//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);																	//根据指定的参数初始化NVIC寄存器
   
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	
	USART_ITConfig(USART1, USART_IT_PE, ENABLE);    //开启PE错误接收中断Bit 8PEIE: PE interrupt enable
  //CR2 开启ERR中断
  USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	USART_Cmd(USART1, ENABLE);//使能串口1
}

/****************************************************************************
* 名    称：DTA0660_Init(void)
* 功    能：DTA0660初始化
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：DTA0660_Init()
****************************************************************************/
void DTA0660_Init(void)
{
	u16 len;	
	u16 times=0;
	DTA_Pins_Config();
	UART1_init();
	
	ClrCS//将CS置为低电平
	while(Read_RDY() != RESET);//等待RDY为低电平
	delay_ms(1000);/*delay_ms(1000);*///2015-3-29：等待DTA将上电初始化数据发过来，以免造成后面数据接收混乱
}

/****************************************************************************
* 名    称：Check_Calibration(void)
* 功    能：DTA0660的检查和校准
* 入口参数：无
* 出口参数：无
* 说    明：一般是校验时使用
* 调用方法：Check_Calibration()
****************************************************************************/
void Check_Calibration(void)
{
	if(dta_receive_flag==1)//在STM32没有向DTA0660发送状态下，接收到的数据CS校准无误（RetOK、RetErr情况）
	{
		dta_receive_flag=0;
		
		if(RxBuffer[0]==RetOK)
		{
			if(RxBuffer[1]==0x01)
			{
				printf("上电初始化完成\r\n");
				lcd_show_num(1111,1,4);
				delay_ms(1000);
			}
			else if(RxBuffer[1]==0x02)	printf("自检1完成\r\n");
			else if(RxBuffer[1]==0x03)	printf("自检2完成\r\n");
			else	printf("返回RetOK");
		}
		else if(RxBuffer[0]==RetErr)
		{
			if(RxBuffer[1]==0x01)	printf("EEPROM错误\r\n");
			else if(RxBuffer[1]==0x02)	printf("EEPROM未初始化\r\n");
			else if(RxBuffer[1]==0x03)	printf("EEPROM写入错误\r\n");
			else if(RxBuffer[1]==0x04)	printf("校正数据超过允许范围\r\n");
			else printf("返回RetErr");
		}
		printf("\r\n");
	}
}

/****************************************************************************
* 名    称：Send(u8 data,...)
* 功    能：控制端向DTA0660发送控制命令
* 入口参数：data			发送的数据（数据类型为：字节）
* 出口参数：无
* 说    明：所有发送命令和返回参数最末尾一个字节数据为前面所发送数据的累加和。
						如设定地址命令为：50H+21H+02H+73H(CS) （50H为命令字，21H+02H为地址，73H为50H+21H+02H的累加和）
						读/写的地址为2字节AddrL+AddrH，低字节在前高字节在后，地址允许范围：
						000H~17FH（IC内部RAM,其中03BH~042H对应LCD BUFFER），200H~2FFH（对应EEPROM地址000H~0FFH）。
* 调用方法：Send(SetAddr,0x3B,0x00,0x8B);//设置LCD地址
****************************************************************************/
void Send(u8 data,...)
{
	u8 cs=0;
	va_list arg_ptr;
	u8 nArgValue = data;
	va_start(arg_ptr,data);//以固定参数的地址为起点确定变参的内存起始地址。
	
	ClrCS
	
	do
	{
		cs+= nArgValue;//计算截止到当前发送值所有发送值的和
		
		USART_SendData(USART1,nArgValue);//输出各参数的值
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
		nArgValue= va_arg(arg_ptr,u8); 		//得到下一个可变参数的值
		
	}while(nArgValue !=  cs);//停止条件：截止到当前发送值所有发送值的和  == 下一个可变参数的值
	
	USART_SendData(USART1,nArgValue);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	
	va_end(arg_ptr);
	
	RxCounter=0;//发送完命令后，即刻将RxCounter置零，使RxBuffer数组从头写入接收数据
}

/****************************************************************************
* 名    称：CheckReceive
* 功    能：等待DTA0660返回数据，校验首字节和末字节
* 入口参数：- command：根据发送的控制命令确定
* 出口参数：- 1：校验成功标志
						- 0：校验失败标志
* 说    明：
* 调用方法：check_flag= CheckReceive(SetRangeAuto)：校验DTA0660设定自动挡成功与否
****************************************************************************/
boolean CheckReceive(u8 command)
{
	u8 j;
	u16 i;
	for(j=0;j<30;j++)//使用两个for，在这段时间内检测数据接收完成标志：接收到，则返回成功（1）；否则返回失败（0）。
	{
		for(i=0;i<10000;i++)
		{
			if(dta_receive_flag==1)//接收到的数据CS校准无误
			{
//				printf("\r\nj=%d,i=%d",j,i);
				dta_receive_flag=0;
				if(RxBuffer[0]==command)	
					return 1;//接收到的数据首字节为控制命令字符无误
			}
		}
	}
	//2015-3-29：如果校验数据失败，则在此等待，以保证DTA发送数据完毕。
	//而不会造成DTA在发送未完成数据，在混合下次要求的数据，将持续无法校验。
	while(Read_RDY() != RESET);//等待RDY为低电平
	dta_receive_flag=0;
	RxCounter=0;
	dta_cs=0;//2015-3-29：之前一直没有把这条语句加入，导致即使重新发送数据，还是在用原来的累加和在计算，而dta_cs是需要接收到每条语句，不论正确与否都要清零的。
	return 0;
}

/****************************************************************************
* 名    称：ReadDTAValue
* 功    能：读取DTA0660的测量数据
* 入口参数：- command：要读取数据类型
* 出口参数：- DTA0660的测量数据
* 说    明：可选入口参数：ReadADC、ReadRMS_DC、ReadResult、ReadFreq、ReadDuty、ReadRange、ReadStatus
* 调用方法：receive_f= ReadDTAValue(ReadResult);
****************************************************************************/
float ReadDTAValue(u8 command)
{
	float temp=0;
	Send(command,command);//设定需要读取的类型
	check_flag= CheckReceive(command);
	if(check_flag==1)
	{
		check_flag= 0;
		//读取ADC值: 05EH+AdcL+AdcM+AdcH+CS
		if(command == ReadADC)																										
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16;
			temp=	receive_data;
		}
		//读取5bytesRMS及3bytes DC值: 05FH+Rms0~Rms4+Dc0~2+CS
		else if(command == ReadRMS_DC)																						
		{
			
		}
		//读取测量的结果：060H + DataL + DataM + DataH + RangeNum + CS
		else if(command == ReadResult)																						
		{
			float positive_negative= 1.0f;
			receive_data = (RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16) & 0xFFFFFF;
			
			if((receive_data>>23)==0x01)//判断数据是否为负数，若是，则令符号标志位-1，并求得绝对值
			{
				positive_negative= -1.0;
				receive_data=(~receive_data+1) & 0xFFFFFF;
			}
			
			if(funcnum==DCmV)//未验证，10月6日添加
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
			else if(funcnum==ACmV)//未验证，10月6日添加
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
			else if(funcnum==DCV)//已验证
			{//2015-8-30：删掉多余的
				temp= (float)receive_data* positive_negative;
			}
			else if(funcnum==ACV)//已验证
			{//2015-7-7：删掉多余的
				temp= (float)receive_data* positive_negative;
			}
			
			else if(funcnum==DCVmV)//已验证
			{//2015-7-7：删掉多余的
				temp= (float)receive_data* positive_negative;
			}
			else if(funcnum==ACVmV)//已验证
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
			else if(funcnum==Ohm)//已验证10-4，输出数据单位：欧姆
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
			else if(funcnum==Cont)//已验证10-4，输出数据单位：欧姆
			{
				temp= receive_data;
				if(RxBuffer[4]==0)
					temp= temp/ 10;
			}
			else if(funcnum==Diode)//已验证10-4，输出数据单位：mV
			{//2015-3-29:修改为输出单位V
				temp= receive_data/1000.0f;
			}
			else if(funcnum==Cap)//已验证10-4，输出数据单位：nF(3位小数)/uF(3位小数)/uF(2位小数)/mF(3位小数)
			{//修改2015-3-28：将输出单位统一为nF
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
			else if(funcnum==Temp)//已验证10-4，输出数据单位：摄氏度
			{
				temp= receive_data;
			}
			
		}
		//读取频率值：061H+Freq0~Freq4+CS、单位为0.001HZ
		else if(command == ReadFreq)
		{
			receive_data = (u32)RxBuffer[1] | ((u32)RxBuffer[2])<<8 | ((u32)RxBuffer[3])<<16 | ((u32)RxBuffer[4])<<24;
			temp= (float)receive_data/ 1000.f+ (float)RxBuffer[5]/ 1000.f* pow(2,32);
		}
		//读取占空比值：062H+DutyL+DutyH+CS、单位为0.01%
		else if(command == ReadDuty)
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8;
			temp= (float)receive_data/ 100;
		}
		//读取档位值：063H+RangeNum+CS
		else if(command == ReadRange)																							
			temp= RxBuffer[1];
		//读取系统状态：067H+Status0~5+CS
		else if(command == ReadStatus)																						
		{
			receive_data = RxBuffer[1] | (RxBuffer[2])<<8 | (RxBuffer[3])<<16;
			temp= 0;
		}
	}
	else
	{
		temp= 0;
//		printf("\r\n*无效命令或校验出错*\r\n");
		lcd_show_Erro(1);//发生于DTA通信出错，在液晶上显示，并等待1s.
		delay_ms(1000);
	}	
	return temp;
}

/****************************************************************************
* 名    称：WriteEeprom
* 功    能：烧写整个EEPROM
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：WriteEeprom();
****************************************************************************/
void WriteEeprom(void)
{
	u16 i=0;//20160312：修改“允许LDC显示”——FDH.Bit7=0时，发现之前一直是u8。
	
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
		printf("*未找到EEPROM或校验出错*\r\n");
	}
}

/****************************************************************************
* 名    称：ReadEeprom
* 功    能：读取整个EEPROM
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：ReadEeprom();
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
//				eeprom[i]=RxBuffer[1];														//记于10月5日：eeprom[]这个数组貌似可以不要，直接上变量得了//2015年3月18日注释
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
		printf("读数据--定位EEPROM出错----ERROR! \r\n");		
}

/****************************************************************************
* 名    称：On_CalMode
* 功    能：开启校验
* 入口参数：无
* 出口参数：- 1:成功
						- 0:失败
* 说    明：
* 调用方法：On_CalMode();
****************************************************************************/
static u8 On_CalMode(void)
{
	Send(SetCalMode,0x55,SetCalMode+0x55);
	check_flag= CheckReceive(SetCalMode);
	if(check_flag==1)
	{
		check_flag=0;
		printf("\r\n开启校正:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
		printf("\r\n开启校正模式\r\n");
	}
	return 1;
}

/****************************************************************************
* 名    称：On_CalMode
* 功    能：关闭校验
* 入口参数：无
* 出口参数：- 1:成功
						- 0:失败
* 说    明：
* 调用方法：On_CalMode();
****************************************************************************/
static u8 Off_CalMode(void)
{
	Send(SetCalMode,0x00,SetCalMode);
	check_flag= CheckReceive(SetCalMode);
	if(check_flag==1)
	{
		check_flag=0;
		printf("\r\n关闭校正:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
		printf("\r\n关闭校正模式\r\n");
	}
	return 1;
}

/****************************************************************************
* 名    称：Self_Calibration
* 功    能：启动自校验，自检1-线路，自检2-放大器
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：Self_Calibration();
****************************************************************************/
void Self_Calibration(void)
{
//		printf("------------------上电初始化等待返回命令-----------------------\r\n");
//		dta_receive_flag=0;
//		while(dta_receive_flag==0)
//		{
//			printf("1——正在上电初始化中，等待返回结果。。。。。\r\n");
//			delay_ms(1000);
//		}
//		Check_Calibration();//初始化之后、未发送数据之前，查看DTA0660返回的状态
		
		printf("------------------进入校正模式----------------------\r\n");		
		printf("------------------开始自检1-----------------------\r\n");
		BUZZER_Open(0);
		On_CalMode();//进入校正模式
		//自检1
		Send(CalSelfTest1,CalSelfTest1);
		while(dta_receive_flag==0)
		{
			printf("自检1步骤1——已发送自检1命令，等待回复中。。。\r\n");
		}
		check_flag= CheckReceive(CalSelfTest1);	
		if(check_flag==1)
		{
			check_flag=0;
			printf("自检1步骤2——自检1线路命令(发送的)返回完成\r\n");
			printf("%x + %x  \r\n",RxBuffer[0],RxBuffer[1]);
			BUZZER_Open(0);
		}
		else
			printf("自检1步骤2——自检1线路命令(发送的)返回失败\r\n");
		while(dta_receive_flag==0)
		{
			printf("自检1步骤3——正在自检线路中，等待返回结果。。。。。（有点久，64s）。。。。。\r\n");
			delay_ms(1000);
		}
		Check_Calibration();
		BUZZER_Open(0);
		
		//printf("------------------开始自检2-----------------------\r\n");
		On_CalMode();//进入校正模式
		//自检2
		Send(CalSelfTest2,CalSelfTest2);
		while(dta_receive_flag==0)
		{
			printf("自检2步骤1——已发送自检2命令，等待回复中。。。\r\n");
		}
		check_flag= CheckReceive(CalSelfTest2);
		
		if(check_flag==1)
		{
			check_flag=0;
			printf("自检2步骤2——自检2放大器命令(发送的)返回完成\r\n");
			printf("%x + %x  \r\n",RxBuffer[0],RxBuffer[1]);
			BUZZER_Open(0);
		}
		else
			printf("自检2步骤2——自检2放大器命令(发送的)返回失败\r\n");
		while(dta_receive_flag==0)
		{
			printf("自检2步骤3——正在自检放大器中，等待返回结果。。。。。\r\n");
			delay_ms(1000);
		}
		Check_Calibration();
		BUZZER_Open(0);
}
/****************************************************************************
* 名    称：PowerOnSelfCheck
* 功    能：上电自校验
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：PowerOnSelfCheck();
****************************************************************************/
void PowerOnSelfCheck(void)
{
//	printf("------------------上电初始化等待返回命令-----------------------\r\n");
//	dta_receive_flag=0;
	while(dta_receive_flag==0)
	{
//		printf("1——正在上电初始化中，等待返回结果。。。。。\r\n");
//		delay_ms(1000);
	}
	Check_Calibration();//初始化之后、未发送数据之前，查看DTA0660返回的状态
}

void display_dta0660(void)
{
	printf("\r\n\%x		1	2	3	4	5	6	7	8	9	10			档位	Hex	Dec	返回值",funcnum);
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
	//设定功能
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

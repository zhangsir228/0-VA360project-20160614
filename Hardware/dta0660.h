#ifndef __DTA0660_H
#define __DTA0660_H	
#include "stm32f37x.h"

//控制引脚
#define DTA_Pin_CS      GPIO_Pin_11
#define DTA_PORT_CS     GPIOA
#define DTA_CLK_CS      RCC_AHBPeriph_GPIOA

#define DTA_Pin_RDY      GPIO_Pin_12
#define DTA_PORT_RDY     GPIOA
#define DTA_CLK_RDY      RCC_AHBPeriph_GPIOA												
//置位、复位
#define SetCS  GPIO_SetBits(DTA_PORT_CS, DTA_Pin_CS);
#define ClrCS  GPIO_ResetBits(DTA_PORT_CS, DTA_Pin_CS);

#define SetRDY  GPIO_SetBits(DTA_PORT_RDY, DTA_Pin_RDY);
#define ClrRDY  GPIO_ResetBits(DTA_PORT_RDY, DTA_Pin_RDY);

//读取RDY
#define	Read_RDY()	GPIO_ReadInputDataBit(DTA_PORT_RDY,DTA_Pin_RDY)

//通讯命令
#define SetAddr					0x50
#define Read						0x51
#define ReadFwd					0x52
#define ReadBwd					0x53
#define Write						0x54
#define WriteFwd				0x55
#define WriteBwd				0x56
#define PowerOff				0x57
#define SetFuncX				0x58
#define SetRangeX				0x59
#define SetRangeAuto		0x5A
#define SetRangeNext		0x5B
#define ReadADC					0x5E
#define ReadRMS_DC			0x5F
#define ReadResult			0x60
#define ReadFreq				0x61
#define ReadDuty				0x62
#define ReadRange				0x63
#define ReadStatus			0x67
#define SetFastFuncX		0x68
#define RetOK						0xA0
#define RetErr					0xA1
#define SetCalMode			0xC0
#define CalSelfTest1		0xC1
#define CalSelfTest2		0xC2
#define CalDataINC			0xC3
#define CalDataDEC			0xC4
#define CalSetData			0xC5
#define CalGetK					0xC6
//功能代码
#define DCmV						0x01
#define ACmV						0x02
#define DCV							0x03
#define ACV							0x04
#define DCVmV						0x05
#define ACVmV						0x06
#define Ohm							0x07

#define Cont						0x09
#define Diode						0x0A
#define Cap							0x0B
#define DCuA 						0x0C		//DCuA 600.0μA/6000μA (或钳表DCA 600.0A/6000A)
#define ACuA  					0x0D		//ACuA 600.0μA/6000μA (或钳表ACA 600.0A/6000A)
#define DCmA 						0x0E		//DCmA 60.00mA/600.0mA (或钳表DCA 60.00A/600.0A)
#define ACmA 						0x0F		//ACmA 60.00mA/600.0mA (或钳表ACA 60.00A/600.0A)
#define DCA 						0x10		//DCA 6.000A/60.00A (或钳表DCA 6.000A/60.00A)
#define ACA 						0x11		//ACA 6.000A/60.00A (或钳表ACA 6.000A/60.00A)
#define Hz_Duty 				0x12
#define Temp 						0x13
#define hFE							0x14

#define DCA_6 					0x16
#define ACA_6 					0x17
#define DCA_60 					0x18
#define ACA_60 					0x19
#define DCA_600 				0x1A
#define ACA_600 				0x1B
#define DCA_6000 				0x1C
#define ACA_6000 				0x1D

/***************全局变量*******************/
extern uint8_t RxCounter;
#define RXBUFFERSIZE   0x20
extern uint8_t RxBuffer[RXBUFFERSIZE];

extern boolean dta_receive_flag,check_flag;
extern u32 receive_data;
extern float receive_f,receive_f1;
extern u8 dta_cs;
extern u8 funcnum;

/***************函数声明********************/
void DTA0660_Init(void);
void Check_Calibration(void);
/* */
void Send(u8 data,...);
boolean CheckReceive(u8 command);
float ReadDTAValue(u8 command);
void WriteEeprom(void);
void ReadEeprom(void);
void Self_Calibration(void);
void PowerOnSelfCheck(void);
void display_dta0660(void);
void FunctionSet(u8 cmd);
void RangeSet(u8 cmd);
#endif

/**************************************************end file**************************************************/

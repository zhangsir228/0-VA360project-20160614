#ifndef __HT1621_H
#define __HT1621_H	
#include "stm32f37x.h"



// LCD segment definitions.
#define a 0x08
#define b 0x80
#define c 0x20
#define d 0x01
#define e 0x02
#define f 0x04
#define g 0x40
#define h 0x10
#define AA a+b+c+e+f+g
#define BB c+d+e+f+g
//#define DD b+c+d+e+g
//#define PP a+b+e+f+g
#define O c+d+e+g
//#define S a+c+d+g+f
#define EE a+d+e+f+g
//#define TT d+e+g+f
//#define Y b+c+d+f+g
//#define FF a+e+f+g
#define RR e+g
#define CC a+f+e+d
#define OO a+b+c+d+e+f
#define LL d+e+f
//#define I  e+f
//#define VV  b+c+d+e+f

//控制引脚
#define HT1621_Pin_WR      GPIO_Pin_8
#define HT1621_PORT_WR     GPIOA
#define HT1621_CLK_WR      RCC_AHBPeriph_GPIOA

#define HT1621_Pin_RD      GPIO_Pin_8
#define HT1621_PORT_RD     GPIOD
#define HT1621_CLK_RD      RCC_AHBPeriph_GPIOD	

#define HT1621_Pin_CS      GPIO_Pin_14
#define HT1621_PORT_CS     GPIOB
#define HT1621_CLK_CS      RCC_AHBPeriph_GPIOB

#define HT1621_Pin_DATA		 GPIO_Pin_15
#define HT1621_PORT_DATA   GPIOB
#define HT1621_CLK_DATA    RCC_AHBPeriph_GPIOB
//复位、置位
#define HT1621_Clr_WR GPIO_ResetBits(HT1621_PORT_WR,HT1621_Pin_WR);
#define HT1621_Set_WR GPIO_SetBits(HT1621_PORT_WR,HT1621_Pin_WR);

#define HT1621_Clr_RD GPIO_ResetBits(HT1621_PORT_RD,HT1621_Pin_RD);
#define HT1621_Set_RD GPIO_SetBits(HT1621_PORT_RD,HT1621_Pin_RD);

#define HT1621_Clr_CS GPIO_ResetBits(HT1621_PORT_CS,HT1621_Pin_CS);
#define HT1621_Set_CS GPIO_SetBits(HT1621_PORT_CS,HT1621_Pin_CS);

#define HT1621_Clr_DATA GPIO_ResetBits(HT1621_PORT_DATA,HT1621_Pin_DATA);
#define HT1621_Set_DATA GPIO_SetBits(HT1621_PORT_DATA,HT1621_Pin_DATA);

extern u8 lcd_show_ram[];

/***************函数声明********************/
void write_mode(boolean mode);
void write_command(u8 Cbyte);
void write_address(u8 Abyte);
void write_data_8bits(u8 Dbyte);
void write_data_4bits(u8 Dbyte);
void lcd_write_4bits(u8 Address,u8 Dbyte4);
void lcd_write_8bits(u8 Address,u8 Dbyte8);
void lcd_write_1bit(u8 Address,u8 n,FunctionalState NewState);

void HT1621_Init(void);
void lcd_clr(void);
void lcd_full_ram(void);
void lcd_full_test(void);
void lcd_show_data(u8 *puts,u8 n);
boolean lcd_show_num(float data,u8 n,u8 size);
void lcd_show_dta_num(float data,u8 n);
void lcd_show_OL(u8 n);
void lcd_show_Erro(u8 n);
void lcd_show_None(u8 n);
void lcd_show_Line(u8 n);
void lcd_show_A___(u8 n);
void lcd_show_A__A(u8 n);
void lcd_show_b__b(u8 n);
void lcd_show_C__C(u8 n);
void lcd_show_____(u8 n);
void lcd_show_Cal(u8 n);
#endif
/***************************************************end file*************************************************/

/********************************************************************************
	*	@file		ht1621.c
	*	@author	Jeff
	*	@date		2014/11/2
	*	@brief	HT1621配置—>驱动LCD。
	*					
	*					未使用外设
	*					
	*
	*					WR					PA8
	*					RD					PD8
	*					CS					PB14
	*					DATA				PB15
	*					
	*					
	*******************************************************************************/
#include "stm32f37x.h"
#include "nvic_systick.h"
#include "ht1621.h"
#include "stdlib.h"
#include "math.h"

const boolean  DAT=1;	//写数据
const boolean  COM=0;	//写命令

#define ht1621_delay	delay_us(10);

//float abs(float __x);

// LCD table
const char char_tab[] = {                 // definitions for digits
  a+b+c+d+e+f,                              // Displays "0"
  b+c,                                      // Displays "1"
  a+b+d+e+g,                                // Displays "2"
  a+b+c+d+g,                                // Displays "3"
  b+c+f+g,                                  // Displays "4"
  a+c+d+f+g,                                // Displays "5"
  a+c+d+e+f+g,                              // Displays "6"
  a+b+c,                                    // Displays "7"
  a+b+c+d+e+f+g,                            // Displays "8"
  a+b+c+d+f+g                               // Displays "9"
};

u8 lcd_show_ram[16]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/****************************************************************************
* 名    称：HT1621_Pins_Config
* 功    能：HT1621引脚配置
* 入口参数：无
* 出口参数：无
* 说    明：将CS、RD、WR、DATA配置为带上拉电阻的推挽输出
* 调用方法：HT1621_Pins_Config();
*						HT1621初始化时调用
****************************************************************************/
static void HT1621_Pins_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(HT1621_CLK_WR | HT1621_CLK_RD | HT1621_CLK_CS | HT1621_CLK_DATA ,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = HT1621_Pin_WR;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(HT1621_PORT_WR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = HT1621_Pin_RD;
  GPIO_Init(HT1621_PORT_RD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = HT1621_Pin_CS;
  GPIO_Init(HT1621_PORT_CS, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = HT1621_Pin_DATA;
  GPIO_Init(HT1621_PORT_DATA, &GPIO_InitStructure);
}

/****************************************************************************
* 名    称：write_mode
* 功    能：模式写入
* 入口参数：mode:COM(命令模式) DAT（数据模式）
* 出口参数：无
* 说    明：100-COMMAND,101-写数据
*						写入模式：数据or命令
* 调用方法：write_mode(mode)
****************************************************************************/
void write_mode(boolean mode)
{
	HT1621_Clr_WR								//	WR = 0;
	ht1621_delay
	HT1621_Set_DATA							//		DA = 1;
	HT1621_Set_WR								//	WR = 1;
	ht1621_delay
	
	HT1621_Clr_WR								//	WR = 0;
	ht1621_delay
	HT1621_Clr_DATA							//		DA = 0;
	HT1621_Set_WR								//	WR = 1;
	ht1621_delay
	
	HT1621_Clr_WR								//	WR = 0;
	ht1621_delay
	if(mode == COM)
	{
		HT1621_Clr_DATA						//		DA = 0;
	}
	else
	{
		HT1621_Set_DATA						//		DA = 1;
	}
	ht1621_delay
	HT1621_Set_WR								//	WR = 1;
	ht1621_delay
}

/****************************************************************************
* 名    称：write_command
* 功    能：命令写入函数
* 入口参数：Cbyte，控制命令字，整个8位
* 出口参数：无
* 说    明：C8C7C6C5C4C3C2C1(C0)
*						先写高位，再写低位，共8+1位
*						写命令代码9bit,最后1bit没用
* 调用方法：write_command(Cbyte)
****************************************************************************/
void write_command(u8 Cbyte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		HT1621_Clr_WR							//	WR = 0;
		ht1621_delay
		if((Cbyte>>(7-i)) & 0x01)
		{
			HT1621_Set_DATA					//		DA = 1;
		}
		else
		{
			HT1621_Clr_DATA					//		DA = 0;
		}
		ht1621_delay
		HT1621_Set_WR							//	WR = 1;
		ht1621_delay
	}
	HT1621_Clr_WR								//	WR = 0;
	ht1621_delay
	HT1621_Clr_DATA							//		DA = 0;第9bit,无作用,但是必须写入
	HT1621_Set_WR								//	WR = 1;
	ht1621_delay
}

/****************************************************************************
* 名    称：write_address
* 功    能：地址写入函数
* 入口参数：Abyte，地址，保存在低6位
* 出口参数：无
* 说    明：A5A4A3A2A1A0
*						先写高位，再写低位，共6位
*						写地址6bit
* 调用方法：write_address(Abyte)
****************************************************************************/
void write_address(u8 Abyte)
{
	u8 i;
	for(i=0;i<6;i++)
	{
		HT1621_Clr_WR							//	WR = 0;
		ht1621_delay
		if((Abyte>>(5-i)) & 0x01)
		{
			HT1621_Set_DATA					//		DA = 1;
		}
		else
		{
			HT1621_Clr_DATA					//		DA = 0;
		}
		ht1621_delay
		HT1621_Set_WR							//	WR = 1;
		ht1621_delay
	}
}

/****************************************************************************
* 名    称：write_data_8bits
* 功    能：数据写入函数
* 入口参数：Dbyte，8位数据
* 出口参数：无
* 说    明：先写低位，再写高位，共8位
*						写数据
* 调用方法：write_data_8bits(Dbyte)
****************************************************************************/
void write_data_8bits(u8 Dbyte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		HT1621_Clr_WR							//	WR = 0;
		ht1621_delay
		if((Dbyte>>i) & 0x01)
		{
			HT1621_Set_DATA					//		DA = 1;
		}
		else
		{
			HT1621_Clr_DATA					//		DA = 0;
		}
		ht1621_delay
		HT1621_Set_WR							//	WR = 1;
		ht1621_delay
	}
}
/****************************************************************************
* 名    称：write_data_4bits
* 功    能：数据写入函数
* 入口参数：Dbyte的低4位，4位数据
* 出口参数：无
* 说    明：先写低位，再写高位，共4位
*						写数据
* 调用方法：write_data_4bits(Dbyte)
****************************************************************************/
void write_data_4bits(u8 Dbyte)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		HT1621_Clr_WR							//	WR = 0;
		ht1621_delay
		if((Dbyte>>i) & 0x01)
		{
			HT1621_Set_DATA					//		DA = 1;
		}
		else
		{
			HT1621_Clr_DATA					//		DA = 0;
		}
		ht1621_delay
		HT1621_Set_WR							//	WR = 1;
		ht1621_delay
	}
}

/****************************************************************************
* 名    称：lcd_write_4bits
* 功    能：向具体地址0-0x1F(31)写入4bits，并将数据保存在lcd_show_ram数组中
* 入口参数：Address:0-0x1F(31)
*						Dbyte4:低4位有效
* 出口参数：
* 说    明：
*						
* 调用方法：lcd_write_4bits(Address,Dbyte4)
****************************************************************************/
void lcd_write_4bits(u8 Address,u8 Dbyte4)
{
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	
	write_address(Address);
	write_data_4bits(Dbyte4);
	
	HT1621_Set_CS								//CS = 1;
	
	if(Address%2==1)//修改后保存
		lcd_show_ram[Address/2]=(Dbyte4<<4)|(lcd_show_ram[Address/2]&0x0F);
	else
		lcd_show_ram[Address/2]=Dbyte4|(lcd_show_ram[Address/2]&0xF0);
}

/****************************************************************************
* 名    称：lcd_write_8bits
* 功    能：向具体地址0-0x1E(30)写入8bits，并将数据保存在lcd_show_ram数组中
* 入口参数：Address:0-0x1E(30)
*						Dbyte8:8位有效
* 出口参数：
* 说    明：
*						
* 调用方法：lcd_write_8bits(Address,Dbyte8)
****************************************************************************/
void lcd_write_8bits(u8 Address,u8 Dbyte8)
{
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	
	write_address(Address);
	write_data_8bits(Dbyte8);
	
	HT1621_Set_CS								//CS = 1;
	//修改后保存
	if(Address%2==1)//当Address为1、3、5。。。。等奇数地址时，写的8位数据在lcd_show_ram数组中跨越2个元素
	{
		lcd_show_ram[Address/2]=(Dbyte8<<4)|(lcd_show_ram[Address/2]&0x0F);//将Dbyte8的低4位，保存在lcd_show_ram[Address/2]的高4位
		lcd_show_ram[Address/2+1]=(Dbyte8>>4)|(lcd_show_ram[Address/2+1]&0xF0);//将Dbyte8的高4位，保存在lcd_show_ram[Address/2+1]的低4位
	}
	else//当Address为0、2、4。。。。等偶数地址时，写的8位数据在lcd_show_ram数组中1个元素内
		lcd_show_ram[Address/2]=Dbyte8;
}



/****************************************************************************
* 名    称：lcd_write_1bit
* 功    能：向具体地址0-0x1F(31)的第（0或1或2或3）位写入0或1
* 入口参数：Address
*						NewState:0 or 1
*						n:具体地址上的第几位，低->高的每一位表示0->3。
* 出口参数：
* 说    明：
*						
* 调用方法：lcd_write_1bit(Address,NewState,n)
****************************************************************************/
void lcd_write_1bit(u8 Address,u8 n,FunctionalState NewState)
{
	u8 data;

	data=lcd_show_ram[Address/2];//先取得Address(0-31)对应的ram的值data(8bits)
	if(Address%2==1)//如果Address(0-31)是1、3、5、、、、之类，则将对应ram的值data右移4位，得到低四位为真实的Address对应的4bits
		data>>=4;
	switch(n)
	{
		case 0:
		{
			if(NewState==DISABLE)	data&=0x0E;
			else	data|=0x01;
		}break;
		case 1:
		{
			if(NewState==DISABLE)	data&=0x0D;
			else	data|=0x02;
		}break;
		case 2:
		{
			if(NewState==DISABLE)	data&=0x0B;
			else	data|=0x04;
		}break;
		case 3:
		{
			if(NewState==DISABLE)	data&=0x07;
			else	data|=0x08;
		}break;
		default:break;
	}
	
	lcd_write_4bits(Address,data);
}

/****************************************************************************
* 名    称：HT1621_Init
* 功    能：HT1621初始化
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：HT1621_Init()
****************************************************************************/
void HT1621_Init(void)
{
	HT1621_Pins_Config();
	HT1621_Clr_CS								//CS = 0;
	write_mode(COM);	//命令模式100
	write_command(0x01);	//Enable System
	write_command(0x03);	//Enable Bias
	write_command(0x04);	//Disable Timer
	write_command(0x05);	//Disable WDT
	write_command(0x08);	//Tone OFF
	write_command(0x18);	//on-chip RC震荡
	write_command(0x29);	//1/4Duty 1/3Bias
	write_command(0x80);	//Disable IRQ
	write_command(0x40);	//Tone Frequency 4kHZ
	write_command(0xE3);	//Normal Mode
	HT1621_Set_CS								//CS = 1;
}

/****************************************************************************
* 名    称：lcd_clr
* 功    能：LCD 清屏函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：lcd_clr()
****************************************************************************/
void lcd_clr(void)
{
	u8 i;
	
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	write_address(0);
	
	for(i=0;i<16;i++)
	{
		write_data_8bits(0x00);
		lcd_show_ram[i]=0x00;
	}
	HT1621_Set_CS								//CS = 1;
}
/*写整个lcd_show_ram[]数组*/
void lcd_full_ram(void)
{
	u8 i;
	
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	write_address(0);
	
	for(i=0;i<16;i++)
	{
		write_data_8bits(lcd_show_ram[i]);
	}
	HT1621_Set_CS								//CS = 1;
}
/****************************************************************************
* 名    称：lcd_full_test
* 功    能：LCD 测试函数，写满屏幕，全亮
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：lcd_full_test()
****************************************************************************/
void lcd_full_test(void)
{
	u8 i;
	
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	write_address(0);
	
	for(i=0;i<16;i++)
	{
		write_data_8bits(0xFF);
//		lcd_show_ram[i]=0xFF;
	}
	HT1621_Set_CS								//CS = 1;
}

/****************************************************************************
* 名    称：lcd_show_data
* 功    能：LCD写数据
* 入口参数：puts：长度为4的数组指针
* 出口参数：无
* 说    明：要求数组长度为4，数组单个数据为8位。
*						n=1:四位数,用于显示第一行的四位数，
*						n=2:四位数,用于显示第二行的四位数，
*						puts数组====》1、2、3、4==》对应千百十个，即数据本身的最高到最低位。
* 调用方法：
****************************************************************************/
void lcd_show_data(u8 *puts,u8 n)
{
	u8 i,Address;
	
	HT1621_Clr_CS								// CS = 0;
	write_mode(DAT);
	if(n==1)
	{
		Address=0x01;
		if(lcd_show_ram[4]&0x01)
		{
			puts[3]|=0x10;
		}
	}
	else if(n==2)
	{
		Address=0x0C;
		if(lcd_show_ram[9]&0x10)
		{
			puts[3]|=0x10;
		}
	}
		
	write_address(Address);
	for(i=0;i<4;i++)
	{
		write_data_8bits(puts[i]);
	}
	
	HT1621_Set_CS								//CS = 1;
	
	//修改后保存
	for(i=0;i<4;i++)
	{
		if(Address%2==1)//当Address为1、3、5。。。。等奇数地址时，写的8位数据在lcd_show_ram数组中跨越2个元素
		{
			lcd_show_ram[Address/2]=(puts[i]<<4)|(lcd_show_ram[Address/2]&0x0F);//将Dbyte8的低4位，保存在lcd_show_ram[Address/2]的高4位
			lcd_show_ram[Address/2+1]=(puts[i]>>4)|(lcd_show_ram[Address/2+1]&0xF0);//将Dbyte8的高4位，保存在lcd_show_ram[Address/2+1]的低4位
		}
		else//当Address为0、2、4。。。。等偶数地址时，写的8位数据在lcd_show_ram数组中1个元素内
			lcd_show_ram[Address/2]=puts[i];
		Address+=2;
	}
}

/****************************************************************************
* 名    称：lcd_show_num
* 功    能：LCD写数字
* 入口参数：data：要显示的数据
*						n=1:用于显示第一行的四位数，
*						n=2:用于显示第二行的四位数，
*						size:表示有几位整数部分，即LCD小数点之前的位数。size=10表示自动
* 出口参数：无
* 说    明：要求data<10000，最大5位数
* 调用方法：
****************************************************************************/
boolean lcd_show_num(float data,u8 n,u8 size)
{
	u8 i,numsize;
	u8 show_buf[4]={0},float_table[5]={0};
	u32 aa;
	
	if(data<0)
	{
		if(n==1)
			lcd_write_1bit(0x0,2,ENABLE);//-亮
		else if(n==2)
			lcd_write_1bit(0x0A,2,ENABLE);//-亮
	}
	else
	{
		if(n==1)
			lcd_write_1bit(0x0,2,DISABLE);//-灭
		else if(n==2)
			lcd_write_1bit(0x0A,2,DISABLE);//-灭
	}
	aa=fabs(data);
	if(aa>=1000)
	{
		numsize=4;
		aa=fabs(data)*10;
	}
	else if(aa>=100)
	{
		numsize=3;
		aa=fabs(data*100);
		show_buf[2]=h;
	}
	else if(aa>=10)
	{
		numsize=2;
		aa=fabs(data*1000);
		show_buf[1]=h;
	}
	else// if(aa>1)//注释掉表示，无论9.999还是0.001都乘以1000
	{
		numsize=1;
		aa=fabs(data*10000);
		show_buf[0]=h;
	}
	
	float_table[0]=(u8)(aa/10000);//获取最高位
	aa%=10000;
	float_table[1]=(u8)(aa/1000);//获取次高位
	aa%=1000;
	float_table[2]=(u8)(aa/100);//......
	aa%=100;
	float_table[3]=(u8)(aa)/10;//获取最低位
	aa%=10;
	float_table[4]=(u8)(aa)/1;
	aa%=1;
	
	if(size!=10)
	{
		if(size==numsize)
		{
			if(float_table[4]>=5)
				float_table[3]++;
		}
		
		else if(size==(numsize+1))//得右移一位
		{
			if(float_table[3]>=5)
				float_table[2]++;
			for(i=3;i>0;i--)
			{
				float_table[i]=float_table[i-1];
				show_buf[i]=show_buf[i-1];
			}
			float_table[0]=0;show_buf[0]=0;
		}
		else if(size==(numsize+2))//得右移两位
		{
			if(float_table[2]>=5)
				float_table[1]++;
			for(i=3;i>1;i--)
			{
				float_table[i]=float_table[i-2];
				show_buf[i]=show_buf[i-2];
			}
			float_table[0]=0;show_buf[0]=0;
			float_table[1]=0;show_buf[1]=0;
		}
		else if(size==(numsize+3))//得右移三位
		{
			if(float_table[1]>=5)
				float_table[0]++;
			float_table[3]=float_table[0];
			float_table[0]=0;show_buf[0]=0;
			float_table[1]=0;show_buf[1]=0;
			float_table[2]=0;show_buf[2]=0;
		}
	}
	else
	{
		
	}
	
	for(i=3;i>0;i--)//遇到末尾是9，被进位成10的情况
	{
		if(float_table[i]==10)
		{
			float_table[i]=0;
			float_table[i-1]++;
		}
	}
	for(i=0;i<4;i++)
	{
		switch(float_table[i])//将十进制数据转换成对应的真值
		{
			case 0:show_buf[i]|=char_tab[0];break;
			case 1:show_buf[i]|=char_tab[1];break;
			case 2:show_buf[i]|=char_tab[2];break;
			case 3:show_buf[i]|=char_tab[3];break;
			case 4:show_buf[i]|=char_tab[4];break;
			case 5:show_buf[i]|=char_tab[5];break;
			case 6:show_buf[i]|=char_tab[6];break;
			case 7:show_buf[i]|=char_tab[7];break;
			case 8:show_buf[i]|=char_tab[8];break;
			case 9:show_buf[i]|=char_tab[9];break;
			default:break;
		}
	}
	
	if((size<numsize) || (float_table[0]==10))//当要显示的数据位数大于range，或者显示的第一位为10
	{
		show_buf[0]=0;
		show_buf[1]=0;
		show_buf[2]=OO;
		show_buf[3]=LL;
		if(size==1)
			show_buf[0]|=h;
		else if(size==2)
			show_buf[1]|=h;
		else if(size==3)
			show_buf[2]|=h;
	}
	
	if(size==4)
		show_buf[3] &= (~h);
	
	lcd_show_data(show_buf,n);
	
	if(size<numsize)
		return 0;
	else
		return 1;
}

void lcd_show_dta_num(float data,u8 n)
{
	u8 i;
	u8 show_buf[4]={0},float_table[5]={0};
	u32 aa;
	
	if(data<0)
	{
		if(n==1)
			lcd_write_1bit(0x0,2,ENABLE);//-亮
		else if(n==2)
			lcd_write_1bit(0x0A,2,ENABLE);//-亮
	}
	else
	{
		if(n==1)
			lcd_write_1bit(0x0,2,DISABLE);//-灭
		else if(n==2)
			lcd_write_1bit(0x0A,2,DISABLE);//-灭
	}
	aa=fabs(data);
//	if(aa>=1000)
//	{
//		aa=fabs(data);
//	}
//	else if(aa>=100)
//	{
//		aa=fabs(data*10);
//	}
//	else if(aa>=10)
//	{
//		aa=fabs(data*100);
//	}
//	else// if(aa>1)//注释掉表示，无论9.999还是0.001都乘以1000
//	{
//		aa=fabs(data*1000);
//	}
	
	float_table[0]=(u8)(aa/1000);//获取最高位
	aa%=1000;
	float_table[1]=(u8)(aa/100);//获取次高位
	aa%=100;
	float_table[2]=(u8)(aa/10);//......
	aa%=10;
	float_table[3]=(u8)(aa);//获取最低位
	aa%=1;
	
	for(i=0;i<4;i++)
	{
		switch(float_table[i])//将十进制数据转换成对应的真值
		{
			case 0:show_buf[i]|=char_tab[0];break;
			case 1:show_buf[i]|=char_tab[1];break;
			case 2:show_buf[i]|=char_tab[2];break;
			case 3:show_buf[i]|=char_tab[3];break;
			case 4:show_buf[i]|=char_tab[4];break;
			case 5:show_buf[i]|=char_tab[5];break;
			case 6:show_buf[i]|=char_tab[6];break;
			case 7:show_buf[i]|=char_tab[7];break;
			case 8:show_buf[i]|=char_tab[8];break;
			case 9:show_buf[i]|=char_tab[9];break;
			default:break;
		}
	}
	
	if(data>9999)
		lcd_show_OL(n);
	else
		lcd_show_data(show_buf,n);
}

void lcd_show_OL(u8 n)
{
	u8 show_buf[4]={0,0,OO,LL};
	lcd_show_data(show_buf,n);
}
void lcd_show_Erro(u8 n)
{
	u8 show_buf[4]={EE,RR,RR,O};
	lcd_show_data(show_buf,n);
}
void lcd_show_None(u8 n)
{
	u8 show_buf[4]={0,0,0,0};
	lcd_show_data(show_buf,n);
}
void lcd_show_Line(u8 n)
{
	u8 show_buf[4]={g,g,g,g};
	lcd_show_data(show_buf,n);
}
void lcd_show_A___(u8 n)
{
	u8 show_buf[4]={AA,g,g,g};
	lcd_show_data(show_buf,n);
}
void lcd_show_A__A(u8 n)
{
	u8 show_buf[4]={AA,g,g,AA};
	lcd_show_data(show_buf,n);
}
void lcd_show_b__b(u8 n)
{
	u8 show_buf[4]={BB,g,g,BB};
	lcd_show_data(show_buf,n);
}
void lcd_show_C__C(u8 n)
{
	u8 show_buf[4]={CC,g,g,CC};
	lcd_show_data(show_buf,n);
}
void lcd_show_____(u8 n)
{
	u8 show_buf[4]={g,g,g,g};
	lcd_show_data(show_buf,n);
}
void lcd_show_Cal(u8 n)
{
	u8 show_buf[3]={CC,AA,LL};
	lcd_show_data(show_buf,n);
}
/***************************************************end file*************************************************/

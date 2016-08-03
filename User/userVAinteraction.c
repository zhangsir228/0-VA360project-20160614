#include "matrixkey.h"
#include "dta0660.h"
#include "userVAinteraction.h"
#include "stmdataprocess.h"
#include "nvic_systick.h"
#include "ht1621.h"
#include "nvic_systick.h"
#include "stdlib.h"
#include "math.h"
#include "timer.h"
#include "comp.h"
#include "powercontrol.h"
#include "buzzer_led.h"
#include "main.h"
#include "sdadc.h"
#include "flash_data.h"
SoftKeystatus funcstatus=state0,btstatus=state0;			//子功能档位状态，蓝牙开关状态
SoftKeystatus max_minstatus=state0,peakstatus=state0;	//最值	，	峰值
SoftKeystatus relstatus=state0,phasestatus=state0;		//			，	相序功能
SoftKeystatus rangestatus=state0,paramstatus=state0;	//量程	，	param按键功能状态
SoftKeystatus holdstatus=state0,lightstatus=state0;		//hold 	，	背光
SoftKeystatus longparamstatus=state0,autostatus=state1;//InRush , 自动量程
SoftKeystatus phaseABCstatus=state0;						//相序测量功能进行到的步奏。


defSysflag Sysflag;//=(defSysflag){.Calonce = 0,.DTArange=0,.rangestatus=0}


extern defSysValue SysValue ;//系统运行时的主要变量参数
extern defFlashCal SaveData;//保存于flash中的必要数据

extern __IO uint16_t CCR4_Val;//TIM58 周期定时脉冲个数

u8 blink;
float max_data1,min_data1,max_data2,min_data2,rel_data1,rel_data2;
float abcde=2.15;//假装的测量值   呵呵
uint32_t TphaseTime=0;

/*三相功率测量时记录值*/
float apparent_power1=0,active_power1=0,reactive_power1=0;//视在功率、有功功率、无功功率
float apparent_power2=0,active_power2=0,reactive_power2=0;//视在功率、有功功率、无功功率
float apparent_power3=0,active_power3=0,reactive_power3=0;//视在功率、有功功率、无功功率
float apparent_power_sum123=0,active_power_sum123=0,reactive_power_sum123=0;//视在功率、有功功率、无功功率

/*display函数用变量*/
u8 size;
u32 phasetimetemp=0;
boolean propershow=0;
float showdata1,showdata2,read_dta_data,read_dta_data1,readstmdata1,readstmdata2,temp_for_over_voltage,abs_readstmdata1;
float readstmdata2_temp;
u8 delay_count;
u8 rangenum;

/*20160727 lea 因为有功能按键时下面的Data_init函数会清空电压电流数值所以定义几个临时变量来传递这几个值，目前只用于REL按键功能电流电压的相对值 基础值*/
float REL_voltage_effective=0;
float REL_voltage_mean=0;
float REL_current_effective=0;
float REL_current_mean=0;

/*20160801 新增全局各档位最终值变量*/
float  	Hz_value=0;
float	VmV_value=0;
float 	ohm_value=0;
float	Cap_value=0;
float	temp_value=0;


void Data_init(void)
{
	REL_voltage_effective=voltage_effective;
	REL_voltage_mean=voltage_mean;
	REL_current_effective=current_effective;
	REL_current_mean=current_mean;
	
	voltage_effective=0;
	voltage_mean=0;
	current_effective=0;
	current_mean=0;

	temp_for_over_voltage=0;

	readstmdata1=0;
	readstmdata2=0;
	showdata1=0;
	showdata2=0;
}

/*STM32外设、DTA0660电源开关的使能和失能——旋转开关改变时调用*/
void MeasureModeChange(void)
{
		if(RotaryKeyValue <= 0x07)//STM32测量，开启STM32相应外设
		{
			Power_On_Voltage();//PA5=0
			Power_On_Current();//PC15=1
			
			SDADC_Cmd(SDADC1, ENABLE);
			SDADC_Cmd(SDADC2, ENABLE);
			TIM_Cmd(TIM19, ENABLE);
//			COMP_Cmd(COMP_Selection_COMP1, DISABLE);//2015-4-28注释，将ENABLE改为DISABLE,在有PHASE测量功能时打开
//			COMP_Cmd(COMP_Selection_COMP2, DISABLE);
//			TIM_Cmd(TIM5, DISABLE);//输入捕获定时器
//			TIM_Cmd(TIM2, DISABLE);//2015-4-28注释，将ENABLE改为DISABLE,在有PHASE测量功能时打开
		}
		else if((RotaryKeyValue >= 0x08) && (RotaryKeyValue != KEY_NULL))//DTA0660测量，关闭STM32相应外设
		{
			Power_Off_Voltage();//PA5=1
			Power_Off_Current();//PC15=0
			
			SDADC_Cmd(SDADC1, DISABLE);
			SDADC_Cmd(SDADC2, DISABLE);
			TIM_Cmd(TIM19, DISABLE);//采样时钟
//			COMP_Cmd(COMP_Selection_COMP1, DISABLE);//比较器
//			COMP_Cmd(COMP_Selection_COMP2, DISABLE);
			TIM_Cmd(TIM5, DISABLE);//输入捕获定时器
//			TIM_Cmd(TIM2, DISABLE);
		}
}

/*测量功能选择——当短按键Func时，DTA0660测量功能改变 ，在旋钮和按键功能检测中调用*/
void MeasureFunctionSelection(void)
{
	switch(RotaryKeyValue)
	{
		case KEY_VALUE_6://DC/ACV
		{
			switch(funcstatus)
			{
				case state0://处理V~——ACV
					FunctionSet(ACV);//设定功能：交流电压
				break;
				
				case state1://处理V-——DCV
					FunctionSet(DCV);//设定功能：直流电压
				break;
				
				case state2://处理V~V-——ACV
					FunctionSet(ACV);//设定功能：交流电压
				break;
				
				default:break;
			}
		}
		break;
		
		case KEY_VALUE_7://W
		{
			FunctionSet(ACV);//设定功能：交流电压
		}
		break;
		
		case KEY_VALUE_8:
		{
			switch(funcstatus)
			{
				case state0://处理V-——DCVmV
					FunctionSet(DCVmV);//设定功能：直流电压
				break;
				
				case state1://处理V~——ACV
				{
					if(paramstatus == state0)
						FunctionSet(ACV);//设定功能：交流电压
					else//Hz_Duty
						FunctionSet(Hz_Duty);//设定功能：Hz_Duty
				}
				break;
				
				default:break;
			}
		}
		break;
		
		case KEY_VALUE_9:
		{
			switch(funcstatus)
			{
				case state0://处理Cont
					FunctionSet(Cont);//设定功能：通断
				break;
				
				case state1://处理Diode
					FunctionSet(Diode);//设定功能：二极管
				break;
				
				default:break;
			}
		}
		break;
		
		case KEY_VALUE_10:
		{
			switch(funcstatus)
			{
				case state0://处理Ohm
					FunctionSet(Ohm);//设定功能：Ohm
				break;
				
				case state1://处理Cap
					FunctionSet(Cap);//设定功能：Cap
				break;
				
				case state2://处理Temp(C)
					FunctionSet(Temp);//设定功能：Temp
				break;
				
				case state3://处理Temp(F)
					FunctionSet(Temp);//设定功能：Temp
				break;
				
				default:break;
			}
		}
		break;
		
		default:break;
	}
	MeasureModeChange();//区分主要由373完成还是由DTA0660完成的档位
}

/*根据相应功能档配置DTA0660,并做当前档位的数据处理。*/
void DataProcessing(void)
{

	if(holdstatus==state0)
	{
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://旋钮VA档:
			{
				rangenum = ReadDTAValue(ReadRange);
				receive_f= ReadDTAValue(ReadResult);	
				
				/*进行DC、AC电压电流计算，THD、CF、Inrush、Peak+-、Phase运算*/
				dealwith_information();
			}break;
			case KEY_VALUE_7://旋钮W档：
			{
				rangenum = ReadDTAValue(ReadRange);
				receive_f= ReadDTAValue(ReadResult);
							
				/*进行功率、多相功率、PF、DPF、THD、KWh、KVAh、Kvarh、KgCO2运算*/
				dealwith_information();
			}break;
			case KEY_VALUE_8://旋钮DCV/ACV档：
			{
				if(paramstatus == state0)
				{//在DCV/ACV档
					/*SDADC不工作，DTA0660工作，进行交流电压、直流电压、毫伏电压测量
					旋转开关需要将硬件电路开关的相应的电压档闭合*/
					receive_f= ReadDTAValue(ReadResult);
				}
				else if(paramstatus == state1)
				{//Hz档
					receive_f= ReadDTAValue(ReadFreq);
				}
				else if(paramstatus == state2)
				{//Duty档
					receive_f1= ReadDTAValue(ReadDuty);
				}
			}break;
			case KEY_VALUE_9://旋钮通断、二极管档：
			{
				/*SDADC不工作，DTA0660工作*/
				switch(funcstatus)
				{
					case state0://处理Cont
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state1://处理Diode
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_10://旋钮欧姆、电容、温度、频率档：
			{
			/*SDADC不工作，DTA0660工作*/
				switch(funcstatus)
				{
					case state0://处理Ohm
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state1://处理Cap
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state2://处理Temp(C)
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state3://处理Temp(F)
					{
						receive_f= ReadDTAValue(ReadResult)*1.8f+32;
					}break;
					default:break;
				}
			}
			break;
			default:break;
		}
	}
}

void display_bt_state(void)/*显示bluetooth状态*/
{
	switch(btstatus)
	{
		case state0:lcd_write_1bit(0x1D,2,DISABLE);break;//BT灭
		case state1:lcd_write_1bit(0x1D,2,ENABLE);break;//BT亮
		default:break;
	}
}

void display_max_min_state(void)/*显示MAX-MIN状态*/
{
	switch(max_minstatus)
	{
		case state0://显示正常值
		{
			lcd_write_1bit(0x0A,0,DISABLE);
			lcd_write_1bit(0x0A,1,DISABLE);
		}break;
		case state1://显示MAX
		{
			lcd_write_1bit(0x0A,0,DISABLE);
			lcd_write_1bit(0x0A,1,ENABLE);
		}break;
		case state2://显示MIN
		{
			lcd_write_1bit(0x0A,0,ENABLE);
			lcd_write_1bit(0x0A,1,DISABLE);
		}break;
		default:break;
	}
}

void display_rel_state(void)/*显示REL状态*/
{
	switch(relstatus)
	{
		case state0://显示正常值
		{
			lcd_write_1bit(0x09,0,DISABLE);
		}break;
		case state1://显示相对值
		{
			lcd_write_1bit(0x09,0,ENABLE);
		}break;
		default:break;
	}
}

void display_range_state(void)/*显示量程*/
{
	if(autostatus == state1)
	{
		lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
		lcd_write_1bit(0x1C,3,DISABLE);//MANU灭
	}
	else
	{
		lcd_write_1bit(0x1C,2,DISABLE);//AUTO灭
		lcd_write_1bit(0x1C,3,ENABLE);//MANU亮
	}
}

float Calculate_max_min_data(float data,u8 n)/*计算、记录最大最小值   n:1 or 2，显示区域，第一行 or 第二行*/
{
	if(n==1)
	{
		switch(max_minstatus)
		{
			case state0://正常值
			{
			}break;
			case state1://MAX
			{
				if(data>max_data1)
					max_data1=data;
				return max_data1;
			}break;
			case state2://MIN
			{
				if(data<min_data1)
					min_data1=data;
				return min_data1;
			}break;
			default:break;
		}
	}
	else if(n==2)
	{
		switch(max_minstatus)
		{
			case state0://正常值
			{
			}break;
			case state1://MAX
			{
				if(data>max_data2)
					max_data2=data;
				return max_data2;
			}break;
			case state2://MIN
			{
				if(data<min_data2)
					min_data2=data;
				return min_data2;
			}break;
			default:break;
		}
	}
	return data;
}

float Calculate_rel_data(float data,u8 n)/*计算、记录相对值   n:1 or 2，显示区域，第一行 or 第二行*/
{
	if(n==1)
		return(data-rel_data1);
	else if(n==2)
		return(data-rel_data2);
}

float deal_1(float read_data,u8 n)/*处理数据---MAX-MIN、REL*/
{
	if(max_minstatus != state0)
	{
		return(Calculate_max_min_data(read_data,n));
	}
	else if(relstatus != state0)
	{
		return(Calculate_rel_data(read_data,n));
	}
	else
		return read_data;
}

float deal_k(float read_data,u8 n)
{
	return(deal_1(read_data,n)/1000);
}

float deal_M(float read_data,u8 n)
{
	return(deal_k(read_data,n)/1000);
}

void LowPowerDetect(void)/*开始时认为：低电压位在内部RAM的LCD的BUFFER：0x41H地址的第低4位——0000,1000*/
{//20160312-2257发现：0x42H的最低位才是低电压显示位
	Send(SetAddr,0x42,0x00,SetAddr+0x42);//定位到LCD的BUFFER：0x42H——最后检测发现是分压电阻选择不对，只有当电压>9.4V时，MES_BAT才会小于1.2V
	check_flag= CheckReceive(SetAddr);
	if(check_flag==1)//定位到0x42H成功
	{
		check_flag= 0;
		Send(Read,Read);
		check_flag= CheckReceive(Read);
		if(check_flag==1)//读当前地址0x42H成功
		{
			check_flag= 0;
			
//			lcd_clr();//20160312时检测是否收到LCD的Buffer时暂时写就
//			lcd_show_dta_num((RxBuffer[1] & 0xF0)>>4,1);
//			lcd_show_dta_num((RxBuffer[1] & 0x0F),2);
			
			if((RxBuffer[1] & 0x01) != 0)
				lcd_write_1bit(0x1D,3,ENABLE);//电池亮
			else
				lcd_write_1bit(0x1D,3,DISABLE);//电池灭
		}
	}
}

/*旋钮、按钮相关操作*/
void manipulate(void)
{
	u8 i;
	/*当有旋转开关改变时*/
	if(RotaryKeyChanged_flag == 1)
	{
		Data_init();
		RotaryKeyChanged_flag = 0;
		BUZZER_Open(0);
		
		funcstatus=state0;
//		btstatus=state0;
		max_minstatus=state0;
		peakstatus=state0;
		relstatus=state0;
		phasestatus=state0;
		rangestatus=state0;
		paramstatus=state0;
		longparamstatus=state0;
		holdstatus=state0;
//		lightstatus=state0;
		
		for(i=0;i<16;i++)
			lcd_show_ram[i]=0x00;
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://V+A 
			{
				lcd_show_ram[14]=0x24;//AC~亮//AUTO亮
				lcd_show_ram[12]=0x08;//V（3）亮
				lcd_show_ram[10]=0x80;//A（2）亮
			}break;
			case KEY_VALUE_7://W
			{
				lcd_show_ram[14]=0x04;//AUTO亮
//				lcd_show_ram[12]=0x10;//W亮
				lcd_show_ram[13]=0x11;//VA（1）亮
				lcd_show_ram[11]=0x04;//PF亮
			}break;
			case KEY_VALUE_8://mV/V
			{
				lcd_show_ram[14]=0x05;//DC-亮//AUTO亮
				lcd_show_ram[12]=0x08;//V（3）亮
			}break;
			case KEY_VALUE_9://CON/DIODE
			{
				lcd_show_ram[0]=0x01;//声音亮
				lcd_show_ram[12]=0x20;//欧姆亮
			}break;
			case KEY_VALUE_10://OHM/CAP/TMP
			{
				lcd_show_ram[12]=0x20;//欧姆亮
				lcd_show_ram[14]=0x04;//AUTO亮
			}break;
			default:break;
		}
		lcd_full_ram();
		display_bt_state();
		MeasureFunctionSelection();
	}
	/*当有软按钮改变时*/
	if(SoftKeyChanged_flag == 1)//当有软按钮改变
	{
		Data_init();
		SoftKeyChanged_flag = 0;
		BUZZER_Open(0);
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://V+A 档位
			{
				if((ShortKey_flag == 1) && (longparamstatus == state1) && (SoftKeyValue != KEY_VALUE_3) && (SoftKeyValue != KEY_VALUE_5) && (holdstatus==state0))
				{//当之前处于Inrush状态，短按任意其他键，退出此状态
					ShortKey_flag=0;
					longparamstatus=state0;
					
					lcd_clr();
					
					lcd_write_1bit(0x1D,1,ENABLE);//AC亮
					lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
					lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
					lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
					inrush_trigger_flag=0;
				}
				switch(SoftKeyValue)//判断是什么软按钮
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (paramstatus == state0) && (holdstatus==state0))
						{//在FUNC出现短按，并且处于非Peak、Phase、Param、HOLD状态下
//							ShortKey_flag=0;//后面改变功能要用
							
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							rangestatus=state0;
							longparamstatus=state0;
							autostatus=state1;
							lcd_clr();
							switch(funcstatus)//必须要先改变FUNC键状态，再根据新状态运行
							{
								case state0:funcstatus = state1;break;//AC V+A		变为DC V+A
								case state1:funcstatus = state2;break;//DC V+A		变为AC+DC V+A
								case state2:funcstatus = state0;break;//AC+DC V+A	变为AC V+A
								default:break;
							}
							
							switch(funcstatus)//显示FUNC
							{
								case state0://处理AC V+A
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC亮
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
									lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
								}break;
								case state1://处理DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC亮
									lcd_write_1bit(0x1D,1,DISABLE);//AC灭
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
									lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
								}break;
								case state2://处理AC+DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC亮
									lcd_write_1bit(0x1D,0,ENABLE);//+亮
									lcd_write_1bit(0x1D,1,ENABLE);//AC亮
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
									lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
								}break;
								default:break;
							}
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//在MAX-MIN出现短按，并且处于非Peak、Phase、HOLD状态下
							ShortKey_flag=0;
							relstatus=state0;
							longparamstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//正常值变为MAX
								case state1:max_minstatus = state2;break;//MAX变为MIN
								case state2:max_minstatus = state0;break;//MIN变为正常值
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{//当按下MAX-MIN键，进入MAX-MIN状态，立即记录下此时的值
								if(funcstatus ==state0)//AC V+A
								{
									max_data1=voltage_effective-voltage_mean;
									min_data1=max_data1;
									
									max_data2=current_effective-current_mean;
									min_data2=max_data2;
								}
								else if(funcstatus ==state1)//DC V+A
								{
									max_data1=voltage_mean;
									min_data1=max_data1;
									
									max_data2=current_mean;
									min_data2=max_data2;
								}
								else if(funcstatus ==state2)//AC+DC V+A
								{
									max_data1=voltage_effective;
									min_data1=max_data1;
									
									max_data2=current_effective;
									min_data2=max_data2;
								}
							}
						}
						if((ShortKey_flag == 1) && (peakstatus != state0)&& (holdstatus==state0))
						{//在MAX-MIN出现短按，并且处于Peak状态下
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state1:max_minstatus = state2;break;//Peak-MAX变为Peak-MIN
								case state2:max_minstatus = state1;break;//Peak-MIN变为Peak-MAX
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
						}
						else if((LongKey_flag == 1) && (phasestatus == state0) && (paramstatus == state0) && (longparamstatus==state0) && (holdstatus==state0))
						{//在MAX-MIN出现长按，并且处于非max-min、Phase、Param、LongParam、HOLD状态下
							LongKey_flag=0;
							relstatus=state0;
							
							if((max_minstatus==state0) && (peakstatus==state0))
							{
								peakstatus = state1;
								max_minstatus = state1;
								lcd_write_1bit(0x09,1,ENABLE);//PEAK亮
							}
							else if(peakstatus!=state0)
							{
								peakstatus = state0;
								max_minstatus = state0;
								lcd_write_1bit(0x09,1,DISABLE);//PEAK灭
							}
							display_max_min_state();
							display_rel_state();
							
							if(peakstatus == state1)
							{//当进入PEAK状态，立即记录下此时的值
								max_data1=maxv_value;
								min_data1=minv_value;
								
								max_data2=maxi_value;
								min_data2=mini_value;
							}
						}
					}break;
					case KEY_VALUE_2://REL    //20160727  lea  修改逻辑为按键后一直显示为相对值 而不是只显示一次。 
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//在REL出现短按，并且处于非Peak、Phase、HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							longparamstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//正常值变为相对值
								case state1:relstatus = state0;break;//相对值变为正常值
								default:break;
							}
							display_rel_state();
							display_max_min_state();	
							
							if(relstatus != state0)//在相对值状态读一个测量值   //20160727 lea 这里的测量值都使用的临时传递变量，其他档位的还没有做相应的修改，留意以下！！！
							{
								if(funcstatus ==state0)//AC V+A
								{
									rel_data1=REL_voltage_effective-fabs(REL_voltage_mean);
									rel_data2=REL_current_effective-fabs(REL_current_mean);
								}
								else if(funcstatus ==state1)//DC V+A
								{
									rel_data1=REL_voltage_mean;
									rel_data2=REL_current_mean;
								}
								else if(funcstatus ==state2)//AC+DC V+A
								{
									rel_data1=REL_voltage_effective;
									rel_data2=REL_current_effective;
								}								
							}
						}
						else if((LongKey_flag == 1) && (peakstatus==state0) && (paramstatus == state0) && (longparamstatus==state0) && (holdstatus==state0))
						{//在REL出现长按，并且处于非Peak、Param、LongParam、HOLD状态下
							LongKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							autostatus=state1;
							phaseABCstatus=state0;
							
							switch(phasestatus)
							{
								case state0:phasestatus = state1;break;//正常值变为Phase
								case state1:phasestatus = state0;break;//Phase变为正常值
								default:break;
							}
							
							switch(phasestatus)
							{
								case state0://正常值
								{
									lcd_write_1bit(0x1B,3,DISABLE);//L1-L2-L3
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
									lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
									TIM_Cmd(TIM14, DISABLE);//0.5s定时关闭
								}break;
								case state1://PHASE
								{
									lcd_write_1bit(0x0,2,DISABLE);//第一行负号灭
									lcd_write_1bit(0xA,2,DISABLE);//第二行负号灭
									lcd_write_1bit(0x1B,3,ENABLE);//L1-L2-L3
									lcd_write_1bit(0x18,3,DISABLE);//V（3）灭
									lcd_write_1bit(0x15,3,DISABLE);//A（2）灭
									lcd_show_None(1);
									lcd_show_None(2);
									TIM_Cmd(TIM14, ENABLE);//0.5s定时开启
									
									TIM5CH4_CAPTURE_STA=0;//定时器5捕获状态，未捕获到时为0，A相0x40，B or C相0x80
									
									COMP_Cmd(COMP_Selection_COMP1, DISABLE);//关闭比较器 20160603lea
									TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);//清比较器中断
									
									TIM_Cmd(TIM19, ENABLE);//20160602lea 进入phase 模式后比较器 用于触发采样相位时间  
																				//所以本来用来过零点触发AC采样的现在时直接开始采样，抖动会厉害一点
									
								}break;
								default:break;
							}
							
							display_rel_state();
							display_max_min_state();
							display_range_state();
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (paramstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//在RANGE出现短按，并且处于非Peak、Phase、Param、HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							
							if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
							{
								autostatus=state0;
								Send(SetRangeNext,SetRangeNext);
								check_flag= CheckReceive(SetRangeNext);
								if(check_flag==1) check_flag=0;
							}
							else
							{
								switch(rangestatus)
								{
									case state0:
									{
										rangestatus = state1;
										RangeSet(state1);
									}break;//6.000V/600.0A变为60.00V/600.0A
									case state1:
									{
										rangestatus = state2;
										RangeSet(state2);
									}break;//60.00V/600.0A变为600.0V/600.0A
									case state2:
									{
										rangestatus = state3;
										RangeSet(state0);
									}break;//600.0V/600.0A变为6.000V/2000A
									case state3:
									{
										rangestatus = state4;
										RangeSet(state1);
									}break;//6.000V/2000A变为60.00V/2000A
									case state4:
									{
										rangestatus = state5;
										RangeSet(state2);
									}break;//60.00V/2000A变为600.0V/2000A
									case state5:
									{
										rangestatus = state0;
										RangeSet(state0);
									}break;//600.0V/2000A变为6.000V/600.0A
									default:break;
								}
							}
							
							display_range_state();
							display_rel_state();
							display_max_min_state();
						}
						else if((ShortKey_flag == 1) && (longparamstatus == state1) && (holdstatus==state0))
						{//在RANGE出现短按，并且处于Inrush状态下
							ShortKey_flag=0;
							
							inrush_trigger_flag=0;//电流浪涌测量的量程改变时，清零触发标志位
							lcd_show_Line(1);
							
							switch(rangestatus)
							{
								case state0:rangestatus = state1;break;//600.0A	变为2000A
								case state1:rangestatus = state0;break;//2000A	变为600.0A
								default:break;
							}
							switch(rangestatus)
							{
								case state0:lcd_show_num(600,2,3);break;//600.0A
								case state1:lcd_show_num(2000,2,4);break;//2000A
								default:break;
							}
						}
						else if((LongKey_flag == 1) && (peakstatus==state0) && (holdstatus==state0))
						{//在RANGE出现长按，并且处于非Peak、HOLD状态下
							LongKey_flag=0;
							
							if(autostatus != state1)//不处于自动挡，即从手动挡换为自动挡
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
								lcd_write_1bit(0x1C,3,DISABLE);//MANU灭
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//在PARAM出现短按，并且处于非Peak、Phase、HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							rangestatus = state0;
							autostatus=state1;
							lcd_clr();
							switch(paramstatus)
							{
								case state0:paramstatus = state1;break;
								case state1:paramstatus = state2;break;
								case state2:paramstatus = state3;break;
								case state3:paramstatus = state0;break;
								default:break;
							}
							
							switch(paramstatus)
							{
								case state0://显示默认值
								{
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
									lcd_write_1bit(0x15,3,ENABLE);//A（2）亮
								}break;
								case state1://显示THD%f
								{
									lcd_write_1bit(0x13,0,ENABLE);//THD%
									lcd_write_1bit(0x15,0,ENABLE);//f
								}break;
								case state2://显示THD%r
								{
									lcd_write_1bit(0x13,0,ENABLE);//THD%
									lcd_write_1bit(0x14,0,ENABLE);//r
								}break;
								case state3://显示CF
								{
									lcd_write_1bit(0x16,1,ENABLE);//CF
								}break;
								default:break;
							}
							switch(funcstatus)//显示FUNC
							{
								case state0://处理AC V+A
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC亮
								}break;
								case state1://处理DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC亮
								}break;
								case state2://处理AC+DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC亮
									lcd_write_1bit(0x1D,0,ENABLE);//+亮
									lcd_write_1bit(0x1D,1,ENABLE);//AC亮
								}break;
								default:break;
							}
							display_max_min_state();
							display_rel_state();
							display_range_state();
						}
						else if(LongKey_flag == 1  && (peakstatus==state0) && (phasestatus == state0) && (paramstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//在PARAM出现长按，并且处于非Peak、Phase、Param、LongParam、HOLD状态下
							LongKey_flag=0;
							
							max_minstatus=state0;
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							longparamstatus = state1;
							autostatus=state1;
							lcd_clr();
							
							lcd_write_1bit(0x17,2,ENABLE);//Inrush亮
							lcd_write_1bit(0x1A,0,ENABLE);//A(1)亮
							lcd_write_1bit(0x15,3,ENABLE);//A(2)亮
							lcd_show_Line(1);
							lcd_show_num(600,2,3);
							
							display_rel_state();
							display_max_min_state();
							display_range_state();
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(Is_Cal_Mode == 1)//校准模式下hold作为校准功能相关
						{
							if(RotaryKeyValue==KEY_VALUE_6)//V+A 档位 
							{
								if(funcstatus ==state1)//DC
								{
									Sysflag.Calonce	=1;
									BUZZER_Open(0);delay_ms(1000);
									BUZZER_Open(0);delay_ms(1000);
								}
							}
						}
						else
						{
							if(ShortKey_flag == 1)
							{//在HOLD出现短按
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD解除
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://显示正常值
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://保持
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//在HOLD出现长按
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//亮灯
									case state1:lightstatus = state0;break;//灭灯
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://亮灯
									{
									}break;
									case state1://灭灯
									{
									}break;
									default:break;
								}
							}
						}
						
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_7://W
			{
				switch(SoftKeyValue)//判断是什么软按钮
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//在FUNC出现短按，并且处于非长按状态下
//							ShortKey_flag=0;//后面改变功能要用
							
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							rangestatus=state0;
//							paramstatus=state0;
							
//							lcd_clr();
							
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//VA变为W
								case state1:funcstatus = state2;break;//W变为var
								case state2:funcstatus = state0;break;//var变为VA
								default:break;
							}
							switch(funcstatus)
							{
								case state0://处理VA
								{
									lcd_write_1bit(0x17,1,DISABLE);//Var灭
									lcd_write_1bit(0x1A,0,ENABLE);//A（1）亮
									lcd_write_1bit(0x1B,0,ENABLE);//V（1）亮
								}break;
								case state1://处理W
								{
									lcd_write_1bit(0x1A,0,DISABLE);//A（1）灭
									lcd_write_1bit(0x1B,0,DISABLE);//V（1）灭
									lcd_write_1bit(0x19,0,ENABLE);//W亮
								}break;
								case state2://处理Var
								{
									lcd_write_1bit(0x19,0,DISABLE);//W灭
									lcd_write_1bit(0x17,1,ENABLE);//Var亮
								}break;
								default:break;
							}
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
						if((ShortKey_flag == 1) && (peakstatus != state0) && (holdstatus==state0))
						{//在FUNC出现短按，并且处于多相功率测量状态下
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//VA变为W
								case state1:funcstatus = state2;break;//W变为var
								case state2:funcstatus = state0;break;//var变为VA
								default:break;
							}
							switch(funcstatus)
							{
								case state0://处理VA
								{
									lcd_write_1bit(0x17,1,DISABLE);//Var灭
									lcd_write_1bit(0x1A,0,ENABLE);//A（1）亮
									lcd_write_1bit(0x1B,0,ENABLE);//V（1）亮
								}break;
								case state1://处理W
								{
									lcd_write_1bit(0x1A,0,DISABLE);//A（1）灭
									lcd_write_1bit(0x1B,0,DISABLE);//V（1）灭
									lcd_write_1bit(0x19,0,ENABLE);//W亮
								}break;
								case state2://处理Var
								{
									lcd_write_1bit(0x19,0,DISABLE);//W灭
									lcd_write_1bit(0x17,1,ENABLE);//Var亮
								}break;
								default:break;
							}
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (holdstatus==state0))
						{//在MAX-MIN出现短按，并且处于非长按状态下
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//正常值变为MAX
								case state1:max_minstatus = state2;break;//MAX变为MIN
								case state2:max_minstatus = state0;break;//MIN变为正常值
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{//当按下MAX-MIN键，进入MAX-MIN状态，立即记录下此时的值
								if(longparamstatus == state0)
								{
									switch(funcstatus)
									{
										case state0:
										{
											max_data1=apparent_power;
											min_data1=max_data1;
										}break;
										case state1:
										{
											max_data1=active_power;
											min_data1=max_data1;
										}break;
										case state2:
										{
											max_data1=reactive_power;
											min_data1=max_data1;
										}break;
										default:break;
									}
									switch(paramstatus)
									{
										case state0:
										{
											max_data2=power_factor;
											min_data2=max_data2;
										}break;
										case state1:
										{
											max_data2=d_power_factor;
											min_data2=max_data2;
										}break;
										case state2:
										{
											max_data2=THD_f_power;
											min_data2=max_data2;
										}break;
										case state3:
										{
											max_data2=THD_r_power;
											min_data2=max_data2;
										}break;
										default:break;
									}
								}
								else if(longparamstatus == state1)
								{
									switch(paramstatus)
									{
										case state0://KWh
										{
											max_data1=kWh;
											min_data1=max_data1;
										}break;
										case state1://KVAh
										{
											max_data1=kVAh;
											min_data1=max_data1;
										}break;
										case state2://Kvarh
										{
											max_data1=kVarh;
											min_data1=max_data1;
										}break;
										case state3://KgCO2
										{
											max_data1=KgCO2;
											min_data1=max_data1;
										}break;
										default:break;
									}
								}
							}
						}
						else if((LongKey_flag == 1) && (max_minstatus==state0) && (holdstatus==state0))
						{//在MAX-MIN出现长按，并且处于非其他长按状态下，多相功率测量功能切换
							LongKey_flag=0;
							relstatus=state0;
							paramstatus=state0;
//							funcstatus = state0;
							phasestatus = state0;
							lcd_clr();
							switch(peakstatus)//改变状态
							{
								case state0:peakstatus = state1;break;//正常模式变为多相功率模式
								case state1:peakstatus = state0;break;//多相功率模式变为正常模式
								default:break;
							}
							switch(peakstatus)//按状态显示
							{
								case state0://正常
								{
									switch(funcstatus)
									{
										case state0://处理VA
										{
											lcd_write_1bit(0x1A,0,ENABLE);//A（1）亮
											lcd_write_1bit(0x1B,0,ENABLE);//V（1）亮
										}break;
										case state1://处理W
										{
											lcd_write_1bit(0x19,0,ENABLE);//W亮
										}break;
										case state2://处理Var
										{
											lcd_write_1bit(0x17,1,ENABLE);//Var亮
										}break;
										default:break;
									}
									lcd_write_1bit(0x16,2,ENABLE);//PF亮
									TIM_Cmd(TIM14, DISABLE);//0.5s定时关闭
								}break;
								case state1://多相功率初始显示1phi
								{
									lcd_write_1bit(0x09,3,ENABLE);//1
									lcd_write_1bit(0x0B,0,ENABLE);//PHI
									switch(funcstatus)//根据当前测量单位，显示相应单位
									{
										case state0://处理VA
										{
											lcd_write_1bit(0x1A,0,ENABLE);//A（1）亮
											lcd_write_1bit(0x1B,0,ENABLE);//V（1）亮
										}break;
										case state1://处理W
										{
											lcd_write_1bit(0x19,0,ENABLE);//W亮
										}break;
										case state2://处理Var
										{
											lcd_write_1bit(0x17,1,ENABLE);//Var亮
										}break;
										default:break;
									}
									TIM_Cmd(TIM14, ENABLE);//0.5s定时开启
								}break;
								default:break;
							}
							display_rel_state();
							display_range_state();
						}
					}break;
					case KEY_VALUE_2://REL
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (holdstatus==state0))
						{//在REL出现短按，并且处于非长按状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//正常值变为相对值
								case state1:relstatus = state0;break;//相对值变为正常值
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//在相对值状态读一个测量值
							{
								if(longparamstatus == state0)
								{
									switch(funcstatus)
									{
										case state0:rel_data1=apparent_power;break;
										case state1:rel_data1=active_power;break;
										case state2:rel_data1=reactive_power;break;
										default:break;
									}
									switch(paramstatus)
									{
										case state0:rel_data2=power_factor;break;
										case state1:rel_data2=d_power_factor;break;
										case state2:rel_data2=THD_f_power;break;
										case state3:rel_data2=THD_r_power;break;
										default:break;
									}
								}
								else if(longparamstatus == state1)
								{
									switch(paramstatus)
									{
										case state0:rel_data1=kWh;break;//KWh
										case state1:rel_data1=kVAh;break;//KVAh
										case state2:rel_data1=kVarh;break;//Kvarh
										case state3:rel_data1=KgCO2;break;//KgCO2
										default:break;
									}
								}
							}
						}
						if((ShortKey_flag == 1) && (peakstatus == state1) && (holdstatus==state0))
						{//在REL出现短按，并且处于多相功率测量功能状态下
							switch(phasestatus)
							{//动作切换功能，并记录下当前功率值
								case state0:
								{
									phasestatus=state1;
									
									apparent_power1=apparent_power;
									active_power1=active_power;
									reactive_power1=reactive_power;
								}break;
								case state1:
								{
									phasestatus=state2;
									
									apparent_power2=apparent_power;
									active_power2=active_power;
									reactive_power2=reactive_power;
								}break;
								case state2:
								{
									phasestatus=state3;
									
									apparent_power3=apparent_power;
									active_power3=active_power;
									reactive_power3=reactive_power;
									
									apparent_power_sum123=apparent_power1+apparent_power2+apparent_power3;
									active_power_sum123=active_power1+active_power2+active_power3;
									reactive_power_sum123=reactive_power1+reactive_power2+reactive_power3;
								}break;
								case state3:phasestatus=state4;break;
								case state4:phasestatus=state5;break;
								case state5:phasestatus=state6;break;
								case state6:phasestatus=state3;break;
								default:break;
							}
							switch(phasestatus)
							{
								case state0://1phi闪
								{
//									lcd_write_1bit(0x09,3,ENABLE);//1，闪烁
								}
								break;
								case state1://2phi闪
								{
									lcd_write_1bit(0x09,3,DISABLE);//1灭
//									lcd_write_1bit(0x0A,3,ENABLE);//2，闪烁
								}
								break;
								case state2://3phi闪
								{
									lcd_write_1bit(0x0A,3,DISABLE);//2灭
//									lcd_write_1bit(0x0B,1,ENABLE);//3，闪烁
								}
								break;
								case state3://1phi不闪
								{
									lcd_write_1bit(0x09,2,DISABLE);//Σ灭
									lcd_write_1bit(0x0B,1,DISABLE);//3灭
									lcd_write_1bit(0x09,3,ENABLE);//1亮
									lcd_write_1bit(0x0B,0,ENABLE);//PHI亮
									TIM_Cmd(TIM14, DISABLE);//0.5s定时关闭
								}
								break;
								case state4://2phi不闪
								{
									lcd_write_1bit(0x09,3,DISABLE);//1灭
									lcd_write_1bit(0x0A,3,ENABLE);//2亮
								}
								break;
								case state5://3phi不闪
								{
									lcd_write_1bit(0x0A,3,DISABLE);//2灭
									lcd_write_1bit(0x0B,1,ENABLE);//3亮
								}
								break;
								case state6://sigma 3phi不闪
								{
									lcd_write_1bit(0x09,2,ENABLE);//Σ亮
//									lcd_write_1bit(0x0B,1,ENABLE);//3亮
								}
								break;
								default:break;
							}
						}
						else if((LongKey_flag == 1) && (peakstatus==state0) && (holdstatus==state0))
						{//在REL出现长按，并且处于非其他长按状态下
							LongKey_flag=0;
							
							{
								lcd_show_Erro(1);
								delay_ms(1000);
							}
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
					}break;
					case KEY_VALUE_4://PARAM
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//在PARAM出现短按，并且处于非长按状态下
							ShortKey_flag=0;
//							max_minstatus=state0;
//							relstatus=state0;
//							rangestatus = state0;
							
//							lcd_clr();
							
							switch(paramstatus)
							{
								case state0:paramstatus = state1;break;
								case state1:paramstatus = state2;break;
								case state2:paramstatus = state3;break;
								case state3:paramstatus = state0;break;
								default:break;
							}
							switch(paramstatus)
							{
								case state0://显示PF
								{
									lcd_write_1bit(0x13,0,DISABLE);//THD%灭
									lcd_write_1bit(0x14,0,DISABLE);//r灭
									lcd_write_1bit(0x16,2,ENABLE);//PF亮
								}break;
								case state1://显示D+PF
								{
									lcd_write_1bit(0x16,3,ENABLE);//D亮
//									lcd_write_1bit(0x16,2,ENABLE);//PF亮
								}break;
								case state2://显示THD%f
								{
									lcd_write_1bit(0x16,3,DISABLE);//D灭
									lcd_write_1bit(0x16,2,DISABLE);//PF灭
									lcd_write_1bit(0x13,0,ENABLE);//THD%亮
									lcd_write_1bit(0x15,0,ENABLE);//f亮
								}break;
								case state3://显示THD%r
								{
									lcd_write_1bit(0x15,0,DISABLE);//f灭
//									lcd_write_1bit(0x13,0,ENABLE);//THD%亮
									lcd_write_1bit(0x14,0,ENABLE);//r亮
								}break;
								default:break;
							}
//							display_max_min_state();
//							display_rel_state();
//							display_range_state();
						}
						else if((ShortKey_flag == 1) && (longparamstatus == state1) && (holdstatus==state0))
						{//在电能模式下，PARAM出现短按
							ShortKey_flag=0;
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							
//							lcd_clr();
							switch(paramstatus)
							{
								case state0:paramstatus = state1;break;//KWh变为KVAh
								case state1:paramstatus = state2;break;//KVAh变为Kvarh
								case state2:paramstatus = state3;break;//Kvarh变为KgCO2
								case state3:paramstatus = state0;break;//KgCO2变为KWh
								default:break;
							}
							switch(paramstatus)
							{
								case state0://处理KWh
								{
									lcd_write_1bit(0x08,0,ENABLE);//k(3)亮
									lcd_write_1bit(0x19,0,ENABLE);//W亮
									lcd_write_1bit(0x18,0,ENABLE);//h亮
									
									lcd_write_1bit(0x17,3,DISABLE);//KgCO2灭
								}break;
								case state1://处理KVAh
								{
//									lcd_write_1bit(0x08,0,ENABLE);//k(3)亮
									lcd_write_1bit(0x1A,0,ENABLE);//V（1）亮
									lcd_write_1bit(0x1B,0,ENABLE);//A（1）亮
//									lcd_write_1bit(0x18,0,ENABLE);//h亮
									
									lcd_write_1bit(0x19,0,DISABLE);//W灭
								}break;
								case state2://处理Kvarh
								{
									lcd_write_1bit(0x18,2,ENABLE);//k(1)亮
									lcd_write_1bit(0x17,1,ENABLE);//var亮
//									lcd_write_1bit(0x18,0,ENABLE);//h亮
									
									lcd_write_1bit(0x08,0,DISABLE);//k(3)灭
									lcd_write_1bit(0x1A,0,DISABLE);//V（1）灭
									lcd_write_1bit(0x1B,0,DISABLE);//A（1）灭
								}break;
								case state3://处理KgCO2
								{
									lcd_write_1bit(0x17,3,ENABLE);//KgCO2亮
									
									lcd_write_1bit(0x18,2,DISABLE);//k(1)灭
									lcd_write_1bit(0x17,1,DISABLE);//var灭
									lcd_write_1bit(0x18,0,DISABLE);//h灭
								}break;
								default:break;
							}
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
						else if((LongKey_flag == 1) && (peakstatus == state0) && (holdstatus==state0))
						{//在PARAM出现长按，并且处于非其他长按状态下
							LongKey_flag=0;
							funcstatus = state0;
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							
							powertimer=0;
							kWh=0;kVAh=0;kVarh=0;
							
							lcd_clr();
							switch(longparamstatus)
							{
								case state0:longparamstatus = state1;break;//正常模式变为电能模式
								case state1:longparamstatus = state0;break;//电能模式变为正常模式
								default:break;
							}
							switch(longparamstatus)
							{
								case state0://主显示总功、副显示PF
								{
									lcd_write_1bit(0x1A,0,ENABLE);//A（1）亮
									lcd_write_1bit(0x1B,0,ENABLE);//V（1）亮
									lcd_write_1bit(0x16,2,ENABLE);//PF亮
									
									TIM_Cmd(TIM12, DISABLE);
								}break;
								case state1://处理KWh
								{
									lcd_write_1bit(0x08,0,ENABLE);//k(3)亮
									lcd_write_1bit(0x19,0,ENABLE);//W亮
									lcd_write_1bit(0x18,0,ENABLE);//h亮
									
									TIM_Cmd(TIM12, ENABLE);/*1s定时*/
									
								}break;
								default:break;
							}
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(ShortKey_flag == 1)
						{//在HOLD出现短按
							ShortKey_flag=0;
							
							switch(holdstatus)
							{
								case state0:holdstatus = state1;break;//HOLD
								case state1:holdstatus = state0;break;//HOLD解除
								default:break;
							}
							
							switch(holdstatus)
							{
								case state0://显示正常值
								{
									lcd_write_1bit(0x1C,1,DISABLE);
								}break;
								case state1://保持
								{
									lcd_write_1bit(0x1C,1,ENABLE);
								}break;
								default:break;
							}
						}
						else if(LongKey_flag == 1)
						{//在HOLD出现长按
							LongKey_flag=0;
							
							switch(lightstatus)
							{
								case state0:lightstatus = state1;break;//亮灯
								case state1:lightstatus = state0;break;//灭灯
								default:break;
							}
							
							switch(lightstatus)
							{
								case state0://亮灯
								{
								}break;
								case state1://灭灯
								{
								}break;
								default:break;
							}
						}
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_8://DCV/ACV
			{
				switch(SoftKeyValue)//判断是什么软按钮
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (paramstatus == state0) && (holdstatus==state0))
						{//在FUNC出现短按，并且处于非Hz-duty/HOLD状态下
//							ShortKey_flag=0;//后面改变功能要用
							
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							rangestatus=state0;
							autostatus=state1;
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//DCV变为ACV
								case state1:funcstatus = state0;break;//ACV变为DCV
								default:break;
							}
							switch(funcstatus)
							{
								case state0://DCV
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC亮
									lcd_write_1bit(0x18,3,ENABLE);//V亮
								}break;
								case state1://ACV
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC亮
									lcd_write_1bit(0x18,3,ENABLE);//V亮
								}break;
								default:break;
							}
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在MAX-MIN出现短按，并且处于非HOLD状态下
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//正常值变为MAX
								case state1:max_minstatus = state2;break;//MAX变为MIN
								case state2:max_minstatus = state0;break;//MIN变为正常值
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)//当进入MAX-MIN状态，记录下此时的值为最大最小值
							{
								if(paramstatus != state2)
								{
									max_data1=receive_f;
									min_data1=max_data1;
								}
								else
								{
									max_data2=receive_f1;
									min_data2=max_data2;
								}
								
							}
						}
					}break;
					case KEY_VALUE_2://REL
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在REL出现短按，并且处于非HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//正常值变为相对值
								case state1:relstatus = state0;break;//相对值变为正常值
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//在相对值状态读一个测量值
								rel_data1 = receive_f;//ReadDTAValue(ReadFreq);//从DTA0660读取数据
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
						if((ShortKey_flag == 1) && (paramstatus == state0) && (holdstatus==state0))
						{//在RANGE出现短按，并且处于非Hz-duty/HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							switch(funcstatus)
							{
								case state0://DCV
								{
									if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
									{
										autostatus=state0;
										Send(SetRangeNext,SetRangeNext);
										check_flag= CheckReceive(SetRangeNext);
										if(check_flag==1) check_flag=0;
									}
									else
									{
										switch(rangestatus)
										{
											case state0://600.0mV变为6.000V
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://6.000V变为60.00V
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://60.00V变为600.0V
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://600.0V变为1000V
											{
												rangestatus = state4;
												RangeSet(state4);
											}break;
											case state4://1000V变为600.0mV
											{
												rangestatus = state0;
												RangeSet(state0);
											}break;
											default:break;
										}
									}
								}break;
								case state1://ACV
								{
									if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
									{
										autostatus=state0;
										Send(SetRangeNext,SetRangeNext);
										check_flag= CheckReceive(SetRangeNext);
										if(check_flag==1) check_flag=0;
									}
									else
									{
										switch(rangestatus)
										{
											case state0://6.000V变为60.00V
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://60.00V变为600.0V
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://600.0V变为1000V
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://1000V变为6.000V
											{
												rangestatus = state0;
												RangeSet(state0);
											}break;
											default:break;
										}
									}
								}break;
								default:break;
							}
							display_range_state();
							display_rel_state();
							display_max_min_state();
						}
						else if((ShortKey_flag == 1) && (paramstatus == state1) && (holdstatus==state0))
						{//在RANGE出现短按，并且处于Hz状态、非HOLD状态下
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							
							if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
							{
								autostatus=state0;
								Send(SetRangeNext,SetRangeNext);
								check_flag= CheckReceive(SetRangeNext);
								if(check_flag==1) check_flag=0;
							}
							else
							{
								switch(rangestatus)
								{
									case state0://99.99变为999.9
									{
										rangestatus = state1;
										RangeSet(state1);
									}break;
									case state1://999.9变为9.999k
									{
										rangestatus = state2;
										RangeSet(state2);
									}break;
									case state2://9.999k变为99.99k
									{
										rangestatus = state3;
										RangeSet(state3);
									}break;
									case state3://99.99k变为999.9k
									{
										rangestatus = state4;
										RangeSet(state4);
									}break;
									case state4://999.9k变为9.999M
									{
										rangestatus = state5;
										RangeSet(state5);
									}break;
									case state5://9.999M变为99.99
									{
										rangestatus = state0;
										RangeSet(state0);
									}break;
									default:break;
								}
							}
							
							display_range_state();
							display_rel_state();
							display_max_min_state();
						}
						else if((LongKey_flag == 1) && (holdstatus==state0))
						{//在RANGE出现长按，并且处于非HOLD状态下
							LongKey_flag=0;
							
							if(autostatus != state1)//不处于自动挡
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
								lcd_write_1bit(0x1C,3,DISABLE);//MANU灭
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
						if((ShortKey_flag == 1) && (funcstatus == state1) && (holdstatus==state0))//切换HZ测量
						{//在PARAM出现短按，并且处于ACV/非HOLD状态下
//							ShortKey_flag=0;//后面改变功能要用
							max_minstatus=state0;
							relstatus=state0;
							rangestatus = state0;
							lcd_clr();
							switch(paramstatus)
							{
								case state0:paramstatus = state1;break;
								case state1:paramstatus = state2;break;
								case state2:paramstatus = state0;break;
								default:break;
							}
							
							if(paramstatus == state0)
							{
								lcd_write_1bit(0x18,3,ENABLE);//V亮
//								lcd_write_1bit(0x14,1,DISABLE);//%灭
//								lcd_show_None(2);//副显示无
							}
							else if(paramstatus == state1)//频率档
							{
//								lcd_write_1bit(0x15,2,ENABLE);//Hz(2)亮
								lcd_write_1bit(0x18,1,ENABLE);//Hz(1)亮
							}
							else if(paramstatus == state2)//占空比档
							{
//								lcd_write_1bit(0x15,2,DISABLE);//Hz(2)灭
								lcd_write_1bit(0x14,1,ENABLE);//%亮
							}
							lcd_write_1bit(0x1D,1,ENABLE);//AC亮
							
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{	
						if(Is_Cal_Mode == 1)//校准模式下hold作为VmV校准功能
						{
							if(VmV_value<1000)//0 
							{
								SaveData.Value.Cal_VmV_zero[rangestatus]=VmV_value/SaveData.Value.Cal_VmV_gain[rangestatus]+SaveData.Value.Cal_VmV_zero[rangestatus];
							}
							else if(VmV_value>3500)//5000
							{
								SaveData.Value.Cal_VmV_gain[rangestatus]= 5000*SaveData.Value.Cal_VmV_gain[rangestatus]/VmV_value;
							}
							updata_flash();	//保存数据	
						}
						else//正常状态
						{
							if(ShortKey_flag == 1)
							{//在HOLD出现短按
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD解除
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://显示正常值
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://保持
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//在HOLD出现长按
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//亮灯
									case state1:lightstatus = state0;break;//灭灯
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://亮灯
									{
									}break;
									case state1://灭灯
									{
									}break;
									default:break;
								}
							}
						}												
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_9://CON/DIODE
			{
				switch(SoftKeyValue)//判断是什么软按钮
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在FUNC出现短按，且非HOLD状态
//							ShortKey_flag=0;//后面改变功能要用
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//CONT变为Diode
								case state1:funcstatus = state0;break;//Diode变为CONT
								default:break;
							}
							switch(funcstatus)
							{
								case state0://CONT
								{
									lcd_write_1bit(0x00,0,ENABLE);//声音亮
									lcd_write_1bit(0x19,1,ENABLE);//欧姆亮
								}break;
								case state1://Diode
								{
									lcd_write_1bit(0x00,1,ENABLE);//二极管亮
									lcd_write_1bit(0x18,3,ENABLE);//V（3）亮
								}break;
								default:break;
							}
							
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在MAX-MIN出现短按，且非HOLD状态
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//正常值变为MAX
								case state1:max_minstatus = state2;break;//MAX变为MIN
								case state2:max_minstatus = state0;break;//MIN变为正常值
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{
								max_data1=receive_f;//ReadDTAValue(ReadResult);
								min_data1=max_data1;
							}
						}
					}break;
					case KEY_VALUE_2://REL
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在REL出现短按，且非HOLD状态
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//正常值变为相对值
								case state1:relstatus = state0;break;//相对值变为正常值
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//在相对值状态读一个测量值
								rel_data1 = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
					}break;
					case KEY_VALUE_4://PARAM
					{
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(ShortKey_flag == 1)
						{//在HOLD出现短按
							ShortKey_flag=0;
							
							switch(holdstatus)
							{
								case state0:holdstatus = state1;break;//HOLD
								case state1:holdstatus = state0;break;//HOLD解除
								default:break;
							}
							
							switch(holdstatus)
							{
								case state0://显示正常值
								{
									lcd_write_1bit(0x1C,1,DISABLE);
								}break;
								case state1://保持
								{
									lcd_write_1bit(0x1C,1,ENABLE);
								}break;
								default:break;
							}
						}
						else if(LongKey_flag == 1)
						{//在HOLD出现长按
							LongKey_flag=0;
							
							switch(lightstatus)
							{
								case state0:lightstatus = state1;break;//亮灯
								case state1:lightstatus = state0;break;//灭灯
								default:break;
							}
							
							switch(lightstatus)
							{
								case state0://亮灯
								{
								}break;
								case state1://灭灯
								{
								}break;
								default:break;
							}
						}
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_10://OHM/CAP/TMP
			{
				switch(SoftKeyValue)//判断是什么软按钮
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在FUNC出现短按，且非HOLD状态
//							ShortKey_flag=0;//后面改变功能要用
							
							max_minstatus=state0;//优先级低的状态清零
							relstatus=state0;
							rangestatus=state0;
							autostatus=state1;
							
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//Ohm变为Cap
								case state1:funcstatus = state2;break;//Cap变为Temp(C)
								case state2:funcstatus = state3;break;//Temp(C)变为Temp(F)
								case state3:funcstatus = state0;break;//Temp(F)变为Ohm
								default:break;
							}
							switch(funcstatus)
							{
								case state0://Ohm
								{
//									lcd_clr();
									lcd_write_1bit(0x19,1,ENABLE);//欧姆亮
									lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
								}break;
								case state1://Cap
								{
									lcd_write_1bit(0x19,3,ENABLE);//法拉亮
								}break;
								case state2://Temp(C)
								{
									lcd_write_1bit(0x15,1,ENABLE);//温度(C)亮
								}break;
								case state3://Temp(F)
								{
									lcd_write_1bit(0x16,0,ENABLE);//温度(F)亮
								}break;
								default:break;
							}
							
							display_max_min_state();//显示MAX-MIN
							display_rel_state();//显示REL
							display_range_state();//显示RANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在MAX-MIN出现短按，且非HOLD状态
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//正常值变为MAX
								case state1:max_minstatus = state2;break;//MAX变为MIN
								case state2:max_minstatus = state0;break;//MIN变为正常值
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{
								max_data1=receive_f;
								min_data1=max_data1;
							}
						}
					}break;
					case KEY_VALUE_2://REL
					{
						if(Is_Cal_Mode == 1)//校准模式下REL作为Cap零点校准功能
						{
							switch(funcstatus)
							{
								case state0://Ohm
								{
									break;
								}
								case state1://Cap
								{																		
									switch(rangestatus)//电容值校准
									{										
										case 1://60.00nF  校准50nF
										{
											SaveData.Value.Cal_Cap_zero[0]=Cap_value/SaveData.Value.Cal_Cap_gain[0]+SaveData.Value.Cal_Cap_zero[0];										
											break;
										}										
										case 3://6.000uF 校准5uF
										{
											SaveData.Value.Cal_Cap_zero[1]=Cap_value/SaveData.Value.Cal_Cap_gain[1]+SaveData.Value.Cal_Cap_zero[1];
											break;
										}										
										case 5://600.0uF 校准500uF
										{
											SaveData.Value.Cal_Cap_zero[2]=Cap_value/SaveData.Value.Cal_Cap_gain[2]+SaveData.Value.Cal_Cap_zero[2];
											break;
										}
										case 6://6.000mF 校准5mF
										{
											SaveData.Value.Cal_Cap_zero[3]=Cap_value/SaveData.Value.Cal_Cap_gain[0]+SaveData.Value.Cal_Cap_zero[3];
											break;
										}																
										default:break;
									}
									updata_flash();	//保存数据	
									break;
								}
								
								default:break;								
							}
						}
						else if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在REL出现短按，且非HOLD状态
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//正常值变为相对值
								case state1:relstatus = state0;break;//相对值变为正常值
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//在相对值状态读一个测量值
								rel_data1 = receive_f;
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//在RANGE出现短按，且非HOLD状态
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							switch(funcstatus)
							{
								case state0://Ohm
								{
									if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
									{
										autostatus=state0;
										Send(SetRangeNext,SetRangeNext);
										check_flag= CheckReceive(SetRangeNext);
										if(check_flag==1) check_flag=0;
									}
									else
									{
										switch(rangestatus)
										{
											case state0://600.0Ω变为6.000kΩ
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://6.000kΩ变为60.00kΩ
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://60.00kΩ变为600.0kΩ
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://600.0kΩ变为6.000MΩ
											{
												rangestatus = state4;
												RangeSet(state4);
											}break;
											case state4://6.000MΩ变为60.00MΩ
											{
												rangestatus = state5;
												RangeSet(state5);
											}break;
											case state5://60.00MΩ变为600.0Ω
											{
												rangestatus = state0;
												RangeSet(state0);
											}break;
											default:break;
										}
									}
								}
								break;
								
								case state1://Cap
								{
									if(autostatus==state1)//若在自动挡，短按RANGE，则退出自动挡
									{
										autostatus=state0;
										Send(SetRangeNext,SetRangeNext);
										check_flag= CheckReceive(SetRangeNext);
										if(check_flag==1) check_flag=0;
									}
									else
									{
										switch(rangestatus)
										{
											case state0://6.000n变为60.00n
											{
												rangestatus = state1;
												RangeSet(state0);//0.000nF~999.999nF
											}break;
											case state1://60.00n变为600.0n
											{
												rangestatus = state2;
												RangeSet(state0);//0.000nF~999.999nF
											}break;
											case state2://600.0n变为6.000u
											{
												rangestatus = state3;
												RangeSet(state1);//0.000uF~9.999uF
											}break;
											case state3://6.000u变为60.00u
											{
												rangestatus = state4;
												RangeSet(state2);//0.00uF~999.99uF
											}break;
											case state4://60.00u变为600.0u
											{
												rangestatus = state5;
												RangeSet(state2);//0.00uF~999.99uF
											}break;
											case state5://600.0u变为6.000m
											{
												rangestatus = state6;
												RangeSet(state3);//0.000mF~99.999mF
											}break;
											case state6://6.000m变为60.00m
											{
												rangestatus = state7;
												RangeSet(state3);//0.000mF~99.999mF
											}break;
											case state7://60.00m变为6.000n
											{
												rangestatus = state0;
												RangeSet(state0);//0.000nF~999.999nF
											}break;
											default:break;
										}
									}
								}
								break;
								
								case state2://Temp(C)
								{
								}
								break;
								
								case state3://Temp(F)
								{
								}
								break;
								
								default:break;
							}
							display_range_state();
							display_rel_state();
							display_max_min_state();
						}
						else if((LongKey_flag == 1) && (holdstatus==state0))
						{//在RANGE出现长按，且非HOLD状态
							LongKey_flag=0;
							
							if(autostatus != state1)//不处于自动挡
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO亮
								lcd_write_1bit(0x1C,3,DISABLE);//MANU灭
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(Is_Cal_Mode == 1)//校准模式下hold作为ohm校准功能
						{
							switch(funcstatus)
							{
								case state0://Ohm
								{
									if(ohm_value<100)//0 Ω
									{
										SaveData.Value.Cal_ohm_zero[rangestatus]=ohm_value/SaveData.Value.Cal_ohm_gain[rangestatus]+SaveData.Value.Cal_ohm_zero[rangestatus];
									}
									else if(ohm_value>3500)//500 Ω
									{
										SaveData.Value.Cal_ohm_gain[rangestatus]= 5000*SaveData.Value.Cal_ohm_gain[rangestatus]/ohm_value;
									}
									updata_flash();	//保存数据	
									break;
								}
								case state1://Cap 校准
								{
									switch(rangestatus)//电容值校准
									{										
										case 1://60.00nF  校准50nF
										{
											if(Cap_value>30)
											{
												SaveData.Value.Cal_Cap_gain[0]= 50*SaveData.Value.Cal_Cap_gain[0]/Cap_value;
											}											
											break;
										}										
										case 3://6.000uF 校准5uF
										{
											if(Cap_value>3000)
											{
												SaveData.Value.Cal_Cap_gain[1]= 5000*SaveData.Value.Cal_Cap_gain[1]/Cap_value;
											}
											break;
										}										
										case 5://600.0uF 校准500uF
										{
											if(Cap_value>300000)
											{
												SaveData.Value.Cal_Cap_gain[2]= 500000*SaveData.Value.Cal_Cap_gain[2]/Cap_value;
											}
											break;
										}
										case 6://6.000mF 校准5mF
										{
											if(Cap_value>3000000)
											{
												SaveData.Value.Cal_Cap_gain[3]= 5000000*SaveData.Value.Cal_Cap_gain[3]/Cap_value;
											}
											break;
										}																
										default:break;
									}
									updata_flash();	//保存数据										
									break;
								}
								case state2://Temp(C)
								{
									break;
								}
								case state3://Temp(F)
								{
									break;
								}
								
								default:break;
							}							
							
						}
						else //正常状态下
						{
							if(ShortKey_flag == 1)
							{//在HOLD出现短按
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD解除
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://显示正常值
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://保持
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//在HOLD出现长按
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//亮灯
									case state1:lightstatus = state0;break;//灭灯
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://亮灯
									{
									}break;
									case state1://灭灯
									{
									}break;
									default:break;
								}
							}
						}
						
					}break;
					default:break;
				}
			}break;
			default:break;
		}
		
		if((SoftKeyValue==KEY_VALUE_0) && (LongKey_flag == 1) && (holdstatus==state0))
		{//对所有旋转按钮情况下都是：在FUNC出现长按，且非HOLD状态，BT电源控制
			LongKey_flag=0;
			
			switch(btstatus)
			{
				case state0:btstatus = state1;break;//正常模式变为BT模式
				case state1:btstatus = state0;break;//BT模式变为正常模式
				default:break;
			}
			
			switch(btstatus)//开启或关闭BT电源
			{
				case state0:Power_Off_Bt();break;
				case state1:Power_On_Bt();break;
				default:break;
			}
		}
		display_bt_state();
		
		if((SoftKeyValue == KEY_VALUE_0) && (ShortKey_flag == 1) && (holdstatus==state0))//在短按Func时，改变测量功能
		{
			ShortKey_flag=0;
			if((RotaryKeyValue == KEY_VALUE_8) && (paramstatus != state0));//当在测量频率、占空比档位时，按Func键无用
			else
			MeasureFunctionSelection();
		}
		else if((RotaryKeyValue == KEY_VALUE_8) && (SoftKeyValue == KEY_VALUE_4) && (ShortKey_flag == 1) && (funcstatus == state1) && (holdstatus==state0))
		{
			ShortKey_flag=0;
			MeasureFunctionSelection();
		}
	}
}

/*显示*/
void display(void)
{
	if(holdstatus==state0)
	{
		LowPowerDetect();//低电压检测
		
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://V+A 档位
			{
				temp_for_over_voltage=voltage_effective;
				if(temp_for_over_voltage>30)
					lcd_write_1bit(0x0,3,ENABLE);//闪电符号亮
				else
					lcd_write_1bit(0x0,3,DISABLE);//闪电符号灭
				
				if((paramstatus == state0) && (longparamstatus == state0) && (peakstatus == state0) && (phasestatus == state0))
				{//当在正常V+A状态（不在Param、Inrush、Peak、Phase状态）
					if(funcstatus ==state0)//AC V+A
					{												
						readstmdata1=voltage_effective-fabs(voltage_mean);
						readstmdata2=current_effective-fabs(current_mean);																			
						
						showdata1=deal_1(readstmdata1,1);
						showdata2=deal_1(readstmdata2,2);
						switch(rangestatus)
						{
							case state0://6.000V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;//加档
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;//加档
									}
								}
								else//手动挡
								{
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=60) && (readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(readstmdata1<=6)
											rangestatus-=1;//减档
										else if(readstmdata1>60)
											rangestatus+=1;//加档
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;//加档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;//减档
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;//加档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;//加档
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=60) && (readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(readstmdata1<=6)
											rangestatus-=1;
										else if(readstmdata1>60)
											rangestatus+=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							default:break;
						}
					}
					else if(funcstatus ==state1)//DC V+A
					{
						readstmdata1=voltage_mean;
						readstmdata2=current_mean;
						
						showdata1=deal_1(readstmdata1,1);
						showdata2=deal_1(readstmdata2,2);
						
						abs_readstmdata1=fabs(readstmdata1);
						
						switch(rangestatus)
						{
							case state0://6.000V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if(abs_readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
									//printf("v:%.6f\r\n",showdata1);
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((abs_readstmdata1<=60) && (abs_readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(abs_readstmdata1<=6)
											rangestatus-=1;
										else if(abs_readstmdata1>60)
											rangestatus+=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((abs_readstmdata1<=600) && (abs_readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(abs_readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if(abs_readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;
									}
									
									if((readstmdata2<=3000) && (readstmdata2>600))//Lea test   20160614 上限增加至3000用于测试显示
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((abs_readstmdata1<=60) && (abs_readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(abs_readstmdata1<=6)
											rangestatus-=1;
										else if(abs_readstmdata1>60)
											rangestatus+=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=3000) && (readstmdata2>600))//Lea test   20160614 上限增加至3000用于测试显示
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((abs_readstmdata1<=600) && (abs_readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(abs_readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							default:break;
						}
//						if(abs_readstmdata1>50)
//							lcd_write_1bit(0x0,3,ENABLE);//闪电符号亮
//						else
//							lcd_write_1bit(0x0,3,DISABLE);//闪电符号灭
					}
					else if(funcstatus ==state2)//AC+DC V+A
					{						
						readstmdata1=voltage_effective;
						readstmdata2=current_effective; 
						
						showdata1=deal_1(readstmdata1,1);
						showdata2=deal_1(readstmdata2,2);

						switch(rangestatus)
						{
							case state0://6.000V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=60) && (readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(readstmdata1<=6)
											rangestatus-=1;
										else if(readstmdata1>60)
											rangestatus+=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.亮
										rangestatus+=3;
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.亮
										rangestatus+=1;
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=60) && (readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(readstmdata1<=6)
											rangestatus-=1;
										else if(readstmdata1>60)
											rangestatus+=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//自动挡
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.亮
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//减档
									}
								}
								else//手动挡
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							default:break;
						}
					}
				}
				else if(paramstatus != state0)
				{//当在Param状态
					switch(paramstatus)
					{
						case state1://THD%f
						{
							readstmdata1=THD_f_voltage;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
							
							readstmdata2=THD_f_current;
							showdata2=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata2,2,10);
						}break;
						case state2://THD%r
						{
							readstmdata1=THD_r_voltage;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
							
							readstmdata2=THD_r_current;
							showdata2=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata2,2,10);
						}break;
						case state3://CF
						{
							readstmdata1=voltage_cf;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,2);
							
							readstmdata2=current_cf;
							showdata2=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata2,2,2);
						}break;
						default:break;
					}
				}
				else if(longparamstatus != state0)
				{//当在Inrush状态
					if(inrush_trigger_flag == 2)//无需判断是600A还是2000A，在按键RANGE时就确定了
					{//采集到100ms内的RMS值。蜂鸣一声，并显示在主显示区上。
						propershow=lcd_show_num(inrush_current_effective_100ms,1,10);
					}
				}
				else if(peakstatus != state0)
				{//当在Peak状态
					switch(max_minstatus)
					{
						case state1://Peak+
						{
							readstmdata1=maxv_value;
							showdata1=readstmdata1;//deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
							
							readstmdata2=maxi_value;
							showdata2=readstmdata2;//deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata2,2,10);
						}break;
						case state2://Peak-
						{
							readstmdata1=minv_value;
							showdata1=readstmdata1;//deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
							
							readstmdata2=mini_value;
							showdata2=readstmdata2;//deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata2,2,10);
						}break;
						default:break;
					}
				}
				else if(phasestatus != state0)
				{//当在Phase状态
					if((TIM5CH4_CAPTURE_STA&0x8000) && (phaseABCstatus==state4))//已经获取相位
					{//此段程序，只进入一次
						if((abs(phasetimetemp) < 20) || (abs(phasetimetemp-360) < 20))//A相
						{
							lcd_show_A__A(2);
						}
						if(abs(phasetimetemp-120) < 20)//B相
						{
							lcd_show_b__b(2);
						}
						else if(abs(phasetimetemp-240) < 20)//C相
						{
							lcd_show_C__C(2);
						}
						else
						{
							lcd_show_Erro(2);
						}							
						propershow=lcd_show_num(phasetimetemp,1,10);//20160603Lea Test 相序时在第一行显示相位差的数值，判断错误原因
						phaseABCstatus=state5;
					}
					else if((TIM5CH4_CAPTURE_STA&0x4000) && (phaseABCstatus<state4))//A相电压已经稳定后，且捕获到了A相经过比较器之后的上升沿，还未捕获到另一相上升沿之前
					{//此段程序在显示A--A时，需要0.5s闪烁第二行的----
						if(powertimer_0_5s_flag == 1)//闪烁显示
						{
							powertimer_0_5s_flag=0;
							switch(blink)
							{
								case 0:blink=1;break;
								case 1:blink=0;break;
								default:break;
							}
							if(blink == 1) lcd_show_____(2);
							else lcd_show_None(2);
							lcd_show_A__A(1);
						}
					}
					else if((TIM5CH4_CAPTURE_STA&0x4000)==0)//还未捕获到A相上升沿
					{
						if(powertimer_0_5s_flag == 1)//显示
						{
							powertimer_0_5s_flag=0;
							switch(blink)
							{
								case 0:blink=1;break;
								case 1:blink=0;break;
								default:break;
							}
							if(blink == 1) lcd_show_A___(1);
							else lcd_show_None(1);
						}
					}
					
					if((TIM5CH4_CAPTURE_STA&0x8000) && (phaseABCstatus==state3))//成功捕获到一次A与B或C相的时间计数,还未计算出相位
					{//此段程序，只会进入一次
						TIM_Cmd(TIM5, DISABLE);//一轮捕获结束
						
//						phasetimetemp=TIM5CH4_CAPTURE_STA&0x3FFF;
//						phasetimetemp*=36000;//溢出时间总和
//						phasetimetemp+=TIM5CH4_CAPTURE_VAL;//得到总的两相相位时间
//						
						TphaseTime = TIM5CH4_CAPTURE_VAL%CCR4_Val;	
						//phasetimetemp%=100;//取得一个周期内相差的相位差计数值。1MHz/50Hz=20000;一个周期内计数20000  约20011
						TphaseTime=TphaseTime*360/CCR4_Val;//将计数值换算成角度0°- 360°
						//propershow=lcd_show_num(phasetimetemp,1,10);
						phasetimetemp = TphaseTime;
						phaseABCstatus=state4;//标志已经获取相位，FINISHED！！！！
					}
					
					else if(TIM5CH4_CAPTURE_STA&0x4000)//在已捕获到A相上升沿、关闭比较器的情况下
					{//先通过判断电压大小<10等待探针移除
						readstmdata2=voltage_effective;
						propershow=lcd_show_num(readstmdata2,2,10);//test 在第二行显示当前电压的有效值
						if((readstmdata2<10) && (phaseABCstatus==state1))
						{
							phaseABCstatus=state2;//表示表笔已经检查到A相，并离开测量端子，准备测量下一相
							BUZZER_Open(0);//蜂鸣一声 提示已判定表笔离开第一相
						}
						else if((readstmdata2>10) && (phaseABCstatus==state1))
						{
							phaseABCstatus=state1;//继续等待表笔离开A相端子
						}
						
						if(phaseABCstatus==state2)
						{
							if(readstmdata2>100)//检测到另一相电压大于100V
							{
								delay_count++;
								if(delay_count==1)
									readstmdata2_temp=readstmdata2;
								if(delay_count==5)
								{
									if(abs(readstmdata2-readstmdata2_temp)<5)
									{
										COMP_Cmd(COMP_Selection_COMP1, ENABLE);
										delay_count=0;
										phaseABCstatus=state3;//标志B or C相电压稳定，开启比较器，检测上升沿
									}
									else
										delay_count=0;
								}
							}
						}
						else if(phaseABCstatus==state3)//在电压已经稳定（还未检测到上升沿时），又检测到电压变小，关闭比较器
						{
							if(readstmdata2<10)
							{
								phaseABCstatus=state2;
//								COMP_Cmd(COMP_Selection_COMP1, DISABLE);
							}
						}
					}
					else if((TIM5CH4_CAPTURE_STA&0x4000)==0)//还没有捕获到A相上升沿的情况下
					{
						readstmdata2=voltage_effective;//
						propershow=lcd_show_num(readstmdata2,2,10);//test 在第二行显示当前电压的有效值
						if((readstmdata2>100) && (phaseABCstatus==state0))//检测到第一相（A相）电压大于100V，且是电压未稳定状态
						{
							if(delay_count==1){readstmdata2_temp=readstmdata2;}
								
							else if(delay_count==5)
							{
								if(abs(readstmdata2-readstmdata2_temp)<5)//若电压抖动不大则继续进行下面的操作
								{
									//开启COMP1和定时器,开始捕获
									COMP_Cmd(COMP_Selection_COMP1, ENABLE);
									
									delay_count=0;
									phaseABCstatus=state1;//标志电压稳定，开始捕获上升沿，此时表笔未离开A相端子，不需要再进入此段程序了。
								}
								else
									delay_count=0;
							}
							delay_count++;
						}
						else if((readstmdata2<100) && (phaseABCstatus==state1))//在电压已经稳定（还未检测到上升沿时），又检测到电压变小，关闭比较器和定时器
						{
							phaseABCstatus=state0;
							COMP_Cmd(COMP_Selection_COMP1, DISABLE);
							TIM_Cmd(TIM5, DISABLE);
						}
					}
				}
			}break;
			case KEY_VALUE_7://W
			{
				temp_for_over_voltage=voltage_effective;
//				if(rangenum == 0)
//					temp_for_over_voltage*=10;
//				else if(rangenum == 1)
//					temp_for_over_voltage*=100;
//				else if(rangenum == 2)
//					temp_for_over_voltage*=1000;
				if(temp_for_over_voltage>30)
					lcd_write_1bit(0x0,3,ENABLE);//闪电符号亮
				else
					lcd_write_1bit(0x0,3,DISABLE);//闪电符号灭
				
				if((peakstatus == state0) && (longparamstatus == state0))
				{//总功率、有功功率、无功功率状态//PF//DPF//THD%f//THD%r
					switch(funcstatus)
					{
						case state0://总功率
						{
							readstmdata1=apparent_power;//VA
							
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state1://有功
						{
							readstmdata1=active_power;//W
							
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state2://无功
						{
							readstmdata1=reactive_power;//var
							
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						default:break;
					}
					switch(paramstatus)
					{
						case state0://PF
						{
							readstmdata2=power_factor;
							showdata1=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata1,2,1);
						}break;
						case state1://DPF
						{
							readstmdata2=d_power_factor;
							showdata1=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata1,2,1);
						}break;
						case state2://THD%f
						{
							readstmdata2=THD_f_power;
							showdata1=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata1,2,10);
						}break;
						case state3://THD%r
						{
							readstmdata2=THD_r_power;
							showdata1=deal_1(readstmdata2,2);
							propershow=lcd_show_num(showdata1,2,10);
						}break;
						default:break;
					}
				}
				else if(longparamstatus != state0)
				{//当在电能状态
					if(timer_1s_blink == 1)
					{
						timer_1s_blink = 0;
						lcd_show_num(powertimer,2,10);
					}
					switch(paramstatus)
					{
						case state0://处理KWh
						{
							readstmdata1=kWh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state1://处理KVAh
						{
							readstmdata1=kVAh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state2://处理Kvarh
						{
							readstmdata1=kVarh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state3://处理KgCO2
						{
							readstmdata1=abcde;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						default:break;
					}
				}
				else if(peakstatus != state0)
				{//当在多相功率状态
					if(powertimer_0_5s_flag == 1)
					{
						powertimer_0_5s_flag=0;
						switch(blink)
						{
							case 0:blink=1;break;
							case 1:blink=0;break;
							default:break;
						}
					}
					switch(phasestatus)
					{
						case state0://1PHI闪烁
						{
							switch(blink)
							{
								case 0:
								{
									lcd_write_1bit(0x09,3,DISABLE);//1
									lcd_write_1bit(0x0B,0,DISABLE);//PHI
								}break;
								case 1:
								{
									lcd_write_1bit(0x09,3,ENABLE);//1
									lcd_write_1bit(0x0B,0,ENABLE);//PHI
								}break;
								default:break;
							}
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power,1,10);break;
								case state1:propershow=lcd_show_num(active_power,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power,1,10);break;
								default:break;
							}
//							propershow=lcd_show_num(active_power,1,10);
						}break;
						case state1://2PHI闪烁
						{
							switch(blink)
							{
								case 0:
								{
									lcd_write_1bit(0x0A,3,DISABLE);//2
									lcd_write_1bit(0x0B,0,DISABLE);//PHI
								}break;
								case 1:
								{
									lcd_write_1bit(0x0A,3,ENABLE);//2
									lcd_write_1bit(0x0B,0,ENABLE);//PHI
								}break;
								default:break;
							}
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power,1,10);break;
								case state1:propershow=lcd_show_num(active_power,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power,1,10);break;
								default:break;
							}
						}break;
						case state2://3PHI闪烁
						{
							switch(blink)
							{
								case 0:
								{
									lcd_write_1bit(0x0B,1,DISABLE);//3
									lcd_write_1bit(0x0B,0,DISABLE);//PHI
									
								}break;
								case 1:
								{
									lcd_write_1bit(0x0B,1,ENABLE);//3
									lcd_write_1bit(0x0B,0,ENABLE);//PHI
								}break;
								default:break;
							}
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power,1,10);break;
								case state1:propershow=lcd_show_num(active_power,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power,1,10);break;
								default:break;
							}
						}break;
						case state3://1PHI不闪烁
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power1,1,10);break;
								case state1:propershow=lcd_show_num(active_power1,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power1,1,10);break;
								default:break;
							}
						}break;
						case state4://2PHI不闪烁
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power2,1,10);break;
								case state1:propershow=lcd_show_num(active_power2,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power2,1,10);break;
								default:break;
							}
						}break;
						case state5://3PHI不闪烁
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power3,1,10);break;
								case state1:propershow=lcd_show_num(active_power3,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power3,1,10);break;
								default:break;
							}
						}break;
						case state6://总PHI不闪烁
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power_sum123,1,10);break;
								case state1:propershow=lcd_show_num(active_power_sum123,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power_sum123,1,10);break;
								default:break;
							}
						}break;
						default:break;
					}
				}
			}break;
			case KEY_VALUE_8://mV/V(DC/AC)
			{
				if(funcstatus ==state0)//DCVmV
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					read_dta_data = (read_dta_data-SaveData.Value.Cal_VmV_zero[rangestatus])*SaveData.Value.Cal_VmV_gain[rangestatus];//20160803Lea 新增MCU内部校准
					VmV_value = read_dta_data;//内部保存测量结果，用于校准和通讯
					
					showdata1=deal_1(read_dta_data,1);
					temp_for_over_voltage=0;
					switch(rangestatus)
					{
						case state0://600.0mV
						{
							lcd_write_1bit(0x1A,2,ENABLE);//m亮
							
							if(autostatus==state1)//自动挡
							{
								if((RxBuffer[4] == 0) && (receive_data<=6200))//receive_data是receive_f的绝对值
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									rangestatus++;
								}
							}
							else//手动挡
							{
								if((RxBuffer[4] == 0) && (receive_data<=6200))
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state1://6.000V
						{
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 1) && (receive_data<=6200) && (receive_data>=580))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<1)
										rangestatus--;
									else if(RxBuffer[4]>1)
										rangestatus++;
								}
							}
							else
							{
								if((RxBuffer[4] == 1) && (receive_data<=6200))
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state2://60.00V
						{
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 2) && (receive_data<=6200) && (receive_data>=580))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data/100.0f;
								}
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<2)
										rangestatus--;
									else if(RxBuffer[4]>2)
										rangestatus++;
								}
							}
							else
							{
								if((RxBuffer[4] == 2) && (receive_data<=6200))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data/100.0f;
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						case state3://600.0V
						{
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 3) && (receive_data<=6200) && (receive_data>=580))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data/10.0f;
								}
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<3)
										rangestatus--;
									else if(RxBuffer[4]>3)
										rangestatus++;
								}
							}
							else
							{
								if((RxBuffer[4] == 3) && (receive_data<=6200))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data/10.0f;
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state4://1000V
						{
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 4) && (receive_data>=580) && (receive_data<=1000))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data;
								}
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<4)
										rangestatus=state3;
								}
							}
							else
							{
								if((RxBuffer[4] == 4) && (receive_data<=1000))
								{
									lcd_show_dta_num(showdata1,1);
									temp_for_over_voltage=receive_data;
								}
								else
									lcd_show_OL(1);
							}
						}break;
						default:break;
					}
					if(temp_for_over_voltage>50)
						lcd_write_1bit(0x0,3,ENABLE);//闪电符号亮
					else
						lcd_write_1bit(0x0,3,DISABLE);//闪电符号灭
				}
				else if(funcstatus ==state1)//ACV
				{
					if(paramstatus == state0)//正常ACV
					{ 
						read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
						showdata1=deal_1(read_dta_data,1);
						temp_for_over_voltage=0;
						switch(rangestatus)
						{
							case state0://6.000V
							{
								if(autostatus==state1)
								{
									if((RxBuffer[4] == 0) && (receive_data<=6200))
										lcd_show_dta_num(showdata1,1);
									else
									{
										lcd_show_Line(1);
										rangestatus++;
									}
								}
								else
								{
									if((RxBuffer[4] == 0) && (receive_data<=6200))
									{
										lcd_show_dta_num(showdata1,1);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x02,0,ENABLE);//.亮
							}break;
							case state1://60.00V
							{
								if(autostatus==state1)
								{
									if((RxBuffer[4] == 1) && (receive_data<=6200) && (receive_data>=580))
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data/100.0f;
									}
									else
									{
										lcd_show_Line(1);
										if(RxBuffer[4]<1)
											rangestatus--;
										else if(RxBuffer[4]>1)
											rangestatus++;
									}
								}
								else
								{
									if((RxBuffer[4] == 1) && (receive_data<=6200))
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data/100.0f;
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x04,0,ENABLE);//.亮
							}break;
							case state2://600.0V
							{
								if(autostatus==state1)
								{
									if((RxBuffer[4] == 2) && (receive_data<=6200) && (receive_data>=580))
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data/10.0f;
									}
									else
									{
										lcd_show_Line(1);
										if(RxBuffer[4]<2)
											rangestatus--;
										else if(RxBuffer[4]>2)
											rangestatus++;
									}
								}
								else
								{
									if((RxBuffer[4] == 2) && (receive_data<=6200))
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data/10.0f;
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x06,0,ENABLE);//.亮
							}break;
							case state3://1000V
							{
								if(autostatus==state1)
								{
									if((RxBuffer[4] == 3) && (receive_data<=1000) && (receive_data>=580))
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data;
									}
									else
									{
										lcd_show_Line(1);
										if(RxBuffer[4]<3)
											rangestatus--;
									}
								}
								else
								{
									if(receive_data<=1000)
									{
										lcd_show_dta_num(showdata1,1);
										temp_for_over_voltage=receive_data;
									}
									else
										lcd_show_OL(1);
								}
							}break;
							default:break;
						}
						if(temp_for_over_voltage>30)
							lcd_write_1bit(0x0,3,ENABLE);//闪电符号亮
						else
							lcd_write_1bit(0x0,3,DISABLE);//闪电符号灭
					}
					else if(paramstatus == state1)//Hz
					{
						read_dta_data = receive_f;//从DTA0660读取数据
						
						showdata1=deal_1(read_dta_data,1);
						switch(rangestatus)
						{
							case state0://99.99Hz
							{
								lcd_write_1bit(0x1B,1,DISABLE);//k(2)
								lcd_write_1bit(0x1A,1,DISABLE);//M
								if(autostatus==state1)
								{
									if(read_dta_data<=99.99f)
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										lcd_show_Line(1);
										rangestatus++;
									}
								}
								else
								{
									if(read_dta_data<=99.99)
									{
										propershow=lcd_show_num(showdata1,1,2);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x04,0,ENABLE);//.亮
							}break;
							case state1://999.9Hz
							{
								lcd_write_1bit(0x1B,1,DISABLE);//k(2)
								lcd_write_1bit(0x1A,1,DISABLE);//M
								if(autostatus==state1)
								{
									if((read_dta_data<=999.9) && (read_dta_data>99.99))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										lcd_show_Line(1);
										if(read_dta_data<=99.99)
											rangestatus--;
										else if(read_dta_data>999.9)
											rangestatus++;
									}
								}
								else
								{
									if(read_dta_data<=999.9)
									{
										propershow=lcd_show_num(showdata1,1,3);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x06,0,ENABLE);//.亮
							}break;
							case state2://9.999kHz
							{
								lcd_write_1bit(0x1B,1,ENABLE);//k(2)
								lcd_write_1bit(0x1A,1,DISABLE);//M
								showdata1/=1000;
								if(autostatus==state1)
								{
									if((read_dta_data/1000<=9.999)  && (read_dta_data>999.9))
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										if(read_dta_data<=999.9)
											rangestatus--;
										else if(read_dta_data/1000>9.999)
											rangestatus++;
									}
								}
								else
								{
									if(read_dta_data/1000<=9.999)
									{
										propershow=lcd_show_num(showdata1,1,1);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x02,0,ENABLE);//.亮
							}break;
							case state3://99.99kHz
							{
								lcd_write_1bit(0x1B,1,ENABLE);//k(2)
								lcd_write_1bit(0x1A,1,DISABLE);//M
								showdata1/=1000;
								if(autostatus==state1)
								{
									if((read_dta_data/1000<=99.99) && (read_dta_data/1000>9.999))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										lcd_show_Line(1);
										if(read_dta_data/1000<=9.999)
											rangestatus--;
										else if(read_dta_data/1000>99.99)
											rangestatus++;
									}
								}
								else
								{
									if(read_dta_data/1000<=99.99)
									{
										propershow=lcd_show_num(showdata1,1,2);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x04,0,ENABLE);//.亮
							}break;
							case state4://999.9kHz
							{
								lcd_write_1bit(0x1B,1,ENABLE);//k(2)
								lcd_write_1bit(0x1A,1,DISABLE);//M
								showdata1/=1000;
								if(autostatus==state1)
								{
									if((read_dta_data/1000<=999.9) && (read_dta_data/1000>99.99))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										lcd_show_Line(1);
										if(read_dta_data/1000<=99.99)
											rangestatus--;
										else if(read_dta_data/1000>999.9)
											rangestatus++;
									}
								}
								else
								{
									if(read_dta_data/1000<=999.9)
									{
										propershow=lcd_show_num(showdata1,1,3);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x06,0,ENABLE);//.亮
							}break;
							case state5://9.999MHz
							{
								lcd_write_1bit(0x1B,1,DISABLE);//k(2)
								lcd_write_1bit(0x1A,1,ENABLE);//M
								showdata1/=1000;
								showdata1/=1000;
								if(autostatus==state1)
								{
									if((read_dta_data/1000/1000<=9.999) && (read_dta_data/1000>999.9))
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										if(read_dta_data/1000<=999.9)
										{
											lcd_show_Line(1);
											rangestatus--;
										}
										else if(read_dta_data/1000/1000>9.999)
											lcd_show_OL(1);
									}
								}
								else
								{
									if(read_dta_data/1000/1000<=9.999)
									{
										propershow=lcd_show_num(showdata1,1,1);
									}
									else
										lcd_show_OL(1);
								}
								lcd_write_1bit(0x02,0,ENABLE);//.亮
							}break;
							default:break;
						}
					}
					else if(paramstatus == state2)//Duty
					{
						read_dta_data = receive_f1;//从DTA0660读取数据
						showdata2=deal_1(read_dta_data,2);
						propershow=lcd_show_num(showdata2,2,2);//占空比
					}
				}
			}break;
			case KEY_VALUE_9://CON/DIODE
			{
				if(funcstatus ==state0)//CONT
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					showdata1=deal_1(read_dta_data,1);
					//if(read_dta_data<=50)
					//	BUZZER_Open(1);
					//else BUZZER_Close();
					if(showdata1<=600)
						propershow=lcd_show_num(showdata1,1,3);
					else 
					{
						lcd_show_OL(1);
						lcd_write_1bit(0x06,0,ENABLE);//.亮
					}
				}
				else if(funcstatus ==state1)//DIODE
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					showdata1=deal_1(read_dta_data,1);
					if(showdata1<=3)
						propershow=lcd_show_num(showdata1,1,1);
					else 
					{
						lcd_show_OL(1);
						lcd_write_1bit(0x02,0,ENABLE);//.亮
					}
				}
			}break;
			case KEY_VALUE_10://OHM/CAP/TMP
			{
				if(funcstatus ==state0)//Ohm
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					read_dta_data = (read_dta_data-SaveData.Value.Cal_ohm_zero[rangestatus])*SaveData.Value.Cal_ohm_gain[rangestatus];//20160802Lea 新增MCU内部校准
					ohm_value = read_dta_data;//内部保存测量结果，用于校准和通讯
					
					showdata1=deal_1(read_dta_data,1);
					switch(rangestatus)
					{
						case state0://600.0Ω
						{
							lcd_write_1bit(0x1B,1,DISABLE);//k(2)灭
							lcd_write_1bit(0x1A,1,DISABLE);//M灭

							if(autostatus==state1)//自动挡
							{
								if((RxBuffer[4] == 0) && (receive_data<=6200))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]>0)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state1://6.000kΩ
						{
							lcd_write_1bit(0x1B,1,ENABLE);//k(2)
							lcd_write_1bit(0x1A,1,DISABLE);//M
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 1) && (receive_data<=6200) && (receive_data>=580))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<1)
										rangestatus--;
									else if(RxBuffer[4]>1)
										rangestatus++;
								}
							}
							else
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state2://60.00kΩ
						{
							lcd_write_1bit(0x1B,1,ENABLE);//k(2)
							lcd_write_1bit(0x1A,1,DISABLE);//M
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 2) && (receive_data<=6200) && (receive_data>=580))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<2)
										rangestatus--;
									else if(RxBuffer[4]>2)
										rangestatus++;
								}
							}
							else
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						case state3://600.0kΩ
						{
							lcd_write_1bit(0x1B,1,ENABLE);//k(2)
							lcd_write_1bit(0x1A,1,DISABLE);//M
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 3) && (receive_data<=6200) && (receive_data>=580))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<3)
										rangestatus--;
									else if(RxBuffer[4]>3)
										rangestatus++;
								}
							}
							else
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);// *1.0101  做电阻软件教主测试 
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state4://6.000MΩ
						{
							lcd_write_1bit(0x1B,1,DISABLE);//k(2)
							lcd_write_1bit(0x1A,1,ENABLE);//M
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 4) && (receive_data<=6200) && (receive_data>=580))
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									if(RxBuffer[4]<4)
										rangestatus--;
									else if(RxBuffer[4]>4)
										rangestatus++;
								}
							}
							else
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state5://60.00MΩ
						{
							lcd_write_1bit(0x1B,1,DISABLE);//k(2)
							lcd_write_1bit(0x1A,1,ENABLE);//M
							if(autostatus==state1)
							{
								if((RxBuffer[4] == 5) && (receive_data<=6200) && (receive_data>=580))
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
								{
									if(RxBuffer[4]<5)
									{
										rangestatus--;
										lcd_show_Line(1);
									}
									else
										lcd_show_OL(1);
								}
							}
							else
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
//									lcd_write_1bit(0x04,0,ENABLE);//.亮
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						default:break;
					}
				}
				else if(funcstatus ==state1)//Cap
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					switch(rangestatus)//电容值修正
					{
						case 0://6.000nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 1://60.00nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 2://600.0nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 3://6.000uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[1])*SaveData.Value.Cal_Cap_gain[1];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 4://60.00uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[2])*SaveData.Value.Cal_Cap_gain[2];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 5://600.0uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[2])*SaveData.Value.Cal_Cap_gain[2];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 6://6.000mF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[3])*SaveData.Value.Cal_Cap_gain[3];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}
						case 7://60.00mF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[3])*SaveData.Value.Cal_Cap_gain[3];//20160803Lea 新增MCU内部校准
							Cap_value = read_dta_data;//内部保存测量结果，用于校准和通讯
							break;
						}										
						default:break;
					}
					
					showdata1=deal_1(read_dta_data,1);
					switch(rangestatus)
					{
						case state0://6.000n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n亮
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,DISABLE);//u灭
							if(autostatus==state1)//自动挡
							{
								if(read_dta_data<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data>6.000)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state1://60.00n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n亮
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,DISABLE);//u灭
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data<=60.00) && (read_dta_data>6.000))
									propershow=lcd_show_num(showdata1,1,2);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data<=6.000)
										rangestatus--;
									else if(read_dta_data>60.00)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						case state2://600.0n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n亮
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,DISABLE);//u灭
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data<=600.0) && (read_dta_data>60.00))
									propershow=lcd_show_num(showdata1,1,3);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data<=60.00)
										rangestatus--;
									else if(read_dta_data>600.0)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data<=600.0)
									propershow=lcd_show_num(showdata1,1,3);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state3://6.000u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n灭
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,ENABLE);//u亮
							showdata1/=1000;
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data/1000<=6.000) && (read_dta_data>600.0))
									propershow=lcd_show_num(showdata1,1,1);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data<=600.0)
										rangestatus--;
									else if(read_dta_data/1000>6.000)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data/1000<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state4://60.00u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n灭
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,ENABLE);//u亮
							showdata1/=1000;
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data/1000<=60.00) && (read_dta_data/1000>6.000))
									propershow=lcd_show_num(showdata1,1,2);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data/1000<=6.000)
										rangestatus--;
									else if(read_dta_data/1000>60.00)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data/1000<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						case state5://600.0u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n灭
							lcd_write_1bit(0x1A,2,DISABLE);//m灭
							lcd_write_1bit(0x19,2,ENABLE);//u亮
							showdata1/=1000;
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data/1000<=600.0)  && (read_dta_data/1000>60.00))
									propershow=lcd_show_num(showdata1,1,3);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data/1000<=60.00)
										rangestatus--;
									else if(read_dta_data/1000>600.0)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data/1000<=600.0)
									propershow=lcd_show_num(showdata1,1,3);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.亮
						}break;
						case state6://6.000m
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n灭
							lcd_write_1bit(0x1A,2,ENABLE);//m亮
							lcd_write_1bit(0x19,2,DISABLE);//u灭
							showdata1/=1000;
							showdata1/=1000;
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data/1000/1000<=6.000) && (read_dta_data/1000>600.0))
									propershow=lcd_show_num(showdata1,1,1);
								else
								{
									lcd_show_Line(1);
									if(read_dta_data/1000<=600.0)
										rangestatus--;
									else if(read_dta_data/1000/1000>6.000)
										rangestatus++;
								}
							}
							else//手动挡
							{
								if(read_dta_data/1000/1000<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.亮
						}break;
						case state7://60.00m
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n灭
							lcd_write_1bit(0x1A,2,ENABLE);//m亮
							lcd_write_1bit(0x19,2,DISABLE);//u灭
							showdata1/=1000;
							showdata1/=1000;
							if(autostatus==state1)//自动挡
							{
								if((read_dta_data/1000/1000<=60.00) && (read_dta_data/1000/1000>=6.000))
									propershow=lcd_show_num(showdata1,1,2);
								else
								{
									if(read_dta_data/1000/1000<=6.000)
									{
										lcd_show_Line(1);
										rangestatus--;
									}
									else
										lcd_show_OL(1);
								}
							}
							else//手动挡
							{
								if(read_dta_data/1000/1000<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.亮
						}break;
						default:break;
					}
				}
				else if(funcstatus ==state2)//Temp(C)
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//从DTA0660读取数据
					showdata1=deal_1(read_dta_data,2);
					propershow=lcd_show_num(showdata1,2,2);
					if(propershow==0)
					{
						propershow=lcd_show_num(showdata1,2,3);
					}
				}
				else if(funcstatus ==state3)//Temp(F)
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult)*1.8+32;;//从DTA0660读取数据
					showdata1=deal_1(read_dta_data,2);
					propershow=lcd_show_num(showdata1,2,2);
					if(propershow==0)
					{
						propershow=lcd_show_num(showdata1,2,3);
					}
				}
			}break;
			default:break;
		}
	}
}

/**/
/**************************************************end file**************************************************/

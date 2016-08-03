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
SoftKeystatus funcstatus=state0,btstatus=state0;			//�ӹ��ܵ�λ״̬����������״̬
SoftKeystatus max_minstatus=state0,peakstatus=state0;	//��ֵ	��	��ֵ
SoftKeystatus relstatus=state0,phasestatus=state0;		//			��	������
SoftKeystatus rangestatus=state0,paramstatus=state0;	//����	��	param��������״̬
SoftKeystatus holdstatus=state0,lightstatus=state0;		//hold 	��	����
SoftKeystatus longparamstatus=state0,autostatus=state1;//InRush , �Զ�����
SoftKeystatus phaseABCstatus=state0;						//����������ܽ��е��Ĳ��ࡣ


defSysflag Sysflag;//=(defSysflag){.Calonce = 0,.DTArange=0,.rangestatus=0}


extern defSysValue SysValue ;//ϵͳ����ʱ����Ҫ��������
extern defFlashCal SaveData;//������flash�еı�Ҫ����

extern __IO uint16_t CCR4_Val;//TIM58 ���ڶ�ʱ�������

u8 blink;
float max_data1,min_data1,max_data2,min_data2,rel_data1,rel_data2;
float abcde=2.15;//��װ�Ĳ���ֵ   �Ǻ�
uint32_t TphaseTime=0;

/*���๦�ʲ���ʱ��¼ֵ*/
float apparent_power1=0,active_power1=0,reactive_power1=0;//���ڹ��ʡ��й����ʡ��޹�����
float apparent_power2=0,active_power2=0,reactive_power2=0;//���ڹ��ʡ��й����ʡ��޹�����
float apparent_power3=0,active_power3=0,reactive_power3=0;//���ڹ��ʡ��й����ʡ��޹�����
float apparent_power_sum123=0,active_power_sum123=0,reactive_power_sum123=0;//���ڹ��ʡ��й����ʡ��޹�����

/*display�����ñ���*/
u8 size;
u32 phasetimetemp=0;
boolean propershow=0;
float showdata1,showdata2,read_dta_data,read_dta_data1,readstmdata1,readstmdata2,temp_for_over_voltage,abs_readstmdata1;
float readstmdata2_temp;
u8 delay_count;
u8 rangenum;

/*20160727 lea ��Ϊ�й��ܰ���ʱ�����Data_init��������յ�ѹ������ֵ���Զ��弸����ʱ�����������⼸��ֵ��Ŀǰֻ����REL�������ܵ�����ѹ�����ֵ ����ֵ*/
float REL_voltage_effective=0;
float REL_voltage_mean=0;
float REL_current_effective=0;
float REL_current_mean=0;

/*20160801 ����ȫ�ָ���λ����ֵ����*/
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

/*STM32���衢DTA0660��Դ���ص�ʹ�ܺ�ʧ�ܡ�����ת���ظı�ʱ����*/
void MeasureModeChange(void)
{
		if(RotaryKeyValue <= 0x07)//STM32����������STM32��Ӧ����
		{
			Power_On_Voltage();//PA5=0
			Power_On_Current();//PC15=1
			
			SDADC_Cmd(SDADC1, ENABLE);
			SDADC_Cmd(SDADC2, ENABLE);
			TIM_Cmd(TIM19, ENABLE);
//			COMP_Cmd(COMP_Selection_COMP1, DISABLE);//2015-4-28ע�ͣ���ENABLE��ΪDISABLE,����PHASE��������ʱ��
//			COMP_Cmd(COMP_Selection_COMP2, DISABLE);
//			TIM_Cmd(TIM5, DISABLE);//���벶��ʱ��
//			TIM_Cmd(TIM2, DISABLE);//2015-4-28ע�ͣ���ENABLE��ΪDISABLE,����PHASE��������ʱ��
		}
		else if((RotaryKeyValue >= 0x08) && (RotaryKeyValue != KEY_NULL))//DTA0660�������ر�STM32��Ӧ����
		{
			Power_Off_Voltage();//PA5=1
			Power_Off_Current();//PC15=0
			
			SDADC_Cmd(SDADC1, DISABLE);
			SDADC_Cmd(SDADC2, DISABLE);
			TIM_Cmd(TIM19, DISABLE);//����ʱ��
//			COMP_Cmd(COMP_Selection_COMP1, DISABLE);//�Ƚ���
//			COMP_Cmd(COMP_Selection_COMP2, DISABLE);
			TIM_Cmd(TIM5, DISABLE);//���벶��ʱ��
//			TIM_Cmd(TIM2, DISABLE);
		}
}

/*��������ѡ�񡪡����̰���Funcʱ��DTA0660�������ܸı� ������ť�Ͱ������ܼ���е���*/
void MeasureFunctionSelection(void)
{
	switch(RotaryKeyValue)
	{
		case KEY_VALUE_6://DC/ACV
		{
			switch(funcstatus)
			{
				case state0://����V~����ACV
					FunctionSet(ACV);//�趨���ܣ�������ѹ
				break;
				
				case state1://����V-����DCV
					FunctionSet(DCV);//�趨���ܣ�ֱ����ѹ
				break;
				
				case state2://����V~V-����ACV
					FunctionSet(ACV);//�趨���ܣ�������ѹ
				break;
				
				default:break;
			}
		}
		break;
		
		case KEY_VALUE_7://W
		{
			FunctionSet(ACV);//�趨���ܣ�������ѹ
		}
		break;
		
		case KEY_VALUE_8:
		{
			switch(funcstatus)
			{
				case state0://����V-����DCVmV
					FunctionSet(DCVmV);//�趨���ܣ�ֱ����ѹ
				break;
				
				case state1://����V~����ACV
				{
					if(paramstatus == state0)
						FunctionSet(ACV);//�趨���ܣ�������ѹ
					else//Hz_Duty
						FunctionSet(Hz_Duty);//�趨���ܣ�Hz_Duty
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
				case state0://����Cont
					FunctionSet(Cont);//�趨���ܣ�ͨ��
				break;
				
				case state1://����Diode
					FunctionSet(Diode);//�趨���ܣ�������
				break;
				
				default:break;
			}
		}
		break;
		
		case KEY_VALUE_10:
		{
			switch(funcstatus)
			{
				case state0://����Ohm
					FunctionSet(Ohm);//�趨���ܣ�Ohm
				break;
				
				case state1://����Cap
					FunctionSet(Cap);//�趨���ܣ�Cap
				break;
				
				case state2://����Temp(C)
					FunctionSet(Temp);//�趨���ܣ�Temp
				break;
				
				case state3://����Temp(F)
					FunctionSet(Temp);//�趨���ܣ�Temp
				break;
				
				default:break;
			}
		}
		break;
		
		default:break;
	}
	MeasureModeChange();//������Ҫ��373��ɻ�����DTA0660��ɵĵ�λ
}

/*������Ӧ���ܵ�����DTA0660,������ǰ��λ�����ݴ���*/
void DataProcessing(void)
{

	if(holdstatus==state0)
	{
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://��ťVA��:
			{
				rangenum = ReadDTAValue(ReadRange);
				receive_f= ReadDTAValue(ReadResult);	
				
				/*����DC��AC��ѹ�������㣬THD��CF��Inrush��Peak+-��Phase����*/
				dealwith_information();
			}break;
			case KEY_VALUE_7://��ťW����
			{
				rangenum = ReadDTAValue(ReadRange);
				receive_f= ReadDTAValue(ReadResult);
							
				/*���й��ʡ����๦�ʡ�PF��DPF��THD��KWh��KVAh��Kvarh��KgCO2����*/
				dealwith_information();
			}break;
			case KEY_VALUE_8://��ťDCV/ACV����
			{
				if(paramstatus == state0)
				{//��DCV/ACV��
					/*SDADC��������DTA0660���������н�����ѹ��ֱ����ѹ��������ѹ����
					��ת������Ҫ��Ӳ����·���ص���Ӧ�ĵ�ѹ���պ�*/
					receive_f= ReadDTAValue(ReadResult);
				}
				else if(paramstatus == state1)
				{//Hz��
					receive_f= ReadDTAValue(ReadFreq);
				}
				else if(paramstatus == state2)
				{//Duty��
					receive_f1= ReadDTAValue(ReadDuty);
				}
			}break;
			case KEY_VALUE_9://��ťͨ�ϡ������ܵ���
			{
				/*SDADC��������DTA0660����*/
				switch(funcstatus)
				{
					case state0://����Cont
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state1://����Diode
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					default:break;
				}
			}break;
			case KEY_VALUE_10://��ťŷķ�����ݡ��¶ȡ�Ƶ�ʵ���
			{
			/*SDADC��������DTA0660����*/
				switch(funcstatus)
				{
					case state0://����Ohm
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state1://����Cap
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state2://����Temp(C)
					{
						receive_f= ReadDTAValue(ReadResult);
					}break;
					case state3://����Temp(F)
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

void display_bt_state(void)/*��ʾbluetooth״̬*/
{
	switch(btstatus)
	{
		case state0:lcd_write_1bit(0x1D,2,DISABLE);break;//BT��
		case state1:lcd_write_1bit(0x1D,2,ENABLE);break;//BT��
		default:break;
	}
}

void display_max_min_state(void)/*��ʾMAX-MIN״̬*/
{
	switch(max_minstatus)
	{
		case state0://��ʾ����ֵ
		{
			lcd_write_1bit(0x0A,0,DISABLE);
			lcd_write_1bit(0x0A,1,DISABLE);
		}break;
		case state1://��ʾMAX
		{
			lcd_write_1bit(0x0A,0,DISABLE);
			lcd_write_1bit(0x0A,1,ENABLE);
		}break;
		case state2://��ʾMIN
		{
			lcd_write_1bit(0x0A,0,ENABLE);
			lcd_write_1bit(0x0A,1,DISABLE);
		}break;
		default:break;
	}
}

void display_rel_state(void)/*��ʾREL״̬*/
{
	switch(relstatus)
	{
		case state0://��ʾ����ֵ
		{
			lcd_write_1bit(0x09,0,DISABLE);
		}break;
		case state1://��ʾ���ֵ
		{
			lcd_write_1bit(0x09,0,ENABLE);
		}break;
		default:break;
	}
}

void display_range_state(void)/*��ʾ����*/
{
	if(autostatus == state1)
	{
		lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
		lcd_write_1bit(0x1C,3,DISABLE);//MANU��
	}
	else
	{
		lcd_write_1bit(0x1C,2,DISABLE);//AUTO��
		lcd_write_1bit(0x1C,3,ENABLE);//MANU��
	}
}

float Calculate_max_min_data(float data,u8 n)/*���㡢��¼�����Сֵ   n:1 or 2����ʾ���򣬵�һ�� or �ڶ���*/
{
	if(n==1)
	{
		switch(max_minstatus)
		{
			case state0://����ֵ
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
			case state0://����ֵ
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

float Calculate_rel_data(float data,u8 n)/*���㡢��¼���ֵ   n:1 or 2����ʾ���򣬵�һ�� or �ڶ���*/
{
	if(n==1)
		return(data-rel_data1);
	else if(n==2)
		return(data-rel_data2);
}

float deal_1(float read_data,u8 n)/*��������---MAX-MIN��REL*/
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

void LowPowerDetect(void)/*��ʼʱ��Ϊ���͵�ѹλ���ڲ�RAM��LCD��BUFFER��0x41H��ַ�ĵڵ�4λ����0000,1000*/
{//20160312-2257���֣�0x42H�����λ���ǵ͵�ѹ��ʾλ
	Send(SetAddr,0x42,0x00,SetAddr+0x42);//��λ��LCD��BUFFER��0x42H��������ⷢ���Ƿ�ѹ����ѡ�񲻶ԣ�ֻ�е���ѹ>9.4Vʱ��MES_BAT�Ż�С��1.2V
	check_flag= CheckReceive(SetAddr);
	if(check_flag==1)//��λ��0x42H�ɹ�
	{
		check_flag= 0;
		Send(Read,Read);
		check_flag= CheckReceive(Read);
		if(check_flag==1)//����ǰ��ַ0x42H�ɹ�
		{
			check_flag= 0;
			
//			lcd_clr();//20160312ʱ����Ƿ��յ�LCD��Bufferʱ��ʱд��
//			lcd_show_dta_num((RxBuffer[1] & 0xF0)>>4,1);
//			lcd_show_dta_num((RxBuffer[1] & 0x0F),2);
			
			if((RxBuffer[1] & 0x01) != 0)
				lcd_write_1bit(0x1D,3,ENABLE);//�����
			else
				lcd_write_1bit(0x1D,3,DISABLE);//�����
		}
	}
}

/*��ť����ť��ز���*/
void manipulate(void)
{
	u8 i;
	/*������ת���ظı�ʱ*/
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
				lcd_show_ram[14]=0x24;//AC~��//AUTO��
				lcd_show_ram[12]=0x08;//V��3����
				lcd_show_ram[10]=0x80;//A��2����
			}break;
			case KEY_VALUE_7://W
			{
				lcd_show_ram[14]=0x04;//AUTO��
//				lcd_show_ram[12]=0x10;//W��
				lcd_show_ram[13]=0x11;//VA��1����
				lcd_show_ram[11]=0x04;//PF��
			}break;
			case KEY_VALUE_8://mV/V
			{
				lcd_show_ram[14]=0x05;//DC-��//AUTO��
				lcd_show_ram[12]=0x08;//V��3����
			}break;
			case KEY_VALUE_9://CON/DIODE
			{
				lcd_show_ram[0]=0x01;//������
				lcd_show_ram[12]=0x20;//ŷķ��
			}break;
			case KEY_VALUE_10://OHM/CAP/TMP
			{
				lcd_show_ram[12]=0x20;//ŷķ��
				lcd_show_ram[14]=0x04;//AUTO��
			}break;
			default:break;
		}
		lcd_full_ram();
		display_bt_state();
		MeasureFunctionSelection();
	}
	/*������ť�ı�ʱ*/
	if(SoftKeyChanged_flag == 1)//������ť�ı�
	{
		Data_init();
		SoftKeyChanged_flag = 0;
		BUZZER_Open(0);
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://V+A ��λ
			{
				if((ShortKey_flag == 1) && (longparamstatus == state1) && (SoftKeyValue != KEY_VALUE_3) && (SoftKeyValue != KEY_VALUE_5) && (holdstatus==state0))
				{//��֮ǰ����Inrush״̬���̰��������������˳���״̬
					ShortKey_flag=0;
					longparamstatus=state0;
					
					lcd_clr();
					
					lcd_write_1bit(0x1D,1,ENABLE);//AC��
					lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
					lcd_write_1bit(0x18,3,ENABLE);//V��3����
					lcd_write_1bit(0x15,3,ENABLE);//A��2����
					inrush_trigger_flag=0;
				}
				switch(SoftKeyValue)//�ж���ʲô��ť
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (paramstatus == state0) && (holdstatus==state0))
						{//��FUNC���ֶ̰������Ҵ��ڷ�Peak��Phase��Param��HOLD״̬��
//							ShortKey_flag=0;//����ı书��Ҫ��
							
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							rangestatus=state0;
							longparamstatus=state0;
							autostatus=state1;
							lcd_clr();
							switch(funcstatus)//����Ҫ�ȸı�FUNC��״̬���ٸ�����״̬����
							{
								case state0:funcstatus = state1;break;//AC V+A		��ΪDC V+A
								case state1:funcstatus = state2;break;//DC V+A		��ΪAC+DC V+A
								case state2:funcstatus = state0;break;//AC+DC V+A	��ΪAC V+A
								default:break;
							}
							
							switch(funcstatus)//��ʾFUNC
							{
								case state0://����AC V+A
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC��
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
									lcd_write_1bit(0x15,3,ENABLE);//A��2����
								}break;
								case state1://����DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC��
									lcd_write_1bit(0x1D,1,DISABLE);//AC��
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
									lcd_write_1bit(0x15,3,ENABLE);//A��2����
								}break;
								case state2://����AC+DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC��
									lcd_write_1bit(0x1D,0,ENABLE);//+��
									lcd_write_1bit(0x1D,1,ENABLE);//AC��
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
									lcd_write_1bit(0x15,3,ENABLE);//A��2����
								}break;
								default:break;
							}
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//��MAX-MIN���ֶ̰������Ҵ��ڷ�Peak��Phase��HOLD״̬��
							ShortKey_flag=0;
							relstatus=state0;
							longparamstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//����ֵ��ΪMAX
								case state1:max_minstatus = state2;break;//MAX��ΪMIN
								case state2:max_minstatus = state0;break;//MIN��Ϊ����ֵ
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{//������MAX-MIN��������MAX-MIN״̬��������¼�´�ʱ��ֵ
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
						{//��MAX-MIN���ֶ̰������Ҵ���Peak״̬��
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state1:max_minstatus = state2;break;//Peak-MAX��ΪPeak-MIN
								case state2:max_minstatus = state1;break;//Peak-MIN��ΪPeak-MAX
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
						}
						else if((LongKey_flag == 1) && (phasestatus == state0) && (paramstatus == state0) && (longparamstatus==state0) && (holdstatus==state0))
						{//��MAX-MIN���ֳ��������Ҵ��ڷ�max-min��Phase��Param��LongParam��HOLD״̬��
							LongKey_flag=0;
							relstatus=state0;
							
							if((max_minstatus==state0) && (peakstatus==state0))
							{
								peakstatus = state1;
								max_minstatus = state1;
								lcd_write_1bit(0x09,1,ENABLE);//PEAK��
							}
							else if(peakstatus!=state0)
							{
								peakstatus = state0;
								max_minstatus = state0;
								lcd_write_1bit(0x09,1,DISABLE);//PEAK��
							}
							display_max_min_state();
							display_rel_state();
							
							if(peakstatus == state1)
							{//������PEAK״̬��������¼�´�ʱ��ֵ
								max_data1=maxv_value;
								min_data1=minv_value;
								
								max_data2=maxi_value;
								min_data2=mini_value;
							}
						}
					}break;
					case KEY_VALUE_2://REL    //20160727  lea  �޸��߼�Ϊ������һֱ��ʾΪ���ֵ ������ֻ��ʾһ�Ρ� 
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//��REL���ֶ̰������Ҵ��ڷ�Peak��Phase��HOLD״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							longparamstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//����ֵ��Ϊ���ֵ
								case state1:relstatus = state0;break;//���ֵ��Ϊ����ֵ
								default:break;
							}
							display_rel_state();
							display_max_min_state();	
							
							if(relstatus != state0)//�����ֵ״̬��һ������ֵ   //20160727 lea ����Ĳ���ֵ��ʹ�õ���ʱ���ݱ�����������λ�Ļ�û������Ӧ���޸ģ��������£�����
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
						{//��REL���ֳ��������Ҵ��ڷ�Peak��Param��LongParam��HOLD״̬��
							LongKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							autostatus=state1;
							phaseABCstatus=state0;
							
							switch(phasestatus)
							{
								case state0:phasestatus = state1;break;//����ֵ��ΪPhase
								case state1:phasestatus = state0;break;//Phase��Ϊ����ֵ
								default:break;
							}
							
							switch(phasestatus)
							{
								case state0://����ֵ
								{
									lcd_write_1bit(0x1B,3,DISABLE);//L1-L2-L3
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
									lcd_write_1bit(0x15,3,ENABLE);//A��2����
									TIM_Cmd(TIM14, DISABLE);//0.5s��ʱ�ر�
								}break;
								case state1://PHASE
								{
									lcd_write_1bit(0x0,2,DISABLE);//��һ�и�����
									lcd_write_1bit(0xA,2,DISABLE);//�ڶ��и�����
									lcd_write_1bit(0x1B,3,ENABLE);//L1-L2-L3
									lcd_write_1bit(0x18,3,DISABLE);//V��3����
									lcd_write_1bit(0x15,3,DISABLE);//A��2����
									lcd_show_None(1);
									lcd_show_None(2);
									TIM_Cmd(TIM14, ENABLE);//0.5s��ʱ����
									
									TIM5CH4_CAPTURE_STA=0;//��ʱ��5����״̬��δ����ʱΪ0��A��0x40��B or C��0x80
									
									COMP_Cmd(COMP_Selection_COMP1, DISABLE);//�رձȽ��� 20160603lea
									TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);//��Ƚ����ж�
									
									TIM_Cmd(TIM19, ENABLE);//20160602lea ����phase ģʽ��Ƚ��� ���ڴ���������λʱ��  
																				//���Ա�����������㴥��AC����������ʱֱ�ӿ�ʼ����������������һ��
									
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
						{//��RANGE���ֶ̰������Ҵ��ڷ�Peak��Phase��Param��HOLD״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							
							if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
									}break;//6.000V/600.0A��Ϊ60.00V/600.0A
									case state1:
									{
										rangestatus = state2;
										RangeSet(state2);
									}break;//60.00V/600.0A��Ϊ600.0V/600.0A
									case state2:
									{
										rangestatus = state3;
										RangeSet(state0);
									}break;//600.0V/600.0A��Ϊ6.000V/2000A
									case state3:
									{
										rangestatus = state4;
										RangeSet(state1);
									}break;//6.000V/2000A��Ϊ60.00V/2000A
									case state4:
									{
										rangestatus = state5;
										RangeSet(state2);
									}break;//60.00V/2000A��Ϊ600.0V/2000A
									case state5:
									{
										rangestatus = state0;
										RangeSet(state0);
									}break;//600.0V/2000A��Ϊ6.000V/600.0A
									default:break;
								}
							}
							
							display_range_state();
							display_rel_state();
							display_max_min_state();
						}
						else if((ShortKey_flag == 1) && (longparamstatus == state1) && (holdstatus==state0))
						{//��RANGE���ֶ̰������Ҵ���Inrush״̬��
							ShortKey_flag=0;
							
							inrush_trigger_flag=0;//������ӿ���������̸ı�ʱ�����㴥����־λ
							lcd_show_Line(1);
							
							switch(rangestatus)
							{
								case state0:rangestatus = state1;break;//600.0A	��Ϊ2000A
								case state1:rangestatus = state0;break;//2000A	��Ϊ600.0A
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
						{//��RANGE���ֳ��������Ҵ��ڷ�Peak��HOLD״̬��
							LongKey_flag=0;
							
							if(autostatus != state1)//�������Զ����������ֶ�����Ϊ�Զ���
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
								lcd_write_1bit(0x1C,3,DISABLE);//MANU��
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (phasestatus == state0) && (holdstatus==state0))
						{//��PARAM���ֶ̰������Ҵ��ڷ�Peak��Phase��HOLD״̬��
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
								case state0://��ʾĬ��ֵ
								{
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
									lcd_write_1bit(0x15,3,ENABLE);//A��2����
								}break;
								case state1://��ʾTHD%f
								{
									lcd_write_1bit(0x13,0,ENABLE);//THD%
									lcd_write_1bit(0x15,0,ENABLE);//f
								}break;
								case state2://��ʾTHD%r
								{
									lcd_write_1bit(0x13,0,ENABLE);//THD%
									lcd_write_1bit(0x14,0,ENABLE);//r
								}break;
								case state3://��ʾCF
								{
									lcd_write_1bit(0x16,1,ENABLE);//CF
								}break;
								default:break;
							}
							switch(funcstatus)//��ʾFUNC
							{
								case state0://����AC V+A
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC��
								}break;
								case state1://����DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC��
								}break;
								case state2://����AC+DC V+A
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC��
									lcd_write_1bit(0x1D,0,ENABLE);//+��
									lcd_write_1bit(0x1D,1,ENABLE);//AC��
								}break;
								default:break;
							}
							display_max_min_state();
							display_rel_state();
							display_range_state();
						}
						else if(LongKey_flag == 1  && (peakstatus==state0) && (phasestatus == state0) && (paramstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//��PARAM���ֳ��������Ҵ��ڷ�Peak��Phase��Param��LongParam��HOLD״̬��
							LongKey_flag=0;
							
							max_minstatus=state0;
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							longparamstatus = state1;
							autostatus=state1;
							lcd_clr();
							
							lcd_write_1bit(0x17,2,ENABLE);//Inrush��
							lcd_write_1bit(0x1A,0,ENABLE);//A(1)��
							lcd_write_1bit(0x15,3,ENABLE);//A(2)��
							lcd_show_Line(1);
							lcd_show_num(600,2,3);
							
							display_rel_state();
							display_max_min_state();
							display_range_state();
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(Is_Cal_Mode == 1)//У׼ģʽ��hold��ΪУ׼�������
						{
							if(RotaryKeyValue==KEY_VALUE_6)//V+A ��λ 
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
							{//��HOLD���ֶ̰�
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD���
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://��ʾ����ֵ
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://����
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//��HOLD���ֳ���
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//����
									case state1:lightstatus = state0;break;//���
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://����
									{
									}break;
									case state1://���
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
				switch(SoftKeyValue)//�ж���ʲô��ť
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (longparamstatus == state0) && (holdstatus==state0))
						{//��FUNC���ֶ̰������Ҵ��ڷǳ���״̬��
//							ShortKey_flag=0;//����ı书��Ҫ��
							
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							rangestatus=state0;
//							paramstatus=state0;
							
//							lcd_clr();
							
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//VA��ΪW
								case state1:funcstatus = state2;break;//W��Ϊvar
								case state2:funcstatus = state0;break;//var��ΪVA
								default:break;
							}
							switch(funcstatus)
							{
								case state0://����VA
								{
									lcd_write_1bit(0x17,1,DISABLE);//Var��
									lcd_write_1bit(0x1A,0,ENABLE);//A��1����
									lcd_write_1bit(0x1B,0,ENABLE);//V��1����
								}break;
								case state1://����W
								{
									lcd_write_1bit(0x1A,0,DISABLE);//A��1����
									lcd_write_1bit(0x1B,0,DISABLE);//V��1����
									lcd_write_1bit(0x19,0,ENABLE);//W��
								}break;
								case state2://����Var
								{
									lcd_write_1bit(0x19,0,DISABLE);//W��
									lcd_write_1bit(0x17,1,ENABLE);//Var��
								}break;
								default:break;
							}
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
						if((ShortKey_flag == 1) && (peakstatus != state0) && (holdstatus==state0))
						{//��FUNC���ֶ̰������Ҵ��ڶ��๦�ʲ���״̬��
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//VA��ΪW
								case state1:funcstatus = state2;break;//W��Ϊvar
								case state2:funcstatus = state0;break;//var��ΪVA
								default:break;
							}
							switch(funcstatus)
							{
								case state0://����VA
								{
									lcd_write_1bit(0x17,1,DISABLE);//Var��
									lcd_write_1bit(0x1A,0,ENABLE);//A��1����
									lcd_write_1bit(0x1B,0,ENABLE);//V��1����
								}break;
								case state1://����W
								{
									lcd_write_1bit(0x1A,0,DISABLE);//A��1����
									lcd_write_1bit(0x1B,0,DISABLE);//V��1����
									lcd_write_1bit(0x19,0,ENABLE);//W��
								}break;
								case state2://����Var
								{
									lcd_write_1bit(0x19,0,DISABLE);//W��
									lcd_write_1bit(0x17,1,ENABLE);//Var��
								}break;
								default:break;
							}
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (peakstatus == state0) && (holdstatus==state0))
						{//��MAX-MIN���ֶ̰������Ҵ��ڷǳ���״̬��
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//����ֵ��ΪMAX
								case state1:max_minstatus = state2;break;//MAX��ΪMIN
								case state2:max_minstatus = state0;break;//MIN��Ϊ����ֵ
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)
							{//������MAX-MIN��������MAX-MIN״̬��������¼�´�ʱ��ֵ
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
						{//��MAX-MIN���ֳ��������Ҵ��ڷ���������״̬�£����๦�ʲ��������л�
							LongKey_flag=0;
							relstatus=state0;
							paramstatus=state0;
//							funcstatus = state0;
							phasestatus = state0;
							lcd_clr();
							switch(peakstatus)//�ı�״̬
							{
								case state0:peakstatus = state1;break;//����ģʽ��Ϊ���๦��ģʽ
								case state1:peakstatus = state0;break;//���๦��ģʽ��Ϊ����ģʽ
								default:break;
							}
							switch(peakstatus)//��״̬��ʾ
							{
								case state0://����
								{
									switch(funcstatus)
									{
										case state0://����VA
										{
											lcd_write_1bit(0x1A,0,ENABLE);//A��1����
											lcd_write_1bit(0x1B,0,ENABLE);//V��1����
										}break;
										case state1://����W
										{
											lcd_write_1bit(0x19,0,ENABLE);//W��
										}break;
										case state2://����Var
										{
											lcd_write_1bit(0x17,1,ENABLE);//Var��
										}break;
										default:break;
									}
									lcd_write_1bit(0x16,2,ENABLE);//PF��
									TIM_Cmd(TIM14, DISABLE);//0.5s��ʱ�ر�
								}break;
								case state1://���๦�ʳ�ʼ��ʾ1phi
								{
									lcd_write_1bit(0x09,3,ENABLE);//1
									lcd_write_1bit(0x0B,0,ENABLE);//PHI
									switch(funcstatus)//���ݵ�ǰ������λ����ʾ��Ӧ��λ
									{
										case state0://����VA
										{
											lcd_write_1bit(0x1A,0,ENABLE);//A��1����
											lcd_write_1bit(0x1B,0,ENABLE);//V��1����
										}break;
										case state1://����W
										{
											lcd_write_1bit(0x19,0,ENABLE);//W��
										}break;
										case state2://����Var
										{
											lcd_write_1bit(0x17,1,ENABLE);//Var��
										}break;
										default:break;
									}
									TIM_Cmd(TIM14, ENABLE);//0.5s��ʱ����
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
						{//��REL���ֶ̰������Ҵ��ڷǳ���״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//����ֵ��Ϊ���ֵ
								case state1:relstatus = state0;break;//���ֵ��Ϊ����ֵ
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//�����ֵ״̬��һ������ֵ
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
						{//��REL���ֶ̰������Ҵ��ڶ��๦�ʲ�������״̬��
							switch(phasestatus)
							{//�����л����ܣ�����¼�µ�ǰ����ֵ
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
								case state0://1phi��
								{
//									lcd_write_1bit(0x09,3,ENABLE);//1����˸
								}
								break;
								case state1://2phi��
								{
									lcd_write_1bit(0x09,3,DISABLE);//1��
//									lcd_write_1bit(0x0A,3,ENABLE);//2����˸
								}
								break;
								case state2://3phi��
								{
									lcd_write_1bit(0x0A,3,DISABLE);//2��
//									lcd_write_1bit(0x0B,1,ENABLE);//3����˸
								}
								break;
								case state3://1phi����
								{
									lcd_write_1bit(0x09,2,DISABLE);//����
									lcd_write_1bit(0x0B,1,DISABLE);//3��
									lcd_write_1bit(0x09,3,ENABLE);//1��
									lcd_write_1bit(0x0B,0,ENABLE);//PHI��
									TIM_Cmd(TIM14, DISABLE);//0.5s��ʱ�ر�
								}
								break;
								case state4://2phi����
								{
									lcd_write_1bit(0x09,3,DISABLE);//1��
									lcd_write_1bit(0x0A,3,ENABLE);//2��
								}
								break;
								case state5://3phi����
								{
									lcd_write_1bit(0x0A,3,DISABLE);//2��
									lcd_write_1bit(0x0B,1,ENABLE);//3��
								}
								break;
								case state6://sigma 3phi����
								{
									lcd_write_1bit(0x09,2,ENABLE);//����
//									lcd_write_1bit(0x0B,1,ENABLE);//3��
								}
								break;
								default:break;
							}
						}
						else if((LongKey_flag == 1) && (peakstatus==state0) && (holdstatus==state0))
						{//��REL���ֳ��������Ҵ��ڷ���������״̬��
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
						{//��PARAM���ֶ̰������Ҵ��ڷǳ���״̬��
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
								case state0://��ʾPF
								{
									lcd_write_1bit(0x13,0,DISABLE);//THD%��
									lcd_write_1bit(0x14,0,DISABLE);//r��
									lcd_write_1bit(0x16,2,ENABLE);//PF��
								}break;
								case state1://��ʾD+PF
								{
									lcd_write_1bit(0x16,3,ENABLE);//D��
//									lcd_write_1bit(0x16,2,ENABLE);//PF��
								}break;
								case state2://��ʾTHD%f
								{
									lcd_write_1bit(0x16,3,DISABLE);//D��
									lcd_write_1bit(0x16,2,DISABLE);//PF��
									lcd_write_1bit(0x13,0,ENABLE);//THD%��
									lcd_write_1bit(0x15,0,ENABLE);//f��
								}break;
								case state3://��ʾTHD%r
								{
									lcd_write_1bit(0x15,0,DISABLE);//f��
//									lcd_write_1bit(0x13,0,ENABLE);//THD%��
									lcd_write_1bit(0x14,0,ENABLE);//r��
								}break;
								default:break;
							}
//							display_max_min_state();
//							display_rel_state();
//							display_range_state();
						}
						else if((ShortKey_flag == 1) && (longparamstatus == state1) && (holdstatus==state0))
						{//�ڵ���ģʽ�£�PARAM���ֶ̰�
							ShortKey_flag=0;
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							
//							lcd_clr();
							switch(paramstatus)
							{
								case state0:paramstatus = state1;break;//KWh��ΪKVAh
								case state1:paramstatus = state2;break;//KVAh��ΪKvarh
								case state2:paramstatus = state3;break;//Kvarh��ΪKgCO2
								case state3:paramstatus = state0;break;//KgCO2��ΪKWh
								default:break;
							}
							switch(paramstatus)
							{
								case state0://����KWh
								{
									lcd_write_1bit(0x08,0,ENABLE);//k(3)��
									lcd_write_1bit(0x19,0,ENABLE);//W��
									lcd_write_1bit(0x18,0,ENABLE);//h��
									
									lcd_write_1bit(0x17,3,DISABLE);//KgCO2��
								}break;
								case state1://����KVAh
								{
//									lcd_write_1bit(0x08,0,ENABLE);//k(3)��
									lcd_write_1bit(0x1A,0,ENABLE);//V��1����
									lcd_write_1bit(0x1B,0,ENABLE);//A��1����
//									lcd_write_1bit(0x18,0,ENABLE);//h��
									
									lcd_write_1bit(0x19,0,DISABLE);//W��
								}break;
								case state2://����Kvarh
								{
									lcd_write_1bit(0x18,2,ENABLE);//k(1)��
									lcd_write_1bit(0x17,1,ENABLE);//var��
//									lcd_write_1bit(0x18,0,ENABLE);//h��
									
									lcd_write_1bit(0x08,0,DISABLE);//k(3)��
									lcd_write_1bit(0x1A,0,DISABLE);//V��1����
									lcd_write_1bit(0x1B,0,DISABLE);//A��1����
								}break;
								case state3://����KgCO2
								{
									lcd_write_1bit(0x17,3,ENABLE);//KgCO2��
									
									lcd_write_1bit(0x18,2,DISABLE);//k(1)��
									lcd_write_1bit(0x17,1,DISABLE);//var��
									lcd_write_1bit(0x18,0,DISABLE);//h��
								}break;
								default:break;
							}
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
						else if((LongKey_flag == 1) && (peakstatus == state0) && (holdstatus==state0))
						{//��PARAM���ֳ��������Ҵ��ڷ���������״̬��
							LongKey_flag=0;
							funcstatus = state0;
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							rangestatus=state0;
							paramstatus=state0;
							
							powertimer=0;
							kWh=0;kVAh=0;kVarh=0;
							
							lcd_clr();
							switch(longparamstatus)
							{
								case state0:longparamstatus = state1;break;//����ģʽ��Ϊ����ģʽ
								case state1:longparamstatus = state0;break;//����ģʽ��Ϊ����ģʽ
								default:break;
							}
							switch(longparamstatus)
							{
								case state0://����ʾ�ܹ�������ʾPF
								{
									lcd_write_1bit(0x1A,0,ENABLE);//A��1����
									lcd_write_1bit(0x1B,0,ENABLE);//V��1����
									lcd_write_1bit(0x16,2,ENABLE);//PF��
									
									TIM_Cmd(TIM12, DISABLE);
								}break;
								case state1://����KWh
								{
									lcd_write_1bit(0x08,0,ENABLE);//k(3)��
									lcd_write_1bit(0x19,0,ENABLE);//W��
									lcd_write_1bit(0x18,0,ENABLE);//h��
									
									TIM_Cmd(TIM12, ENABLE);/*1s��ʱ*/
									
								}break;
								default:break;
							}
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(ShortKey_flag == 1)
						{//��HOLD���ֶ̰�
							ShortKey_flag=0;
							
							switch(holdstatus)
							{
								case state0:holdstatus = state1;break;//HOLD
								case state1:holdstatus = state0;break;//HOLD���
								default:break;
							}
							
							switch(holdstatus)
							{
								case state0://��ʾ����ֵ
								{
									lcd_write_1bit(0x1C,1,DISABLE);
								}break;
								case state1://����
								{
									lcd_write_1bit(0x1C,1,ENABLE);
								}break;
								default:break;
							}
						}
						else if(LongKey_flag == 1)
						{//��HOLD���ֳ���
							LongKey_flag=0;
							
							switch(lightstatus)
							{
								case state0:lightstatus = state1;break;//����
								case state1:lightstatus = state0;break;//���
								default:break;
							}
							
							switch(lightstatus)
							{
								case state0://����
								{
								}break;
								case state1://���
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
				switch(SoftKeyValue)//�ж���ʲô��ť
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (paramstatus == state0) && (holdstatus==state0))
						{//��FUNC���ֶ̰������Ҵ��ڷ�Hz-duty/HOLD״̬��
//							ShortKey_flag=0;//����ı书��Ҫ��
							
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							rangestatus=state0;
							autostatus=state1;
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//DCV��ΪACV
								case state1:funcstatus = state0;break;//ACV��ΪDCV
								default:break;
							}
							switch(funcstatus)
							{
								case state0://DCV
								{
									lcd_write_1bit(0x1C,0,ENABLE);//DC��
									lcd_write_1bit(0x18,3,ENABLE);//V��
								}break;
								case state1://ACV
								{
									lcd_write_1bit(0x1D,1,ENABLE);//AC��
									lcd_write_1bit(0x18,3,ENABLE);//V��
								}break;
								default:break;
							}
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��MAX-MIN���ֶ̰������Ҵ��ڷ�HOLD״̬��
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//����ֵ��ΪMAX
								case state1:max_minstatus = state2;break;//MAX��ΪMIN
								case state2:max_minstatus = state0;break;//MIN��Ϊ����ֵ
								default:break;
							}
							
							display_max_min_state();
							display_rel_state();
							
							if(max_minstatus == state1)//������MAX-MIN״̬����¼�´�ʱ��ֵΪ�����Сֵ
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
						{//��REL���ֶ̰������Ҵ��ڷ�HOLD״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//����ֵ��Ϊ���ֵ
								case state1:relstatus = state0;break;//���ֵ��Ϊ����ֵ
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//�����ֵ״̬��һ������ֵ
								rel_data1 = receive_f;//ReadDTAValue(ReadFreq);//��DTA0660��ȡ����
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
						if((ShortKey_flag == 1) && (paramstatus == state0) && (holdstatus==state0))
						{//��RANGE���ֶ̰������Ҵ��ڷ�Hz-duty/HOLD״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							switch(funcstatus)
							{
								case state0://DCV
								{
									if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
											case state0://600.0mV��Ϊ6.000V
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://6.000V��Ϊ60.00V
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://60.00V��Ϊ600.0V
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://600.0V��Ϊ1000V
											{
												rangestatus = state4;
												RangeSet(state4);
											}break;
											case state4://1000V��Ϊ600.0mV
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
									if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
											case state0://6.000V��Ϊ60.00V
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://60.00V��Ϊ600.0V
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://600.0V��Ϊ1000V
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://1000V��Ϊ6.000V
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
						{//��RANGE���ֶ̰������Ҵ���Hz״̬����HOLD״̬��
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							
							if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
									case state0://99.99��Ϊ999.9
									{
										rangestatus = state1;
										RangeSet(state1);
									}break;
									case state1://999.9��Ϊ9.999k
									{
										rangestatus = state2;
										RangeSet(state2);
									}break;
									case state2://9.999k��Ϊ99.99k
									{
										rangestatus = state3;
										RangeSet(state3);
									}break;
									case state3://99.99k��Ϊ999.9k
									{
										rangestatus = state4;
										RangeSet(state4);
									}break;
									case state4://999.9k��Ϊ9.999M
									{
										rangestatus = state5;
										RangeSet(state5);
									}break;
									case state5://9.999M��Ϊ99.99
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
						{//��RANGE���ֳ��������Ҵ��ڷ�HOLD״̬��
							LongKey_flag=0;
							
							if(autostatus != state1)//�������Զ���
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
								lcd_write_1bit(0x1C,3,DISABLE);//MANU��
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
						if((ShortKey_flag == 1) && (funcstatus == state1) && (holdstatus==state0))//�л�HZ����
						{//��PARAM���ֶ̰������Ҵ���ACV/��HOLD״̬��
//							ShortKey_flag=0;//����ı书��Ҫ��
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
								lcd_write_1bit(0x18,3,ENABLE);//V��
//								lcd_write_1bit(0x14,1,DISABLE);//%��
//								lcd_show_None(2);//����ʾ��
							}
							else if(paramstatus == state1)//Ƶ�ʵ�
							{
//								lcd_write_1bit(0x15,2,ENABLE);//Hz(2)��
								lcd_write_1bit(0x18,1,ENABLE);//Hz(1)��
							}
							else if(paramstatus == state2)//ռ�ձȵ�
							{
//								lcd_write_1bit(0x15,2,DISABLE);//Hz(2)��
								lcd_write_1bit(0x14,1,ENABLE);//%��
							}
							lcd_write_1bit(0x1D,1,ENABLE);//AC��
							
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{	
						if(Is_Cal_Mode == 1)//У׼ģʽ��hold��ΪVmVУ׼����
						{
							if(VmV_value<1000)//0 
							{
								SaveData.Value.Cal_VmV_zero[rangestatus]=VmV_value/SaveData.Value.Cal_VmV_gain[rangestatus]+SaveData.Value.Cal_VmV_zero[rangestatus];
							}
							else if(VmV_value>3500)//5000
							{
								SaveData.Value.Cal_VmV_gain[rangestatus]= 5000*SaveData.Value.Cal_VmV_gain[rangestatus]/VmV_value;
							}
							updata_flash();	//��������	
						}
						else//����״̬
						{
							if(ShortKey_flag == 1)
							{//��HOLD���ֶ̰�
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD���
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://��ʾ����ֵ
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://����
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//��HOLD���ֳ���
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//����
									case state1:lightstatus = state0;break;//���
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://����
									{
									}break;
									case state1://���
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
				switch(SoftKeyValue)//�ж���ʲô��ť
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��FUNC���ֶ̰����ҷ�HOLD״̬
//							ShortKey_flag=0;//����ı书��Ҫ��
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//CONT��ΪDiode
								case state1:funcstatus = state0;break;//Diode��ΪCONT
								default:break;
							}
							switch(funcstatus)
							{
								case state0://CONT
								{
									lcd_write_1bit(0x00,0,ENABLE);//������
									lcd_write_1bit(0x19,1,ENABLE);//ŷķ��
								}break;
								case state1://Diode
								{
									lcd_write_1bit(0x00,1,ENABLE);//��������
									lcd_write_1bit(0x18,3,ENABLE);//V��3����
								}break;
								default:break;
							}
							
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��MAX-MIN���ֶ̰����ҷ�HOLD״̬
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//����ֵ��ΪMAX
								case state1:max_minstatus = state2;break;//MAX��ΪMIN
								case state2:max_minstatus = state0;break;//MIN��Ϊ����ֵ
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
						{//��REL���ֶ̰����ҷ�HOLD״̬
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//����ֵ��Ϊ���ֵ
								case state1:relstatus = state0;break;//���ֵ��Ϊ����ֵ
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//�����ֵ״̬��һ������ֵ
								rel_data1 = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
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
						{//��HOLD���ֶ̰�
							ShortKey_flag=0;
							
							switch(holdstatus)
							{
								case state0:holdstatus = state1;break;//HOLD
								case state1:holdstatus = state0;break;//HOLD���
								default:break;
							}
							
							switch(holdstatus)
							{
								case state0://��ʾ����ֵ
								{
									lcd_write_1bit(0x1C,1,DISABLE);
								}break;
								case state1://����
								{
									lcd_write_1bit(0x1C,1,ENABLE);
								}break;
								default:break;
							}
						}
						else if(LongKey_flag == 1)
						{//��HOLD���ֳ���
							LongKey_flag=0;
							
							switch(lightstatus)
							{
								case state0:lightstatus = state1;break;//����
								case state1:lightstatus = state0;break;//���
								default:break;
							}
							
							switch(lightstatus)
							{
								case state0://����
								{
								}break;
								case state1://���
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
				switch(SoftKeyValue)//�ж���ʲô��ť
				{
					case KEY_VALUE_0://FUNC
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��FUNC���ֶ̰����ҷ�HOLD״̬
//							ShortKey_flag=0;//����ı书��Ҫ��
							
							max_minstatus=state0;//���ȼ��͵�״̬����
							relstatus=state0;
							rangestatus=state0;
							autostatus=state1;
							
							lcd_clr();
							switch(funcstatus)
							{
								case state0:funcstatus = state1;break;//Ohm��ΪCap
								case state1:funcstatus = state2;break;//Cap��ΪTemp(C)
								case state2:funcstatus = state3;break;//Temp(C)��ΪTemp(F)
								case state3:funcstatus = state0;break;//Temp(F)��ΪOhm
								default:break;
							}
							switch(funcstatus)
							{
								case state0://Ohm
								{
//									lcd_clr();
									lcd_write_1bit(0x19,1,ENABLE);//ŷķ��
									lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
								}break;
								case state1://Cap
								{
									lcd_write_1bit(0x19,3,ENABLE);//������
								}break;
								case state2://Temp(C)
								{
									lcd_write_1bit(0x15,1,ENABLE);//�¶�(C)��
								}break;
								case state3://Temp(F)
								{
									lcd_write_1bit(0x16,0,ENABLE);//�¶�(F)��
								}break;
								default:break;
							}
							
							display_max_min_state();//��ʾMAX-MIN
							display_rel_state();//��ʾREL
							display_range_state();//��ʾRANGE
						}
					}break;
					case KEY_VALUE_1://MAX-MIN
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��MAX-MIN���ֶ̰����ҷ�HOLD״̬
							ShortKey_flag=0;
							relstatus=state0;
							
							switch(max_minstatus)
							{
								case state0:max_minstatus = state1;break;//����ֵ��ΪMAX
								case state1:max_minstatus = state2;break;//MAX��ΪMIN
								case state2:max_minstatus = state0;break;//MIN��Ϊ����ֵ
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
						if(Is_Cal_Mode == 1)//У׼ģʽ��REL��ΪCap���У׼����
						{
							switch(funcstatus)
							{
								case state0://Ohm
								{
									break;
								}
								case state1://Cap
								{																		
									switch(rangestatus)//����ֵУ׼
									{										
										case 1://60.00nF  У׼50nF
										{
											SaveData.Value.Cal_Cap_zero[0]=Cap_value/SaveData.Value.Cal_Cap_gain[0]+SaveData.Value.Cal_Cap_zero[0];										
											break;
										}										
										case 3://6.000uF У׼5uF
										{
											SaveData.Value.Cal_Cap_zero[1]=Cap_value/SaveData.Value.Cal_Cap_gain[1]+SaveData.Value.Cal_Cap_zero[1];
											break;
										}										
										case 5://600.0uF У׼500uF
										{
											SaveData.Value.Cal_Cap_zero[2]=Cap_value/SaveData.Value.Cal_Cap_gain[2]+SaveData.Value.Cal_Cap_zero[2];
											break;
										}
										case 6://6.000mF У׼5mF
										{
											SaveData.Value.Cal_Cap_zero[3]=Cap_value/SaveData.Value.Cal_Cap_gain[0]+SaveData.Value.Cal_Cap_zero[3];
											break;
										}																
										default:break;
									}
									updata_flash();	//��������	
									break;
								}
								
								default:break;								
							}
						}
						else if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��REL���ֶ̰����ҷ�HOLD״̬
							ShortKey_flag=0;
							max_minstatus=state0;
							switch(relstatus)
							{
								case state0:relstatus = state1;break;//����ֵ��Ϊ���ֵ
								case state1:relstatus = state0;break;//���ֵ��Ϊ����ֵ
								default:break;
							}
							display_rel_state();
							display_max_min_state();
							
							if(relstatus != state0)//�����ֵ״̬��һ������ֵ
								rel_data1 = receive_f;
						}
					}break;
					case KEY_VALUE_3://RANGE
					{
						if((ShortKey_flag == 1) && (holdstatus==state0))
						{//��RANGE���ֶ̰����ҷ�HOLD״̬
							ShortKey_flag=0;
							max_minstatus=state0;
							relstatus=state0;
							switch(funcstatus)
							{
								case state0://Ohm
								{
									if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
											case state0://600.0����Ϊ6.000k��
											{
												rangestatus = state1;
												RangeSet(state1);
											}break;
											case state1://6.000k����Ϊ60.00k��
											{
												rangestatus = state2;
												RangeSet(state2);
											}break;
											case state2://60.00k����Ϊ600.0k��
											{
												rangestatus = state3;
												RangeSet(state3);
											}break;
											case state3://600.0k����Ϊ6.000M��
											{
												rangestatus = state4;
												RangeSet(state4);
											}break;
											case state4://6.000M����Ϊ60.00M��
											{
												rangestatus = state5;
												RangeSet(state5);
											}break;
											case state5://60.00M����Ϊ600.0��
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
									if(autostatus==state1)//�����Զ������̰�RANGE�����˳��Զ���
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
											case state0://6.000n��Ϊ60.00n
											{
												rangestatus = state1;
												RangeSet(state0);//0.000nF~999.999nF
											}break;
											case state1://60.00n��Ϊ600.0n
											{
												rangestatus = state2;
												RangeSet(state0);//0.000nF~999.999nF
											}break;
											case state2://600.0n��Ϊ6.000u
											{
												rangestatus = state3;
												RangeSet(state1);//0.000uF~9.999uF
											}break;
											case state3://6.000u��Ϊ60.00u
											{
												rangestatus = state4;
												RangeSet(state2);//0.00uF~999.99uF
											}break;
											case state4://60.00u��Ϊ600.0u
											{
												rangestatus = state5;
												RangeSet(state2);//0.00uF~999.99uF
											}break;
											case state5://600.0u��Ϊ6.000m
											{
												rangestatus = state6;
												RangeSet(state3);//0.000mF~99.999mF
											}break;
											case state6://6.000m��Ϊ60.00m
											{
												rangestatus = state7;
												RangeSet(state3);//0.000mF~99.999mF
											}break;
											case state7://60.00m��Ϊ6.000n
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
						{//��RANGE���ֳ������ҷ�HOLD״̬
							LongKey_flag=0;
							
							if(autostatus != state1)//�������Զ���
							{
								autostatus = state1;
								Send(SetRangeAuto,SetRangeAuto);
								check_flag= CheckReceive(SetRangeAuto);
								if(check_flag==1) check_flag=0;
								lcd_write_1bit(0x1C,2,ENABLE);//AUTO��
								lcd_write_1bit(0x1C,3,DISABLE);//MANU��
							}
						}
					}break;
					case KEY_VALUE_4://PARAM
					{
					}break;
					case KEY_VALUE_5://HOLD-LIGHT
					{
						if(Is_Cal_Mode == 1)//У׼ģʽ��hold��ΪohmУ׼����
						{
							switch(funcstatus)
							{
								case state0://Ohm
								{
									if(ohm_value<100)//0 ��
									{
										SaveData.Value.Cal_ohm_zero[rangestatus]=ohm_value/SaveData.Value.Cal_ohm_gain[rangestatus]+SaveData.Value.Cal_ohm_zero[rangestatus];
									}
									else if(ohm_value>3500)//500 ��
									{
										SaveData.Value.Cal_ohm_gain[rangestatus]= 5000*SaveData.Value.Cal_ohm_gain[rangestatus]/ohm_value;
									}
									updata_flash();	//��������	
									break;
								}
								case state1://Cap У׼
								{
									switch(rangestatus)//����ֵУ׼
									{										
										case 1://60.00nF  У׼50nF
										{
											if(Cap_value>30)
											{
												SaveData.Value.Cal_Cap_gain[0]= 50*SaveData.Value.Cal_Cap_gain[0]/Cap_value;
											}											
											break;
										}										
										case 3://6.000uF У׼5uF
										{
											if(Cap_value>3000)
											{
												SaveData.Value.Cal_Cap_gain[1]= 5000*SaveData.Value.Cal_Cap_gain[1]/Cap_value;
											}
											break;
										}										
										case 5://600.0uF У׼500uF
										{
											if(Cap_value>300000)
											{
												SaveData.Value.Cal_Cap_gain[2]= 500000*SaveData.Value.Cal_Cap_gain[2]/Cap_value;
											}
											break;
										}
										case 6://6.000mF У׼5mF
										{
											if(Cap_value>3000000)
											{
												SaveData.Value.Cal_Cap_gain[3]= 5000000*SaveData.Value.Cal_Cap_gain[3]/Cap_value;
											}
											break;
										}																
										default:break;
									}
									updata_flash();	//��������										
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
						else //����״̬��
						{
							if(ShortKey_flag == 1)
							{//��HOLD���ֶ̰�
								ShortKey_flag=0;
								
								switch(holdstatus)
								{
									case state0:holdstatus = state1;break;//HOLD
									case state1:holdstatus = state0;break;//HOLD���
									default:break;
								}
								
								switch(holdstatus)
								{
									case state0://��ʾ����ֵ
									{
										lcd_write_1bit(0x1C,1,DISABLE);
									}break;
									case state1://����
									{
										lcd_write_1bit(0x1C,1,ENABLE);
									}break;
									default:break;
								}
							}
							else if(LongKey_flag == 1)
							{//��HOLD���ֳ���
								LongKey_flag=0;
								
								switch(lightstatus)
								{
									case state0:lightstatus = state1;break;//����
									case state1:lightstatus = state0;break;//���
									default:break;
								}
								
								switch(lightstatus)
								{
									case state0://����
									{
									}break;
									case state1://���
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
		{//��������ת��ť����¶��ǣ���FUNC���ֳ������ҷ�HOLD״̬��BT��Դ����
			LongKey_flag=0;
			
			switch(btstatus)
			{
				case state0:btstatus = state1;break;//����ģʽ��ΪBTģʽ
				case state1:btstatus = state0;break;//BTģʽ��Ϊ����ģʽ
				default:break;
			}
			
			switch(btstatus)//������ر�BT��Դ
			{
				case state0:Power_Off_Bt();break;
				case state1:Power_On_Bt();break;
				default:break;
			}
		}
		display_bt_state();
		
		if((SoftKeyValue == KEY_VALUE_0) && (ShortKey_flag == 1) && (holdstatus==state0))//�ڶ̰�Funcʱ���ı��������
		{
			ShortKey_flag=0;
			if((RotaryKeyValue == KEY_VALUE_8) && (paramstatus != state0));//���ڲ���Ƶ�ʡ�ռ�ձȵ�λʱ����Func������
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

/*��ʾ*/
void display(void)
{
	if(holdstatus==state0)
	{
		LowPowerDetect();//�͵�ѹ���
		
		switch(RotaryKeyValue)
		{
			case KEY_VALUE_6://V+A ��λ
			{
				temp_for_over_voltage=voltage_effective;
				if(temp_for_over_voltage>30)
					lcd_write_1bit(0x0,3,ENABLE);//���������
				else
					lcd_write_1bit(0x0,3,DISABLE);//���������
				
				if((paramstatus == state0) && (longparamstatus == state0) && (peakstatus == state0) && (phasestatus == state0))
				{//��������V+A״̬������Param��Inrush��Peak��Phase״̬��
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
								if(autostatus==state1)//�Զ���
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;//�ӵ�
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;//�ӵ�
									}
								}
								else//�ֶ���
								{
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//�Զ���
								{
									if((readstmdata1<=60) && (readstmdata1>6))
										propershow=lcd_show_num(showdata1,1,2);
									else
									{
										if(readstmdata1<=6)
											rangestatus-=1;//����
										else if(readstmdata1>60)
											rangestatus+=1;//�ӵ�
										lcd_show_Line(1);
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;//�ӵ�
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//�Զ���
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;//����
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;//�ӵ�
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;//�ӵ�
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//�Զ���
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
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
//										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
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
								if(autostatus==state1)//�Զ���
								{
									if(abs_readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
									//printf("v:%.6f\r\n",showdata1);
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//�Զ���
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
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//�Զ���
								{
									if((abs_readstmdata1<=600) && (abs_readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(abs_readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if(abs_readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;
									}
									
									if((readstmdata2<=3000) && (readstmdata2>600))//Lea test   20160614 ����������3000���ڲ�����ʾ
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//�Զ���
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
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if((readstmdata2<=3000) && (readstmdata2>600))//Lea test   20160614 ����������3000���ڲ�����ʾ
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if((abs_readstmdata1<=600) && (abs_readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(abs_readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							default:break;
						}
//						if(abs_readstmdata1>50)
//							lcd_write_1bit(0x0,3,ENABLE);//���������
//						else
//							lcd_write_1bit(0x0,3,DISABLE);//���������
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
								if(autostatus==state1)//�Զ���
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
										propershow=lcd_show_num(showdata1,1,1);
										propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state1://60.00V/600.0A
							{
								if(autostatus==state1)//�Զ���
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
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state2://600.0V/600.0A
							{
								if(autostatus==state1)//�Զ���
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if(readstmdata2<=600)
										propershow=lcd_show_num(showdata2,2,3);
									else
									{
										lcd_show_Line(2);
										lcd_write_1bit(0x11,0,ENABLE);//.��
										rangestatus+=3;
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,3);
									propershow=lcd_show_num(showdata2,2,3);
								}
							}break;
							case state3://6.000V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if(readstmdata1<=6)
										propershow=lcd_show_num(showdata1,1,1);
									else
									{
										lcd_show_Line(1);
										lcd_write_1bit(0x02,0,ENABLE);//.��
										rangestatus+=1;
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,1);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state4://60.00V/2000A
							{
								if(autostatus==state1)//�Զ���
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
										lcd_write_1bit(0x04,0,ENABLE);//.��
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
								{
									propershow=lcd_show_num(showdata1,1,2);
									propershow=lcd_show_num(showdata2,2,4);
								}
							}break;
							case state5://600.0V/2000A
							{
								if(autostatus==state1)//�Զ���
								{
									if((readstmdata1<=600) && (readstmdata1>60))
										propershow=lcd_show_num(showdata1,1,3);
									else
									{
										if(readstmdata1<=60)
											rangestatus-=1;
										lcd_show_Line(1);
										lcd_write_1bit(0x06,0,ENABLE);//.��
									}
									
									if((readstmdata2<=2000) && (readstmdata2>600))
										propershow=lcd_show_num(showdata2,2,4);
									else if(readstmdata2<=600)
									{
										lcd_show_Line(2);
										rangestatus-=3;//����
									}
								}
								else//�ֶ���
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
				{//����Param״̬
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
				{//����Inrush״̬
					if(inrush_trigger_flag == 2)//�����ж���600A����2000A���ڰ���RANGEʱ��ȷ����
					{//�ɼ���100ms�ڵ�RMSֵ������һ��������ʾ������ʾ���ϡ�
						propershow=lcd_show_num(inrush_current_effective_100ms,1,10);
					}
				}
				else if(peakstatus != state0)
				{//����Peak״̬
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
				{//����Phase״̬
					if((TIM5CH4_CAPTURE_STA&0x8000) && (phaseABCstatus==state4))//�Ѿ���ȡ��λ
					{//�˶γ���ֻ����һ��
						if((abs(phasetimetemp) < 20) || (abs(phasetimetemp-360) < 20))//A��
						{
							lcd_show_A__A(2);
						}
						if(abs(phasetimetemp-120) < 20)//B��
						{
							lcd_show_b__b(2);
						}
						else if(abs(phasetimetemp-240) < 20)//C��
						{
							lcd_show_C__C(2);
						}
						else
						{
							lcd_show_Erro(2);
						}							
						propershow=lcd_show_num(phasetimetemp,1,10);//20160603Lea Test ����ʱ�ڵ�һ����ʾ��λ�����ֵ���жϴ���ԭ��
						phaseABCstatus=state5;
					}
					else if((TIM5CH4_CAPTURE_STA&0x4000) && (phaseABCstatus<state4))//A���ѹ�Ѿ��ȶ����Ҳ�����A�ྭ���Ƚ���֮��������أ���δ������һ��������֮ǰ
					{//�˶γ�������ʾA--Aʱ����Ҫ0.5s��˸�ڶ��е�----
						if(powertimer_0_5s_flag == 1)//��˸��ʾ
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
					else if((TIM5CH4_CAPTURE_STA&0x4000)==0)//��δ����A��������
					{
						if(powertimer_0_5s_flag == 1)//��ʾ
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
					
					if((TIM5CH4_CAPTURE_STA&0x8000) && (phaseABCstatus==state3))//�ɹ�����һ��A��B��C���ʱ�����,��δ�������λ
					{//�˶γ���ֻ�����һ��
						TIM_Cmd(TIM5, DISABLE);//һ�ֲ������
						
//						phasetimetemp=TIM5CH4_CAPTURE_STA&0x3FFF;
//						phasetimetemp*=36000;//���ʱ���ܺ�
//						phasetimetemp+=TIM5CH4_CAPTURE_VAL;//�õ��ܵ�������λʱ��
//						
						TphaseTime = TIM5CH4_CAPTURE_VAL%CCR4_Val;	
						//phasetimetemp%=100;//ȡ��һ��������������λ�����ֵ��1MHz/50Hz=20000;һ�������ڼ���20000  Լ20011
						TphaseTime=TphaseTime*360/CCR4_Val;//������ֵ����ɽǶ�0��- 360��
						//propershow=lcd_show_num(phasetimetemp,1,10);
						phasetimetemp = TphaseTime;
						phaseABCstatus=state4;//��־�Ѿ���ȡ��λ��FINISHED��������
					}
					
					else if(TIM5CH4_CAPTURE_STA&0x4000)//���Ѳ���A�������ء��رձȽ����������
					{//��ͨ���жϵ�ѹ��С<10�ȴ�̽���Ƴ�
						readstmdata2=voltage_effective;
						propershow=lcd_show_num(readstmdata2,2,10);//test �ڵڶ�����ʾ��ǰ��ѹ����Чֵ
						if((readstmdata2<10) && (phaseABCstatus==state1))
						{
							phaseABCstatus=state2;//��ʾ����Ѿ���鵽A�࣬���뿪�������ӣ�׼��������һ��
							BUZZER_Open(0);//����һ�� ��ʾ���ж�����뿪��һ��
						}
						else if((readstmdata2>10) && (phaseABCstatus==state1))
						{
							phaseABCstatus=state1;//�����ȴ�����뿪A�����
						}
						
						if(phaseABCstatus==state2)
						{
							if(readstmdata2>100)//��⵽��һ���ѹ����100V
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
										phaseABCstatus=state3;//��־B or C���ѹ�ȶ��������Ƚ��������������
									}
									else
										delay_count=0;
								}
							}
						}
						else if(phaseABCstatus==state3)//�ڵ�ѹ�Ѿ��ȶ�����δ��⵽������ʱ�����ּ�⵽��ѹ��С���رձȽ���
						{
							if(readstmdata2<10)
							{
								phaseABCstatus=state2;
//								COMP_Cmd(COMP_Selection_COMP1, DISABLE);
							}
						}
					}
					else if((TIM5CH4_CAPTURE_STA&0x4000)==0)//��û�в���A�������ص������
					{
						readstmdata2=voltage_effective;//
						propershow=lcd_show_num(readstmdata2,2,10);//test �ڵڶ�����ʾ��ǰ��ѹ����Чֵ
						if((readstmdata2>100) && (phaseABCstatus==state0))//��⵽��һ�ࣨA�ࣩ��ѹ����100V�����ǵ�ѹδ�ȶ�״̬
						{
							if(delay_count==1){readstmdata2_temp=readstmdata2;}
								
							else if(delay_count==5)
							{
								if(abs(readstmdata2-readstmdata2_temp)<5)//����ѹ���������������������Ĳ���
								{
									//����COMP1�Ͷ�ʱ��,��ʼ����
									COMP_Cmd(COMP_Selection_COMP1, ENABLE);
									
									delay_count=0;
									phaseABCstatus=state1;//��־��ѹ�ȶ�����ʼ���������أ���ʱ���δ�뿪A����ӣ�����Ҫ�ٽ���˶γ����ˡ�
								}
								else
									delay_count=0;
							}
							delay_count++;
						}
						else if((readstmdata2<100) && (phaseABCstatus==state1))//�ڵ�ѹ�Ѿ��ȶ�����δ��⵽������ʱ�����ּ�⵽��ѹ��С���رձȽ����Ͷ�ʱ��
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
					lcd_write_1bit(0x0,3,ENABLE);//���������
				else
					lcd_write_1bit(0x0,3,DISABLE);//���������
				
				if((peakstatus == state0) && (longparamstatus == state0))
				{//�ܹ��ʡ��й����ʡ��޹�����״̬//PF//DPF//THD%f//THD%r
					switch(funcstatus)
					{
						case state0://�ܹ���
						{
							readstmdata1=apparent_power;//VA
							
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state1://�й�
						{
							readstmdata1=active_power;//W
							
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state2://�޹�
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
				{//���ڵ���״̬
					if(timer_1s_blink == 1)
					{
						timer_1s_blink = 0;
						lcd_show_num(powertimer,2,10);
					}
					switch(paramstatus)
					{
						case state0://����KWh
						{
							readstmdata1=kWh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state1://����KVAh
						{
							readstmdata1=kVAh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state2://����Kvarh
						{
							readstmdata1=kVarh;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						case state3://����KgCO2
						{
							readstmdata1=abcde;
							showdata1=deal_1(readstmdata1,1);
							propershow=lcd_show_num(showdata1,1,10);
						}break;
						default:break;
					}
				}
				else if(peakstatus != state0)
				{//���ڶ��๦��״̬
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
						case state0://1PHI��˸
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
						case state1://2PHI��˸
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
						case state2://3PHI��˸
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
						case state3://1PHI����˸
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power1,1,10);break;
								case state1:propershow=lcd_show_num(active_power1,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power1,1,10);break;
								default:break;
							}
						}break;
						case state4://2PHI����˸
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power2,1,10);break;
								case state1:propershow=lcd_show_num(active_power2,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power2,1,10);break;
								default:break;
							}
						}break;
						case state5://3PHI����˸
						{
							switch(funcstatus)
							{
								case state0:propershow=lcd_show_num(apparent_power3,1,10);break;
								case state1:propershow=lcd_show_num(active_power3,1,10);break;
								case state2:propershow=lcd_show_num(reactive_power3,1,10);break;
								default:break;
							}
						}break;
						case state6://��PHI����˸
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
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					read_dta_data = (read_dta_data-SaveData.Value.Cal_VmV_zero[rangestatus])*SaveData.Value.Cal_VmV_gain[rangestatus];//20160803Lea ����MCU�ڲ�У׼
					VmV_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
					
					showdata1=deal_1(read_dta_data,1);
					temp_for_over_voltage=0;
					switch(rangestatus)
					{
						case state0://600.0mV
						{
							lcd_write_1bit(0x1A,2,ENABLE);//m��
							
							if(autostatus==state1)//�Զ���
							{
								if((RxBuffer[4] == 0) && (receive_data<=6200))//receive_data��receive_f�ľ���ֵ
									lcd_show_dta_num(showdata1,1);
								else
								{
									lcd_show_Line(1);
									rangestatus++;
								}
							}
							else//�ֶ���
							{
								if((RxBuffer[4] == 0) && (receive_data<=6200))
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.��
						}break;
						case state1://6.000V
						{
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							
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
							lcd_write_1bit(0x02,0,ENABLE);//.��
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
							lcd_write_1bit(0x04,0,ENABLE);//.��
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
							lcd_write_1bit(0x06,0,ENABLE);//.��
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
						lcd_write_1bit(0x0,3,ENABLE);//���������
					else
						lcd_write_1bit(0x0,3,DISABLE);//���������
				}
				else if(funcstatus ==state1)//ACV
				{
					if(paramstatus == state0)//����ACV
					{ 
						read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
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
								lcd_write_1bit(0x02,0,ENABLE);//.��
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
								lcd_write_1bit(0x04,0,ENABLE);//.��
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
								lcd_write_1bit(0x06,0,ENABLE);//.��
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
							lcd_write_1bit(0x0,3,ENABLE);//���������
						else
							lcd_write_1bit(0x0,3,DISABLE);//���������
					}
					else if(paramstatus == state1)//Hz
					{
						read_dta_data = receive_f;//��DTA0660��ȡ����
						
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
								lcd_write_1bit(0x04,0,ENABLE);//.��
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
								lcd_write_1bit(0x06,0,ENABLE);//.��
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
								lcd_write_1bit(0x02,0,ENABLE);//.��
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
								lcd_write_1bit(0x04,0,ENABLE);//.��
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
								lcd_write_1bit(0x06,0,ENABLE);//.��
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
								lcd_write_1bit(0x02,0,ENABLE);//.��
							}break;
							default:break;
						}
					}
					else if(paramstatus == state2)//Duty
					{
						read_dta_data = receive_f1;//��DTA0660��ȡ����
						showdata2=deal_1(read_dta_data,2);
						propershow=lcd_show_num(showdata2,2,2);//ռ�ձ�
					}
				}
			}break;
			case KEY_VALUE_9://CON/DIODE
			{
				if(funcstatus ==state0)//CONT
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					showdata1=deal_1(read_dta_data,1);
					//if(read_dta_data<=50)
					//	BUZZER_Open(1);
					//else BUZZER_Close();
					if(showdata1<=600)
						propershow=lcd_show_num(showdata1,1,3);
					else 
					{
						lcd_show_OL(1);
						lcd_write_1bit(0x06,0,ENABLE);//.��
					}
				}
				else if(funcstatus ==state1)//DIODE
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					showdata1=deal_1(read_dta_data,1);
					if(showdata1<=3)
						propershow=lcd_show_num(showdata1,1,1);
					else 
					{
						lcd_show_OL(1);
						lcd_write_1bit(0x02,0,ENABLE);//.��
					}
				}
			}break;
			case KEY_VALUE_10://OHM/CAP/TMP
			{
				if(funcstatus ==state0)//Ohm
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					read_dta_data = (read_dta_data-SaveData.Value.Cal_ohm_zero[rangestatus])*SaveData.Value.Cal_ohm_gain[rangestatus];//20160802Lea ����MCU�ڲ�У׼
					ohm_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
					
					showdata1=deal_1(read_dta_data,1);
					switch(rangestatus)
					{
						case state0://600.0��
						{
							lcd_write_1bit(0x1B,1,DISABLE);//k(2)��
							lcd_write_1bit(0x1A,1,DISABLE);//M��

							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(receive_data<=6200)
								{
									lcd_show_dta_num(showdata1,1);
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.��
						}break;
						case state1://6.000k��
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
							lcd_write_1bit(0x02,0,ENABLE);//.��
						}break;
						case state2://60.00k��
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
							lcd_write_1bit(0x04,0,ENABLE);//.��
						}break;
						case state3://600.0k��
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
									lcd_show_dta_num(showdata1,1);// *1.0101  ����������������� 
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.��
						}break;
						case state4://6.000M��
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
							lcd_write_1bit(0x02,0,ENABLE);//.��
						}break;
						case state5://60.00M��
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
//									lcd_write_1bit(0x04,0,ENABLE);//.��
								}
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.��
						}break;
						default:break;
					}
				}
				else if(funcstatus ==state1)//Cap
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					switch(rangestatus)//����ֵ����
					{
						case 0://6.000nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 1://60.00nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 2://600.0nF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[0])*SaveData.Value.Cal_Cap_gain[0];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 3://6.000uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[1])*SaveData.Value.Cal_Cap_gain[1];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 4://60.00uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[2])*SaveData.Value.Cal_Cap_gain[2];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 5://600.0uF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[2])*SaveData.Value.Cal_Cap_gain[2];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 6://6.000mF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[3])*SaveData.Value.Cal_Cap_gain[3];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}
						case 7://60.00mF
						{
							read_dta_data = (read_dta_data-SaveData.Value.Cal_Cap_zero[3])*SaveData.Value.Cal_Cap_gain[3];//20160803Lea ����MCU�ڲ�У׼
							Cap_value = read_dta_data;//�ڲ�����������������У׼��ͨѶ
							break;
						}										
						default:break;
					}
					
					showdata1=deal_1(read_dta_data,1);
					switch(rangestatus)
					{
						case state0://6.000n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,DISABLE);//u��
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.��
						}break;
						case state1://60.00n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,DISABLE);//u��
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.��
						}break;
						case state2://600.0n
						{
							lcd_write_1bit(0x1B,2,ENABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,DISABLE);//u��
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data<=600.0)
									propershow=lcd_show_num(showdata1,1,3);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.��
						}break;
						case state3://6.000u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,ENABLE);//u��
							showdata1/=1000;
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data/1000<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.��
						}break;
						case state4://60.00u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,ENABLE);//u��
							showdata1/=1000;
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data/1000<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.��
						}break;
						case state5://600.0u
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n��
							lcd_write_1bit(0x1A,2,DISABLE);//m��
							lcd_write_1bit(0x19,2,ENABLE);//u��
							showdata1/=1000;
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data/1000<=600.0)
									propershow=lcd_show_num(showdata1,1,3);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x06,0,ENABLE);//.��
						}break;
						case state6://6.000m
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n��
							lcd_write_1bit(0x1A,2,ENABLE);//m��
							lcd_write_1bit(0x19,2,DISABLE);//u��
							showdata1/=1000;
							showdata1/=1000;
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data/1000/1000<=6.000)
									propershow=lcd_show_num(showdata1,1,1);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x02,0,ENABLE);//.��
						}break;
						case state7://60.00m
						{
							lcd_write_1bit(0x1B,2,DISABLE);//n��
							lcd_write_1bit(0x1A,2,ENABLE);//m��
							lcd_write_1bit(0x19,2,DISABLE);//u��
							showdata1/=1000;
							showdata1/=1000;
							if(autostatus==state1)//�Զ���
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
							else//�ֶ���
							{
								if(read_dta_data/1000/1000<=60.00)
									propershow=lcd_show_num(showdata1,1,2);
								else
									lcd_show_OL(1);
							}
							lcd_write_1bit(0x04,0,ENABLE);//.��
						}break;
						default:break;
					}
				}
				else if(funcstatus ==state2)//Temp(C)
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult);//��DTA0660��ȡ����
					showdata1=deal_1(read_dta_data,2);
					propershow=lcd_show_num(showdata1,2,2);
					if(propershow==0)
					{
						propershow=lcd_show_num(showdata1,2,3);
					}
				}
				else if(funcstatus ==state3)//Temp(F)
				{
					read_dta_data = receive_f;//ReadDTAValue(ReadResult)*1.8+32;;//��DTA0660��ȡ����
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

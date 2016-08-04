#include "stm32f37x.h"
#include <stdio.h>
#include "sdadc.h"
#include <math.h>
#include "comp.h"
#include "matrixkey.h"
#include "stmdataprocess.h"
#include "arm_math.h" 
#include "userVAinteraction.h"
#include "timer.h"
#include "flash_data.h"
#include "CalA.h"

#define TEST_LENGTH_SAMPLES 2048
 
 
////���ڶ��У�����������										
//extern float Cal_A_tab[][2];

////���ڶ��У�����������										
//extern float Cal_A_gain[][2];	
 
 extern defSysValue SysValue ;//ϵͳ����ʱ����Ҫ��������
 extern defFlashCal SaveData;	//������flash�еı�Ҫ����
 
extern const u16 length;

extern u16 count_for_Standby;//���߼���  ��������AC�Զ�����ʧЧʱ ��ʱ1�봥��һ��AC����
uint16_t count_for_ac=0;
/* ------------------------------------------------------------------- 
* External Input and Output buffer Declarations for FFT Bin Example 
* ------------------------------------------------------------------- */ 
 
//float Input[TEST_LENGTH_SAMPLES]={0};//�ƶ���RAMsave����
	//float Output[TEST_LENGTH_SAMPLES/2];
	float phase_angle[TEST_LENGTH_SAMPLES/4];
	float THD[TEST_LENGTH_SAMPLES/4];

union RAMsavedef RAMsave;

#define Test_Fs 10240
#define PI2 (6.283185307179586476925286766559)
//  
/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint16_t fftSize = TEST_LENGTH_SAMPLES/2; 
uint8_t ifftFlag = 0; 
uint8_t doBitReverse = 1;

uint32_t refIndex = 213, testIndex = 0; 
//static float32_t maxValue=0; 
unsigned int iiflag=0;
/*******************************������FFT�ĳ�ʼ��*************************************/


/*key*/
u8 key=0,page=0;

/******************����***********************/
uint16_t t;
uint8_t count=0;
uint8_t fft_count=0;
uint8_t addcount=5;//��ֵ�˲�����
uint16_t datasize=1024;
u16 inrush_current_100ms_count=0;
/******************��־***********************/

uint8_t pause_flag=0;//Һ����ʾ��ͣ��־��0-����ͣ��1-��ͣ
uint8_t af_flag=0;//��Ƶ��ʾ��־,1-��ѹ��2-������3-����
uint8_t r_or_f_flag=0;//THDѡ��ģʽ��0-THD%r,1-THD%f
u8 adjust_flag=0;
u8 inrush_trigger_flag=0;
/******************SDADC�ɼ��Ͳ����Ƶ��ɱ�־***********************/
uint8_t collect_finished=0;//SDADC������־
uint8_t voltage_capture_finished=0 , current_capture_finished=0;//������⣬�����Ƶ��־

/******************��ѹ�����������ʴ洢����***********************/
float SDADC1_value[1024]={0},SDADC2_value[1024]={0};
float POWER_value[1024]={0};

/******************��Ҫ�������***********************/
float voltage_effective=0,current_effective=0;//��Ч��ѹ����Ч����
float inrush_current_effective_100ms=0,inrush_current_effective_100ms_sum;//100ms��ӿ����
float maxv_value,maxi_value,minv_value,mini_value;//�����С|��ѹ������
float voltage_cf=0,current_cf=0;//��������[CF]
float apparent_power=0,active_power=0,reactive_power=0;//���ڹ��ʡ��й����ʡ��޹�����
float kWh,kVAh,kVarh,KgCO2;
float power_factor=0,d_power_factor=0;//��������[PF]��λ�ƹ�������[DPF]
float THD_r_voltage=0,THD_f_voltage=0,THD_r_current=0,THD_f_current=0,THD_r_power=0,THD_f_power=0;//��г�������ʣ����������Чֵ������+г��������[THD%r]/��Ի�����Чֵ[THD%f]

///******************�����г��ֵ��м���***********************/
//float voltage_sum=0,current_sum=0,power_sum=0;//����Vrms��Arms���й�����
//float voltage_temp=0,voltage_effective_before=0;
//float current_temp=0,current_effective_before=0;
//float power_temp=0,active_power_before=0;

float maxv=0,maxi=0,minv=0,mini=0;//���������Сֵ

float voltage_peak_value=0,current_peak_value=0;//�����ֵ->CF

float voltage_foudamental_effective=0,current_foudamental_effective=0,power_foudamental_effective=0;//���������Чֵ->THD%r��THD%f
//20160607lea �ܵ�г����������г����ѹ�ľ�����ֵ��������ѹ�����ܵ�ѹ��
float THDV2sum=0,THDA2sum=0,THDP2sum=0;//����г����ƽ���͡�

float voltage_foudamental_phase=0,current_foudamental_phase=0,power_foudamental_phase=0;//���������λ->DPF

float voltage_fundamental_frequency = 0 , current_fundamental_frequency = 0;
float voltage_effective_sum=0,current_effective_sum=0,active_power_sum=0;//����ۼ���ͣ��Ա��ֵ����

float voltage_mean,current_mean;
float voltage_mean_temp ,current_mean_temp;//20160616 �������ڵ�ѹ����ƽ��ֵ���˲�

float voltage_effective_tab[20]={0},current_effective_tab[20]={0},active_power_tab[20]={0}; //2016-07-25 lea  ���ô����˲� �ӿ���ֵ�����ٶ� 
float voltage_mean_tab[20]={0},current_mean_tab[20]={0};
char  Window_num=10;//�����˲���С;


u8 timer_1s_blink;
u16 powertimercounter;
float kWh_sum,kVAh_sum,kVarh_sum,KgCO2_sum;
uint8_t VadENA=0;

float testAdjV=0;

float Current_multiplication=2.0f/0.6f;
void dealwith_information(void)
{
	/******************�����г��ֵ��м���***********************/
	float voltage_sum=0,current_sum=0,power_sum=0;//����Vrms��Arms���й�����
	float voltage_temp=0,voltage_effective_before=0;
	float current_temp=0,current_effective_before=0;
	float power_temp=0,active_power_before=0;
	float voltage_mean_sum,current_mean_sum;//�����ѹ������ƽ��ֵDC
	
	float v_temp,a_temp;
	
	char    chardata[32];
	uint16_t   	Tloop=0,Tab_i=0;
	float temp_mean=0;

	/*********************************************
	*	�ɼ�������־������������Ӧ����
	*********************************************/
	if(collect_finished == 1)
	{
		collect_finished = 0;
		
		//������������ ��ȡ�������1024����
		get_formed1024();	
		
		for(t = 0;t < datasize;t++)
		{//������������		
			if(RotaryKeyValue==KEY_VALUE_6)//V+A ��λ  
			{
				if((funcstatus ==state0)||(funcstatus ==state2))//AC V+A    AC\DC
				{
					if(rangenum ==0)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);
					}
					else if(rangenum ==1)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);
					}
					else if(rangenum ==2)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);
					}	
				}
				else if(funcstatus ==state1)//DC
				{					
					if(rangenum ==0)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
							
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_6VD_zero=temp_mean;
								sprintf(chardata, "set 6VD_zero %.4f", SaveData.Value.cal_6VD_zero);
								printf(chardata); updata_flash();
							}							
							else//����
							{
								SaveData.Value.cal_6VD_gain=5.0f/(temp_mean-SaveData.Value.cal_6VD_zero);
								sprintf(chardata, "set 6VD_gain %.4f", SaveData.Value.cal_6VD_gain);
								printf(chardata); 
								updata_flash();
							}
						}					
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);	//						
					}
					else if(rangenum ==1)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_60VD_zero=temp_mean;
								sprintf(chardata, "set 60VD_zero %.4f", SaveData.Value.cal_60VD_zero);
								printf(chardata); updata_flash();
							}							
							else//����
							{
								SaveData.Value.cal_60VD_gain=50.0f/(temp_mean-SaveData.Value.cal_60VD_zero);
								sprintf(chardata, "set 60VD_gain %.4f", SaveData.Value.cal_60VD_gain);
								printf(chardata); 
								updata_flash();
							}					
						}
						
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);							
					}
					else if(rangenum ==2)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_600VD_zero=temp_mean;
								sprintf(chardata, "set 600VD_zero %.4f", SaveData.Value.cal_600VD_zero);
								printf(chardata);updata_flash(); 
							}							
							else//����
							{
								SaveData.Value.cal_600VD_gain=500.0f/(temp_mean-SaveData.Value.cal_600VD_zero);
								sprintf(chardata, "set 600VD_gain %.4f", SaveData.Value.cal_600VD_gain);
								printf(chardata); 
								updata_flash();
							}					
						}
						
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);							
					}	
				}											
			}
			else if(RotaryKeyValue==KEY_VALUE_7)// W��λ
			{
				if(rangenum ==0)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);
					}
					else if(rangenum ==1)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);
					}
					else if(rangenum ==2)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);
					}	
			}
						
			//У����������  С��600A����ʱ��������У׼��ʽ
			if(SDADC2->CONFCHR2 == 0x01)//
			{
				SDADC2_value[t] = (SDADC2_value[t]-SaveData.Value.cal_600A_zero)*SaveData.Value.cal_600A_gain;		
			}
			else
			{
				SDADC2_value[t]=(SDADC2_value[t]-SaveData.Value.cal_A1_zero)*SaveData.Value.cal_A1_gain;
				if(Is_Cal_Mode != 1)//��У׼ģʽ�¶Դ��������ֵ����������
				{//1  ��������ģʽ���Դ�������ж��У׼��				
					SDADC2_value[t] = Adj_Nline(SDADC2_value[t]);
				}	
			}
			
			

			
			//�����ֵ//����Сֵ
			if(t == 0)
			{
				maxv = SDADC1_value[t];minv = SDADC1_value[t];maxi = SDADC2_value[t];mini = SDADC2_value[t];
			}
			if(maxv < SDADC1_value[t])	maxv = SDADC1_value[t];
			if(minv > SDADC1_value[t])	minv = SDADC1_value[t];
			
			if(maxi < SDADC2_value[t]) 	maxi = SDADC2_value[t];
			if(mini > SDADC2_value[t])	mini = SDADC2_value[t];
			
			if((RotaryKeyValue == KEY_VALUE_6) && (longparamstatus != state0))
			{//������Inrush����
				if(inrush_trigger_flag == 0)
				{//δ����ʱ���ж��ں��������´���
					if(((rangestatus == state0) && (maxi >= 10)) || ((rangestatus == state1) && (maxi >= 100)))//1A@600A || 10A@2000A
					{//�����(maxi >= 1)��(maxi >= 10)�е�1��10������ֵ��Ҫת����maxi��Ӧ-32768~32767
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag=1;
					}
				}
				else if(inrush_trigger_flag == 1)
				{//�Ѿ�����,����100ms(50Hzʱ,100ms��5�����ڣ�����Ƶ��Ϊ5120Hzʱ����5*5120/50=512����)
					if(inrush_current_100ms_count < 512)
					{
						inrush_current_effective_100ms_sum += SDADC2_value[t] * SDADC2_value[t];//SDADC2_value[t];
						inrush_current_100ms_count++;
						
						printf("%5f\r\n",SDADC2_value[t]); //20160615 test ���� ����ӿ�������ݴ���
					}
					else if(inrush_current_100ms_count == 512)
					{
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms = sqrt(inrush_current_effective_100ms_sum / 512);
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag = 2;
					}
				}
				else if(inrush_trigger_flag == 2)//20160531lea�ɹ���ȡ��ӿ������Ĳ���
				{
					
				}
			}

			//������㹦��˲ʱֵ ��˲ʱֵ�ľ�ֵ�õ��й�����    **************************************
			POWER_value[t] = SDADC1_value[t] * SDADC2_value[t];
			
			//��ѹ��������������Чֵ���㣬����һ���֡�����ƽ���ͣ�V/A������ͣ�W��
			
			voltage_sum += SDADC1_value[t] * SDADC1_value[t];//* SDADC_CAL_COEF2;//�������ʱ�������ѹ
			current_sum += SDADC2_value[t] * SDADC2_value[t];//* SDADC_CAL_COEF2;//�������ʱ���������
																				//20160419 lea  ��������ǯͷ����  
			power_sum += SDADC1_value[t] * SDADC2_value[t];//���й����� //ƽ������
			
			voltage_mean_sum += SDADC1_value[t] ;//* SDADC_CAL_COEF;
			current_mean_sum += SDADC2_value[t] ;//* SDADC_CAL_COEF;
		}//һ֡����Ԥ�������************************************************************************************
/**************************************************************************************************************/
	
		
		//��ѹ������ƽ��ֵ��DC
		voltage_mean = voltage_mean_sum / datasize;
		current_mean = current_mean_sum / datasize;
		
		//�����˲�����ƽ��ֵ
		voltage_mean = Windows_Filter(voltage_mean,voltage_mean_tab,Window_num);
		current_mean = Windows_Filter(current_mean,current_mean_tab,Window_num);

		
		//��ѹ��������������Чֵ���㣬������һ���֡��������������V/A���������W��
		voltage_temp = sqrt(voltage_sum / datasize);
		current_temp = sqrt(current_sum / datasize);//20160419lea ����ֵ��ȡ 
		power_temp = power_sum / datasize;
/************************************************************************************************************************/		
		SysValue.curr_now=current_mean;//Ŀǰֻ����������ʾ֮ǰ���ݸ�У׼���֣�ȫ����Ȼʹ��֮ǰ�ı������߼�
				
//		printf("v,c:%.4f,%.4f",voltage_temp,current_temp);
//		printf("vm,cm:%.4f,%.4f\r\n",voltage_mean,current_mean);
/*************************************************************************************************************************/
//		//�ۼ�addcount�Σ����ں���addcount�ε�������ƽ��ֵ
//		voltage_effective_sum += voltage_temp;
//		current_effective_sum += current_temp;
//		active_power_sum += power_temp;		

		if(SaveData.Value.cal_adjv==0)//
		{//0 У׼ģʽ�µ���У����ʾ 
			current_mean = Adj_Nline(current_mean);
		}
		
		count=5;
		//count++;
		//����addcount���Ѿ��õ�������ֵ�˲���ĵ�ѹ�͵�����Чֵ���й�����
		if(count == addcount)
		{			
//			//addcount�εľ�ֵ�����õ���ѹ��������������Чֵ
//			voltage_effective=voltage_effective_sum / 5;//addcount;
//			current_effective=current_effective_sum / 5;//addcount;
//			active_power=active_power_sum / 5;//addcount;//ƽ����� �й�����ֵ
			
			voltage_effective = Windows_Filter(voltage_temp,voltage_effective_tab,Window_num);
			current_effective = Windows_Filter(current_temp,current_effective_tab,Window_num);
			active_power 	  = Windows_Filter(power_temp,  active_power_tab,     Window_num);

			//printf("ve,ce:%.4f,%.4f\r\n",voltage_effective,current_effective);
			
			if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
				fft_count++;
			
			if(((RotaryKeyValue==KEY_VALUE_6) && (peakstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3)))
			{//VA��Peak����VA����CF��
				//�����Сֵ����
//				maxv_value = (float)maxv ;
//				minv_value = (float)minv ;
//				maxi_value = (float)maxi ;
//				mini_value = (float)mini ;
				if(maxv > maxv_value)	maxv_value = maxv;
				if(minv < minv_value)	minv_value = minv;
				if(maxi > maxi_value)	maxi_value = maxi;
				if(mini < mini_value)	mini_value = mini;
				
				//�����ֵ����ҪΪ�˽������ѹ�������Ĳ�������
				if(maxv_value > fabs(minv_value))//��ֵ(2014-10-31ע�ͣ�ֻ������������������ֵ)
					voltage_peak_value = maxv_value;
				else	voltage_peak_value = fabs(minv_value);
				if(maxi_value > fabs(mini_value))
					current_peak_value = maxi_value;
				else	current_peak_value = fabs(mini_value);
			}
			if((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3))
			{//VA����CF��
				if(voltage_effective == 0)	voltage_cf = 0;
				else	voltage_cf = voltage_peak_value / voltage_effective;//��������(CF)
				if(current_effective == 0)	current_cf = 0;
				else	current_cf = current_peak_value / current_effective;
			}
			
			if(RotaryKeyValue==KEY_VALUE_7)
			{//�������ڹ��ʡ��޹����ʡ�����������PF��
				apparent_power = voltage_effective * current_effective;//���ڹ���(VA)
				
				if(apparent_power <= active_power)
					reactive_power = 0;
				else
					reactive_power = sqrt(apparent_power * apparent_power - active_power * active_power);//�޹�����(Var)
			}
			
			
			if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0) && (paramstatus == state0))
			{//W����PF��
				power_factor = active_power / apparent_power;//��������(PF)
				if(power_factor > 1)	power_factor = 1;
			}
			//count=0;//�����˲�������ü���ֵ��ֻ��Ϊ�ʼ����������ֵ
			voltage_effective_sum = 0;
			current_effective_sum = 0;
			active_power_sum = 0;
		}//����������� ��addcount���������ʼ
			
		if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus != state0))
		{//W���������
//			if(powertimer==1)//�ſ�ʼ�������ʱ������������
//			{
//				kWh=0;kVAh=0;kVarh=0;
//			}
			powertimercounter++;//
			/*kWh,kVAh,kVarh,KgCO2;*/
			kWh_sum += active_power;// * (float)powertimer/3600/1000;
			kVAh_sum += apparent_power;// * (float)powertimer/3600/1000;
			kVarh_sum += reactive_power;// * (float)powertimer/3600/1000;
			if(powertimer_1s_flag == 1)//1���ӱ�־����
			{
				powertimer_1s_flag=0;
				timer_1s_blink=1;
				kWh += kWh_sum / powertimercounter/3600/1000;
				kVAh += kVAh_sum / powertimercounter/3600/1000;
				kVarh += kVarh_sum / powertimercounter/3600/1000;
				
				powertimercounter=0;
				kWh_sum=0;kVAh_sum=0;kVarh_sum=0;
			}
		}
			
		maxv=0;maxi=0;minv=0;mini=0;
		voltage_sum=0;current_sum=0;power_sum=0;
		voltage_mean_sum=0;current_mean_sum=0;//�޸�һ������֮ǰ������current_mean_sum=0;�޸�һ��Ϊvoltage_mean_sum=0;
		
		/*********************************************
		*	FFT�������,ֻ����ťVA+W������
		*********************************************/
		if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
		{//VA����THD%r��THD%f����W����Param��
			if(fft_count>=1)						//ÿ��fft_count�λ������㣬����һ��FFT
			{
				FFT();//��Ƶ
				
				if((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1))
				{
					//����λ�ƹ���������DPF��
					d_power_factor= cos((current_foudamental_phase- voltage_foudamental_phase)*PI/180);
				}
				
				//��ѹ��г��������(THD%r%f)
				if(voltage_effective< voltage_foudamental_effective)
					voltage_effective= voltage_foudamental_effective;
				
				if(voltage_effective == 0)	THD_r_voltage=0;
				//else	THD_r_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_effective* voltage_effective));		
				else THD_r_voltage=sqrt(THDV2sum)/voltage_effective;//V THD%r
				
				if(voltage_foudamental_effective == 0)	THD_f_voltage=0;
				//else	THD_f_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_foudamental_effective* voltage_foudamental_effective));
				else THD_f_voltage=sqrt(THDV2sum)/voltage_foudamental_effective;//V THD%f
				
				//������г��������(THD%r%f)
				if(current_effective< current_foudamental_effective)
					current_effective= current_foudamental_effective;
				
				if(current_effective == 0)	THD_r_current=0;
				//else	THD_r_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_effective* current_effective));
				else THD_r_current=sqrt(THDA2sum)/current_effective;//A THD%r
				
				if(current_foudamental_effective == 0)	THD_f_current=0;
				//else	THD_f_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_foudamental_effective* current_foudamental_effective));
				else THD_f_current=sqrt(THDA2sum)/current_foudamental_effective;//A THD%f
				
				//������г��������(THD%r%f)
				if(active_power< power_foudamental_effective)
					active_power= power_foudamental_effective;
				
				if(active_power == 0)	THD_r_power=0;
				//else	THD_r_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(active_power* active_power));
				else THD_r_power=sqrt(THDP2sum)/active_power;//P THD%r
				
				if(power_foudamental_effective == 0)	THD_f_power=0;
				//else	THD_f_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(power_foudamental_effective* power_foudamental_effective));
				else THD_f_power=sqrt(THDP2sum)/power_foudamental_effective;	//P THD%f
				
				//����
				fft_count = 0;
				voltage_effective_sum = 0;
				current_effective_sum = 0;
				active_power_sum = 0;
			}
		}
		
		if(funcstatus ==state0)//ac
		{
			VadENA=0;//SDADC��ʹ�ܱ�־���ݵ��Ƚ����жϣ�����㿪ʼADC�ɼ�
			if(phasestatus != state0)
			{
				TIM_Cmd(TIM19, ENABLE);
			}
			//TIM_Cmd(TIM19, ENABLE);//test  ����ʱ��ACֱ�Ӳ���
		}
		else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   
		{
			TIM_Cmd(TIM19, ENABLE);
		}
		//VadENA=0;//SDADC��ʹ�ܱ�־���ݵ��Ƚ����жϣ�����㿪ʼADC�ɼ�
		//TIM_Cmd(TIM19, ENABLE);
		
		count_for_ac = count_for_Standby;
	}
	//��ʱ����һ��AC����������ʱ����    2Sһ��   ������ֵ��addcount�˲���������ֵ���»�Ƚ�����
	if(count_for_Standby-count_for_ac>=2)
	{
		TIM_Cmd(TIM19, ENABLE);
	}
}


//2015-10-30JEFF����ѹ����������ʱ��ʱ����Ϊstm32�����õ��ĵ�ѹֵ
void FFT(void)
{
  unsigned int ii=0;
	uint8_t fundamental_flag=0;
	float temp_frequency;
	
	
	
  arm_status status;
  arm_cfft_radix4_instance_f32 S;  
	
     
  status = ARM_MATH_SUCCESS;
   
  /* Initialize the CFFT/CIFFT module */  
  status = arm_cfft_radix4_init_f32(&S, fftSize,ifftFlag, doBitReverse);
	
/*****************************************��ѹFFT********************************************************/
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = ((float)SDADC1_value[ii]);//
			RAMsave.Input[2*ii+1] = 0;
		}	 
		arm_cfft_radix4_f32(&S,RAMsave.Input);//FFT����
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);//�����ֵ	 
		//���ڶ�����һ���������ڣ��˴�output ��ռ��input����ĺ���һ���֣���Ӱ�����ǰ��ʹ�г���ļ��㡣
		
//		printf("---------------------��ѹ-----------------------------\r\n");
//		printf("��ѹƵ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		THDV2sum=0;
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("ֱ����ѹ ,  V=%2.5f V\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-50)<0.5 /*|| fabs(temp_frequency-60)<0.5*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;//������iiֵ����fundamental_flag����������
						voltage_foudamental_phase= phase_angle[ii];
						voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDV2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//����������ѹ��ֵ
					}
					else if(fabs(temp_frequency-60)<0.5)//�жϻ�Ƶ�ǲ���60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>voltage_foudamental_effective)
						{
							fundamental_flag= ii;//������iiֵ����fundamental_flag����������
							voltage_foudamental_phase= phase_angle[ii];
							voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDV2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//����������ѹ��ֵ
						}						
						//voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(voltage_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ voltage_effective;//THD%r
					}
					else
					{
						if(voltage_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ voltage_foudamental_effective;//THD%f
					}
					THDV2sum+=RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("V=%2.5f V  ,  ", RAMsave.K4_tab.Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}

/*******************************************����FFT******************************************************/	
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)SDADC2_value[ii];					//�ֶ������һ����ֵΪ10��ֱ������
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
				
//		printf("---------------------����-----------------------------\r\n");
//		
//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("ֱ������ ,  I=%2.5f A\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-50)<0.5f /*|| fabs(temp_frequency-60)<0.5f*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;
						current_foudamental_phase= phase_angle[ii];
						current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDA2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//��������������ֵ
					}
					else if(fabs(temp_frequency-60)<0.5)//�жϻ�Ƶ�ǲ���60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>current_foudamental_effective)
						{
							fundamental_flag= ii;
							current_foudamental_phase= phase_angle[ii];
							current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDA2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//��������������ֵ							
						}						
						//voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(current_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ current_effective;//THD%r
					}
					else
					{
						if(current_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ current_foudamental_effective;//THD%f
					}
					THDA2sum+=RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("I=%2.5f A  ,  ", Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
/************************************************����FFT*************************************************/		
	if(RotaryKeyValue==KEY_VALUE_7)//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)POWER_value[ii];					//�ֶ������һ����ֵΪ10��ֱ������
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
		
//		printf("---------------------����-----------------------------\r\n");

//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
//					printf("ֱ��(�ֶ����10W)  ,  Pw=%2.5f W\r\n", Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-50)<0.5f /*|| fabs(temp_frequency-120)<0.5f*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;
						power_foudamental_phase= phase_angle[ii];
						power_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDP2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//�����������ʷ�ֵ
					}
					else if(fabs(temp_frequency-60)<0.5)//�жϻ�Ƶ�ǲ���60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>power_foudamental_effective)
						{
							fundamental_flag= ii;
							current_foudamental_phase= phase_angle[ii];
							current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDP2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//�����������ʷ�ֵ							
						}												
					}					
					
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(active_power==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ active_power;//THD%r
					}
					else
					{
						if(power_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ power_foudamental_effective;//THD%f
					}
					THDP2sum+=RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("Pw=%2.5f W  ,  ", Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
}
/************************************************************************************************
*���������������������Ӧ�ĵ�ѹֵ
*�����������޺󼴿�ʼ����
*������ʽΪ y=a1*exp(x/t1)+y0;
* 0.00137*exp(x/(206.147))+1.5688
*/
//float Adj_V(float sdadc_value)
//{
//	float new_value=0;
//	
//	if(sdadc_value>1500.0f)
//	{
//		//SaveData.Value.cal_A_a1=0.000640519;
//		//SaveData.Value.cal_A_t1=1484.56402;
//		//SaveData.Value.cal_A_y0=64.17008;
//		
//		//new_value=0.00064*exp((sdadc_value/1484.56402))+64.17008;
//		new_value=SaveData.Value.cal_A_a1*exp((sdadc_value/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
//		return new_value;
//	}
//	else if((sdadc_value<-1500.0f))
//	{
//		sdadc_value*=-1;
//		new_value=SaveData.Value.cal_A_a1*exp((sdadc_value/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
//		return new_value*-1;
//	}
//	else 
//	{
//		return 0;
//	}			
//}


/*********************************************/
//����������
//ʹ��SDADC�������
/***********************************************/
uint8_t get_formed1024(void)
{
	int16_t tt=0,loop=0;
	int16_t temp[5]={0};//��������������ơ�

	if(funcstatus ==state0)//ac ֱ�ӽ����ɹ����Ƚϵõ���1024����
	{
		for(loop = 0;loop < 1024;loop++)//��ȡ1024�������ڼ���
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]>>16);	
		}
	}
	else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   ��ȡ1024��
	{
		for(tt=1;tt<256;tt++)
		{
			temp[0] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt-1]&0xFFFF;
			temp[1] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt]&0xFFFF;
			temp[2] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+1]&0xFFFF;
			temp[3] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+2]&0xFFFF;
			temp[4] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+3]&0xFFFF;
			if(temp[1]>0)
			{
				if((temp[1]>temp[0])&&(temp[2]>temp[1])&&(temp[2]<temp[3])&&(temp[3]<temp[4]))
				{
					tt--;
					break;
				}
			}	
			else
			{				
			}	
		}
		
		for(loop = 0;loop < 1024;loop++)//��ȡ1024�������ڼ���
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]>>16);	
		}
	}
	return 1;

}


/***************************************************************************************************/
//��һ������������һ���������鲢����ƽ��ֵ
//������������ø������������е�һ����[0]�Ĳ�ֵ���� [0]*0.2  ���ø���ֵ��䵱ǰ����
//���������	New_Data �´���Ĳ�����
//			Data_tab ��ǰ�������ڵĴ�������
//			num  ���ڴ�С�������С��
//
//�������أ�	�����˲�ֵ
/****************************************************************************************************/
float Windows_Filter(float New_Data,float *Data_tab,char num)
{
	char i=0;
	float value=0;
	
	if(Data_tab==current_mean_tab)//�����˲����������Ŵ�һ��
	 {
		if(fabs(New_Data-Data_tab[0])>fabs(Data_tab[0]*0.2))//��ƫ��ֵ�ϴ�������˲��Ĵ���
		{
			for(i=0;i<num;i++)
			{
				Data_tab[i]=New_Data;
			}
		}
		else//����ֵ�ƶ���������
		{
			for(i=(num-1);i>0;i--)
			{
				Data_tab[i] = Data_tab[i-1];
			}	
			Data_tab[0]=New_Data;
		}		
	}
	else
	{
		if(fabs(New_Data-Data_tab[0])>fabs(Data_tab[0]*0.01))//��ƫ��ֵ�ϴ�������˲��Ĵ���
		{
			for(i=0;i<num;i++)
			{
				Data_tab[i]=New_Data;
			}
		}
		else//����ֵ�ƶ���������
		{
			for(i=(num-1);i>0;i--)
			{
				Data_tab[i] = Data_tab[i-1];
			}	
			Data_tab[0]=New_Data;
		}
	}
	
	
	for(i=0;i<num;i++)
	{
		value +=Data_tab[i];
	}
	
	return value/num;
}









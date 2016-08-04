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
 
 
////用于多段校正大电流曲线										
//extern float Cal_A_tab[][2];

////用于多段校正大电流曲线										
//extern float Cal_A_gain[][2];	
 
 extern defSysValue SysValue ;//系统运行时的主要变量参数
 extern defFlashCal SaveData;	//保存于flash中的必要数据
 
extern const u16 length;

extern u16 count_for_Standby;//休眠计数  这里用于AC自动触发失效时 超时1秒触发一次AC采样
uint16_t count_for_ac=0;
/* ------------------------------------------------------------------- 
* External Input and Output buffer Declarations for FFT Bin Example 
* ------------------------------------------------------------------- */ 
 
//float Input[TEST_LENGTH_SAMPLES]={0};//移动到RAMsave里面
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
/*******************************以上是FFT的初始化*************************************/


/*key*/
u8 key=0,page=0;

/******************计数***********************/
uint16_t t;
uint8_t count=0;
uint8_t fft_count=0;
uint8_t addcount=5;//均值滤波窗口
uint16_t datasize=1024;
u16 inrush_current_100ms_count=0;
/******************标志***********************/

uint8_t pause_flag=0;//液晶显示暂停标志，0-不暂停，1-暂停
uint8_t af_flag=0;//幅频显示标志,1-电压，2-电流，3-功率
uint8_t r_or_f_flag=0;//THD选择模式，0-THD%r,1-THD%f
u8 adjust_flag=0;
u8 inrush_trigger_flag=0;
/******************SDADC采集和捕获基频完成标志***********************/
uint8_t collect_finished=0;//SDADC采样标志
uint8_t voltage_capture_finished=0 , current_capture_finished=0;//过零点检测，捕获基频标志

/******************电压、电流、功率存储数组***********************/
float SDADC1_value[1024]={0},SDADC2_value[1024]={0};
float POWER_value[1024]={0};

/******************需要计算的量***********************/
float voltage_effective=0,current_effective=0;//有效电压、有效电流
float inrush_current_effective_100ms=0,inrush_current_effective_100ms_sum;//100ms浪涌电流
float maxv_value,maxi_value,minv_value,mini_value;//最大、最小|电压、电流
float voltage_cf=0,current_cf=0;//波峰因数[CF]
float apparent_power=0,active_power=0,reactive_power=0;//视在功率、有功功率、无功功率
float kWh,kVAh,kVarh,KgCO2;
float power_factor=0,d_power_factor=0;//功率因数[PF]、位移功率因数[DPF]
float THD_r_voltage=0,THD_f_voltage=0,THD_r_current=0,THD_f_current=0,THD_r_power=0,THD_f_power=0;//总谐波畸变率，相对整个有效值（基波+谐波分量）[THD%r]/相对基波有效值[THD%f]

///******************计算中出现的中间量***********************/
//float voltage_sum=0,current_sum=0,power_sum=0;//计算Vrms、Arms、有功功率
//float voltage_temp=0,voltage_effective_before=0;
//float current_temp=0,current_effective_before=0;
//float power_temp=0,active_power_before=0;

float maxv=0,maxi=0,minv=0,mini=0;//计算最大、最小值

float voltage_peak_value=0,current_peak_value=0;//计算峰值->CF

float voltage_foudamental_effective=0,current_foudamental_effective=0,power_foudamental_effective=0;//计算基波有效值->THD%r、THD%f
//20160607lea 总的谐波畸变率由谐波电压的均方根值除基波电压（或总电压）
float THDV2sum=0,THDA2sum=0,THDP2sum=0;//各次谐波的平方和。

float voltage_foudamental_phase=0,current_foudamental_phase=0,power_foudamental_phase=0;//计算基波相位->DPF

float voltage_fundamental_frequency = 0 , current_fundamental_frequency = 0;
float voltage_effective_sum=0,current_effective_sum=0,active_power_sum=0;//多次累加求和，以便均值处理

float voltage_mean,current_mean;
float voltage_mean_temp ,current_mean_temp;//20160616 新增用于电压电流平均值的滤波

float voltage_effective_tab[20]={0},current_effective_tab[20]={0},active_power_tab[20]={0}; //2016-07-25 lea  换用窗口滤波 加快数值更新速度 
float voltage_mean_tab[20]={0},current_mean_tab[20]={0};
char  Window_num=10;//窗口滤波大小;


u8 timer_1s_blink;
u16 powertimercounter;
float kWh_sum,kVAh_sum,kVarh_sum,KgCO2_sum;
uint8_t VadENA=0;

float testAdjV=0;

float Current_multiplication=2.0f/0.6f;
void dealwith_information(void)
{
	/******************计算中出现的中间量***********************/
	float voltage_sum=0,current_sum=0,power_sum=0;//计算Vrms、Arms、有功功率
	float voltage_temp=0,voltage_effective_before=0;
	float current_temp=0,current_effective_before=0;
	float power_temp=0,active_power_before=0;
	float voltage_mean_sum,current_mean_sum;//计算电压、电流平均值DC
	
	float v_temp,a_temp;
	
	char    chardata[32];
	uint16_t   	Tloop=0,Tab_i=0;
	float temp_mean=0;

	/*********************************************
	*	采集结束标志到来，进行相应运算
	*********************************************/
	if(collect_finished == 1)
	{
		collect_finished = 0;
		
		//换到单独函数 获取过零点后的1024个数
		get_formed1024();	
		
		for(t = 0;t < datasize;t++)
		{//处理数据数组		
			if(RotaryKeyValue==KEY_VALUE_6)//V+A 档位  
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
						if(Sysflag.Calonce==1)//校准模式
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
							
							if(temp_mean<2000)//零点
							{
								SaveData.Value.cal_6VD_zero=temp_mean;
								sprintf(chardata, "set 6VD_zero %.4f", SaveData.Value.cal_6VD_zero);
								printf(chardata); updata_flash();
							}							
							else//增益
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
						if(Sysflag.Calonce==1)//校准模式
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//零点
							{
								SaveData.Value.cal_60VD_zero=temp_mean;
								sprintf(chardata, "set 60VD_zero %.4f", SaveData.Value.cal_60VD_zero);
								printf(chardata); updata_flash();
							}							
							else//增益
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
						if(Sysflag.Calonce==1)//校准模式
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//零点
							{
								SaveData.Value.cal_600VD_zero=temp_mean;
								sprintf(chardata, "set 600VD_zero %.4f", SaveData.Value.cal_600VD_zero);
								printf(chardata);updata_flash(); 
							}							
							else//增益
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
			else if(RotaryKeyValue==KEY_VALUE_7)// W档位
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
						
			//校正电流数据  小于600A电流时采用线性校准方式
			if(SDADC2->CONFCHR2 == 0x01)//
			{
				SDADC2_value[t] = (SDADC2_value[t]-SaveData.Value.cal_600A_zero)*SaveData.Value.cal_600A_gain;		
			}
			else
			{
				SDADC2_value[t]=(SDADC2_value[t]-SaveData.Value.cal_A1_zero)*SaveData.Value.cal_A1_gain;
				if(Is_Cal_Mode != 1)//非校准模式下对大电流测量值进行修正。
				{//1  正常工作模式，对大电流进行多点校准。				
					SDADC2_value[t] = Adj_Nline(SDADC2_value[t]);
				}	
			}
			
			

			
			//求最大值//求最小值
			if(t == 0)
			{
				maxv = SDADC1_value[t];minv = SDADC1_value[t];maxi = SDADC2_value[t];mini = SDADC2_value[t];
			}
			if(maxv < SDADC1_value[t])	maxv = SDADC1_value[t];
			if(minv > SDADC1_value[t])	minv = SDADC1_value[t];
			
			if(maxi < SDADC2_value[t]) 	maxi = SDADC2_value[t];
			if(mini > SDADC2_value[t])	mini = SDADC2_value[t];
			
			if((RotaryKeyValue == KEY_VALUE_6) && (longparamstatus != state0))
			{//当处于Inrush功能
				if(inrush_trigger_flag == 0)
				{//未触发时，判断在合适条件下触发
					if(((rangestatus == state0) && (maxi >= 10)) || ((rangestatus == state1) && (maxi >= 100)))//1A@600A || 10A@2000A
					{//这里的(maxi >= 1)、(maxi >= 10)中的1和10具体数值需要转换，maxi对应-32768~32767
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag=1;
					}
				}
				else if(inrush_trigger_flag == 1)
				{//已经触发,接收100ms(50Hz时,100ms是5个周期，采样频率为5120Hz时，采5*5120/50=512个点)
					if(inrush_current_100ms_count < 512)
					{
						inrush_current_effective_100ms_sum += SDADC2_value[t] * SDADC2_value[t];//SDADC2_value[t];
						inrush_current_100ms_count++;
						
						printf("%5f\r\n",SDADC2_value[t]); //20160615 test 测试 将浪涌电流数据传出
					}
					else if(inrush_current_100ms_count == 512)
					{
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms = sqrt(inrush_current_effective_100ms_sum / 512);
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag = 2;
					}
				}
				else if(inrush_trigger_flag == 2)//20160531lea成功获取浪涌电流后的操作
				{
					
				}
			}

			//下面计算功率瞬时值 由瞬时值的均值得到有功功率    **************************************
			POWER_value[t] = SDADC1_value[t] * SDADC2_value[t];
			
			//电压、电流、功率有效值计算，先算一部分——求平方和（V/A）、求和（W）
			
			voltage_sum += SDADC1_value[t] * SDADC1_value[t];//* SDADC_CAL_COEF2;//差分输入时，计算电压
			current_sum += SDADC2_value[t] * SDADC2_value[t];//* SDADC_CAL_COEF2;//差分输入时，计算电流
																				//20160419 lea  新增电流钳头部分  
			power_sum += SDADC1_value[t] * SDADC2_value[t];//求有功功率 //平均功率
			
			voltage_mean_sum += SDADC1_value[t] ;//* SDADC_CAL_COEF;
			current_mean_sum += SDADC2_value[t] ;//* SDADC_CAL_COEF;
		}//一帧数据预处理完毕************************************************************************************
/**************************************************************************************************************/
	
		
		//电压、电流平均值：DC
		voltage_mean = voltage_mean_sum / datasize;
		current_mean = current_mean_sum / datasize;
		
		//窗口滤波电流平均值
		voltage_mean = Windows_Filter(voltage_mean,voltage_mean_tab,Window_num);
		current_mean = Windows_Filter(current_mean,current_mean_tab,Window_num);

		
		//电压、电流、功率有效值计算，再算另一部分——求均、开根（V/A）、求均（W）
		voltage_temp = sqrt(voltage_sum / datasize);
		current_temp = sqrt(current_sum / datasize);//20160419lea 电流值获取 
		power_temp = power_sum / datasize;
/************************************************************************************************************************/		
		SysValue.curr_now=current_mean;//目前只用用于在显示之前传递给校准部分，全局仍然使用之前的变量与逻辑
				
//		printf("v,c:%.4f,%.4f",voltage_temp,current_temp);
//		printf("vm,cm:%.4f,%.4f\r\n",voltage_mean,current_mean);
/*************************************************************************************************************************/
//		//累计addcount次，便于后面addcount次到来后求平均值
//		voltage_effective_sum += voltage_temp;
//		current_effective_sum += current_temp;
//		active_power_sum += power_temp;		

		if(SaveData.Value.cal_adjv==0)//
		{//0 校准模式下电流校正显示 
			current_mean = Adj_Nline(current_mean);
		}
		
		count=5;
		//count++;
		//计数addcount后，已经得到经过均值滤波后的电压和电流有效值、有功功率
		if(count == addcount)
		{			
//			//addcount次的均值处理，得到电压、电流、功率有效值
//			voltage_effective=voltage_effective_sum / 5;//addcount;
//			current_effective=current_effective_sum / 5;//addcount;
//			active_power=active_power_sum / 5;//addcount;//平均后的 有功功率值
			
			voltage_effective = Windows_Filter(voltage_temp,voltage_effective_tab,Window_num);
			current_effective = Windows_Filter(current_temp,current_effective_tab,Window_num);
			active_power 	  = Windows_Filter(power_temp,  active_power_tab,     Window_num);

			//printf("ve,ce:%.4f,%.4f\r\n",voltage_effective,current_effective);
			
			if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
				fft_count++;
			
			if(((RotaryKeyValue==KEY_VALUE_6) && (peakstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3)))
			{//VA档Peak档，VA档的CF档
				//最大最小值换算
//				maxv_value = (float)maxv ;
//				minv_value = (float)minv ;
//				maxi_value = (float)maxi ;
//				mini_value = (float)mini ;
				if(maxv > maxv_value)	maxv_value = maxv;
				if(minv < minv_value)	minv_value = minv;
				if(maxi > maxi_value)	maxi_value = maxi;
				if(mini < mini_value)	mini_value = mini;
				
				//计算峰值，还要为了接着求电压、电流的波峰因数
				if(maxv_value > fabs(minv_value))//峰值(2014-10-31注释：只求了正负两者中最大的值)
					voltage_peak_value = maxv_value;
				else	voltage_peak_value = fabs(minv_value);
				if(maxi_value > fabs(mini_value))
					current_peak_value = maxi_value;
				else	current_peak_value = fabs(mini_value);
			}
			if((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3))
			{//VA档的CF档
				if(voltage_effective == 0)	voltage_cf = 0;
				else	voltage_cf = voltage_peak_value / voltage_effective;//波峰因数(CF)
				if(current_effective == 0)	current_cf = 0;
				else	current_cf = current_peak_value / current_effective;
			}
			
			if(RotaryKeyValue==KEY_VALUE_7)
			{//计算视在功率、无功功率、功率因数（PF）
				apparent_power = voltage_effective * current_effective;//视在功率(VA)
				
				if(apparent_power <= active_power)
					reactive_power = 0;
				else
					reactive_power = sqrt(apparent_power * apparent_power - active_power * active_power);//无功功率(Var)
			}
			
			
			if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0) && (paramstatus == state0))
			{//W档的PF档
				power_factor = active_power / apparent_power;//功率因数(PF)
				if(power_factor > 1)	power_factor = 1;
			}
			//count=0;//窗口滤波不清零该计数值，只作为最开始的五次填充数值
			voltage_effective_sum = 0;
			current_effective_sum = 0;
			active_power_sum = 0;
		}//基本运算结束 由addcount计数满足后开始
			
		if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus != state0))
		{//W档计算电能
//			if(powertimer==1)//才开始计算电能时，各电能清零
//			{
//				kWh=0;kVAh=0;kVarh=0;
//			}
			powertimercounter++;//
			/*kWh,kVAh,kVarh,KgCO2;*/
			kWh_sum += active_power;// * (float)powertimer/3600/1000;
			kVAh_sum += apparent_power;// * (float)powertimer/3600/1000;
			kVarh_sum += reactive_power;// * (float)powertimer/3600/1000;
			if(powertimer_1s_flag == 1)//1秒钟标志到来
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
		voltage_mean_sum=0;current_mean_sum=0;//修改一个错误，之前是两个current_mean_sum=0;修改一个为voltage_mean_sum=0;
		
		/*********************************************
		*	FFT相关运算,只在旋钮VA+W档进行
		*********************************************/
		if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
		{//VA档的THD%r、THD%f档，W档的Param档
			if(fft_count>=1)						//每做fft_count次基本运算，就做一次FFT
			{
				FFT();//幅频
				
				if((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1))
				{
					//计算位移功率因数（DPF）
					d_power_factor= cos((current_foudamental_phase- voltage_foudamental_phase)*PI/180);
				}
				
				//电压总谐波畸变率(THD%r%f)
				if(voltage_effective< voltage_foudamental_effective)
					voltage_effective= voltage_foudamental_effective;
				
				if(voltage_effective == 0)	THD_r_voltage=0;
				//else	THD_r_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_effective* voltage_effective));		
				else THD_r_voltage=sqrt(THDV2sum)/voltage_effective;//V THD%r
				
				if(voltage_foudamental_effective == 0)	THD_f_voltage=0;
				//else	THD_f_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_foudamental_effective* voltage_foudamental_effective));
				else THD_f_voltage=sqrt(THDV2sum)/voltage_foudamental_effective;//V THD%f
				
				//电流总谐波畸变率(THD%r%f)
				if(current_effective< current_foudamental_effective)
					current_effective= current_foudamental_effective;
				
				if(current_effective == 0)	THD_r_current=0;
				//else	THD_r_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_effective* current_effective));
				else THD_r_current=sqrt(THDA2sum)/current_effective;//A THD%r
				
				if(current_foudamental_effective == 0)	THD_f_current=0;
				//else	THD_f_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_foudamental_effective* current_foudamental_effective));
				else THD_f_current=sqrt(THDA2sum)/current_foudamental_effective;//A THD%f
				
				//功率总谐波畸变率(THD%r%f)
				if(active_power< power_foudamental_effective)
					active_power= power_foudamental_effective;
				
				if(active_power == 0)	THD_r_power=0;
				//else	THD_r_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(active_power* active_power));
				else THD_r_power=sqrt(THDP2sum)/active_power;//P THD%r
				
				if(power_foudamental_effective == 0)	THD_f_power=0;
				//else	THD_f_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(power_foudamental_effective* power_foudamental_effective));
				else THD_f_power=sqrt(THDP2sum)/power_foudamental_effective;	//P THD%f
				
				//清零
				fft_count = 0;
				voltage_effective_sum = 0;
				current_effective_sum = 0;
				active_power_sum = 0;
			}
		}
		
		if(funcstatus ==state0)//ac
		{
			VadENA=0;//SDADC非使能标志传递到比较中中断，过零点开始ADC采集
			if(phasestatus != state0)
			{
				TIM_Cmd(TIM19, ENABLE);
			}
			//TIM_Cmd(TIM19, ENABLE);//test  测试时打开AC直接测量
		}
		else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   
		{
			TIM_Cmd(TIM19, ENABLE);
		}
		//VadENA=0;//SDADC非使能标志传递到比较中中断，过零点开始ADC采集
		//TIM_Cmd(TIM19, ENABLE);
		
		count_for_ac = count_for_Standby;
	}
	//超时开启一次AC采样（开定时器）    2S一次   由于数值被addcount滤波，所以数值更新会比较慢！
	if(count_for_Standby-count_for_ac>=2)
	{
		TIM_Cmd(TIM19, ENABLE);
	}
}


//2015-10-30JEFF：电压、电流测量时暂时还是为stm32测量得到的电压值
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
	
/*****************************************电压FFT********************************************************/
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = ((float)SDADC1_value[ii]);//
			RAMsave.Input[2*ii+1] = 0;
		}	 
		arm_cfft_radix4_f32(&S,RAMsave.Input);//FFT运算
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);//计算幅值	 
		//由于定义在一个联合体内，此处output 会占用input数组的后面一部分，不影响对于前面低次谐波的计算。
		
//		printf("---------------------电压-----------------------------\r\n");
//		printf("电压频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		THDV2sum=0;
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("直流电压 ,  V=%2.5f V\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//幅值
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//相角
					
					if(fabs(temp_frequency-50)<0.5 /*|| fabs(temp_frequency-60)<0.5*/)//确定基频，为工频
					{
						fundamental_flag= ii;//基波的ii值赋给fundamental_flag，保存下来
						voltage_foudamental_phase= phase_angle[ii];
						voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDV2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波电压方值
					}
					else if(fabs(temp_frequency-60)<0.5)//判断基频是不是60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>voltage_foudamental_effective)
						{
							fundamental_flag= ii;//基波的ii值赋给fundamental_flag，保存下来
							voltage_foudamental_phase= phase_angle[ii];
							voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDV2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波电压方值
						}						
						//voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
					//计算各谐波分量的畸变率
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
//					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
//					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}

/*******************************************电流FFT******************************************************/	
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)SDADC2_value[ii];					//手动添加了一个数值为10的直流分量
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
				
//		printf("---------------------电流-----------------------------\r\n");
//		
//		printf("电流频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("直流电流 ,  I=%2.5f A\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//幅值
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//相角
					
					if(fabs(temp_frequency-50)<0.5f /*|| fabs(temp_frequency-60)<0.5f*/)//确定基频，为工频
					{
						fundamental_flag= ii;
						current_foudamental_phase= phase_angle[ii];
						current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDA2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波电流方值
					}
					else if(fabs(temp_frequency-60)<0.5)//判断基频是不是60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>current_foudamental_effective)
						{
							fundamental_flag= ii;
							current_foudamental_phase= phase_angle[ii];
							current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDA2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波电流方值							
						}						
						//voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
					//计算各谐波分量的畸变率
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
//					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
//					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
/************************************************功率FFT*************************************************/		
	if(RotaryKeyValue==KEY_VALUE_7)//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)POWER_value[ii];					//手动添加了一个数值为10的直流分量
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
		
//		printf("---------------------功率-----------------------------\r\n");

//		printf("功率频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
//					printf("直流(手动添加10W)  ,  Pw=%2.5f W\r\n", Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//幅值
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//相角
					
					if(fabs(temp_frequency-50)<0.5f /*|| fabs(temp_frequency-120)<0.5f*/)//确定基频，为工频
					{
						fundamental_flag= ii;
						power_foudamental_phase= phase_angle[ii];
						power_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDP2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波功率方值
					}
					else if(fabs(temp_frequency-60)<0.5)//判断基频是不是60HZ
					{
						if((RAMsave.K4_tab.Output[ii]/1.41421356f)>power_foudamental_effective)
						{
							fundamental_flag= ii;
							current_foudamental_phase= phase_angle[ii];
							current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
							THDP2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//减掉基波功率方值							
						}												
					}					
					
					//计算各谐波分量的畸变率
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
//					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
//					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
}
/************************************************************************************************
*调整测量到的输入电流对应的电压值
*超过调整门限后即开始调整
*调整公式为 y=a1*exp(x/t1)+y0;
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
//软件过零点检测
//使用SDADC结果数组
/***********************************************/
uint8_t get_formed1024(void)
{
	int16_t tt=0,loop=0;
	int16_t temp[5]={0};//计数单向递增趋势。

	if(funcstatus ==state0)//ac 直接接受由过零点比较得到的1024个点
	{
		for(loop = 0;loop < 1024;loop++)//获取1024个点用于计算
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]>>16);	
		}
	}
	else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   抽取1024点
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
		
		for(loop = 0;loop < 1024;loop++)//获取1024个点用于计算
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]>>16);	
		}
	}
	return 1;

}


/***************************************************************************************************/
//将一个浮点数传给一个浮点数组并返回平均值
//特殊情况：若该浮点数与数组中第一个数[0]的差值大于 [0]*0.2  则用该数值填充当前数组
//输入参数：	New_Data 新传入的参数。
//			Data_tab 当前数据所在的窗口数组
//			num  窗口大小（数组大小）
//
//函数返回：	窗口滤波值
/****************************************************************************************************/
float Windows_Filter(float New_Data,float *Data_tab,char num)
{
	char i=0;
	float value=0;
	
	if(Data_tab==current_mean_tab)//电流滤波的容许误差放大一点
	 {
		if(fabs(New_Data-Data_tab[0])>fabs(Data_tab[0]*0.2))//若偏差值较大则填充滤波寄存器
		{
			for(i=0;i<num;i++)
			{
				Data_tab[i]=New_Data;
			}
		}
		else//将新值移动到窗口中
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
		if(fabs(New_Data-Data_tab[0])>fabs(Data_tab[0]*0.01))//若偏差值较大则填充滤波寄存器
		{
			for(i=0;i<num;i++)
			{
				Data_tab[i]=New_Data;
			}
		}
		else//将新值移动到窗口中
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









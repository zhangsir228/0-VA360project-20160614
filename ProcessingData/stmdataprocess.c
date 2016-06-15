#include "stm32f37x.h"
#include "sdadc.h"
#include <math.h>
#include "comp.h"
#include "matrixkey.h"
#include "stmdataprocess.h"
#include "arm_math.h" 


//#define TEST_LENGTH_SAMPLES 2048 
// 
///* ------------------------------------------------------------------- 
//* External Input and Output buffer Declarations for FFT Bin Example 
//* ------------------------------------------------------------------- */ 
//static float Input[TEST_LENGTH_SAMPLES]={0}; 
//static float32_t Output[TEST_LENGTH_SAMPLES/2];
//float phase_angle[TEST_LENGTH_SAMPLES/4];
//float THD[TEST_LENGTH_SAMPLES/4];

//#define Test_Fs 10240
//#define PI2 (6.283185307179586476925286766559)
////  
///* ------------------------------------------------------------------ 
//* Global variables for FFT Bin Example 
//* ------------------------------------------------------------------- */ 
//uint16_t fftSize = TEST_LENGTH_SAMPLES/2; 
//uint8_t ifftFlag = 0; 
//uint8_t doBitReverse = 1;

//uint32_t refIndex = 213, testIndex = 0; 
////static float32_t maxValue=0; 
//unsigned int iiflag=0;
/*******************************以上是FFT的初始化*************************************/


/*key*/
u8 key=0,page=0;

/******************计数***********************/
uint16_t t;
uint8_t count=0;
uint8_t fft_count=0;
const uint8_t addcount=5;
uint16_t datasize;

/******************标志***********************/
uint32_t sdadc_cali_flag=0;//SDADC配置成功与否标志，0-成功，1-INITRDY未置位，2-EOCAL未置位
uint8_t pause_flag=0;//液晶显示暂停标志，0-不暂停，1-暂停
uint8_t af_flag=0;//幅频显示标志,1-电压，2-电流，3-功率
uint8_t r_or_f_flag=0;//THD选择模式，0-THD%r,1-THD%f
u8 adjust_flag=0;

/******************SDADC采集和捕获基频完成标志***********************/
uint8_t collect_finished=0;//SDADC采样标志
uint8_t voltage_capture_finished=0 , current_capture_finished=0;//过零点检测，捕获基频标志

/******************电压、电流、功率存储数组***********************/
int16_t SDADC1_value[1024]={0},SDADC2_value[1024]={0};
//int32_t POWER_value[1024]={0};

/******************需要计算的量***********************/
float voltage_effective=0,current_effective=0;//有效电压、有效电流
float maxv_value,maxi_value,minv_value,mini_value;//最大、最小|电压、电流
float voltage_cf=0,current_cf=0;//波峰因数[CF]
float apparent_power=0,active_power=0,reactive_power=0;//视在功率、有功功率、无功功率
float power_factor=0,d_power_factor=0;//功率因数[PF]、位移功率因数[DPF]
float THD_r_voltage=0,THD_f_voltage=0,THD_r_current=0,THD_f_current=0,THD_r_power=0,THD_f_power=0;//总谐波畸变率，相对整个有效值（基波+谐波分量）[THD%r]/相对基波有效值[THD%f]

/******************计算中出现的中间量***********************/
float voltage_sum=0,current_sum=0,power_sum=0;//计算Vrms、Arms、有功功率
float voltage_temp=0,voltage_effective_before=0;
float current_temp=0,current_effective_before=0;
float power_temp=0,active_power_before=0;

int16_t maxv=0,maxi=0,minv=0,mini=0;//计算最大、最小值

float voltage_peak_value=0,current_peak_value=0;//计算峰值->CF

float voltage_foudamental_effective=0,current_foudamental_effective=0,power_foudamental_effective=0;//计算基波有效值->THD%r、THD%f

float voltage_foudamental_phase=0,current_foudamental_phase=0,power_foudamental_phase=0;//计算基波相位->DPF

float voltage_fundamental_frequency = 0 , current_fundamental_frequency = 0;



void dealwith_information(void)
{
	int16_t v_temp,a_temp;
	/*********************************************
	*	FFT相关运算,只在旋钮W档进行
	*********************************************/
	if((RotaryKeyValue == 0x07) && (fft_count==3))						//每做十次基本运算，就做一次FFT
	{
//		FFT();//2015月3月18日注释掉
		
		//计算位移功率因数（DPF）
		d_power_factor= cos(current_foudamental_phase- voltage_foudamental_phase);
		
		//电压总谐波畸变率
		if(voltage_effective< voltage_foudamental_effective)
			voltage_effective= voltage_foudamental_effective;
		
		if(voltage_effective == 0)	THD_r_voltage=0;
		else	THD_r_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_effective* voltage_effective));
		
		if(voltage_foudamental_effective == 0)	THD_f_voltage=0;
		else	THD_f_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_foudamental_effective* voltage_foudamental_effective));
		
		//电流总谐波畸变率
		if(current_effective< current_foudamental_effective)
			current_effective= current_foudamental_effective;
		
		if(current_effective == 0)	THD_r_current=0;
		else	THD_r_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_effective* current_effective));
		
		if(current_foudamental_effective == 0)	THD_f_current=0;
		else	THD_f_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_foudamental_effective* current_foudamental_effective));
		
		//功率总谐波畸变率
		if(active_power< power_foudamental_effective)
			active_power= power_foudamental_effective;
		
		if(active_power == 0)	THD_r_power=0;
		else	THD_r_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(active_power* active_power));
		
		if(voltage_foudamental_effective == 0)	THD_f_power=0;
		else	THD_f_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(power_foudamental_effective* power_foudamental_effective));
		
		
		//清零
		fft_count = 0;
		voltage_effective = 0;
		current_effective = 0;
		active_power = 0;
	}
	/*********************************************
	*	采集结束标志到来，进行相应运算
	*********************************************/
	if(collect_finished == 1)
	{
		collect_finished = 0;
		
//		if(pause_flag==0 && af_flag==0)
//			ili9320_Clear(Blue);
		for(t = 0;t < datasize;t++)
		{
			//得到2路采集数据，存入数组中
			v_temp = (InjectedConvData[t]&0xFFFF);	//获得低16位，为测量电压
			a_temp = (InjectedConvData[t]>>16);		//获得高16位，为测量电流
			SDADC1_value[t] = v_temp;
			SDADC2_value[t] = a_temp;
			
//			//实时显示
//			if(pause_flag==0 && af_flag==0)
//			{
//				ili9320_SetPoint(10+t*300/datasize, 130-v_temp* SDADC_CAL_COEF*200/1,Red);
//				ili9320_SetPoint(10+t*300/datasize, 130-a_temp* SDADC_CAL_COEF*200/1,Black);
//			}
			
			//求最大值//求最小值
			if(t == 0)
			{
				maxv = v_temp;minv = v_temp;maxi = a_temp;mini = a_temp;
			}
			if(maxv < v_temp)	maxv = v_temp;
			if(minv > v_temp)	minv = v_temp;
			
			if(maxi < a_temp)	maxi = a_temp;
			if(mini > a_temp)	mini = a_temp;
			
//			//功率瞬时值
//			POWER_value[t] = v_temp * a_temp;
			
			//电压、电流、功率有效值计算，先算一部分——求平方和（V/A）、求和（W）
			voltage_sum += v_temp * v_temp * SDADC_CAL_COEF2;//差分输入时，计算电压
			current_sum += a_temp * a_temp * SDADC_CAL_COEF2;//差分输入时，计算电流
			power_sum += v_temp * a_temp * SDADC_CAL_COEF2;//求有功功率//平均功率
		}
		
		//电压、电流、功率有效值计算，再算另一部分——求均、开根（V/A）、求均（W）
		voltage_temp = sqrt(voltage_sum / datasize);
		current_temp = sqrt(current_sum / datasize);
		power_temp = current_sum / datasize;
		
		//累计addcount次，求平均值
		voltage_effective += voltage_temp / addcount;
		current_effective += current_temp / addcount;
		active_power += power_temp / addcount;
		
		//计数addcount后，已经得到经过均值滤波后的电压和电流有效值、有功功率
		if(count == addcount)
		{
			fft_count++;
			
			//最大最小值换算，int16_t -> float
			maxv_value = (float)maxv * SDADC_CAL_COEF;
			minv_value = (float)minv * SDADC_CAL_COEF;
			maxi_value = (float)maxi * SDADC_CAL_COEF;
			mini_value = (float)mini * SDADC_CAL_COEF;
			
			//计算峰值，接着求电压、电流的波峰因数
			if(maxv_value > fabs(minv_value))//峰值(2014-10-31注释：只求了正负两者中最大的值)
				voltage_peak_value = maxv_value;
			else	voltage_peak_value = fabs(minv_value);
			if(maxi_value > fabs(mini_value))
				current_peak_value = maxi_value;
			else	current_peak_value = fabs(mini_value);
			
			if(voltage_effective == 0)	voltage_cf = 0;
			else	voltage_cf = voltage_peak_value / voltage_effective;//波峰因数
			if(current_effective == 0)	current_cf = 0;
			else	current_cf = current_peak_value / current_effective;
			
			//计算视在功率、无功功率、功率因数（PF）
			apparent_power = voltage_effective * current_effective;//视在功率
			
			if(apparent_power <= active_power)
				reactive_power = 0;
			else
				reactive_power = sqrt(apparent_power * apparent_power - active_power * active_power);//无功功率
			
			power_factor = active_power / apparent_power;//功率因数
			if(power_factor > 1)	power_factor = 1;
			
			printf("---------------------------|------------------------\r\n");
			printf("Vmax=         %2.5f V ,  |   Vmin=%2.5f V  \r\n", maxv_value, minv_value);
			printf("Imax=         %2.5f A ,  |   Imin=%2.5f A  \r\n", maxi_value, mini_value);
			printf("Vrms=         %2.5f V ,  |   Irms=%2.5f A  \r\n", voltage_effective, current_effective);
			printf("S=            %2.5f VA , |   P=%2.5f W,	         |    Q=%2.5f VAR  \r\n", apparent_power, active_power, reactive_power);
			printf("Vrms(基波)=   %2.5f V,   |   Arms(基波)=%2.5f A  \r\n", voltage_foudamental_effective, current_foudamental_effective);
			printf("V_CF=         %2.5f ,	   |   A_CF=%2.5f   \r\n", voltage_cf, current_cf);
			printf("PF=           %2.5f ,    |   DPF=%2.5f   \r\n", power_factor, d_power_factor);
			printf("THD_r_voltage=%2.5f ,    |   THD_f_voltage=%2.5f   \r\n", THD_r_voltage, THD_f_voltage);
			printf("THD_r_current=%2.5f ,    |   THD_f_current=%2.5f   \r\n", THD_r_current, THD_f_current);
			printf("THD_r_power=  %2.5f ,	   |   THD_f_power=%2.5f   \r\n", THD_r_power, THD_f_power);
			printf("---------------------------|------------------------\r\n");
			
			count=0;
			
			//当未到FFT计算时，清掉在计算中存在累加的数。若达到FFT计算，在FFT计算后再清掉
			if(fft_count<3)
			{
				voltage_effective=0;
				current_effective=0;
				active_power=0;
			}
		}
		count++;
		maxv=0;maxi=0;minv=0;mini=0;
		voltage_sum=0;current_sum=0;power_sum=0;
		
//			TIM_Cmd(TIM19, DISABLE);
//		DMA_Cmd(DMA2_Channel3, DISABLE);  //关闭DMA2 所指示的通道			
//		DMA_SetCurrDataCounter(DMA2_Channel3,DMA2_MEM_LEN);//DMA通道的DMA缓存的大小
//		DMA_Cmd(DMA2_Channel3, ENABLE);  //使能DMA2 所指示的通道 	
		TIM_Cmd(TIM19, ENABLE);
	}
}







// 
//void FFT(void)
//{
//  unsigned int ii=0;
//	uint8_t fundamental_flag=0;
//	float temp_frequency;
//  arm_status status;
//  arm_cfft_radix4_instance_f32 S;  
//     
//  status = ARM_MATH_SUCCESS;
//   
//  /* Initialize the CFFT/CIFFT module */  
//  status = arm_cfft_radix4_init_f32(&S, fftSize,ifftFlag, doBitReverse);
//	
///*****************************************电压FFT********************************************************/
//	if(pause_flag==0 && af_flag==1)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ ((float)SDADC1_value[ii]* SDADC_CAL_COEF)* 1;//(0.5-0.5*cos(PI2*ii/1023));					//手动添加了一个数值为10的直流分量
//			Input[2*ii+1] = 0;
//		}		 
//		arm_cfft_radix4_f32(&S,Input);//FFT运算
//		arm_cmplx_mag_f32(Input, Output, fftSize);//计算幅值	
//	
////		ili9320_Clear(Blue);//2015年3月18日注释掉
//		
//		printf("---------------------电压-----------------------------\r\n");
//		printf("电压频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015年3月18日注释掉
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("直流电压(手动添加10V)  ,  V=%2.5f V\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
//					Output[ii]=Output[ii]*2/fftSize;//幅值
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//相角
//					
//					if(fabs(temp_frequency-50)<0.5 || fabs(temp_frequency-60)<0.5)//确定基频，为工频
//					{
//						fundamental_flag= ii;//基波的ii值赋给fundamental_flag，保存下来
//						voltage_foudamental_phase= temp_frequency;
//						voltage_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					//计算各谐波分量的畸变率
////					if(r_or_f_flag==0)
////					{
////						if(voltage_effective==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ voltage_effective;//THD%r
////					}
////					else
////					{
////						if(voltage_foudamental_effective==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ voltage_foudamental_effective;//THD%f
////					}
//					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("V=%2.5f V  ,  ", Output[ii]);
////					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
////					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}

///*******************************************电流FFT******************************************************/	
//	if(pause_flag==0 && af_flag==2)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ (float)SDADC2_value[ii]* SDADC_CAL_COEF;					//手动添加了一个数值为10的直流分量
//			Input[2*ii+1] = 0;
//		}
//		arm_cfft_radix4_f32(&S,Input);   
//		arm_cmplx_mag_f32(Input, Output, fftSize);
//		
////		ili9320_Clear(Blue);//2015年3月18日注释掉
//		
//		printf("---------------------电流-----------------------------\r\n");
//		
//		printf("电流频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015年3月18日注释掉
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("直流电流(手动添加10A)  ,  I=%2.5f A\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
//					Output[ii]=Output[ii]*2/fftSize;//幅值
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//相角
//					
//					if(fabs(temp_frequency-50)<0.5f || fabs(temp_frequency-60)<0.5f)//确定基频，为工频
//					{
//						fundamental_flag= ii;
//						current_foudamental_phase= temp_frequency;
//						current_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					
//					//计算各谐波分量的畸变率
////					if(r_or_f_flag==0)
////					{
////						if(current_effective==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ current_effective;//THD%r
////					}
////					else
////					{
////						if(current_foudamental_effective==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ current_foudamental_effective;//THD%f
////					}
//					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("I=%2.5f A  ,  ", Output[ii]);
////					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
////					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}
///************************************************功率FFT*************************************************/		
//	if(pause_flag==0 && af_flag==3)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ (float)SDADC1_value[ii] * SDADC2_value[ii] * SDADC_CAL_COEF2;					//手动添加了一个数值为10的直流分量
//			Input[2*ii+1] = 0;
//		}
//		arm_cfft_radix4_f32(&S,Input);
//		arm_cmplx_mag_f32(Input, Output, fftSize);
//		
////		ili9320_Clear(Blue);//2015年3月18日注释掉
//		
//		printf("---------------------功率-----------------------------\r\n");

//		printf("功率频率分辨率-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015年3月18日注释掉
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("直流(手动添加10W)  ,  Pw=%2.5f W\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//频率
//					Output[ii]=Output[ii]*2/fftSize;//幅值
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//相角
//					
//					if(fabs(temp_frequency-100)<0.5f || fabs(temp_frequency-120)<0.5f)//确定基频，为工频
//					{
//						fundamental_flag= ii;
//						power_foudamental_phase= temp_frequency;
//						power_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					
//					//计算各谐波分量的畸变率
////					if(r_or_f_flag==0)
////					{
////						if(active_power==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ active_power;//THD%r
////					}
////					else
////					{
////						if(power_foudamental_effective==0)	THD[ii]=0;
////						else	THD[ii]= Output[ii]/1.41421356f/ power_foudamental_effective;//THD%f
////					}
//					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("Pw=%2.5f W  ,  ", Output[ii]);
////					printf("Phase=%2.5f 度  ，  ", phase_angle[ii]);
////					printf("THD=%2.5f(百分数) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}
///*************************************************************************************************/
//	
//	
//}

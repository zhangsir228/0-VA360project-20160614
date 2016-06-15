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
/*******************************������FFT�ĳ�ʼ��*************************************/


/*key*/
u8 key=0,page=0;

/******************����***********************/
uint16_t t;
uint8_t count=0;
uint8_t fft_count=0;
const uint8_t addcount=5;
uint16_t datasize;

/******************��־***********************/
uint32_t sdadc_cali_flag=0;//SDADC���óɹ�����־��0-�ɹ���1-INITRDYδ��λ��2-EOCALδ��λ
uint8_t pause_flag=0;//Һ����ʾ��ͣ��־��0-����ͣ��1-��ͣ
uint8_t af_flag=0;//��Ƶ��ʾ��־,1-��ѹ��2-������3-����
uint8_t r_or_f_flag=0;//THDѡ��ģʽ��0-THD%r,1-THD%f
u8 adjust_flag=0;

/******************SDADC�ɼ��Ͳ����Ƶ��ɱ�־***********************/
uint8_t collect_finished=0;//SDADC������־
uint8_t voltage_capture_finished=0 , current_capture_finished=0;//������⣬�����Ƶ��־

/******************��ѹ�����������ʴ洢����***********************/
int16_t SDADC1_value[1024]={0},SDADC2_value[1024]={0};
//int32_t POWER_value[1024]={0};

/******************��Ҫ�������***********************/
float voltage_effective=0,current_effective=0;//��Ч��ѹ����Ч����
float maxv_value,maxi_value,minv_value,mini_value;//�����С|��ѹ������
float voltage_cf=0,current_cf=0;//��������[CF]
float apparent_power=0,active_power=0,reactive_power=0;//���ڹ��ʡ��й����ʡ��޹�����
float power_factor=0,d_power_factor=0;//��������[PF]��λ�ƹ�������[DPF]
float THD_r_voltage=0,THD_f_voltage=0,THD_r_current=0,THD_f_current=0,THD_r_power=0,THD_f_power=0;//��г�������ʣ����������Чֵ������+г��������[THD%r]/��Ի�����Чֵ[THD%f]

/******************�����г��ֵ��м���***********************/
float voltage_sum=0,current_sum=0,power_sum=0;//����Vrms��Arms���й�����
float voltage_temp=0,voltage_effective_before=0;
float current_temp=0,current_effective_before=0;
float power_temp=0,active_power_before=0;

int16_t maxv=0,maxi=0,minv=0,mini=0;//���������Сֵ

float voltage_peak_value=0,current_peak_value=0;//�����ֵ->CF

float voltage_foudamental_effective=0,current_foudamental_effective=0,power_foudamental_effective=0;//���������Чֵ->THD%r��THD%f

float voltage_foudamental_phase=0,current_foudamental_phase=0,power_foudamental_phase=0;//���������λ->DPF

float voltage_fundamental_frequency = 0 , current_fundamental_frequency = 0;



void dealwith_information(void)
{
	int16_t v_temp,a_temp;
	/*********************************************
	*	FFT�������,ֻ����ťW������
	*********************************************/
	if((RotaryKeyValue == 0x07) && (fft_count==3))						//ÿ��ʮ�λ������㣬����һ��FFT
	{
//		FFT();//2015��3��18��ע�͵�
		
		//����λ�ƹ���������DPF��
		d_power_factor= cos(current_foudamental_phase- voltage_foudamental_phase);
		
		//��ѹ��г��������
		if(voltage_effective< voltage_foudamental_effective)
			voltage_effective= voltage_foudamental_effective;
		
		if(voltage_effective == 0)	THD_r_voltage=0;
		else	THD_r_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_effective* voltage_effective));
		
		if(voltage_foudamental_effective == 0)	THD_f_voltage=0;
		else	THD_f_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_foudamental_effective* voltage_foudamental_effective));
		
		//������г��������
		if(current_effective< current_foudamental_effective)
			current_effective= current_foudamental_effective;
		
		if(current_effective == 0)	THD_r_current=0;
		else	THD_r_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_effective* current_effective));
		
		if(current_foudamental_effective == 0)	THD_f_current=0;
		else	THD_f_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_foudamental_effective* current_foudamental_effective));
		
		//������г��������
		if(active_power< power_foudamental_effective)
			active_power= power_foudamental_effective;
		
		if(active_power == 0)	THD_r_power=0;
		else	THD_r_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(active_power* active_power));
		
		if(voltage_foudamental_effective == 0)	THD_f_power=0;
		else	THD_f_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(power_foudamental_effective* power_foudamental_effective));
		
		
		//����
		fft_count = 0;
		voltage_effective = 0;
		current_effective = 0;
		active_power = 0;
	}
	/*********************************************
	*	�ɼ�������־������������Ӧ����
	*********************************************/
	if(collect_finished == 1)
	{
		collect_finished = 0;
		
//		if(pause_flag==0 && af_flag==0)
//			ili9320_Clear(Blue);
		for(t = 0;t < datasize;t++)
		{
			//�õ�2·�ɼ����ݣ�����������
			v_temp = (InjectedConvData[t]&0xFFFF);	//��õ�16λ��Ϊ������ѹ
			a_temp = (InjectedConvData[t]>>16);		//��ø�16λ��Ϊ��������
			SDADC1_value[t] = v_temp;
			SDADC2_value[t] = a_temp;
			
//			//ʵʱ��ʾ
//			if(pause_flag==0 && af_flag==0)
//			{
//				ili9320_SetPoint(10+t*300/datasize, 130-v_temp* SDADC_CAL_COEF*200/1,Red);
//				ili9320_SetPoint(10+t*300/datasize, 130-a_temp* SDADC_CAL_COEF*200/1,Black);
//			}
			
			//�����ֵ//����Сֵ
			if(t == 0)
			{
				maxv = v_temp;minv = v_temp;maxi = a_temp;mini = a_temp;
			}
			if(maxv < v_temp)	maxv = v_temp;
			if(minv > v_temp)	minv = v_temp;
			
			if(maxi < a_temp)	maxi = a_temp;
			if(mini > a_temp)	mini = a_temp;
			
//			//����˲ʱֵ
//			POWER_value[t] = v_temp * a_temp;
			
			//��ѹ��������������Чֵ���㣬����һ���֡�����ƽ���ͣ�V/A������ͣ�W��
			voltage_sum += v_temp * v_temp * SDADC_CAL_COEF2;//�������ʱ�������ѹ
			current_sum += a_temp * a_temp * SDADC_CAL_COEF2;//�������ʱ���������
			power_sum += v_temp * a_temp * SDADC_CAL_COEF2;//���й�����//ƽ������
		}
		
		//��ѹ��������������Чֵ���㣬������һ���֡��������������V/A���������W��
		voltage_temp = sqrt(voltage_sum / datasize);
		current_temp = sqrt(current_sum / datasize);
		power_temp = current_sum / datasize;
		
		//�ۼ�addcount�Σ���ƽ��ֵ
		voltage_effective += voltage_temp / addcount;
		current_effective += current_temp / addcount;
		active_power += power_temp / addcount;
		
		//����addcount���Ѿ��õ�������ֵ�˲���ĵ�ѹ�͵�����Чֵ���й�����
		if(count == addcount)
		{
			fft_count++;
			
			//�����Сֵ���㣬int16_t -> float
			maxv_value = (float)maxv * SDADC_CAL_COEF;
			minv_value = (float)minv * SDADC_CAL_COEF;
			maxi_value = (float)maxi * SDADC_CAL_COEF;
			mini_value = (float)mini * SDADC_CAL_COEF;
			
			//�����ֵ���������ѹ�������Ĳ�������
			if(maxv_value > fabs(minv_value))//��ֵ(2014-10-31ע�ͣ�ֻ������������������ֵ)
				voltage_peak_value = maxv_value;
			else	voltage_peak_value = fabs(minv_value);
			if(maxi_value > fabs(mini_value))
				current_peak_value = maxi_value;
			else	current_peak_value = fabs(mini_value);
			
			if(voltage_effective == 0)	voltage_cf = 0;
			else	voltage_cf = voltage_peak_value / voltage_effective;//��������
			if(current_effective == 0)	current_cf = 0;
			else	current_cf = current_peak_value / current_effective;
			
			//�������ڹ��ʡ��޹����ʡ�����������PF��
			apparent_power = voltage_effective * current_effective;//���ڹ���
			
			if(apparent_power <= active_power)
				reactive_power = 0;
			else
				reactive_power = sqrt(apparent_power * apparent_power - active_power * active_power);//�޹�����
			
			power_factor = active_power / apparent_power;//��������
			if(power_factor > 1)	power_factor = 1;
			
			printf("---------------------------|------------------------\r\n");
			printf("Vmax=         %2.5f V ,  |   Vmin=%2.5f V  \r\n", maxv_value, minv_value);
			printf("Imax=         %2.5f A ,  |   Imin=%2.5f A  \r\n", maxi_value, mini_value);
			printf("Vrms=         %2.5f V ,  |   Irms=%2.5f A  \r\n", voltage_effective, current_effective);
			printf("S=            %2.5f VA , |   P=%2.5f W,	         |    Q=%2.5f VAR  \r\n", apparent_power, active_power, reactive_power);
			printf("Vrms(����)=   %2.5f V,   |   Arms(����)=%2.5f A  \r\n", voltage_foudamental_effective, current_foudamental_effective);
			printf("V_CF=         %2.5f ,	   |   A_CF=%2.5f   \r\n", voltage_cf, current_cf);
			printf("PF=           %2.5f ,    |   DPF=%2.5f   \r\n", power_factor, d_power_factor);
			printf("THD_r_voltage=%2.5f ,    |   THD_f_voltage=%2.5f   \r\n", THD_r_voltage, THD_f_voltage);
			printf("THD_r_current=%2.5f ,    |   THD_f_current=%2.5f   \r\n", THD_r_current, THD_f_current);
			printf("THD_r_power=  %2.5f ,	   |   THD_f_power=%2.5f   \r\n", THD_r_power, THD_f_power);
			printf("---------------------------|------------------------\r\n");
			
			count=0;
			
			//��δ��FFT����ʱ������ڼ����д����ۼӵ��������ﵽFFT���㣬��FFT����������
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
//		DMA_Cmd(DMA2_Channel3, DISABLE);  //�ر�DMA2 ��ָʾ��ͨ��			
//		DMA_SetCurrDataCounter(DMA2_Channel3,DMA2_MEM_LEN);//DMAͨ����DMA����Ĵ�С
//		DMA_Cmd(DMA2_Channel3, ENABLE);  //ʹ��DMA2 ��ָʾ��ͨ�� 	
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
///*****************************************��ѹFFT********************************************************/
//	if(pause_flag==0 && af_flag==1)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ ((float)SDADC1_value[ii]* SDADC_CAL_COEF)* 1;//(0.5-0.5*cos(PI2*ii/1023));					//�ֶ������һ����ֵΪ10��ֱ������
//			Input[2*ii+1] = 0;
//		}		 
//		arm_cfft_radix4_f32(&S,Input);//FFT����
//		arm_cmplx_mag_f32(Input, Output, fftSize);//�����ֵ	
//	
////		ili9320_Clear(Blue);//2015��3��18��ע�͵�
//		
//		printf("---------------------��ѹ-----------------------------\r\n");
//		printf("��ѹƵ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015��3��18��ע�͵�
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("ֱ����ѹ(�ֶ����10V)  ,  V=%2.5f V\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
//					Output[ii]=Output[ii]*2/fftSize;//��ֵ
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//���
//					
//					if(fabs(temp_frequency-50)<0.5 || fabs(temp_frequency-60)<0.5)//ȷ����Ƶ��Ϊ��Ƶ
//					{
//						fundamental_flag= ii;//������iiֵ����fundamental_flag����������
//						voltage_foudamental_phase= temp_frequency;
//						voltage_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					//�����г�������Ļ�����
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
////					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
////					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}

///*******************************************����FFT******************************************************/	
//	if(pause_flag==0 && af_flag==2)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ (float)SDADC2_value[ii]* SDADC_CAL_COEF;					//�ֶ������һ����ֵΪ10��ֱ������
//			Input[2*ii+1] = 0;
//		}
//		arm_cfft_radix4_f32(&S,Input);   
//		arm_cmplx_mag_f32(Input, Output, fftSize);
//		
////		ili9320_Clear(Blue);//2015��3��18��ע�͵�
//		
//		printf("---------------------����-----------------------------\r\n");
//		
//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015��3��18��ע�͵�
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("ֱ������(�ֶ����10A)  ,  I=%2.5f A\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
//					Output[ii]=Output[ii]*2/fftSize;//��ֵ
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//���
//					
//					if(fabs(temp_frequency-50)<0.5f || fabs(temp_frequency-60)<0.5f)//ȷ����Ƶ��Ϊ��Ƶ
//					{
//						fundamental_flag= ii;
//						current_foudamental_phase= temp_frequency;
//						current_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					
//					//�����г�������Ļ�����
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
////					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
////					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}
///************************************************����FFT*************************************************/		
//	if(pause_flag==0 && af_flag==3)
//	{
//		for(ii=0;ii<fftSize;ii++)
//		{
//			Input[2*ii] = 0.0f+ (float)SDADC1_value[ii] * SDADC2_value[ii] * SDADC_CAL_COEF2;					//�ֶ������һ����ֵΪ10��ֱ������
//			Input[2*ii+1] = 0;
//		}
//		arm_cfft_radix4_f32(&S,Input);
//		arm_cmplx_mag_f32(Input, Output, fftSize);
//		
////		ili9320_Clear(Blue);//2015��3��18��ע�͵�
//		
//		printf("---------------------����-----------------------------\r\n");

//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
//		for(ii=0;ii<fftSize/2;ii++)
//		{
////			GUI_Line(10+ii*300*2/fftSize,230,10+ii*300*2/fftSize, 230- Output[ii]*2/fftSize*200/0.8,Red);//2015��3��18��ע�͵�
//			
//			if(Output[ii]/fftSize>0.01f)
//			{
//				if(ii==0)
//				{
//					printf("ֱ��(�ֶ����10W)  ,  Pw=%2.5f W\r\n", Output[ii]/fftSize);
//				}
//				else
//				{
//					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
//					Output[ii]=Output[ii]*2/fftSize;//��ֵ
////					phase_angle[ii]= atan2(Input[2*ii+1],Input[2*ii+1])* 360/PI2;//���
//					
//					if(fabs(temp_frequency-100)<0.5f || fabs(temp_frequency-120)<0.5f)//ȷ����Ƶ��Ϊ��Ƶ
//					{
//						fundamental_flag= ii;
//						power_foudamental_phase= temp_frequency;
//						power_foudamental_effective= Output[ii]/1.41421356f;
//					}
//					
//					//�����г�������Ļ�����
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
////					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
////					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
//				}
//			}
//		}
//	}
///*************************************************************************************************/
//	
//	
//}

#ifndef __STMDATAPROCESS_H
#define __STMDATAPROCESS_H
#include "stm32f37x.h"



extern uint16_t datasize;


/*sdadc.c交互信息*/
extern float SDADC1_value[];
extern float SDADC2_value[];
extern float POWER_value[];
/*功率档测量参数*/
extern float maxv_value,maxi_value,minv_value,mini_value;//最大、最小|电压、电流
extern float voltage_effective,current_effective;//有效电压、有效电流
extern float inrush_current_effective_100ms;//100ms浪涌电流
extern float voltage_mean,current_mean;//平均电压、平均电流
extern float apparent_power,active_power,reactive_power;//视在功率、有功功率、无功功率
extern float kWh,kVAh,kVarh,KgCO2;
extern float voltage_foudamental_effective,current_foudamental_effective,power_foudamental_effective;//电压/电流/功率基波有效值
extern float voltage_cf,current_cf;//电压波峰因数、电流波峰因数
extern float power_factor,d_power_factor;//功率因数[PF]、位移功率因数[DPF]
extern float THD_r_voltage,THD_f_voltage,THD_r_current,THD_f_current,THD_r_power,THD_f_power;//总谐波畸变率，相对整个有效值（基波+谐波分量）[THD%r]/相对基波有效值[THD%f]

extern float voltage_fundamental_frequency , current_fundamental_frequency;//电压/电流基波频率
extern float voltage_foudamental_phase , current_foudamental_phase , power_foudamental_phase;//电压/电流/功率基波相位
/*标志*/
extern uint8_t collect_finished;
extern uint8_t voltage_capture_finished , current_capture_finished;


//extern uint32_t SAMPLING_FREQ_FFT;

extern uint8_t pause_flag,af_flag,r_or_f_flag;
extern u8 inrush_trigger_flag;

extern u8 timer_1s_blink;



union RAMsavedef
{
	float Input[2048+256];
	
	struct
	{		
		int32_t InjectedConvData[1280];		
		float Output[1024];
	}K4_tab;
	
	
};


extern union RAMsavedef RAMsave;

void dealwith_information(void);




void FFT(void);
//void Test_FFT(void) ;
float Adj_V(float sdadc_value);
uint8_t get_formed1024(void);
float Windows_Filter(float New_Data,float *Data_tab,char num);
#endif

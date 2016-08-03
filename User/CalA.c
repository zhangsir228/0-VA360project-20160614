/****************************************************************************************
	*	@file		CalA.c
	*	@author	Lea
	*	@date		2016/6/13
	*	@brief	本程序实现功能：
	*					1）通过特定的校准点校准目标钳头测量电流输出电压。
	*					2）校正公式为 y=a1*exp(x/t1)+y0    x为输入线性校正电流值  y为大电流校正后的值。
	*					3）使每一组输入的（xi，yi）满足以上公式 （使误差控制在一个特定差值以内）。
	*					4）
	*
	*		其他：
	*
	*
	*					
***************************************************************************************/
#include "CalA.h"	



//大电流校准6个点
float CalA_point[8]={1600,1800,2000,2200,2400,2600,2800,3000};
//对应电流校准点下的电流测量值
//float CalA_data[8] ={
//	1604.61,	//1600
//	1796.3,	//1800
//	1980.4,	//2000
//	2158.21,	//2200
//	2317.46,	//2400
//	2441.77,	//2600
//	2600, //2800 以下两组数据未经测量
//	2750, //3000
//};
//按照当前参数计算出来的各点校准结果
float CalA_temp[8] ={0,0,0,0,0,0,0,0};


float Adj_Nline(float Value)
{
	float new_value=0;
	uint8_t i=0;
	
	if(Value>1500)
	{
		for(i=0;i<8;i++)
		{
			if(CalA_data[i]<=0){break;}
			else if(Value<CalA_data[i])
			{
				new_value=CalA_point[i-1]+((Value-CalA_data[i-1])/(CalA_data[i]-CalA_data[i-1]))*(CalA_point[i]-CalA_point[i-1]);
				return new_value;
			}
		}
//		while(Value<CalA_data[i])
//		{
//			i++;
//			if(CalA_data[i]==0)break;//
//		}
//		new_value=CalA_point[i]+((Value-CalA_data[i-1])/(CalA_data[i]-CalA_data[i-1]))*(CalA_point[i]-CalA_point[i-1]);
//	
//		return new_value;
	}
	return Value;
}



/*****************************************************************************
* 循环调整系数a1  t1  y0 使得每个校正点的测量数据经过校正曲线公式后逼近校正点的值
* CalA_point  ，  CalA_data
* float 返回最大误差占其对应校准点的百分比值
*	前四个校准点调整a1 后四个校准点调整a0    y0作为起始点的误差值暂时不予考虑
*
*
* Lea
* 
********************************************************************************/
float Adj_EXP(void)	
{
	uint8_t i=0;
	float Err_percent=0;
//	float test=0,test1=0,test2=0;

//	test=exp(2);
//	test1=log(test);
//	test2=exp(1);

	for(i=0;i<8;i++)
	{
		if(CalA_data[i]==0)continue;//若当前校准点处的测量数据为零则跳过
		
		CalA_temp[i] = CalA_data[i]+MyEXP(CalA_data[i]);
		
		if(((CalA_point[i]-CalA_temp[i])>1)||((CalA_point[i]-CalA_temp[i])<-1))
		{//测量值校正后误差超过1A
			//SaveData.Value.cal_A_a1*exp((CalA_data[i]/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0=CalA_point[i];
			if(i<4)
			{//adj a1   同时以四点之外的高点调整t1
				//a1=(y-y0)/(exp(x/t1));
				SaveData.Value.cal_A_a1=((CalA_point[i]-CalA_data[i])-SaveData.Value.cal_A_y0)/(exp(CalA_data[i]/SaveData.Value.cal_A_t1));
				if(CalA_data[i+4]==0)continue;
				SaveData.Value.cal_A_t1=CalA_data[i+4]/(log(((CalA_point[i+4]-CalA_data[i+4])-SaveData.Value.cal_A_y0)/(SaveData.Value.cal_A_a1)));
			}
			else
			{//adj t1		同时以四点之外的低点调整a1
				//t1=x/(log((y-y0)/(a1)));
				SaveData.Value.cal_A_t1=CalA_data[i]/(log(((CalA_point[i]-CalA_data[i])-SaveData.Value.cal_A_y0)/(SaveData.Value.cal_A_a1)));
				if(CalA_data[i-4]==0)continue;
				SaveData.Value.cal_A_a1=((CalA_point[i-4]-CalA_data[i-4])-SaveData.Value.cal_A_y0)/(exp(CalA_data[i-4]/SaveData.Value.cal_A_t1));
			}
		}
		
		
	}
	
	
	
	return Err_percent;
}

	
/*****************************************************************************
* 代入y=a1*exp(x/t1)+y0
* float 返回方程输出值
*	目前大电流调整门限暂定为1600仍需调整。
* 
* Lea
********************************************************************************/
float MyEXP(float inputA)
{
	float new_value=0;
	if(inputA>1500.0f)
	{
		//SaveData.Value.cal_A_a1=0.00137;
		//SaveData.Value.cal_A_t1=206.147;
		//SaveData.Value.cal_A_y0=1.5688;
		
		//new_value=0.00064*exp((sdadc_value/1484.56402))+64.17008;
		new_value=SaveData.Value.cal_A_a1*exp((inputA/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
		return new_value;
	}
	else if((inputA<-1500.0f))
	{
		inputA*=-1;
		new_value=SaveData.Value.cal_A_a1*exp((inputA/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
		return new_value*-1;
	}
	else 
	{
		return 0;
	}			
	//return SaveData.Value.cal_A_a1*exp((inputA/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
}	
	


	
	
	
	
	

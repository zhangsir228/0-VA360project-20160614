/****************************************************************************************
	*	@file		CalA.c
	*	@author	Lea
	*	@date		2016/6/13
	*	@brief	������ʵ�ֹ��ܣ�
	*					1��ͨ���ض���У׼��У׼Ŀ��ǯͷ�������������ѹ��
	*					2��У����ʽΪ y=a1*exp(x/t1)+y0    xΪ��������У������ֵ  yΪ�����У�����ֵ��
	*					3��ʹÿһ������ģ�xi��yi���������Ϲ�ʽ ��ʹ��������һ���ض���ֵ���ڣ���
	*					4��
	*
	*		������
	*
	*
	*					
***************************************************************************************/
#include "CalA.h"	



//�����У׼6����
float CalA_point[8]={1600,1800,2000,2200,2400,2600,2800,3000};
//��Ӧ����У׼���µĵ�������ֵ
//float CalA_data[8] ={
//	1604.61,	//1600
//	1796.3,	//1800
//	1980.4,	//2000
//	2158.21,	//2200
//	2317.46,	//2400
//	2441.77,	//2600
//	2600, //2800 ������������δ������
//	2750, //3000
//};
//���յ�ǰ������������ĸ���У׼���
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
* ѭ������ϵ��a1  t1  y0 ʹ��ÿ��У����Ĳ������ݾ���У�����߹�ʽ��ƽ�У�����ֵ
* CalA_point  ��  CalA_data
* float ����������ռ���ӦУ׼��İٷֱ�ֵ
*	ǰ�ĸ�У׼�����a1 ���ĸ�У׼�����a0    y0��Ϊ��ʼ������ֵ��ʱ���迼��
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
		if(CalA_data[i]==0)continue;//����ǰУ׼�㴦�Ĳ�������Ϊ��������
		
		CalA_temp[i] = CalA_data[i]+MyEXP(CalA_data[i]);
		
		if(((CalA_point[i]-CalA_temp[i])>1)||((CalA_point[i]-CalA_temp[i])<-1))
		{//����ֵУ��������1A
			//SaveData.Value.cal_A_a1*exp((CalA_data[i]/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0=CalA_point[i];
			if(i<4)
			{//adj a1   ͬʱ���ĵ�֮��ĸߵ����t1
				//a1=(y-y0)/(exp(x/t1));
				SaveData.Value.cal_A_a1=((CalA_point[i]-CalA_data[i])-SaveData.Value.cal_A_y0)/(exp(CalA_data[i]/SaveData.Value.cal_A_t1));
				if(CalA_data[i+4]==0)continue;
				SaveData.Value.cal_A_t1=CalA_data[i+4]/(log(((CalA_point[i+4]-CalA_data[i+4])-SaveData.Value.cal_A_y0)/(SaveData.Value.cal_A_a1)));
			}
			else
			{//adj t1		ͬʱ���ĵ�֮��ĵ͵����a1
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
* ����y=a1*exp(x/t1)+y0
* float ���ط������ֵ
*	Ŀǰ��������������ݶ�Ϊ1600���������
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
	


	
	
	
	
	

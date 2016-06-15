#ifndef __CalA_H__
#define __CalA_H__

#include <math.h>
#include "stm32f37x.h"
#include "flash_data.h"

#define CalA_data   SaveData.Value.CalA_data 

//大电流校准6个点
extern float CalA_point[8];
//对应电流校准点下的电流测量值
//extern float CalA_data[8];
//按照当前参数计算出来的各点校准结果
extern float CalA_temp[8];
	
extern defFlashCal SaveData;


float Adj_Nline(float Value);

float Adj_EXP();

float MyEXP(float inputA);












#endif 


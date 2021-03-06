#ifndef __comm_h
#define __comm_h

#include "main.h"
#include "sdadc.h"
#include <math.h>
#include "comp.h"
#include "matrixkey.h"
#include "stmdataprocess.h"
#include "arm_math.h" 
#include "userVAinteraction.h"
#include "timer.h"


/****************************************
//用于分析参数数组字符

******************************************/
void CMD_analyze_R(char * string);

/*******************************************************************************
* Function Name  : findCMD
* Description    : 查找对应主指令位置
* Input          : find CMD.
*                : 
* Output         : s8
* Return         : none
*******************************************************************************/
int8_t findCMD(char *szCommand);

/*******************************************************************************
* Function Name  : findCMD2
* Description    : 查找对应次指令位置
* Input          : find CMD2.
*                : 
* Output         : s8
* Return         : none
*******************************************************************************/
int8_t findCMD2(char *szCommand);

/*******************************************************************************
* Function Name  : findCMD2
* Description    : 查找对应次指令位置
* Input          : find CMD2.
*                : 
* Output         : s8
* Return         : none
*******************************************************************************/
int8_t findCMD3(char *szCommand);

/*******************************************************************************
* Function Name  : Communicate
* Description    : 串口命令解析与应答
* Input          : RxBuffer[]
*                : 
* Output         : 
* Return         : none
*******************************************************************************/
void Communicate(void);

//通信处理汇总函数
void Communication_Service(void);

//主要用于DTA的校准指令
void DTA_CAL(void);
#endif

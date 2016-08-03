#ifndef __USERVAINTERACTION_H
#define __USERVAINTERACTION_H
#include "stm32f37x.h"
typedef enum
{
	state0 = 0,
	state1,
	state2,
	state3,
	state4,
	state5,
	state6,
	state7,
	state8,
}SoftKeystatus;



typedef struct
{
	uint8_t Calonce;//校准对应的档位
	uint8_t DTArange;
	uint8_t rangestatus;
	
	
}defSysflag;
extern defSysflag Sysflag;
	
extern SoftKeystatus funcstatus,btstatus;
extern SoftKeystatus max_minstatus,peakstatus;
extern SoftKeystatus relstatus,phasestatus;
extern SoftKeystatus rangestatus,paramstatus;
extern SoftKeystatus holdstatus,lightstatus;
extern SoftKeystatus longparamstatus,autostatus;
extern SoftKeystatus phaseABCstatus;

extern u8 rangenum;

void DataProcessing(void);

void MeasureFunctionSelection(void);
void manipulate(void);
void display(void);
#endif

/**************************************************end file**************************************************/

#include<string.h>

#include "flash_data.h"


defFlashCal SaveData;

uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t NbrOfPage = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

//读取保存数据，初始化校准变量
void Init_flash(void)
{
	uint8_t 	i=0;
	uint32_t 	*pt;	
	pt=(uint32_t *)NewFlash_addr;	
	if(*pt==NewFlash_value)//不是新机器则读取校准数据
	{
		for(i=0;i<FLASH_CAL_BUFFER_SIZE;i++)
		{
			SaveData.Table[i]=*(uint32_t *)(FLASH_USER_START_ADDR+i*4);
		}
		
		SaveData.Value.VAfact[29]=0;
		SaveData.Value.VAmodel[29]=0;
		SaveData.Value.VAsn[29]=0;
		SaveData.Value.VAver[29]=0;
	}
	else
	{	
			Default_flash();
	}
}
	
void Default_flash(void)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{while (1){}}
	}
	FLASH_Lock();
	

	SaveData.Value.cal_6VA_zero=0;
	SaveData.Value.cal_6VA_gain=1;
	SaveData.Value.cal_60VA_zero=0;
	SaveData.Value.cal_60VA_gain=1;
	SaveData.Value.cal_600VA_zero=0;
	SaveData.Value.cal_600VA_gain=1;
	
	SaveData.Value.cal_6VD_zero=0;
	SaveData.Value.cal_6VD_gain=1;
	SaveData.Value.cal_60VD_zero=0;
	SaveData.Value.cal_60VD_gain=1;
	SaveData.Value.cal_600VD_zero=0;
	SaveData.Value.cal_600VD_gain=1;
	

	SaveData.Value.cal_A_a1=0.00137;
	SaveData.Value.cal_A_t1=206.147;
	SaveData.Value.cal_A_y0=1.5688;
	
	SaveData.Value.cal_A1_zero=0;
	SaveData.Value.cal_A1_gain=1;
	SaveData.Value.cal_A2_zero=0;
	SaveData.Value.cal_A2_gain=1;
	
	SaveData.Value.cal_A_err3=0;
	SaveData.Value.cal_A_err4=0;
	SaveData.Value.cal_adjv=1;//是否启用调整

			
	strcpy(SaveData.Value.VAsn,"123456789");
	strcpy(SaveData.Value.VAmodel,"VA362");
	strcpy(SaveData.Value.VAfact,"V&A");
	strcpy(SaveData.Value.VAver,"V1.2");	


//	float CalA_data[8] ={
//		1604.61,	//1600
//		1796.3,	//1800
//		1980.4,	//2000
//		2158.21,	//2200
//		2317.46,	//2400
//		2441.77,	//2600
//		2600, //2800 以下两组数据未经测量
//		2750, //3000
//	};
	SaveData.Value.CalA_data[0]=1604.11;
	SaveData.Value.CalA_data[1]=1796.22;
	SaveData.Value.CalA_data[2]=1980.33;
	SaveData.Value.CalA_data[3]=2158.44;
	SaveData.Value.CalA_data[4]=2317.55;
	SaveData.Value.CalA_data[5]=2441.66;
	SaveData.Value.CalA_data[6]=2600.77;
	SaveData.Value.CalA_data[7]=2750.88;
		
	updata_flash();	
}	
//更新数据至flash
void updata_flash(void)
{
	uint16_t Wloop=0;
  FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{while (1){}}
	}	
			
	FLASH_Unlock();			
	for(Wloop=0;Wloop<FLASH_CAL_BUFFER_SIZE;Wloop++)
  {	
		//FLASH_ProgramHalfWord((FLASH_USER_START_ADDR+Wloop*2), SaveData.Table[Wloop]);
		//if (FLASH_ProgramHalfWord((FLASH_USER_START_ADDR+Wloop*2), SaveData.Table[Wloop]) != FLASH_COMPLETE)
			if (FLASH_ProgramWord((FLASH_USER_START_ADDR+Wloop*4), SaveData.Table[Wloop]) != FLASH_COMPLETE)
			{ while (1){}}  		
  } 
	
	FLASH_Lock();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	if (FLASH_ProgramWord(NewFlash_addr, NewFlash_value) != FLASH_COMPLETE)//写入标志，确认已修改新机器默认参数
		{ while (1){}}  				
	FLASH_Lock();
	
	FLASH_Lock(); 
}










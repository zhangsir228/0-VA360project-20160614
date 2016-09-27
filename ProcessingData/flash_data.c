#include<string.h>

#include "flash_data.h"
#include "ht1621.h"
#include "dta0660.h"

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
	uint8_t i=0;
	uint8_t show_buf[4]={EE,EE,EE,EE};
	
	WriteEeprom();
	
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
	
	SaveData.Value.cal_6VD_zero=-415.644073;
	SaveData.Value.cal_6VD_gain=0.000283658475;
	SaveData.Value.cal_60VD_zero=-417.363678;
	SaveData.Value.cal_60VD_gain=0.00787606463;
	SaveData.Value.cal_600VD_zero=-417.82489;
	SaveData.Value.cal_600VD_gain=0.0680495873;
	
	
	SaveData.Value.cal_600A_zero=319.617615; //600A档校准线
	SaveData.Value.cal_600A_gain=0.0518837385;
	SaveData.Value.cal_A_y0=1.5688;
	
	SaveData.Value.cal_A1_zero=164.322266;	 //2000A档校准线
	SaveData.Value.cal_A1_gain=0.103888966;
	SaveData.Value.cal_A2_zero=0;
	SaveData.Value.cal_A2_gain=1;
	
	SaveData.Value.cal_A_err3=0;
	SaveData.Value.cal_A_err4=0;
	SaveData.Value.cal_adjv=1;//是否启用调整

			
	strcpy(SaveData.Value.VAsn,"123456789");
	strcpy(SaveData.Value.VAmodel,"VA362");
	strcpy(SaveData.Value.VAfact,"V&A");
	strcpy(SaveData.Value.VAver,"V1.3");	


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
	SaveData.Value.CalA_data[0]=1593.57;
	SaveData.Value.CalA_data[1]=1782.27;
	SaveData.Value.CalA_data[2]=1961.04;
	SaveData.Value.CalA_data[3]=2085.61;
	SaveData.Value.CalA_data[4]=2238.41;
	SaveData.Value.CalA_data[5]=2330.60;
	SaveData.Value.CalA_data[6]=2409.98;
	SaveData.Value.CalA_data[7]=2469.98;
		
	
	//初始化DTA0660相关校准档位参数
	for(i=0;i<5;i++) 
	{
		SaveData.Value.Cal_VmV_zero[i] = 0;
		SaveData.Value.Cal_VmV_gain[i] = 1;
	}
	for(i=0;i<6;i++)
	{
		SaveData.Value.Cal_ohm_zero[i] = 0;
		SaveData.Value.Cal_ohm_gain[i] = 1;
	}
	for(i=0;i<4;i++)
	{
		SaveData.Value.Cal_Cap_zero[i] = 0;
		SaveData.Value.Cal_Cap_gain[i] = 1;
	}

	updata_flash();	
	
	lcd_clr();	
	lcd_show_data(show_buf,1);//显示EEEE,表示内部EEPROM被清空	
	delay_ms(2000);
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










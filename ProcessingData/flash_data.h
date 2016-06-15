#ifndef __flash_data_H__
#define __flash_data_H__

#include "stm32f37x.h"
#include "nvic_systick.h"

#include "main.h"
#include "math.h"

/*�궨��*/

#define false 0
#define true 	1

#define NewFlash_addr						((uint32_t)0x0803fff0) 		/*if not 0xaa55aa55 should be reset*/
#define NewFlash_value					((uint32_t)0xaa55aa55)
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x0803f800)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08040000)   /* End @ of user Flash area */
#define DATA_32                 ((uint32_t)0x12345678)

#define	FLASH_CAL_BUFFER_SIZE  	60		//write by word (halfword before)

typedef struct {
	//SDADC1У׼���� ���+����
	//AC cal param
	float cal_6VA_zero;
	float cal_6VA_gain;
	float cal_60VA_zero;
	float cal_60VA_gain;
	float cal_600VA_zero;
	float cal_600VA_gain;

	//DC cal param
	float cal_6VD_zero;
	float cal_6VD_gain;
	float cal_60VD_zero;
	float cal_60VD_gain;
	float cal_600VD_zero;
	float cal_600VD_gain;	//12

	//current cal
	float cal_A_a1;//�����������ʽ
	float cal_A_t1;//y=a1*exp(x/t1)+y0;
	float cal_A_y0;				//3

	float cal_A1_zero;//��ĳ������ֵʱУ׼������ƫ�á�
	float cal_A1_gain;//��ĳ���ϴ����ʱУ׼���Զε����档
	float cal_A2_zero;//��ĳ������ֵʱУ׼������ƫ�á�
	float cal_A2_gain;//��ĳ���ϴ����ʱУ׼���Զε����档
	float cal_A_err3;//			2400
	float cal_A_err4;//			2600
	uint32_t cal_adjv;//�Ƿ����õ���	//7
	
	char VAsn[30];
	char VAmodel[30];
	char VAfact[30];//������
	char VAver[30];//����汾 //30  ��52
	
	float CalA_data[8]; //8   ��60
	
	}defCalValue;

typedef union{
 	  defCalValue	Value;	
		uint32_t 		Table[FLASH_CAL_BUFFER_SIZE];	
	  //uint16_t 		Table[FLASH_CAL_BUFFER_SIZE];
	  }defFlashCal;

void Init_flash(void);//��ȡ��������
void Default_flash(void);//��ʼ��У׼����
void updata_flash(void);//����������flash


#endif

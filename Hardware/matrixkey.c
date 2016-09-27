/********************************************************************************
	*	@file		matrixkey.c
	*	@author	Jeff
	*	@date		2014/11/1
	*	@brief	����������á�
	*					���������3��4�У�����Ϊ��3�С������������4�С����������롣
	*					ʵ�ַ���������stm32���ԣ�û�а�������ʱ��������ߵ�ƽ����Ϊ�������룬
	*										���а������£����߼�⵽�������жϴ����źţ����ⲿ�жϴ�������
	*										�ر��ⲿ�жϣ���Ϊ��ʱ��ɨ��������ʱ����������ɨ�衪���ֱ�
	*										��ÿ��Ϊ�ߵ�ƽ��⣬�������ڼ�����ⲿ�жϣ����Թص�������ʱ��ɨ��
	*										��ɺ��ٴ򿪣������򿪶�ʱ���жϡ���ʱ��ÿ10msɨ��һ�Σ�ɨ��õ�
	*										�����Ķ̰��ͷš������źţ��Թ�������
	*					
	*					ʹ�����裺TIM3(0,0),EXTI9_5_IRQn(2,0)
	*
	*					����					PF6/7/PB5(��������)
	*					����					PB6/7/8/9(��������)
	*					
	*										����|����|����|����
	*										����|����|����|����
	*										����|����|����|����
	*										����|����|����|����
	*					
	*******************************************************************************/
#include "stm32f37x.h"
#include "matrixkey.h"


/* ���尴������״̬ */
#define        KEY_STATE_INIT           0
#define        KEY_STATE_WOBBLE         1
#define        KEY_STATE_PRESS          2
#define        KEY_STATE_LONG           3
//#define        KEY_STATE_CONTINUE      	4
#define        KEY_STATE_RELEASE        5
    
//#define        KEY_CONTINUE_PERIOD      50        /* 500ms */

boolean RotaryKeyChanged_flag = 0;
boolean SoftKeyChanged_flag = 0;
boolean ShortKey_flag,LongKey_flag;

u8 KeyValue;
u8 RotaryKeyValue,RotaryKeyValue_before=KEY_NULL;
u8 SoftKeyValue,SoftKeyValue_before=KEY_NULL;
u8 oncekey = KEY_NULL;


/*1��2��3�������*/
static void KEY_LINE_write_low_all(void)
{
//	GPIO_Write(GPIOF, GPIOF->ODR &= (~0x00C0));
//	GPIO_Write(GPIOB, GPIOB->ODR &= (~0x0020));
	GPIO_ResetBits(Keyboard_Line_Port_1,Keyboard_Line_1);
	GPIO_ResetBits(Keyboard_Line_Port_2,Keyboard_Line_2);
	GPIO_ResetBits(Keyboard_Line_Port_3,Keyboard_Line_3);
}
/*1��2��3�������*/
void KEY_LINE_write_high_all(void)
{
//	GPIO_Write(GPIOF, GPIOF->ODR |= 0x00C0);
//	GPIO_Write(GPIOB, GPIOB->ODR |= 0x0020);
	GPIO_SetBits(Keyboard_Line_Port_1,Keyboard_Line_1);
	GPIO_SetBits(Keyboard_Line_Port_2,Keyboard_Line_2);
	GPIO_SetBits(Keyboard_Line_Port_3,Keyboard_Line_3);
}
/*ָ��(0/1/2)�������*/
static void KEY_LINE_write_high(uint8_t i)
{
	KEY_LINE_write_low_all();
	if(i == 0)
		GPIO_SetBits(Keyboard_Line_Port_1 ,Keyboard_Line_1);
	else if(i == 1)
		GPIO_SetBits(Keyboard_Line_Port_2 ,Keyboard_Line_2);
	else if(i == 2)
		GPIO_SetBits(Keyboard_Line_Port_3 ,Keyboard_Line_3);
}
/*��ȡ�����е�ֵ(PB6-PB9)�������Ϊ0x0F(����λ)��ʽ
*ע�⣺###����ж�Ӧ��IO�ڸı䣬�˴���ı�###
*/
static u8 KEY_ROW_read(void)
{
	return ((GPIO_ReadInputData(Keyboard_Row_Port)>>6)&0x0F);
}

/**************************************************************** 
*	��������: TIM3_Config 
*	��    ��: ��ʱ��3����10ms�����ж�           
*	��    ��: �� 
*	�� �� ֵ: ��  
*****************************************************************/
static void TIM3_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  /* Time base configuration */
					/*Ƶ�ʼ��㣺f = 72,000,000/(99+1)/(7199+1) = 100Hz������10ms */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/(99+1)/100-1;//7199
  TIM_TimeBaseStructure.TIM_Prescaler = 99;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	/* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, ENABLE);
}

/**************************************************************** 
*	��������: Init_Keyboard_Interrupt 
*	��    ��: ���̳�ʼ��Ϊ�ж�ɨ��ģʽ            
*						��ʼ��������Ҫ��IO��Line1-Line3��Ϊ�����           
*						Row1-Row4 ���������裬ʹ���������ж� 
*	��    ��: �� 
*	����ֵ  : ��  
*****************************************************************/
void Init_Keyboard_Interrupt(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* Enable GPIOB/F clock */
  RCC_AHBPeriphClockCmd(Keyboard_Line_1_CLK | Keyboard_Line_2_CLK | Keyboard_Line_3_CLK | Keyboard_Row_CLK, ENABLE);
	/* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	TIM3_Config();
	EXTI_DeInit();
	
	//Line1,Line2:��PF6,PF7       
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Keyboard_Line_Port_1 ,&GPIO_InitStructure);
	GPIO_SetBits(Keyboard_Line_Port_1 ,Keyboard_Line_1);
	
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_2;
	GPIO_Init(Keyboard_Line_Port_2 ,&GPIO_InitStructure);	
	GPIO_SetBits(Keyboard_Line_Port_2 ,Keyboard_Line_2);
	
	//Line3:��PB5
	GPIO_InitStructure.GPIO_Pin = Keyboard_Line_3;
	GPIO_Init(Keyboard_Line_Port_3 ,&GPIO_InitStructure);	
	GPIO_SetBits(Keyboard_Line_Port_3 ,Keyboard_Line_3);
	
	//Row1-Row4:��PB6-PB9������Ϊ�������룬���������������ж�
	GPIO_InitStructure.GPIO_Pin = Keyboard_Row;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//��������
	GPIO_Init(Keyboard_Row_Port ,&GPIO_InitStructure);
	
	/* Enable and set EXTI9_5 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI6 Line to PB6 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
	//PB6:�����ش���
	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	//PB7
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
	EXTI_Init(&EXTI_InitStructure);
	//PB8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);
	//PB9
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
	EXTI_Init(&EXTI_InitStructure);
	
/*********************************************************************************/
	//20160728Lea �����ⲿ�ж�PB2 ���������⵵λ���ߺ��ܹ�ʵ�ֻ������ѡ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//��������
	GPIO_Init(GPIOB ,&GPIO_InitStructure);
	
		
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_TS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI2 Line to PB2 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	//PB2:�����ش���
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
}



/**************************************************************************************************************
* ��������: OnceSoftKey
* ����˵��: ����һ�μ��̰���ֵ��ɨ���ȡ
* ��    ��: ��
* ��    ��: u8	ɨ�赽�İ���ֵ
* ע������: 1������0x0F(KEY_NULL)��ʾû�а���������
*						2��������ɨ�谴��û�н��������������ScanKey()�����ڱ������Ļ����Ͻ����˻���״̬�����ƵĻ�е��������
****************************************************************************************************************/
u8 OnceSoftKey(void)
{
	u8 i = 0;
	u8 keytemp=KEY_NULL;
	
	KEY_LINE_write_high_all();
	if(KEY_ROW_read() == 0)
	{
//		KEY_LINE_write_high_all();
		return KEY_NULL;         		//û�а������£�ֱ�ӷ���
	}
	
	for(i = 0;i < 3;i++)//���������������ң��У�0��1��2.
	{
		KEY_LINE_write_low_all();
		KEY_LINE_write_high(i);
		oncekey = (KEY_ROW_read() & 0x03);//ֻȡ���ݵĺ���λ
		switch(oncekey)
		{
			case (1 & 0x0F)://PB6����
				keytemp = i;
				break;
			case (2 & 0x0F)://PB7����
				keytemp = 3 + i;
				break;
			default:
				keytemp = 0x0F;
				break;
		}
	
		if(keytemp < 0x06)
		{
			return keytemp;
		}
	}
	
	return KEY_NULL;
}

u8 OnceRotaryKey(void)
{
	u8 i = 0;
	u8 keytemp=KEY_NULL;
	
	KEY_LINE_write_high_all();
	if(KEY_ROW_read() == 0)
	{
//		KEY_LINE_write_high_all();
		return KEY_NULL;         		//û�а������£�ֱ�ӷ���
	}
	
	for(i = 0;i < 3;i++)//���������������ң��У�0��1��2.
	{
		KEY_LINE_write_low_all();
		KEY_LINE_write_high(i);
		oncekey = (KEY_ROW_read() & 0x0C);//ֻȡ���ݵĵ���4-3��λ
		switch(oncekey)
		{
			case (4 & 0x0F)://PB8����
				keytemp = 6 + i;
				break;
			case (8 & 0x0F)://PB9����
			{
				//if(i<2)
					keytemp = 9 + i;
			}
				break;
			default:
				keytemp = 0x0F;break;
		}
	
		if((keytemp <= 0x0B) && (keytemp >= 0x06))
		{
			return keytemp;
		}
	}
	
	return KEY_NULL;
}
/***************************************************************************** 
* ��������: ScanKey
* ����˵��: ɨ���ȡ����ֵ����״̬
* ��    ��: ��
* ��    ��: u8	����ֵ����״̬
*						����λ������״̬
*						����λ������ֵ
* ע������: 0�����������ֵ��KEY_NULL(0x0F)��1001####���̰��ͷ���Ӧ����0100####��������ʱ��Ӧ����0101####�������ͷš�û���ã�
*															0x0#(���õķ���ֵ)//��355��ע�ͣ�1��28�գ���ʹ��ע�ͺ��û�����õķ���ֵ�ˡ�
*															Ҳ���Բ��øģ���Ϊ�ڵ���ScanKey�󣬻��жϷ���ֵ��״̬�ͼ�ֵ��
*						1�����룺�̰�����1001####����Ҫ�ͷŰ�ťʶ��
*						2�����룺��������0100####������ִ��
******************************************************************************/
u8 ScanKey(void)
{
	static u8 KeyState = KEY_STATE_INIT;
	static u8 KeyTimeCounter = 0;
	static u8 LastKey = KEY_NULL;
	u8 KeyTemp = KEY_NULL;//ÿ�ν���˺���������KeyTemp = 0x0F
	
	KeyTemp = OnceSoftKey();//��ⰴ����KeyTemp��ֵΪ��0000####��0x00-0x0B,0x0F��
//	KEY_LINE_write_high_all();
	switch(KeyState)
	{
		case KEY_STATE_INIT://0
		{
			if(KEY_NULL != (KeyTemp))//�а�������
			{
				KeyState = KEY_STATE_WOBBLE;//���밴������״̬
			}
		}
		break;
		
		case KEY_STATE_WOBBLE://1
		{
			KeyState = KEY_STATE_PRESS;//���밴������״̬
		}
		break;
		
		case KEY_STATE_PRESS://2
		{
			if(KEY_NULL != (KeyTemp))//�а�������
			{
				KeyTemp |= KEY_DOWN;//���水�����±�־(0x80)��KeyTemp��8λ��Ϊ�� 1000####
				LastKey = KeyTemp;//�����ֵ�Ͷ̰���־���Ա����ͷ�״̬���ؼ�ֵ
				KeyState = KEY_STATE_LONG;//���밴������״̬
			}
			else
			{
				KeyState = KEY_STATE_INIT;//�������δ���£���Ϊ�Ƕ��������س�ʼ״̬
			}
		}
		break;
		
		case KEY_STATE_LONG://3 
		{
			
			if(KEY_NULL != (KeyTemp))//�а�������
			{
				if(++KeyTimeCounter > KEY_LONG_PERIOD)//����ʱ��������ڳ�����ֵ
				{
					KeyTimeCounter = 0;
					KeyTemp |= KEY_LONG;//���水��������־(0x40)��KeyTemp��8λ��Ϊ��0100####
					LastKey = KeyTemp;//�����ֵ�ͳ�����־���Ա����ͷ�״̬���ؼ�ֵ
//					KeyState = KEY_STATE_CONTINUE;//���밴������״̬����˼��һֱ�����ţ��ȳ���ʱ�䳤��
					//KeyState = KEY_STATE_RELEASE;//20160830 Lea ���������ﵽ��͵ȴ���һ�������ͷŴ�����  Ȼ���ж��оͻ�ִ�г�ʱ����
				}
			}
			else
			{
				KeyTimeCounter = 0;
				KeyState = KEY_STATE_RELEASE;//�ڳ���ʱ�䵽��ǰ������⵽�����ɿ������밴���ͷ�״̬��0x10)�����ж�Ϊ�̰�
			}																//�����ڳ���֮���⵽�����ɿ�
		}
		break;
		
		/*2014-10-25-19:01,�ڽ�����ñ�ĵĹ���ʱע�͵�һ��ʼ��д�ĺ�������״̬�ĳ���*/
//		case KEY_STATE_CONTINUE://4
//		{
//			if(KEY_NULL != (KeyTemp))//�а�������
//			{
//				if(++KeyTimeCounter > KEY_CONTINUE_PERIOD)
//				{
//					KeyTimeCounter = 0;
//					KeyTemp |= KEY_CONTINUE;//���水��������־(0x20)��KeyTemp��8λ��Ϊ��0010####
//					LastKey = KeyTemp;//�����ֵ��������־���Ա����ͷ�״̬���ؼ�ֵ
//				}
//			}
//			else
//			{
//				KeyTimeCounter = 0;
//				KeyState = KEY_STATE_RELEASE;//����ʱ�䵽��ǰ�������ж�Ϊ�������������Ѿ��й�����״̬�������ж�Ϊ������������⵽�����ɿ������밴���ͷ�״̬
//			}
//		}
//		break;
		
		case KEY_STATE_RELEASE://5
		{
			LastKey |= KEY_UP;//�����ͷű�־(0x10)���˱�־ֻ�����Ѿ����ڶ̰���������������־������³���
			KeyTemp = LastKey;//KeyTemp��8λ��Ϊ���̰��ͷš���1001####�������ͷš���0101####�������ͷš���0011####
			KeyState = KEY_STATE_INIT;
			return KeyTemp;//�̰���Ҫ���ֲ���ʶ��//
		}
		break;
		
		default:break;
	}
	
	if(KeyTemp < 0x80 && ((KeyTemp&0xF0) != 0))//1��28�գ����������Ǻţ������޸�Ϊif((KeyTemp < 80) && ((KeyTemp&0xF0) != 0))
		return KeyTemp; //������������������ִ��//��������0100####����������0010####
	else
		return KEY_NULL;
}

/**************************************************end file**************************************************/

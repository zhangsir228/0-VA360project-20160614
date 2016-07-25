#ifndef __MATRIXKEY_H
#define __MATRIXKEY_H	
#include "stm32f37x.h"


/* 定义按键值 */
#define        KEY_NULL        					0x0F
#define        KEY_VALUE_0              0x00
#define        KEY_VALUE_1              0x01
#define        KEY_VALUE_2              0x02
#define        KEY_VALUE_3              0x03
#define        KEY_VALUE_4              0x04
#define        KEY_VALUE_5              0x05
#define        KEY_VALUE_6              0x06
#define        KEY_VALUE_7              0x07
#define        KEY_VALUE_8              0x08
#define        KEY_VALUE_9              0x09
#define        KEY_VALUE_10             0x0A
#define        KEY_VALUE_11             0x0B
/* 定义按键状态值 */
#define        KEY_DOWN                	0x80
#define        KEY_LONG                 0x40
#define        KEY_CONTINUE             0x20
#define        KEY_UP                   0x10

//选择扫描模式  
#define Interrupt_Scan   //中断扫描模式 ,要在NVIC在中打开对应中断 
/*可以自己定义其它扫描方式*/ 

//#define DELAY_COUNT    0x0FFFF 

/* 键盘控制引脚定义 */  
#define Keyboard_Line_1_CLK		RCC_AHBPeriph_GPIOF
#define Keyboard_Line_2_CLK		RCC_AHBPeriph_GPIOF
#define Keyboard_Line_3_CLK		RCC_AHBPeriph_GPIOB
#define Keyboard_Row_CLK   		RCC_AHBPeriph_GPIOB

#define Keyboard_Line_Port_1   GPIOF//(PF6)
#define Keyboard_Line_Port_2   GPIOF//(PF7)
#define Keyboard_Line_Port_3   GPIOB//(PB5)
#define Keyboard_Row_Port   GPIOB

#define Keyboard_Line_1    GPIO_Pin_6
#define Keyboard_Line_2    GPIO_Pin_7
#define Keyboard_Line_3    GPIO_Pin_5

#define Keyboard_Row_1    GPIO_Pin_6//(PB6)软
#define Keyboard_Row_2    GPIO_Pin_7//(PB7)软
#define Keyboard_Row_3    GPIO_Pin_8//(PB8)旋转
#define Keyboard_Row_4    GPIO_Pin_9//(PB9)旋转

#define Keyboard_LineBase Keyboard_Line_1
#define Keyboard_RowBase  Keyboard_Row_1
////IO若变了，此处相应改变
//#define Keyboard_Line_F  (Keyboard_Line_1 | Keyboard_Line_2)
//#define Keyboard_Line_B  Keyboard_Line_3
//#define Keyboard_Line  Keyboard_Line_F | Keyboard_Line_B
#define Keyboard_Row   (Keyboard_Row_1 | Keyboard_Row_2 | Keyboard_Row_3 | Keyboard_Row_4)

#ifdef Interrupt_Scan   /* 中断扫描模式宏定义 */
#define Keyboard_EXTI_Row1   EXTI_Line6
#define Keyboard_EXTI_Row2   EXTI_Line7
#define Keyboard_EXTI_Row3   EXTI_Line8
#define Keyboard_EXTI_Row4   EXTI_Line9

#define Keyboard_EXTI_Line    (Keyboard_EXTI_Row1 | Keyboard_EXTI_Row2 | Keyboard_EXTI_Row3 | Keyboard_EXTI_Row4)
#endif  /* 中断扫描模式宏定义 */

/* 键盘全局变量声明 */
extern boolean RotaryKeyChanged_flag;
extern boolean SoftKeyChanged_flag;
extern boolean ShortKey_flag,LongKey_flag;

extern u8 KeyValue;
extern u8 RotaryKeyValue,RotaryKeyValue_before;
extern u8 SoftKeyValue,SoftKeyValue_before;
/* 键盘接口函数声明 */ 
#ifdef Interrupt_Scan
void Init_Keyboard_Interrupt(void); //键盘初始化为——外部中断进入后，关闭外部中断并定时器扫描——相配合的模式
#endif
u8 OnceSoftKey(void);

u8 OnceRotaryKey(void);
u8 ScanKey(void);
void KEY_LINE_write_high_all(void);

#endif /* KEYBOARD_H */

/**************************************************end file**************************************************/

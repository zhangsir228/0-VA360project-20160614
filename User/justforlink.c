int main(void)
{
	start_configration();
	
	while(1)
	{
		DataProcessing();
		
		/*当有旋转开关改变时*/
		if(RotaryKeyChanged_flag == 1)
		{
			MeasureModeChange();//测量模式选择：DTA0660<->STM32
			
			DataProcessing();
			
			SoftKeyState_AllInit();//软按钮状态初始化
			Display_RotaryKeyChange();//旋转开关时状态字显示初始化
			
			RotaryKeyChanged_flag = 0;
		}
		/*当有软按钮改变时*/
		if(SoftKeyChanged_flag == 1)
		{
			SoftKeyStateChange();//软按键发生时，改变相应状态(及软按键6时测量功能的改变)
			
			DataProcessing();
			
			Display_SoftKeyChange();//软按键发生时，针对相应已改变的状态进行显示状态字
			SoftKeyChanged_flag = 0;
		}
		
		Display();//还没写，2014-12-15日
	}
}
/**************************************************end file**************************************************/

int main(void)
{
	start_configration();
	
	while(1)
	{
		DataProcessing();
		
		/*������ת���ظı�ʱ*/
		if(RotaryKeyChanged_flag == 1)
		{
			MeasureModeChange();//����ģʽѡ��DTA0660<->STM32
			
			DataProcessing();
			
			SoftKeyState_AllInit();//��ť״̬��ʼ��
			Display_RotaryKeyChange();//��ת����ʱ״̬����ʾ��ʼ��
			
			RotaryKeyChanged_flag = 0;
		}
		/*������ť�ı�ʱ*/
		if(SoftKeyChanged_flag == 1)
		{
			SoftKeyStateChange();//��������ʱ���ı���Ӧ״̬(������6ʱ�������ܵĸı�)
			
			DataProcessing();
			
			Display_SoftKeyChange();//��������ʱ�������Ӧ�Ѹı��״̬������ʾ״̬��
			SoftKeyChanged_flag = 0;
		}
		
		Display();//��ûд��2014-12-15��
	}
}
/**************************************************end file**************************************************/

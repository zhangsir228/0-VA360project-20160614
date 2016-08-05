#include <string.h>

#include "main.h"
#include "comm.h"
#include "flash_data.h"
#include "dac.h"
#include "CalA.h"	
#include "dta0660.h"
#include "buzzer_led.h"


#define name_to_str(data)  (#data)

extern defSysValue SysValue ;//ϵͳ����ʱ����Ҫ��������
extern defFlashCal SaveData;//������flash�еı�Ҫ����

extern uint8_t 	Rx2Buffer[];
extern uint8_t	Rx2Counter;
extern uint8_t 	Flag_Rx2End;



char tempR[50],tempS[50],tempT[50],tempU[50],tempV[50],tempW[50];


typedef struct
{
  char type[3];
  signed long long numb_i[3];
  float numb_f[3];
  char f_CMD3[3];
} tempR_type;
tempR_type tp;

const char *CMDmap1[50]={
         // ϵͳ���� 0~11
				 "*idn",				"idn",//0
					"rst",				"rst",//2
					"output",			"out",//4
					"input",			"inp",//6
					"read",				"read",//8
					"program",		"prog",//10
					"callibration","cal",//12
					"voltage",		"volt",//14
					"state",			"state",//16	�����ǰ��ϵͳ��־	
					"factory",		"fact",//18
					"measurement","meas",//20
					"config",     "conf",//22
					"cala",								//24  ���������У׼  ex:  cala:1800  cala:2400
					"dta_calv",		//25У׼DTA��ѹ
					"dta_eeprom_read",		//26��ȡDTA���ò���		
					"dta_eeprom_write", //27��ʼ��DTA���ò���
					"",			
					"",
					"",	
	
			};

const char *CMDmap2[100]={
          "mvoltage",		"mvol",//0
          "voltage",			"volt",//2
          "mcurrent",		"mcur",//4
          "ma%",			"ma%",//6
          "tc",				"tc",//8
          "in",				"in",//10	
          "out",			"out",//12 
          "set",			"set",//14
          "end",			"end",//16 ������ȡ����
          "run",			"run",//18
          "state",		"state",//20 program_out���鵱ǰ״̬
			"goon",			"goon",		//22 ������ȡ����
			"cel",      "c",		//24
			"fah",      "f",		//26
			"mv",				"mv",		//28
			"v",				"v",		//30
			"ma",				"ma",		//32
			"tck",			"tck",//34
			"tcj",			"tcj",//36
			"tct",			"tct",//38
			"tce",			"tce",//40
			"tcr",			"tcr",//42
			"tcs",			"tcs",//44
			"tcb",			"tcb",//46
			"tcn",			"tcn",//48
			"initialflash",			"tcx",//50  51û��
			
			"fact",			"model",//52�������� 53�ͺ�  
			"sn",				"ver",//54���к�  55�汾��  
			"calmode",	"normode",//56У׼ģʽ  57����ģʽ
			"setkey",			"tcx",//58��λһ�ΰ���ֵ
			"cal_600a_zero","cal_600a_gain",//60  	//61
			"y0","sdadc1",	//62	//63
			"cal_6va_zero",		"cal_6va_gain",		//64	//65
			"cal_60va_zero",	"cal_60va_gain",	//66	//67
			"cal_600va_zero",	"cal_600va_gain",	//68	//69						
			"cal_6vd_zero",		"cal_6vd_gain",			//70	//71 ֱ��У׼����  6VDC��
			"cal_60vd_zero",	"cal_60vd_gain",		//72	//73
			"cal_600vd_zero",	"cal_600vd_gain",		//74	//75
			"cal_a_err",			//76 �ڶ�Ӧ����A��У��ƫ��ֵ cal_A_errN    
			"cal_a_zero",			//77  �ڶ�Ӧ������У�����ƫ��ֵ	
			"cal_a_gain",			//78	��ĳ���ϴ����ʱУ׼���Զε�����
			"cal1800a",				//79	������DC1800A���ĵ�ѹֵΪ�����У׼��ѹ��ʼ��
			"cal_adjv",				//80
			"daout",								//81 ����DAC�����ѹ
			"up",								//82 ����У׼DTA��ѹ
			"down",								//83 ����У׼DTA��ѹ
			"five",								//84
			"sdadc2",								//85
			"",								//86
			"",								//87
			"",								//88
			"",								//89
			"",								//90
		 };

const char *CMDmap3[]={
				 "k",			"j",//0
				 "t",			"e",//2
				 "r",			"s",//4
				 "b",			"n",//6
				 "l",			"u",//8
				 			 
		 };
                            //tempR     temp

/****************************************
//���ڷ������������ַ�

******************************************/
void CMD_analyze_R(char * string)
{

  char tc1[20];
  uint8_t j=0;
  tp.type[0]=0;
  tp.type[1]=0;
  tp.type[2]=0;
  tp.numb_i[0]=0;
  tp.numb_i[1]=0;
  tp.numb_i[2]=0;
  
  while(*string!='\0')
  {
    if((*string=='+')||(*string=='-')||((*string>=0x30)&&(*string<=0x39)))
    {
        // get number
      int spoint=-1;
      int ssigne=0;
      
      if(*string=='+')
      {
        string++;
      }
      else if(*string=='-')
      {
        ssigne=-1;
        string++;
      }
      while((*string!='\0')&&(*string!=','))
      {
        if((*string>=0x30)&&(*string<=0x39))
        {
          if(spoint>=0) spoint++;
          tp.numb_i[j]=tp.numb_i[j]*10+(*string-0x30);
        }
        else if(*string=='.')
        {
          if(spoint==-1) spoint=0;
          else 
          {
            // ���С����
            tp.type[j]=-3;
            break;
          }
        }
        else
        {
          // �������ַ�
          tp.type[j]=-3;
        break;
        }
        string++;
      }
      if(tp.type[j]==0)
      {
        if(spoint<=0)  // ��С����
        {
          if(ssigne==0)  
          {  // �޸���
            tp.type[j]=1;
          }
          else
          { // ����
            tp.type[j]=-1;
            tp.numb_i[j]*=-1;
          }
          tp.numb_f[j]=tp.numb_i[j];
        }
        else
        {
          if(ssigne==0)  
          {  // �޸���
            tp.type[j]=2;
            tp.numb_f[j]=tp.numb_i[j];
          }
          else
          { // ����
          tp.type[j]=-2;
          tp.numb_i[j]*=-1;
          tp.numb_f[j]=tp.numb_i[j];
          }
          while(spoint--)
          {
            tp.numb_f[j]*=0.1;
          }
        
        }
      }
    
    }
    else
    {
      // get cmd3
      uint8_t i=0;
      while((*string!='\0')&&(*string!=','))
      {
        tc1[i]=*string;
        string++;i++;
      }
      tp.f_CMD3[j]=findCMD3(tc1);
      if(tp.f_CMD3[j]==-2)
      {
        tp.type[j]=-3;
      }
      else
      { tp.type[j]=3;}
    }
    if(*string==',')
    {
      string++;j++;
    }
  }
  //return tp;
}



/*******************************************************************************
* Function Name  : findCMD
* Description    : ���Ҷ�Ӧ��ָ��λ��
* Input          : find CMD.
*                : 
* Output         : int8_t
* Return         : none
*******************************************************************************/
int8_t findCMD(char *szCommand) 
{
		uint8_t i;
    for(i=0;i<50;i++) 
        { 
                if(strcmp(szCommand,CMDmap1[i])==0) 
                      return i; 
        } 
        return   -2; 
}

/*******************************************************************************
* Function Name  : findCMD2
* Description    : ���Ҷ�Ӧ��ָ��λ��
* Input          : find CMD2.
*                : 
* Output         : int8_t
* Return         : none
*******************************************************************************/
int8_t findCMD2(char *szCommand) 
{
		uint8_t i;
    for(i=0;i<100;i++) 
        { 
                if(strcmp(szCommand,CMDmap2[i])==0) 
                      return i; 
        } 
        return   -2; 
}
/*******************************************************************************
* Function Name  : findCMD3
* Description    : ���Ҷ�Ӧ��ָ��λ��
* Input          : find CMD2.
*                : 
* Output         : int8_t
* Return         : none
*******************************************************************************/
int8_t findCMD3(char *szCommand) 
{
		uint8_t i;
    for(i=0;i<10;i++) 
        { 
                if(strcmp(szCommand,CMDmap3[i])==0) 
                      return i; 
        } 
        return   -2; 
}
/*******************************************************************************
* Function Name  : Command_service
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Communicate(void)
{
  int i,j,k;
	char CalH=0,CalL=0;
	
	char    chardata[16];
	uint8_t Buf_temp[100];
  uint8_t Buf_temp_count;
  uint8_t tempQ=0;
  int8_t tempC=-1;
  int8_t tempT1=-1;
  int8_t tempT2=-1;
	uint16_t input_data=0;

  //char tempR[50],tempS[50],tempT[50],tempU[50],tempV[50],tempW[50];
  char Buf_temp2[50];
  char str[100+1]; // output string

   /***************************************************/
    //����ȡ��ֵ��������
    //CMD_analyze_R(testchar);

  //��ȡ ����ͨѶ�˿ڵ���������
  str[0]=0;
	 // ��ȡ��һ���ֵ������� ����Buf_temp,Ȼ�����RxBuffer 
	 j=0;k=0;
	 for(i=0;i<=Rx2Counter;i++)
	 {
	   if((j==0)&&((Rx2Buffer[i]<0x20)||(Rx2Buffer[i]>0x7d)))
	   {k++;}
	   else
	   {
	     if((Rx2Buffer[i]==0x0a)||(Rx2Buffer[i]==0x0d)||(Rx2Buffer[i]==0))
	     {
	       Buf_temp[i-k]=0;
		   i++;
	       break;
	     }
	     else 
	     {
	       Buf_temp[i-k]=Rx2Buffer[i];
	     }
		 j=1;
	   }
	 }
	 Buf_temp_count=i-k;
	 j=0;k=0;
	 while(i<=Rx2Counter)
	 {
	     Rx2Buffer[j]=Rx2Buffer[i];
	     j++;i++;
	 }
	 Rx2Counter=Rx2Counter-Buf_temp_count;
	 i=0;j=0;
    while((((Rx2Buffer[0]<0x20)||(Rx2Buffer[0]>0x7d)))&&(Rx2Counter>0))
	{
	  for(i=0;i<Rx2Counter;i++)
	  {Rx2Buffer[i]=Rx2Buffer[i+1];}
	  Rx2Counter--;
	}

	 // תСд
	 i=0;
	 j=Buf_temp_count;
	 while(j--)
	 {
	  Buf_temp[i]=tolower(Buf_temp[i]);
	  i++;
	 }
 

  // ����ȡ����Buf_temp�������� �����������
	// ָ��� ָ��+ð�Ż��߿ո�[+ָ��+ð�Ż��߿ո�]+����    ��ʽ���н�����
  // �����ؼ��ֳ����ִ��룬��һ����tempC;�ڶ�����tempT1;��������tempT2;����tempR;ѯ�ʱ�־tempQ;
	i=0;j=100;
	while(j--)
	{
	  if((Buf_temp[i]==':')||(Buf_temp[i]=='?')||(Buf_temp[i]==';')||(Buf_temp[i]==0x00))
	    break;
	  Buf_temp2[i]=Buf_temp[i];
	  i++;
	}
	Buf_temp2[i]='\0';
	tempC=findCMD(Buf_temp2);//��ȡ����һ��ָ�
 
	if(Buf_temp[i]==':')
	{
	  j=0;i++;k=100;
	  while(k--)
	  {
	    if((Buf_temp[i]==':')||(Buf_temp[i]=='?')||(Buf_temp[i]==';')||(Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
	    break;
	    Buf_temp2[j]=Buf_temp[i];
	    i++;j++;
	  }
	  Buf_temp2[j]='\0';
		if(Buf_temp2[0]<0x40)//����ǲ�����ֵ
		{
			//�ս� �ո�� ȡ����
       strcpy(tempR,Buf_temp2);
			 //CMD_analyze_R(Buf_temp2);
			 tempT1=-3;
		}
		else//��Ϊָ���ȡ��Ӧ��ָ��
		{
			tempT1=findCMD2(Buf_temp2);
		}
	  
	  if(Buf_temp[i]==':') 
	  {
	    // �ж�tempT2
			j=0;i++;k=100;
	    while(k--)
	    {
	      if((Buf_temp[i]==':')||(Buf_temp[i]=='?')||(Buf_temp[i]==';')||(Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
	        break;
	      Buf_temp2[j]=Buf_temp[i];
	      i++;j++;
	    }
	    Buf_temp2[j]='\0';
	    tempT2=findCMD2(Buf_temp2);
			if(Buf_temp[i]==':')
			{
				// ��3��ð�ţ�����
				//sys_info.Update_DSC_IP=FALSE;
			}
			else
			{
				if(Buf_temp[i]==0x20)
				{
					// �ո�� ȡ����
					j=0;i++;
					while(1)
						{
							if(Buf_temp[i]==0x00)
								break;
							tempR[j]=Buf_temp[i];
							i++;j++;
						}
					tempR[j]='\0';
				}
				else
				{
					if(Buf_temp[i]=='?')
				{tempQ=1;}
							tempR[0]='\0';
				}
			}
	  }
	  else
	  {
			if(((tempC==4)||(tempC==5))&&(tempT1==-3))//��outָ��ʱ��ֵ�ڵ�λ����֮ǰ����ȡ��λ�жϵ�λ
			 {
					 j=0;i++;k=100;
					while(k--)
					 {
						 if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
								break;
							tempS[j]=Buf_temp[i];                 //��1��������ֵ
							i++;j++;
					 }
					 tempS[j]='\0';
				 tempT1=findCMD2(tempS);//��ȡtempT1 ȷ��������λ
			 }
	    else if(Buf_temp[i]==0x20)
      {
        //�ս� �ո�� ȡ����
        j=0;i++;k=100;
        while(k--)
         {
           if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
              break;
            tempR[j]=Buf_temp[i];                 //��1��������ֵ
            i++;j++;
         }
         tempR[j]='\0';
         if(Buf_temp[i]==0x20)
         {
            //�ս� �ո�� ȡ����
            j=0;i++;k=100; 
             while(k--)
             {
               if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
                  break;
                tempS[j]=Buf_temp[i];              //��2��������ֵ
                i++;j++;
             }
             tempS[j]='\0';
              if(Buf_temp[i]==0x20)
               {
                  //�ս� �ո�� ȡ����
                  j=0;i++;k=100; 
                   while(k--)
                   {
                     if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
                        break;
                      tempT[j]=Buf_temp[i];              //��3��������ֵ
                      i++;j++;
                   }
                   tempT[j]='\0'; 
                    if(Buf_temp[i]==0x20)
                     {
                        //�ս� �ո�� ȡ����
                        j=0;i++;k=100; 
                         while(k--)
                         {
                           if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
                              break;
                            tempU[j]=Buf_temp[i];          //��4��������ֵ
                            i++;j++;
                         }
                         tempU[j]='\0';
                          if(Buf_temp[i]==0x20)
                           {
                              //�ս� �ո�� ȡ����
                              j=0;i++;k=100; 
                               while(k--)
                               {
                                 if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
                                    break;
                                  tempV[j]=Buf_temp[i];      //��5��������ֵ
                                  i++;j++;
                               }
                               tempV[j]='\0';
                                if(Buf_temp[i]==0x20)
                                 {
                                    //�ս� �ո�� ȡ����
                                    j=0;i++;k=100; 
                                     while(k--)
                                     {
                                       if((Buf_temp[i]==0x00)||(Buf_temp[i]==0x20))
                                          break;
                                        tempW[j]=Buf_temp[i]; //��6��������ֵ
                                        i++;j++;
                                     }
                                    tempW[j]='\0';    
                                 }
                                  
                           }
                            
                     }
                     
               }
               
         }              
        tempT2=-1;
      }
      else
      {
        if(Buf_temp[i]=='?')
           {tempQ=1;}
          tempR[0]='\0';
      }
      tempT2=-1;
	  }
	}
	else
	{
	  if(Buf_temp[i]=='?')
	  {tempQ=1;}
	  tempT1=-1;
	  tempT2=-1;
	  tempR[0]='\0';
	}

   // ��������ɵ������Ӧ���������ִ��
   // �жϵ�һ�ιؼ��ֵ�����
  switch (tempC)
	{
		case 0: //-----------------------------------------------------��ѯ *IDN? �����
		case 1:
		{
      //Uart_send_string("VA701 industrial process calibrator\r\n");
			//V&A,VAT810,8001,V1.01
			//V&A,VA701,8001,V1.0
      //strcat(str,"V&A,VA702,8001,V1.0");
			strcat(str,SaveData.Value.VAfact); 	strcat(str,",");
			strcat(str,SaveData.Value.VAmodel); strcat(str,",");
			strcat(str,SaveData.Value.VAsn); 		strcat(str,",");
			strcat(str,SaveData.Value.VAver);		
      break;
		}
		
		case 8://read
		case 9://���ԣ���ȡ����1024�����ݡ�
		{
			switch(tempT1)
			{
				case 0xfe:
				{
					for(i=0;i<1024;i++)
					{				
						printf("%.4f\r\n",(float)(SDADC2_value[i]));
					}
					break;
				}
				case 85://SDADC2
				{
					for(i=0;i<1024;i++)
					{				
						printf("%.4f\r\n",(float)(SDADC2_value[i]));
					}
					break;
				}
				
				case 63://SDADC1
				{
					for(i=0;i<1024;i++)
					{				
						printf("%.4f\r\n",(float)(SDADC1_value[i]));
					}
					break;
				}
					//SaveData.Value.cal_A_a1=0.000640519;
					//SaveData.Value.cal_A_t1=1484.56402;
					//SaveData.Value.cal_A_y0=64.17008;
				case 60://				"a1"//60
				{
					//sprintf(chardata, " %.10f", SaveData.Value.cal_A_a1);
					//strcat(str,chardata);
					break;
				}
				case 61://					"t1"//61
				{
					//sprintf(chardata, " %.10f", SaveData.Value.cal_A_t1);
					//strcat(str,chardata);
					break;
				}
				case 62://					"y0"//62
				{
					//sprintf(chardata, " %.10f", SaveData.Value.cal_A_y0);
					//strcat(str,chardata);
					break;
				}
				default:	break;	
			}
			break;
		}
		
		case 12:
		case 13://cal
		{
			switch(tempT1)
			{
				case -1:
				{
					
					break;
				}								
				case 60://				//60 cal_600A_zero
				{
					CMD_analyze_R(tempR); 
					input_data=tp.numb_f[0];
					if((input_data>0)&&(input_data<200))
					{													
						SaveData.Value.cal_600A_zero=SysValue.curr_now/SaveData.Value.cal_600A_gain+SaveData.Value.cal_600A_zero;					
						updata_flash();
						strcat(str,"cal 600A zero");
					}
					break;
				}
				case 61://					//61 cal_600A_gain
				{
					CMD_analyze_R(tempR);
					input_data=tp.numb_f[0];
					if((input_data>200)&&(input_data<600))
					{										
						SaveData.Value.cal_600A_gain=input_data/(SysValue.curr_now/SaveData.Value.cal_600A_gain);
						updata_flash();
						strcat(str,"cal curr gain");
					}			
					break;
				}
				case 62://					"y0"//62
				{
//					CMD_analyze_R(tempR);
//					SaveData.Value.cal_A_y0=tp.numb_f[0];
//					sprintf(chardata, " %.10f", SaveData.Value.cal_A_y0);
//					strcat(str,chardata);
//					updata_flash();
					break;
				}
				case 64://6 zero
				{
//					if(rangenum == 0)
//					{
//						SaveData.Value.cal_6VA_zero=voltage_effective*10;
//						sprintf(chardata, "set 6VA_zero %.4f", SaveData.Value.cal_6VA_zero);
//						strcat(str,chardata);
//						updata_flash();
//					}
						Sysflag.Calonce	=1;
					break;
				}
				case 65://6 gain
				{
//					if(rangenum == 0)
//					{
//						SaveData.Value.cal_6VA_gain=5.0/(voltage_effective*10-SaveData.Value.cal_6VA_zero);
//						sprintf(chardata, "set 6VA_gain %.4f", SaveData.Value.cal_6VA_gain);
//						strcat(str,chardata);
//						updata_flash();
//					}
						Sysflag.Calonce	=1;
					break;
				}	
				case 66://60 zero
				{
//					if(rangenum == 1)
//					{
//						SaveData.Value.cal_60VA_zero=voltage_effective*10;
//						sprintf(chardata, "set 60VA_zero %.4f", SaveData.Value.cal_60VA_zero);
//						strcat(str,chardata);
//						updata_flash();
//					}
						Sysflag.Calonce	=1;		
					break;
				}
				case 67://60 gain
				{
//					if(rangenum == 1)
//					{
//						SaveData.Value.cal_60VA_gain=5.0/(voltage_effective*10-SaveData.Value.cal_60VA_zero);
//						sprintf(chardata, "set 60VA_gain %.4f", SaveData.Value.cal_60VA_gain);
//						strcat(str,chardata);
//						updata_flash();
//					}
						Sysflag.Calonce	=1;
					break;
				}	
				case 68://600 zero
				{
					if(rangenum == 2)
					{
						SaveData.Value.cal_600VA_zero=voltage_effective*10;
						sprintf(chardata, "set 600VA_zero %.4f", SaveData.Value.cal_600VA_zero);
						strcat(str,chardata);
						updata_flash();
					}
							
					break;
				}
				case 69://600 gain
				{
					if(rangenum == 2)
					{
						SaveData.Value.cal_600VA_gain=5.0/(voltage_effective*10-SaveData.Value.cal_600VA_zero);
						sprintf(chardata, "set 600VA_gain %.4f", SaveData.Value.cal_600VA_gain);
						strcat(str,chardata);
						updata_flash();
					}
					break;
				}	
				case 70://6D zero
				{
					Sysflag.Calonce	=1;
					break;
				}
				case 71://6D gain
				{					
					Sysflag.Calonce	=1;
					break;
				}	
				case 72://60D zero
				{
					Sysflag.Calonce	=1;
					break;
				}
				case 73://60D gain
				{					
					Sysflag.Calonce	=1;
					break;
				}	
				case 74://600D zero
				{
					Sysflag.Calonce	=1;
					break;
				}
				case 75://600D gain
				{					
					Sysflag.Calonce	=1;
					break;
				}	
				
//			"cal_a_err",			//76 �ڶ�Ӧ����A��У��ƫ��ֵ cal_A_errN    
//			"cal_a_zero",			//77  �ڶ�Ӧ������У�����ƫ��ֵ	
//			"cal_a_gain",			//78	��ĳ���ϴ����ʱУ׼���Զε�����
//			"cal1800a",				//79	������DC1800A���ĵ�ѹֵΪ�����У׼��ѹ��ʼ��
//			"cal_adjv",				//80
				case 76: //76 �ڶ�Ӧ����A��У��ƫ��ֵ cal_A_errN   
				{
					
					break;
				}
				case 77: //77  �ڶ�Ӧ������У�����ƫ��ֵ	
				{									
					CMD_analyze_R(tempR); 
					input_data=tp.numb_f[0];
					if((input_data>0)&&(input_data<1200))
					{				
						//SaveData.Value.cal_A_zero+=((input_data-SysValue.curr_now)/SaveData.Value.cal_A_gain);	
						SaveData.Value.cal_A1_zero=SysValue.curr_now/SaveData.Value.cal_A1_gain+SaveData.Value.cal_A1_zero;
						//SaveData.Value.cal_A1_zero=((input_data-SysValue.curr_now)/SaveData.Value.cal_A1_gain);
						updata_flash();
						strcat(str,"cal curr zero");
					}
									
					break;
				}
				case 78: //78	��ĳ���ϴ����ʱУ׼���Զε�����
				{
					CMD_analyze_R(tempR);
					input_data=tp.numb_f[0];
					if((input_data>1200)&&(input_data<1900))
					{			
						//SaveData.Value.cal_A_zero+=(SysValue.curr_now-input_data)/SaveData.Value.cal_A_gain;	
						SaveData.Value.cal_A1_gain=input_data/(SysValue.curr_now/SaveData.Value.cal_A1_gain);
						updata_flash();
						strcat(str,"cal curr gain");
					}			
					break;
				}
				case 79: //79 ������DC1800A���ĵ�ѹֵΪ�����У׼��ѹ��ʼ��
				{
					
					
					break;
				}
				case 80://�Ƿ����õ���
				{
					char *ttpp;
					CMD_analyze_R(tempR);
					SaveData.Value.cal_adjv=tp.numb_f[0];
					updata_flash();
					strcat(str,"cal adjv:");
					strcat(str,tempR);
					break;
				}
				
				case 81://"daout",								//81 ����DAC�����ѹ
				{
					char *ttpp;
					CMD_analyze_R(tempR);					
					
					Dac1_Set_Vol(tp.numb_f[0]);
					strcat(str,"daout:");
					strcat(str,tempR);
					break;
				}
				
				default:break;
			}
			break;	
		}
		
		case 18:
		case 19://factory fact  ���� �ͺ����к��ṩ�� ����汾 ���޸�
		{	
			//		"fact",			"model",//52�������� 53�ͺ�  
			//		"sn",				"ver",//54���к�  55�汾��  
			switch(tempT1)
			{
				case 50:
				{//��ʼ��Flash�б���Ĳ�������ֹ������������û��޷��ָ���
					if(tempR[0]==0x31) //ָ���ʽ   fact:initialflash 1
					{
						Default_flash();
						strcat(str,"Write default value to flash, This Machine need calibration to use.\r\n");
					}			
					break;
				}
				case 52://fact
				{
					for(i=0;i<30;i++)
					{
						if((tempR[i]>96)&&(tempR[i]<123))//תΪ��ֵ
						SaveData.Value.VAfact[i]=tempR[i]-32;
						else
						SaveData.Value.VAfact[i]=tempR[i];
					}
					updata_flash();
					strcat(str,"write fact ok!");
					break;
				}
				case 53://model
				{
					for(i=0;i<30;i++)
					{
						if((tempR[i]>96)&&(tempR[i]<123))
						SaveData.Value.VAmodel[i]=tempR[i]-32;
						else
						SaveData.Value.VAmodel[i]=tempR[i];
					}
					updata_flash();
					strcat(str,"write model ok!");
					break;
				}
				case 54://sn
				{
					for(i=0;i<30;i++)
					{
						if((tempR[i]>96)&&(tempR[i]<123))
						SaveData.Value.VAsn[i]=tempR[i]-32;
						else
						SaveData.Value.VAsn[i]=tempR[i];
					}
					updata_flash();
					strcat(str,"write sn ok!");
					break;
				}
				case 55://ver
				{
					for(i=0;i<30;i++)
					{
						if((tempR[i]>96)&&(tempR[i]<123))
						SaveData.Value.VAver[i]=tempR[i]-32;
						else
						SaveData.Value.VAver[i]=tempR[i];
					}
					updata_flash();
					strcat(str,"write ver ok!");
					break;			
				}
				
				default:break;
			}
			break;
		}
		case 24://24  ���������У׼  ex:  cala:1800  cala:2400
		{
			//			//�����У׼6����
			//			float CalA_point[8]={1600,1800,2000,2200,2400,2600,2800,3000};
			//			//��Ӧ����У׼���µĵ�������ֵ
			//			float CalA_data[8] ={0,0,0,0,0,0,0,0};
			//			//���յ�ǰ������������ĸ���У׼���
			//			float CalA_temp[8] ={0,0,0,0,0,0,0,0};
			CMD_analyze_R(tempR);//����Ĳ�������ΪCalA_point ָ���ĵ㡣
			i=(uint32_t)tp.numb_f[0];
			//			switch(i)
			//			{
			//				case 1800: CalA_data[(i-1600)/200]=0;
			//					
			//			}
			CalA_data[(i-1600)/200]=SysValue.curr_now;
			updata_flash();
			sprintf(chardata, "Set CalA_data[%d]= %.4f",((i-1600)/200),SysValue.curr_now);
			strcat(str,chardata);		
			break;
		}
				
		
		case 25://25У׼DTA��ѹ
		{
			Send(SetCalMode,0x55,SetCalMode+0x55);//����У��ģʽ
			check_flag= CheckReceive(SetCalMode);
			if(check_flag==1)
			{
				check_flag=0;
				printf("\r\n����У��:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
				printf("\r\n����У��ģʽ\r\n");
			}
			delay_ms(80000);//��Ϊ��ʱ  û�������ʱУ׼����ȥ
			
			if(tempT1==82)//82 ����У׼DTA��ѹ			
			{
				Send(CalDataINC,CalDataINC);
				while(dta_receive_flag==0)
				{
					printf("�ȴ�У׼��\r\n");
					delay_ms(1000);
				}
				check_flag= CheckReceive(CalDataINC);	
			}
			else if(tempT1==83)//83 ����У׼DTA��ѹ
			{
				Send(CalDataDEC,CalDataDEC);
				while(dta_receive_flag==0)
				{
					printf("�ȴ�У׼��\r\n");
				}
				check_flag= CheckReceive(CalDataDEC);	
			}
			else if(tempT1==84)//84 ����Ϊ5000
			{
				CMD_analyze_R(tempR);			
				
				if(tp.numb_f[0]!=0)
				{
					sprintf(chardata, "dta_calv:five  %.10f", tp.numb_f[0]);
					CalH = ((int)tp.numb_f[0]>>8)&0xff;
					CalL = (int)tp.numb_f[0]&0xff;
					
					Send(CalSetData,CalL,CalH,CalSetData+CalL+CalH);//У׼�ض�ֵ
					check_flag= CheckReceive(CalSetData);
				}
				else
				{
					if(rangestatus==state4)//1000V��У׼
					{																		
						//Send(CalSetData,0xf4,0x01,CalSetData+0xf4+0x01);//����У׼���������⣬Ŀǰ1000V��û��У׼��
						Send(CalSetData,0xE8,0x03,CalSetData+0xE8+0x03);//У׼1000V
						check_flag= CheckReceive(CalSetData);
					}
					else
					{
						Send(CalSetData,0x88,0x13,CalSetData+0x88+0x13);
						check_flag= CheckReceive(CalSetData);
					}	
				}		
			}	
			
			if(check_flag==1)
			{
				check_flag=0;
				printf("У׼���\r\n");
				
				BUZZER_Open(0);
			}
			else
			{
				printf("У׼ʧ��\r\n");
			}
			
			Send(SetCalMode,0x00,SetCalMode);
			check_flag= CheckReceive(SetCalMode);
			if(check_flag==1)
			{
				check_flag=0;
				printf("\r\n�ر�У��:	%x	%x	%x	||",RxBuffer[0],RxBuffer[1],RxBuffer[2]);
				printf("\r\n�ر�У��ģʽ\r\n");
			}
			
			break;
		}
		
		case 26://26��ȡDTA���ò�
		{
			
			ReadEeprom();
			break;
		}
		case 27://27��ʼ��DTA���ò���
		{
			WriteEeprom();			
			break;
		}
		
		
		
		default:break;

	}
	
  strcat(str,"\r\n");
  printf(str);  
}

void Communication_Service()
{
	char Ostr[120]="\0";
	char    chardata[10];
	float Data=0;
	float temp=0;	
	uint8_t temp_nm[12];
	uint8_t i=0,j=0;
	
	if(Flag_Rx2End)
	{
		Flag_Rx2End=0;
		Communicate(); 		
	}
	
//	if(T05_4>=2)//�˴�Ϊ���������� 0.5*4=2S
//	{
//		T05_4=0;
//			
//			strcat(Ostr,"\r\n");
//			Uart_send_string(Ostr);
//			
//			
//		}
//	}
	
}


//��Ҫ����DTA��У׼ָ��
void DTA_CAL(void)
{
	
}




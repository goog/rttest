#include "oled.h"
#include "oled_iic.h"
//#include "stdlib.h"
#include "oledfont.h"  	 
//#include "bsp_SysTick.h"
//#include "config.h"
//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//   ...     ...
//[17]0 1 2 3 ... 127	
//[18]0 1 2 3 ... 127	
//[19]0 1 2 3 ... 127
	
u8 OLED_GRAM[128][20];

u8 CABNUM[12]={"12345678901"};
u8 SENDT[7]={" 00.0C"};
u8 BACKT[7]={" 11.1C"};
u8 SETT[7]={" 22.2C"};
u8 TIME1[9]={"  /  /  "};
u8 TIME2[9]={"  :  :  "};
u8 TIME3[20]={"00/00/00  00:00:00"};


u8 OLED_Clear_Flag=0;					//������־λ
u8 MenuCursor=0;							//�˵����
LanguageTypedef			OLEDLanguage = OLED_Chinese;		//ϵͳ����
MenuStatusTypedef		MENUStatus	 = MENU_Main;

/********************************************
// Delay
********************************************/
void Delay_50ms(unsigned int Del_50ms)
{
	unsigned int m;
	for(;Del_50ms>0;Del_50ms--)
		for(m=6245;m>0;m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
	unsigned char j;
	while(Del_1ms--)
	{	
		for(j=0;j<123;j++);
	}
}

void Delay_ms(unsigned int Del_1ms)
{
	unsigned char j;
	while(Del_1ms--)
	{	
		for(j=0;j<123;j++);
	}
}

/********************************************
// ��OLEDд����/����
// cmd: 0,OLED_CMD;	1,OLED_DATA
********************************************/
void OLED_WR_Byte(unsigned int dat,unsigned int cmd)
{
	if(cmd)
	{		
		Write_IIC_Data(dat);
	}
	else 
	{
		Write_IIC_Command(dat);
	}
}

/********************************************
// ����ĳ����ɫ������Ļ
// fill_Data: 0x00,ȫ��,��û����һ��
//						0xff,ȫ��,ȫ������
********************************************/
void OLED_Fill(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<20;m++)
	{
		
		OLED_WR_Byte(0xb0,OLED_CMD);
		OLED_WR_Byte(0x00+m,OLED_CMD);
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0x11,OLED_CMD); 
    for(n=0;n<128;n++)
    {
			OLED_WR_Byte(fill_Data,OLED_DATA);
		}
	}
}

/********************************************
// ����
********************************************/
void OLED_Clear(void)
{
	OLED_Fill(0x00);
}


/********************************************
// ����
********************************************/
void OLED_Clear_Data(void)
{
	u8 x,y;
	for(x=0;x<128;x++)
		for(y=0;y<160;y++)
		{
			OLED_DrawPoint(x,y,0);
		}	
}

/********************************************
//����OLED��ʾ 
********************************************/
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

/********************************************
//�ر�OLED��ʾ   
********************************************/
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 


/********************************************
//�����Դ浽OLED	
********************************************/
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<20;i++)  
	{  
		OLED_WR_Byte(0xb0,OLED_CMD);
		OLED_WR_Byte(0x00+i,OLED_CMD);
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0x11,OLED_CMD);  
		for(n=0;n<128;n++)	OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

/********************************************
//���²�����ʾ����
//x1,y1,x2,y2  ��������ĶԽ�����
//ȷ��x1<=x2;y1<=y2		0<=x1<=127  0<=y1<=159
********************************************/
void OLED_REfresh_Part(u8 x1,u8 y1,u8 x2,u8 y2)
{
	u8 x,y;
}

/********************************************
//���ػ��� 
//x:0~127
//y:0~159
//t:1 ��� 0,���	
********************************************/
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx;
	u8 temp=0;
	
	if(x>127||y>159)	return;		//������Χ��
	//pos = 19-y/8;
	pos = y/8;
	bx = y%8;
	temp = 0x80>>(7-bx);
	if(t)	OLED_GRAM[x][pos]|=temp;
	else	OLED_GRAM[x][pos]&=~temp;
}

/********************************************
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~159
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 12/16/24
********************************************/
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			
	u8 hahah;
	
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize = (size/8 + ( (size%8)?1:0) ) * (size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ		 
  for(t=0;t<csize;t++)
  {		
		if(size==12)	temp=asc2_1206[chr][t];				//����1206����
		else if(size==16)	temp=asc2_1608[chr][t];		//����1608����
		else if(size==24)	temp=asc2_2412[chr][t];		//����2412����
		else return;									//û�е��ֿ�
		
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)	OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
	}          
}

/********************************************
//m^n����
********************************************/
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			

/********************************************
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 	
********************************************/
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size,u8 mode)
{
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,mode);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,mode); 
	}
}

/********************************************
//��ʾ�ַ���
// x,y: �������  
//size: �����С 
// *p : �ַ�����ʼ��ַ 
********************************************/
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size,u8 mode)
{	
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>(128-(size/2)))	{x=0;y+=size;}
        if(y>160)	{y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,size,mode);	 
        x+=size/2;
        p++;
    }  
	
}	

/********************************************
//��ʾһ������
// x,y: �������  
//addr: ���ֵ�ַ
********************************************/
void OLED_ShowHzk(u8 x,u8 y,u8 addr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize = (size/8 + ( (size%8)?1:0) ) * size;		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	
  for(t=0;t<csize;t++)
  {
		if(size==12)	temp=chi_word12[addr][t];
		else if(size==16)	temp=chi_word16[addr][t];
		else if(size==24) temp=chi_word24[addr][t];
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)	OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
	} 					
	
}

/********************************************
//��ʾ����
// x,y: �������  
//addr: ��ʼ��ַ
// len: ����
********************************************/
void OLED_ShowSentense(u8 x,u8 y,u8 addr,u8 len,u8 size,u8 mode)
{      			    
	u8 n;

	for(n=addr;n<addr+(len*2);n+=2)
	{
		if(x>(128-size)){x=0;y+=size;}
    if(y>(160-size)){y=x=0;OLED_Clear();}
		OLED_ShowHzk(x,y,n,size,mode);
    x+=size;
	}
	
}


///***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
//void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
//{ 	
// unsigned int j=0;
// unsigned char x,y;
//  
//  if(y1%8==0) y=y1/8;      
//  else y=y1/8+1;
//	for(y=y0;y<y1;y++)
//	{
//		OLED_Set_Pos(x0,y);
//    for(x=x0;x<x1;x++)
//	    {      
//	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
//	    }
//	}
//} 

//��ʼ��OLED
				    
void OLED_Init(void)
{ 	

 	//GPIO_InitTypeDef  GPIO_InitStructure;
 	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��B�˿�ʱ��
	//GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14;	 
 	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	//GPIO_Init(GPIOB, &GPIO_InitStructure);	  //��ʼ��GPIOD3,6
 	//GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14);	

	Delay_ms(200);
	
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x20,OLED_CMD);//---set memory addressing mode
	OLED_WR_Byte(0x81,OLED_CMD);//---set contrast control
	OLED_WR_Byte(0x90,OLED_CMD);//--  
	OLED_WR_Byte(0xa0,OLED_CMD);//--Segment remap
	OLED_WR_Byte(0xa6,OLED_CMD); // Normal display
	OLED_WR_Byte(0xa9,OLED_CMD);//--Set Display Resolution 
	OLED_WR_Byte(0x02,OLED_CMD);//--160*128
	OLED_WR_Byte(0xad,OLED_CMD);//--Set external VPP
	OLED_WR_Byte(0x80,OLED_CMD);//--
	OLED_WR_Byte(0xc0,OLED_CMD);//--Set Common scan direction
	OLED_WR_Byte(0xd5,OLED_CMD);//--Divide Ratio/Oscillator Frequency Mode Set
	OLED_WR_Byte(0xf1,OLED_CMD);//--
	OLED_WR_Byte(0xd9,OLED_CMD);//--Set Dis-charge/Pre-charge Period
	OLED_WR_Byte(0x2a,OLED_CMD);//--
	OLED_WR_Byte(0xdb,OLED_CMD);//--Set Vcomh voltage
	OLED_WR_Byte(0x2b,OLED_CMD);//--
	OLED_WR_Byte(0xdc,OLED_CMD);//--Set VSEGM Deselect Level
	OLED_WR_Byte(0x35,OLED_CMD);//--
	OLED_WR_Byte(0x30,OLED_CMD);//--Set Discharge VSL Level,0V
			
	OLED_Fill(OLED_CLEAR);		//����
	
	OLED_WR_Byte(0xaf,OLED_CMD);//--Display ON
		
}  

#if 1
void OLED_Menu_Data(void)
{
	u8 year,mon,mday,hour,min,sec;
	
	//year=systmtime.tm_year %100;    //����RTCʱ��
//	year=year%100;
    year = 19;
	//mon=systmtime.tm_mon;
	mon = 4;
	//mday=systmtime.tm_mday;
	mday = 11;
	//hour=systmtime.tm_hour;
	//min=systmtime.tm_min;
	//sec=systmtime.tm_sec;
	hour = 0;
	min = 0;
	sec = 0;
	
	if(OLED_Chinese==OLEDLanguage)
		{
			
			if(MENU_Sleep==MENUStatus)
				{
					OLED_Clear_Data();
					OLED_Clear();
				}
			
			if(MENU_Main==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/
						
					OLED_ShowSentense(39,0,ADDR_MENU,2,24,1);						//�˵�
					OLED_ShowString(94,8,"Uxdar",12,1); //
					OLED_ShowSentense(31,24+12,ADDR_NET,2,16,((MenuCursor-0)?1:0));
					OLED_ShowSentense(31+32,24+12,ADDR_SITUATION,2,16,((MenuCursor-0)?1:0));			//�������
					
					OLED_ShowSentense(31,24+16+16,ADDR_CONNECT,2,16,((MenuCursor-1)?1:0));
					OLED_ShowSentense(31+32,24+16+16,ADDR_STATUR,2,16,((MenuCursor-1)?1:0));			//����״̬

					OLED_ShowSentense(31,24+32+20,ADDR_READCAB,2,16,((MenuCursor-2)?1:0));
					OLED_ShowSentense(31+32,24+32+20,ADDR_DETAIL,2,16,((MenuCursor-2)?1:0));			//��������

					OLED_ShowSentense(31,24+48+24,ADDR_ACTIVE,2,16,((MenuCursor-3)?1:0));
					OLED_ShowSentense(31+32,24+48+24,ADDR_READCAB,2,16,((MenuCursor-3)?1:0));			//��������
					
					OLED_ShowSentense(31,24+64+28,ADDR_LANG,4,16,((MenuCursor-4)?1:0));						//����ѡ��
						
//					OLED_ShowSentense(0,147,ADDR_SURE,2,12,1);
//					OLED_ShowSentense(103,147,ADDR_BACK,2,12,1);
//					OLED_ShowString(40,147-12,TIME1,12,1);
//					OLED_ShowString(40,147,TIME2,12,1);					
					OLED_ShowString(9,147,TIME3,12,1);					//ʱ��
					
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);

				}
				
			if(MENU_Net==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(39-24,0,ADDR_NET,2,24,1);
					OLED_ShowSentense(39+24,0,ADDR_SITUATION,2,24,1);						//�������

					OLED_ShowSentense(0,24+16,ADDR_SIGNAL,2,16,1);
					OLED_ShowSentense(0+32,24+16,ADDR_STRENGTH,2,16,1);					//�ź�ǿ��
					//if(GPRS_CONNECT==GPRSStatus)		OLED_ShowString(48+16,24+16,GPRS_CSQ,16,1);
					//else		OLED_ShowString(48+16,24+16,"     ",16,1);
					
					OLED_ShowSentense(0,24+16+16+8,ADDR_OPERATOR,3,16,1);				//��Ӫ��
					#if 0
					if(CMCC==GPRSOperator)		OLED_ShowString(48+16,24+16+16+8,"CMCC",16,1);					//�ƶ�
					else if(CUCC==GPRSOperator)		OLED_ShowString(48+16,24+16+16+8,"CUCC",16,1);					//��ͨ
					else		OLED_ShowString(48+16,24+16+16+8,"    ",16,1);					
					#endif
					OLED_ShowSentense(0,24+16+16+16+16,ADDR_YES,1,16,1);
					OLED_ShowSentense(0+16,24+16+16+16+16,ADDR_NO,1,16,1);
					OLED_ShowSentense(0+32,24+16+16+16+16,ADDR_CONNECT,2,16,1);
					OLED_ShowSentense(0+32+32,24+16+16+16+16,ADDR_NET,2,16,1);		//�Ƿ���������
					#if 0
					if(GPRSNET_flag)	OLED_ShowSentense(0+32+32+32+8,24+16+16+16+16,ADDR_YES,1,16,1);		//��
					else	OLED_ShowSentense(0+32+32+32+8,24+16+16+16+16,ADDR_NO,1,16,1);			//��
					#endif
					OLED_ShowString(9,147,TIME3,12,1);					//ʱ��
						
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);						
				}
				
			if(MENU_Connet==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(39-24,0,ADDR_CONNECT,2,24,1);
					OLED_ShowSentense(39+24,0,ADDR_STATUR,2,24,1);									//����״̬

					OLED_ShowSentense(0,24+16,ADDR_NET,2,16,1);
					OLED_ShowSentense(0+32,24+16,ADDR_CONNECT,2,16,1);
					OLED_ShowSentense(0+64,24+16,ADDR_STATUR,2,16,1);								//��������״̬
					//if(GPRSCONNET_flag)		OLED_ShowSentense(0+32+32+32+8,24+16,ADDR_GOU,1,16,1);		//����
					//else		OLED_ShowSentense(0+32+32+32+8,24+16,ADDR_CHA,1,16,1);		//�Ͽ�
					
					OLED_ShowSentense(0,24+16+16+8,ADDR_READCAB,3,16,1);
					OLED_ShowSentense(0+32,24+16+16+8,ADDR_CONNECT,2,16,1);
					OLED_ShowSentense(0+64,24+16+16+8,ADDR_STATUR,2,16,1);					//��������״̬
					
					OLED_ShowString(0,24+16+16+16+16,"GPS",16,1);
					OLED_ShowSentense(0+24,24+16+16+16+16,ADDR_CONNECT,2,16,1);
					OLED_ShowSentense(0+24+32,24+16+16+16+16,ADDR_STATUR,2,16,1);		//GPS����״̬
					//if(GNSSCONNECT_flag)		OLED_ShowSentense(0+32+32+32+8,24+16+16+16+16,ADDR_GOU,1,16,1);		//����
					//else		OLED_ShowSentense(0+32+32+32+8,24+16+16+16+16,ADDR_CHA,1,16,1);		//�Ͽ�
					
					OLED_ShowString(9,147,TIME3,12,1);						//ʱ��
					
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);					
				}
				
			if(MENU_Temp==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(39-24,0,ADDR_READCAB,2,24,1);
					OLED_ShowSentense(39+24,0,ADDR_DETAIL,2,24,1);							//��������				

					OLED_ShowSentense(12,24*1+8,ADDR_CABNUM,2,12,1);
					OLED_ShowChar(42,24*1+8,'.',12,1);
					OLED_ShowString(48,24*1+8,CABNUM,12,1);											//���
		
					OLED_ShowSentense(0,16*2+16,ADDR_AIRS,2,16,1);
					OLED_ShowSentense(0+32,16*2+16,ADDR_TEMP,2,16,1);
					OLED_ShowChar(64,16*2+16,':',16,1);	
					OLED_ShowString(72,16*2+16,SENDT,16,1);											//�ͷ��¶�
		
					OLED_ShowSentense(0,16*4+8,ADDR_AIRR,2,16,1);
					OLED_ShowSentense(0+32,16*4+8,ADDR_TEMP,2,16,1);
					OLED_ShowChar(64,16*4+8,':',16,1);	
					OLED_ShowString(72,16*4+8,BACKT,16,1);											//�ط��¶�
	
					OLED_ShowSentense(0,16*5+16,ADDR_SET,2,16,1);
					OLED_ShowSentense(0+32,16*5+16,ADDR_TEMP,2,16,1);
					OLED_ShowChar(64,16*5+16,':',16,1);	
					OLED_ShowString(72,16*5+16,SETT,16,1);											//�����¶�

					OLED_ShowString(9,147,TIME3,12,1);													//ʱ��			
				
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);				
				}
				
			if(MENU_Active_Ing==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(16,16*2+16,ADDR_ING,2,16,1);
					OLED_ShowSentense(16+32,16*2+16,ADDR_READCAB,2,16,1);
					OLED_ShowString(16+32+16*2,16*2+16,"...",16,1);
					
					
					OLED_ShowString(9,147,TIME3,12,1);
					
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);
						
					MENUStatus = MENU_Active_Result;
				}
				
			if(MENU_Active_Result==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(16,16*2+16,ADDR_READCAB,2,16,1);
					OLED_ShowSentense(16+32,16*2+16,ADDR_SUCCESS,2,16,1);					
					OLED_ShowString(16+32+16*2,16*2+16,"!",16,1);
					
					
					OLED_ShowString(9,147,TIME3,12,1);
				
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);						
				}
				
			if(MENU_Language_Choose==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
						
					OLED_ShowSentense(39-24,0,ADDR_LANG,4,24,1);
					
					OLED_ShowSentense(31,24+16+16,ADDR_CHINESE,4,16,1);
					
					OLED_ShowString(31,24+16+16+16+16+8,"English",16,1);
					
					OLED_ShowString(9,147,TIME3,12,1);					
				
					OLED_ShowNum(9,147,year,2,12,1);
					OLED_ShowNum(9+18,147,mon,2,12,1);
					if(mon<10)	OLED_ShowChar(9+18,147,'0',12,1);
					OLED_ShowNum(9+18+18,147,mday,2,12,1);
					if(mday<10)	OLED_ShowChar(9+18+18,147,'0',12,1);	

					OLED_ShowNum(9+18+18+24,147,hour,2,12,1);
					if(hour<10)	OLED_ShowChar(9+18+18+24,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18,147,min,2,12,1);
					if(min<10)	OLED_ShowChar(9+18+18+24+18,147,'0',12,1);
					OLED_ShowNum(9+18+18+24+18+18,147,sec,2,12,1);
					if(sec<10)	OLED_ShowChar(9+18+18+24+18+18,147,'0',12,1);				
				}

		}
	if(OLED_English==OLEDLanguage)
		{
			
			if(MENU_Sleep==MENUStatus)
				{
					OLED_Clear_Data();
					OLED_Clear();
				}
			
			if(MENU_Main==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/


				}
				
			if(MENU_Net==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/
						

				}
				
			if(MENU_Connet==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	

						
				}
				
			if(MENU_Temp==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	

						
				}
				
			if(MENU_Active_Ing==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	

						
				}
				
			if(MENU_Active_Result==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	

						
				}
				
			if(MENU_Language_Choose==MENUStatus)
				{
					if(OLED_Clear_Flag)
						{
							OLED_Clear_Data();
							OLED_Clear_Flag=0;
						}
					/*************��ʾ����**************/	
					OLED_ShowSentense(39-24,0,ADDR_LANG,4,24,1);
					
					OLED_ShowSentense(31,24+16+16,ADDR_CHINESE,4,16,1);
					
					OLED_ShowString(31,24+16+16+16+16+8,"English",16,1);
					
					OLED_ShowString(9,147,TIME3,12,1);					
				}

		}
}
#endif























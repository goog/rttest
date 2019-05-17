#ifndef __OLED_IIC_H
#define __OLED_IIC_H	
//#include "sys.h"
//#include "stdlib.h"	   
#include "pins.h"  					   

#define OLED_SCLK_Clr() SCL_CLEAR()
#define OLED_SCLK_Set() SCL_SET()

#define OLED_SDIN_Clr() SDA_CLEAR()
#define OLED_SDIN_Set() SDA_SET()

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

//-----------------IIC控制用函数-------------------
void IIC_Start();
void IIC_Stop();
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);

void IIC_Wait_Ack();
#endif  
	 

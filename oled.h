#ifndef __OLED_H
#define __OLED_H			  	 
//#include "sys.h"
#include "stdlib.h"
#include <stdint.h>	    	
//#define OLED_MODE 0
//#define SIZE 8
//#define XLevelL		0x00
//#define XLevelH		0x10
//#define Max_Column	128
//#define Max_Row		64
//#define	Brightness	0xFF 
//#define X_WIDTH 	128
//#define Y_WIDTH 	64	
typedef uint8_t u8;
typedef unsigned int u32;

#define OLED_CLEAR	0X00
#define OLED_LIGHT	0XFF

#define ADDR_TITLE			0			
#define LEN_TITLE				12		//深圳市优鲜达科技有限公司
#define ADDR_MENU				24		//菜单		
#define ADDR_SURE				28		//确定		
#define ADDR_BACK				32		//返回
#define	ADDR_NET				36		//网络
#define	ADDR_SITUATION	40		//情况
#define ADDR_CONNECT		44		//连接
#define ADDR_STATUR			48		//状态
#define ADDR_READCAB		52		//读柜
#define	ADDR_DETAIL			56		//详情
#define ADDR_ACTIVE			60		//主动
#define ADDR_SIGNAL			64		//信号
#define ADDR_STRENGTH		68		//强度
#define	ADDR_OPERATOR		72		//运营商
#define ADDR_YES				78		//是
#define ADDR_NO					80		//否
#define	ADDR_CABNUM			82		//柜号
#define ADDR_AIRS				86		//送风
#define ADDR_AIRR				90		//回风
#define ADDR_SET				94		//设置
#define	ADDR_TEMP				98		//温度
#define ADDR_ING				102		//正在
#define	ADDR_SUCCESS		106		//成功
#define	ADDR_FAIL				110		//失败
#define	ADDR_LANG				114		//语言选择
#define ADDR_CHINESE		122		//简体中文
#define ADDR_GOU				130		//√
#define ADDR_CHA				132		//×


typedef enum										//系统语言
{
	OLED_Chinese=0,								//中文
	OLED_English,									//英文
}
LanguageTypedef;

typedef enum
{
	MENU_Sleep=0,									//熄屏
	MENU_Main,										//主菜单
	MENU_Net,											//网络情况
	MENU_Connet,									//连接状态
	MENU_Temp,										//读柜详情
	MENU_Active_Ing,							//正在读柜
	MENU_Active_Result,						//读柜成功/失败
	
	MENU_Language_Choose,					//系统语言选择
}
MenuStatusTypedef;

extern MenuStatusTypedef		MENUStatus;
extern u8 MenuCursor;									//菜单光标
extern u8 OLED_Clear_Flag;						//清屏标志位

//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);  				//OLED写命令/数据
//void OLED_Display_On(void);													//OLED显示开启
//void OLED_Display_Off(void);												//OLED显示关闭	   							   		    
extern void OLED_Init(void);
void OLED_Fill(unsigned char fill_Data);							//满屏幕显示
void OLED_Clear(void);																//清屏
void OLED_Clear_Data(void);														//清数据
void OLED_Refresh_Gram(void);													//屏幕刷新
void OLED_DrawPoint(u8 x,u8 y,u8 t);									//在屏幕上画点
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);		//显示一个字符

u32 OLED_Pow(u8 m,u8 n);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size,u8 mode);		//在屏幕上显示数字

void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size,u8 mode);		//在屏幕上显示字符串


void OLED_Set_Pos(unsigned char x, unsigned char y);

void OLED_ShowHzk(u8 x,u8 y,u8 addr,u8 size,u8 mode);
void OLED_ShowSentense(u8 x,u8 y,u8 addr,u8 len,u8 size,u8 mode);


void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);

void OLED_Menu_Data(void);

#endif  
	 

#include "adc.h"
//#include "delay.h"		 


#define RCC_BASE 0x40023800
#define ADC1_BASE 0x40012000

#define RCC_APB2ENR  (*(volatile uint32_t *)0x40023844)
#define RCC_APB2RSTR (*(volatile uint32_t *)0x40023824)
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE+0x30))
    
#define ADC1_CR1 (*(volatile uint32_t *)(ADC1_BASE+0x04))
#define ADC1_CR2 (*(volatile uint32_t *)(ADC1_BASE+0x08))
#define ADC1_SQR1 (*(volatile uint32_t *)(ADC1_BASE+0x2c))
#define ADC1_SMPR1 (*(volatile uint32_t *)(ADC1_BASE+0x0c))
#define ADC_CCR (*(volatile uint32_t *)(ADC1_BASE+0x304))
#define ADC_CR2_SWSTART  ((uint32_t)0x40000000)
#define ADC1_DR (*(volatile uint32_t *)(ADC1_BASE+0x4C))
   
#define ADC1_SQR3 (*(volatile uint32_t *)(ADC1_BASE+0x34))
#define ADC1_SR   (*(volatile uint32_t *)(ADC1_BASE))


																	   
void  stm32_adc_init(void)
{    

     
 	RCC_APB2ENR|= 1<<8;    	 
	RCC_AHB1ENR|= 1<<2;  	  
	   

	RCC_APB2RSTR |=1<<8;   	//reset ADCs
	RCC_APB2RSTR &= ~(1<<8);	//
    printf("RCC_APB2RSTR %08x \n", RCC_APB2RSTR);	 
	ADC_CCR = 1<<16;			//ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
 	printf("ADC_CCR %08x \n", ADC_CCR);
    
	//ADC1_CR1=0;   			
	//ADC1_CR2=0;
    printf("after reset cr 1 %08x cr2 %08x \n", ADC1_CR1, ADC1_CR2);   			
	ADC1_CR1 |= 0<<24;      	//12位模式
	
    //ADC1_CR1|=0<<8;    	//
    ADC_CCR &= 0xffffffe0;  // independence
    ADC1_CR1 &= ~(1<<8);  // disable scan	
	ADC1_CR2 &= ~(1<<1);    	//single
 	ADC1_CR2 &= ~(1<<11);     // right align   		
	ADC1_CR2 &= 0xcfffffff;
	printf("after set mode CR2 %08x \n", ADC1_CR2);
    
    
	ADC1_SQR1&=(uint32_t)~(0XF<<20);
	ADC1_SQR1 |=0<<20;
    printf(" sqr 1 %08x \n", ADC1_SQR1); 			   
	
	ADC1_SMPR1&=~(7<<(3*4));	  
 	ADC1_SMPR1|=7<<(3*4);	 
 	ADC1_CR2|=1<<0;	   	//adc ON
    printf(" CR2 %08x \n", ADC1_CR2);
    printf("adc init end\n");	  
}				  



uint16_t Get_Adc(uint8_t ch)   
{
    	  		 
	ADC1_SQR3 &=0XFFFFFFE0;
	ADC1_SQR3 |= 0xe;
    printf("ADC1_SQR3 %08x \n", ADC1_SQR3);
    printf("ADC1_SR %08X \n", ADC1_SR);		  			    
	ADC1_CR2 |= 1<<30;
    printf("adc CR2 %08x \n", ADC1_CR2);
    printf("wait to conv in get adc func\n");
    printf("adc SR %08x \n", ADC1_SR);    
	while(!(ADC1_SR & 1<<1));
    printf("adc end %08x \n", ADC1_DR);	 	   
	return ADC1_DR;		
    	
}



uint16_t stm32_get_adc(uint8_t ch, uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val +=Get_Adc(ch);
		//delay_ms(5);
        usleep(5000);
	}
	return temp_val/times;
}  










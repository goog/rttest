#include <stdlib.h>
#include "spi.h"                  


typedef unsigned char u8;
typedef unsigned short u16;


// pb12
stm32f4_gpio_config spi1_cs =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(1, 12),
	    .pin_last = STM32F4_GPIO_PIN(1, 12),
	    .mode = STM32F4_GPIO_MODE_OUTPUT,
	    .otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	    .ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	    .pupd = STM32F4_GPIO_PULL_UP,
	    .output = 1,
	    .af = 0
        }
};

void printBits(const size_t size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);

            if(j == 4 || j == 0)
                printf(" ");
        }
    }
    puts("");
}


void spi1_init()
{

    //stm32f4_gpio_set_config(&spi1_cs);

    
    printf("init begin, spi CR1 %08x\n", SPI2_CR1);
    RCC_AHB1ENR |= 1<<1;       //PORTB时钟使能    
    RCC_APB1ENR |= 1<<14;      //SPI2时钟使能 

    #if 0
    GPIOA_MODER &= 0xffff03ff;
    GPIOA_MODER |= 0x0000a800;  // GPIO port mode af

    GPIOA_OSPEEDR |= 0x5400;
    GPIOA_PUPDR |= 0X5400;  //pull up
    GPIOA_AFRL |= 0x55500000;  // af spi
    // af5
    #endif
    GPIOB_MODER &= 0x03ffffff;
    GPIOB_MODER |= 0xa8000000;  // GPIO port mode af

    //GPIOB_OSPEEDR |= 0x5400;
    GPIOB_PUPDR |= 0X54000000;  //pull up
    GPIOB_AFRH |= 0x55500000;  // af spi

    //SPI2_I2SCFGR &=~(1<<11);

    RCC_APB2RSTR |= 1<<12;  //reset spi
    RCC_APB2RSTR &= ~(1<<12);
       
    SPI2_CR1 |= 1<<9; //nss Software slave management
    SPI2_CR1 |= 1<<8;  

    SPI2_CR1|=1<<2; //SPI master
    //SPI1_CR1|=0<<11;//8bit数据格式 
    SPI2_CR1|=1<<1;  //CPOL=1
    SPI2_CR1|=1<<0;  //CPHA=1  

    SPI2_CR1 |= 7<<3; //Fsck  125khz
    //SPI1_CR1|=0<<7; //MSBfirst   
    SPI2_CR1 |= 1<<6; //SPI enable
    printf("spi CR1 %08x\n", SPI2_CR1);
    printBits(4, &SPI2_CR1);

    printf("spi SR %08x\n", SPI2_SR);
    printBits(4, &SPI2_SR);

    
    
    SPI1_ReadWriteByte(0xff);     
}   



void spi1_setspeed(u8 SpeedSet)
{
    #if 0
    SpeedSet &=0X07;         //限制范围
    SPI1_CR1&=0XFFC7; 
    SPI1_CR1 |=SpeedSet<<3; //设置SPI1速度  
    SPI1_CR1 |=1<<6;        //SPI设备使能
    #endif
    
} 


u8 SPI1_ReadWriteByte(u8 TxData)
{       
    u16 retry=0;                 
    while((SPI2_SR&1<<1) == 0)  // Transmit buffer empty
    {
        retry++;
        if(retry > 0XFFFE) return 0;

        //printf("spi tx buf nott empty\n");
    }             

    SPI2_DR = TxData;    

    retry=0;                
    while((SPI2_SR&1<<0)==0)
    {
        retry++;
        if(retry>0XFFFE)return 0;

        //printf("spi rx buf is empty\n");
    }                               

    //printf("spi dr: %08x\n", SPI2_DR);
    return (unsigned char)SPI2_DR;
}


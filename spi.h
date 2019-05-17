#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <bsp/io.h>


#define RCC_BASE 0x40023800
//RCC_APB1ENR
#define RCC_APB1ENR  (*(volatile uint32_t *)0x40023840)
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40023844)
#define RCC_APB2RSTR (*(volatile uint32_t *)0x40023824)
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE+0x30))

#define GPIOB_BASE 0x40020400
#define GPIOB_MODER  (*(volatile uint32_t *)GPIOB_BASE)
#define GPIOB_ODR  (*(volatile uint32_t *)(GPIOB_BASE+0x14))
#define GPIOB_PUPDR (*(volatile uint32_t *)(GPIOB_BASE+0x0c))
#define GPIOB_AFRH (*(volatile uint32_t *)(GPIOB_BASE+0x24))
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE+0x08))

#define SPI2_BASE 0x40003800
#define SPI2_CR1 (*(volatile uint32_t *)SPI2_BASE)
#define SPI2_SR (*(volatile uint32_t *)(SPI2_BASE+0x08))
#define SPI2_DR (*(volatile uint32_t *)(SPI2_BASE+0x0C))
#define SPI2_I2SCFGR (*(volatile uint32_t *)(SPI2_BASE+0x1C))


extern stm32f4_gpio_config spi1_cs;
#define SPI_CS_SET()   stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,12), 1)
#define SPI_CS_RESET()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,12), 0)
void spi1_init();
void spi1_setspeed(unsigned     char SpeedSet);
unsigned char SPI1_ReadWriteByte(unsigned char TxData);










#endif
#ifndef __PINS_H
#define __PINS_H

#include <bsp/io.h>


extern stm32f4_gpio_config pwrkeyconfig;

#define PWR_KEY_INIT() stm32f4_gpio_set_config(&pwrkeyconfig)
#define PWR_KEY_ON()   stm32f4_gpio_set_output(STM32F4_GPIO_PIN(3,2), 1)
#define PWR_KEY_OFF()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(3,2), 0)



extern stm32f4_gpio_config pwrenconfig;

#define PWR_ENABLE_INIT()    stm32f4_gpio_set_config(&pwrenconfig)
#define PWR_ENABLE_SET(a)    stm32f4_gpio_set_output(STM32F4_GPIO_PIN(2,5), a)
//#define PWR_KEY_OFF()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(5,9), 0)


//pb8
extern stm32f4_gpio_config iic_scl;

#define SCL_INIT()   stm32f4_gpio_set_config(&iic_scl)
#define SCL_SET()    stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,8), 1)
#define SCL_CLEAR()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,8), 0)


//pb9
extern stm32f4_gpio_config iic_sda;

#define SDA_INIT()   stm32f4_gpio_set_config(&iic_sda)
#define SDA_SET()    stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,9), 1)
#define SDA_CLEAR()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(1,9), 0)


#define STM32_IO_DTR_ON() stm32f4_gpio_set_output(STM32F4_GPIO_PIN(2,8), 1)


#endif

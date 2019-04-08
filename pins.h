

#ifndef __PINS_H
#define __PINS_H

//#define __LONG_LED

#include <stdio.h>
#include <bsp/io.h> //Everything we need is in io.h

//STM32F4 Discovery Board, LED4: PORTD, 12
// f9
stm32f4_gpio_config pwrkeyconfig =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(2, 8),
	.pin_last = STM32F4_GPIO_PIN(2, 8),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 0,
	.af = 0
   }
};

#define PWR_KEY_INIT() stm32f4_gpio_set_config(&pwrkeyconfig)
#define PWR_KEY_ON()   stm32f4_gpio_set_output(STM32F4_GPIO_PIN(2,8), 1)
#define PWR_KEY_OFF()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(2,8), 0)



stm32f4_gpio_config pwrenconfig =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(2, 9),
	.pin_last = STM32F4_GPIO_PIN(2, 9),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};

#define PWR_ENABLE_INIT()    stm32f4_gpio_set_config(&pwrenconfig)
#define PWR_ENABLE_SET(a)    stm32f4_gpio_set_output(STM32F4_GPIO_PIN(2,9), a)
//#define PWR_KEY_OFF()  stm32f4_gpio_set_output(STM32F4_GPIO_PIN(5,9), 0)

#endif

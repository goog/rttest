#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

void  stm32_adc_init(void);
uint16_t stm32_get_adc(uint8_t ch, uint8_t times);



#endif
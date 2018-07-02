#ifndef __M_DRIVER__STM32F1XX__
#define __M_DRIVER__STM32F1XX__
#include "stm32f1xx_hal.h"

#define PULSE_TOTAL 16383
#define PULSE_BASE 64

uint16_t oe_pulse(uint8_t duration);
void oe_pulse_wait();

#endif /* __M_DRIVER__STM32F1XX__ */
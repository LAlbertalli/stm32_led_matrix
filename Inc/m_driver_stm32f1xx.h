#ifndef __M_DRIVER__STM32F1XX__
#define __M_DRIVER__STM32F1XX__
#include "stm32f1xx_hal.h"

#define PULSE_TOTAL 65535
#define PULSE_BASE 256

uint16_t oe_pulse(uint8_t duration);
void oe_pulse_wait();

#endif /* __M_DRIVER__STM32F1XX__ */
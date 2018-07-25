#ifndef __M_DRIVER__STM32F1XX__
#define __M_DRIVER__STM32F1XX__
#include "stm32f1xx_hal.h"

#define PULSE_TOTAL 16383
#define PULSE_BASE 64

uint16_t oe_pulse(uint8_t duration);
void oe_pulse_wait();

void clock_pixel(uint8_t r1, uint8_t g1, uint8_t b1,
    uint8_t r2, uint8_t g2, uint8_t b2);

void clock_row(uint8_t row);

void clock_latch();

#endif /* __M_DRIVER__STM32F1XX__ */
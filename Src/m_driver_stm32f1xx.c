#include "m_driver_stm32f1xx.h"

uint16_t oe_pulse(uint8_t duration){
    assert_param(duration <= PULSE_TOTAL/PULSE_BASE);
    uint16_t pulse_duration = PULSE_TOTAL - duration*PULSE_BASE;
    TIM1->ARR = PULSE_TOTAL; // Set the total duration of the PULSE (On and Off)
    TIM1->CCR1 = pulse_duration;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_OPM;
    TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_OC1M;
    TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_CC1S;
    TIM1->CCMR1 |= TIM_OCMODE_PWM1; // Set PWM1 (inverted mode) because OE is off
    TIM1->CCER &= (uint16_t)~TIM_CCER_CC1P;
    TIM1->CCER |= TIM_OCPOLARITY_HIGH;
    TIM1->CCER = TIM_CCER_CC1E; 
    TIM1->BDTR |= TIM_BDTR_MOE; 
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->ARR = 0;
    return 0;
}

void oe_pulse_wait(){
    while((TIM1->CR1 & TIM_CR1_CEN) != 0)
        asm("NOP");
}

void clock_pixel(uint8_t r1, uint8_t g1, uint8_t b1,
    uint8_t r2, uint8_t g2, uint8_t b2){
    
    uint32_t w_out = 0;
    r1 = r1 ? 0x01 : 0;
    g1 = g1 ? 0x02 : 0;
    b1 = b1 ? 0x04 : 0;
    r2 = r2 ? 0x10 : 0;
    g2 = g2 ? 0x20 : 0;
    b2 = b2 ? 0x40 : 0;

    w_out = (uint32_t)(r1 | g1 | b1 | r2 | g2 | b2 | 0x08) << 3U;
    w_out |= (~w_out) << 16U;
    
    GPIOB->BSRR = w_out;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_6 << 16U;
}

void clock_row(uint8_t row){
    uint32_t w_out = row << 9U;
    w_out |= (~w_out) << 16U;
    
    GPIOA->BSRR = w_out;
}

void clock_latch(){
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U;
    GPIOA->BSRR = GPIO_PIN_15;
}
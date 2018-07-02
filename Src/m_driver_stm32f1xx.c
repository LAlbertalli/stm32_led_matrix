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
#ifndef STM32_LIB_H
#define STM32_LIB_H

#include "stm32f0xx.h"
#include <stdint.h>

/* Function Prototypes */
void init_EXTI(void);
void init_NVIC(void);
void init_Ports(void);
void init_ADC(void);
void init_Tim6(void);
void init_TIM14(void);
void init_PWM(void);
void TIM6_DAC_IRQHandler(void);
int ADC_POT(int pot, int resolution);
int ADC_DATA(void);
void delay_ms(uint32_t counter);
void EXTI0_1_IRQHandler(void);

#endif /* STM32_LIB_H */ 

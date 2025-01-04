/******************************************************************************
 * @file    stm32_lib.c
 * @brief   STM32 library functions
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32_lib.h"
#include "lcd_stm32f0.h"

/**
 * @brief  Initialize External Interrupt
 */
void init_EXTI(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable clock for sys config controller
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // Map PA1 to EXTI1
    EXTI->IMR |= EXTI_IMR_MR1; // Unmask EXTI1 in interrupt mask register
    EXTI->FTSR |= EXTI_FTSR_TR1; // Falling trigger enable for input line 1 (SW1)
}

/**
 * @brief  Initialize GPIO ports
 */
void init_Ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 |
                     GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                     GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER12_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);
}

/**
 * @brief  Initialize NVIC
 */
void init_NVIC(void) {
    NVIC_EnableIRQ(EXTI0_1_IRQn); // Enable EXTI0_1 in NVIC
}

/**
 * @brief  Initialize ADC
 */
void init_ADC(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable ADC clock
    ADC1->CFGR1 |= 0x10; // Set ADC resolution
    ADC1->CHSELR = 0b100000; // Select channel for pot-0
    ADC1->CR |= ADC_CR_ADEN; // Enable ADC
    while ((ADC1->ISR & 0x01) == 0); // Wait for ADC ready
}

/**
 * @brief  Initialize Timer 6
 */
void init_Tim6(void) {
    RCC->APB1ENR |= (1 << 4); // Enable TIM6 clock
    TIM6->PSC = 2829; // Set prescaler
    TIM6->ARR = 2829; // Set auto-reload register
    TIM6->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable counter and auto-reload preload
    NVIC_EnableIRQ(TIM6_IRQn); // Enable TIM6 interrupt
}

/**
 * @brief  Initialize TIM14
 */
void init_TIM14(void) {
    TIM14->PSC = 1;
    TIM14->ARR = 39999;
    TIM14->DIER |= TIM_DIER_UIE;
}

/**
 * @brief  Initialize PWM
 */
void init_PWM(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2 & (GPIO_AF2 << 8)) | (GPIO_AFRH_AFR11_AF2 & (GPIO_AF2 << 12));
    TIM2->ARR = 8000;
    TIM2->PSC = 0;
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief  Timer 6 Interrupt Handler
 */
void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~(1 << 0); // Acknowledge interrupt request
}

/**
 * @brief  ADC Potentiometer Reading
 * @param  pot: Potentiometer number
 * @param  resolution: ADC resolution
 * @retval ADC value
 */
int ADC_POT(int pot, int resolution) {
    ADC1->CFGR1 |= (resolution << 3); // Configure resolution
    ADC1->CHSELR = (1 << (5 + pot)); // Channel select (Pot0 = PA5, Pot1 = PA6)
    ADC1->CR |= (0b1 << 2); // Start conversion
    while ((ADC1->ISR & 0b100) == 0); // Wait for conversion
    return ADC1->DR; // Return value
}

/**
 * @brief  ADC Data Reading
 * @retval ADC data
 */
int ADC_DATA(void) {
    ADC1->CR |= ADC_CR_ADSTART; // Start ADC conversion
    while ((ADC1->ISR & 0b100) == 0); // Wait for end of conversion
    return ADC1->DR; // Return data
}

/**
 * @brief  Millisecond Delay
 * @param  delaylength: Length of delay
 */
void delay_ms(uint32_t delaylength) {
    int counter = delaylength * 735;
    while (counter > 0) {
        counter--;
    }
}

/**
 * @brief  External Interrupt Handler
 */
void EXTI0_1_IRQHandler(void) {
    delay_ms(200); // Debouncing
    EXTI->PR |= EXTI_PR_PR1; // Clear interrupt pending bit
}
















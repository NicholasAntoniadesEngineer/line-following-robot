/******************************************************************************
 * @file    PWM.c
 * @brief   PWM configuration and control for STM32
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32_PWM.h"
#include "delay.h"

/**
 * @brief  Configure GPIO for PWM
 */
void PWM_configure_GPIO(void) 
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock for GPIOB
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // Set PB10 and PB11 to AF
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2 & (0b10 << 8)) | (GPIO_AFRH_AFR11_AF2 & (0b10 << 12)); // Enable AF2 for PB10 and PB11
}

/**
 * @brief  Configure TIM2 for PWM
 */
void PWM_configure_TIM2(void) 
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1 for CH3 and CH4
    TIM2->ARR = 48000; // Set auto-reload register for 1 KHz frequency
    TIM2->PSC = 0; // Set prescaler
    TIM2->CCR3 = 0 * 480; // Set duty cycle for CH3
    TIM2->CCR4 = 20 * 480; // Set duty cycle for CH4
    TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable output compare for CH3 and CH4
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
}

/**
 * @brief  Initialize PWM
 */
void PWM_init(void) 
{
    PWM_configure_GPIO();
    PWM_configure_TIM2();
}

/**
 * @brief  Soft start for PWM
 * @param  max_value: Maximum value for PWM
 * @param  step_delay: Delay between steps
 */
void PWM_softstart(uint32_t max_value, uint32_t step_delay) 
{
    float percent_j;
    uint32_t j;

    TIM2->CCR4 = 0;
    TIM2->CCR3 = 0;

    for (uint32_t i = 1; i <= max_value; i++) 
    {
        percent_j = (i / (float)max_value);
        j = percent_j * 100;
        TIM2->CCR3 = j * 480;
        TIM2->CCR4 = j * 480;

        // Delay to control the speed of the soft start
        delay_ms(step_delay);
    }
    
}

/**
 * @brief  Handle brake
 */
void PWM_handle_brake(void) 
{
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    GPIOB->ODR = 0;
    GPIOB->ODR |= 0b10000000;
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    init_TIM14();
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
}

/**
 * @brief  Handle drive
 */
void PWM_handle_drive(void) 
{
    GPIOB->ODR = 0b01;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

/**
 * @brief  Handle soft start
 */
void PWM_handle_softstart(void) 
{
    GPIOB->ODR = 0b101;
    PWM_softstart(100, 10);
}

/**
 * @brief  Handle reverse operation
 */
void PWM_handle_reverse(void) 
{
    GPIOB->ODR = 0b100;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

/**
 * @brief  Potentiometer PWM control
 * @retval Status
 */
int PWM_pot_control(void) 
{
    while (1) 
    {
        float P0_val = ADC_POT(0, 8);
        float P1_val = ADC_POT(1, 8);
        TIM2->CCR3 = (P0_val / 255) * 80000;
        TIM2->CCR4 = (P1_val / 255) * 80000;

        if (!(GPIOA->IDR & (GPIO_IDR_0 | GPIO_IDR_1 | GPIO_IDR_2 | GPIO_IDR_3))) 
        {
            return 0;
        }
        
    }
}

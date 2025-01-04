/******************************************************************************
 * @file    main.c
 * @brief   Main program for STM32 line-following car
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdint.h>
#include "stm32_lib.h"

/* Global Constants ----------------------------------------------------------*/
uint16_t centi_sec = 0;

/**
 * @brief  TIM14 interrupt handler
 */
void TIM14_IRQHandler(void) {
    TIM14->SR &= ~TIM_SR_UIF;
    if (++centi_sec == 25) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
        centi_sec = 0;
    }
}

/**
 * @brief  Main function
 * @retval int
 */
int main(void) {
    init_Ports();
    init_ADC();
    init_LCD();
    init_PWM();
    init_NVIC();

    lcd_command(CLEAR);
    lcd_putstring("SW0: Brakes");
    lcd_command(LINE_TWO);
    lcd_putstring("SW1: Reactivate");

    while (1) 
    {
        PWM_pot_control();

        if (!(GPIOA->IDR & GPIO_IDR_0)) 
        {
            PWM_handle_brake();

            lcd_command(CLEAR);
            lcd_putstring("     Brake");
            lcd_command(LINE_TWO);
            lcd_putstring("    Activated");

        } else if (!(GPIOA->IDR & GPIO_IDR_1)) 
        {
            PWM_handle_drive();

        } else if (!(GPIOA->IDR & GPIO_IDR_2)) 
        {
            lcd_command(CLEAR);
            lcd_putstring("    Softstart");
            lcd_command(LINE_TWO);
            lcd_putstring("    Activated");

            PWM_handle_softstart();
            PWM_handle_drive();

            lcd_command(CLEAR);
            lcd_putstring("Driving motor...");

        } else if (!(GPIOA->IDR & GPIO_IDR_3)) 
        {
            lcd_command(CLEAR);
            lcd_putstring("     Reverse");  
            
            PWM_handle_reverse();
        }
    }

    return 0;
}

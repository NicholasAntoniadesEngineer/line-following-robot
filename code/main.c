/******************************************************************************
 * @file    main.c
 * @brief   Main program for STM32 line-following car
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables */
static volatile uint8_t centi_sec = 0;

/**
 * @brief Initialize all STM32 peripherals and subsystems
 */
static void init_stm32(void) 
{
    init_Ports();
    init_ADC();
    init_LCD();
    init_PWM();
    init_NVIC();
}

/**
 * @brief Display message on LCD
 * @param line1 Text for first line
 * @param line2 Text for second line
 */
static void lcd_display(const char* line1, const char* line2) 
{
    lcd_command(CLEAR);
    lcd_putstring(line1);

    lcd_command(LINE_TWO);
    lcd_putstring(line2);
}

/**
 * @brief Handle robot state transitions based on button inputs
 */
static void robot_state_machine(void) 
{
    if (!(GPIOA->IDR & BRAKE_PIN)) 
    {
        stm32_pwm_handle_brake();
        lcd_display("     Brake", "    Activated");
    } 
    else if (!(GPIOA->IDR & DRIVE_PIN)) 
    {
        stm32_pwm_handle_drive();
    } 
    else if (!(GPIOA->IDR & SOFTSTART_PIN)) 
    {
        lcd_display("    Softstart", "    Activated");
        stm32_pwm_handle_softstart();
        stm32_pwm_handle_drive();
        lcd_display("Driving motor...", "");
    } 
    else if (!(GPIOA->IDR & REVERSE_PIN)) 
    {
        lcd_display("     Reverse", "");
        stm32_pwm_handle_reverse();
    }
}

/**
 * @brief  Main function
 * @retval int
 */
int main(void) 
{
    init_stm32();
    
    lcd_display("SW0: Brakes", "SW1: Reactivate");

    while (1) 
    {
        stm32_pwm_set_speed();
        robot_state_machine();
    }
    return 0;
}

/**
 * @brief  TIM14 interrupt handler
 */
void TIM14_IRQHandler(void) 
{
    TIM14->SR &= ~TIM_SR_UIF;
    if (++centi_sec == CENTI_SEC_MAX) 
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
        centi_sec = 0;
    }
}

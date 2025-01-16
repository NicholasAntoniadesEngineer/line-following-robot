/******************************************************************************
 * @file    main.c
 * @brief   Main program for STM32 line-following car
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32_lib.h"
#include "stm32_pwm.h"

/* Global variables */
static volatile uint8_t centi_sec = 0;

/**
 * @brief Initialize all STM32 peripherals and subsystems with config
 */
static void init_stm32(const stm32_system_config_t *config) 
{
    if (!config) return;
    
    // Initialize the system with configuration
    stm32_lib_init(config);
    init_LCD();
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
    // Create system configuration
    stm32_system_config_t system_config = {
        .adc_states = (stm32_adc_state_t[2]){
            {   // Left sensor
                .channel = 0,
                .resolution = 8,
                .sample_time = ADC_SAMPLETIME_1CYCLE_5,
                .conversion_mode = ADC_CONVERSIONMODE_SINGLE
            },
            {   // Right sensor
                .channel = 1,
                .resolution = 8,
                .sample_time = ADC_SAMPLETIME_1CYCLE_5,
                .conversion_mode = ADC_CONVERSIONMODE_SINGLE
            }
        },
        .num_adc_channels = 2,
        .pwm_states = (stm32_pwm_state_t[2]){
            {   // Left motor
                .timer = &htim2,
                .channel = TIM_CHANNEL_3,
                .frequency = 48000,
                .duty_cycle = 50,
                .period = 80000
            },
            {   // Right motor
                .timer = &htim2,
                .channel = TIM_CHANNEL_4,
                .frequency = 48000,
                .duty_cycle = 50,
                .period = 80000
            }
        },
        .num_pwm_channels = 2,
        .port_state = {
            .pins_a = (pin_config_t[]){
                { .pin = 0, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Brake button
                { .pin = 1, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Drive button
                { .pin = 2, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Softstart button
                { .pin = 3, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP }    // Reverse button
            },
            .num_pins_a = 4,
            .pins_b = (pin_config_t[]){
                { .pin = 10, .mode = GPIO_MODE_AF, .pull = GPIO_NOPULL },     // PWM output
                { .pin = 11, .mode = GPIO_MODE_AF, .pull = GPIO_NOPULL }      // PWM output
            },
            .num_pins_b = 2
        }
    };

    // Initialize all STM32 peripherals and subsystems
    init_stm32(&system_config);

    // Display initial message on LCD
    lcd_display("SW0: Brakes", "SW1: Reactivate");

    while (1) 
    {
        // Read ADC values from IR sensors
        stm32_lib_adc_input(system_config.adc_states[0].channel, system_config.adc_states[0].resolution);
        system_config.adc_states[0].current_value = stm32_lib_adc_data();
        
        stm32_lib_adc_input(system_config.adc_states[1].channel, system_config.adc_states[1].resolution);
        system_config.adc_states[1].current_value = stm32_lib_adc_data();

        // Update PWM values based on ADC readings
        system_config.pwm_states[0].pulse = (system_config.adc_states[0].current_value / ((1 << system_config.adc_states[0].resolution) - 1)) * system_config.pwm_states[0].period;
        system_config.pwm_states[1].pulse = (system_config.adc_states[1].current_value / ((1 << system_config.adc_states[1].resolution) - 1)) * system_config.pwm_states[1].period;
        
        // Update timer compare values
        TIM2->CCR3 = system_config.pwm_states[0].pulse;
        TIM2->CCR4 = system_config.pwm_states[1].pulse;

        // Handle state transitions based on button inputs
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

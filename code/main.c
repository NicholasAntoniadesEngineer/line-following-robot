/******************************************************************************
 * @file    main.c
 * @brief   Main program for STM32 line-following car
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"
#include "robot_control.h"

/**
 * @brief Set up system state
 */
static void _init_state(robot_control_state_t *state)
{
    if (!state) return;

    // Configure ADC states
    state->adc_states = (stm32_adc_state_t[2]){
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
    };
    state->num_adc_channels = 2;

    // Configure PWM states
    state->pwm_states = (stm32_pwm_state_t[2]){
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
    };
    state->num_pwm_channels = 2;

    // Configure port states
    state->port_state.pins_a = (pin_config_t[]){
        { .pin = 0, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Brake button
        { .pin = 1, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Drive button
        { .pin = 2, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP },   // Softstart button
        { .pin = 3, .mode = GPIO_MODE_INPUT, .pull = GPIO_PULLUP }    // Reverse button
    };
    state->port_state.num_pins_a = 4;

    state->port_state.pins_b = (pin_config_t[]){
        { .pin = 10, .mode = GPIO_MODE_AF, .pull = GPIO_NOPULL },     // PWM output
        { .pin = 11, .mode = GPIO_MODE_AF, .pull = GPIO_NOPULL }      // PWM output
    };
    state->port_state.num_pins_b = 2;

    // Configure button states
    state->button_state.brake_pin = BRAKE_PIN;
    state->button_state.drive_pin = DRIVE_PIN;
    state->button_state.softstart_pin = SOFTSTART_PIN;
    state->button_state.reverse_pin = REVERSE_PIN;

    // Configure softstart parameters
    state->softstart_state.max_value = 100;
    state->softstart_state.step_delay = 50;
}

/**
 * @brief  Main function
 * @retval int
 */
int main(void) 
{
    // Create and initialize system state
    robot_control_state_t system_state;
    _init_state(&system_state);
    
    robot_control_system_init(&system_state);

    while (1) 
    {
        // Read sensor values
        robot_control_read_sensors(&system_state);

        // Update motor speeds based on sensor readings
        robot_control_update_motor_speeds(&system_state);

        // Handle state transitions based on button inputs
        robot_control_state_machine(&system_state);
    }
    return 0;
}

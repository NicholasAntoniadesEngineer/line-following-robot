/******************************************************************************
 * @file    robot_control.c
 * @brief   Robot control functions for line following car
 ******************************************************************************/

#include "robot_control.h"
#include "stm32_lib.h"

/**
 * @brief Initialize STM32 peripherals for the robot
 * @param state Pointer to system state
 */
void robot_control_stm32_init(const stm32_system_state_t *state)
{
    if (!state) return;

    // Initialize ports with pin configurations
    stm32_lib_port_init(&state->port_state);

    // Initialize ADC for sensors
    stm32_lib_init_adc(state->adc_states, state->num_adc_channels);

    // Initialize PWM for each motor
    for (uint8_t i = 0; i < state->num_pwm_channels; i++) 
    {
        stm32_lib_init_pwm_tim(state->pwm_states[i].timer, 
                              state->pwm_states[i].channel, 
                              state->pwm_states[i].frequency);
    }

    // Initialize LCD
    stm32_lib_init_lcd();

    // Initialize interrupts
    stm32_lib_init_nvic();
}

/**
 * @brief Handle brake state
 * @param state Pointer to system state
 */
void robot_control_handle_brake(const stm32_system_state_t *state)
{
    if (!state) return;

    // Stop both motors
    for (uint8_t i = 0; i < state->num_pwm_channels; i++)
    {
        stm32_lib_timer_set_compare(state->pwm_states[i].timer,
                                  state->pwm_states[i].channel,
                                  0);
    }

    // Set brake pin
    GPIOB->ODR = 0;
    GPIOB->ODR |= 0b10000000;

    // Disable PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    stm32_lib_init_tim6(0, 0);  // Initialize timer with 0 values
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
}

/**
 * @brief Handle drive state
 * @param state Pointer to system state
 */
void robot_control_handle_drive(const stm32_system_state_t *state)
{
    if (!state) return;

    GPIOB->ODR = 0b01;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

/**
 * @brief Handle soft start
 * @param state Pointer to system state
 * @param max_value Maximum PWM value
 * @param step_delay Delay between steps
 */
void robot_control_handle_softstart(const stm32_system_state_t *state, uint32_t max_value, uint32_t step_delay)
{
    if (!state) return;

    GPIOB->ODR = 0b101;

    // Gradually increase PWM values
    for (uint32_t i = 1; i <= max_value; i++) 
    {
        float percent = i / (float)max_value;
        uint32_t pwm_value = (percent * 100) * 480;

        // Update both motors
        for (uint8_t j = 0; j < state->num_pwm_channels; j++) 
        {
            stm32_lib_timer_set_compare(state->pwm_states[j].timer,
                                      state->pwm_states[j].channel, 
                                      pwm_value);
        }

        stm32_lib_delay(step_delay);
    }
}

/**
 * @brief Handle reverse state
 * @param state Pointer to system state
 */
void robot_control_handle_reverse(const stm32_system_state_t *state)
{
    if (!state) return;

    GPIOB->ODR = 0b100;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

/**
 * @brief Read sensor values and update ADC states
 * @param state Pointer to system state
 */
void robot_control_read_sensors(const stm32_system_state_t *state)
{
    if (!state) return;

    // Read ADC values from IR sensors
    for (uint8_t i = 0; i < state->num_adc_channels; i++) 
    {
        stm32_lib_adc_input(state->adc_states[i].channel, 
                           state->adc_states[i].resolution);
        state->adc_states[i].current_value = stm32_lib_adc_data();
    }
}

/**
 * @brief Update motor speeds based on current ADC values
 * @param state Pointer to system state
 */
void robot_control_update_motor_speeds(const stm32_system_state_t *state)
{
    if (!state) return;

    // Update PWM values based on ADC readings
    for (uint8_t i = 0; i < state->num_pwm_channels; i++) 
    {
        state->pwm_states[i].pulse = (state->adc_states[i].current_value / 
                                    ((1 << state->adc_states[i].resolution) - 1)) * 
                                    state->pwm_states[i].period;

        // Update timer compare values
        stm32_lib_timer_set_compare(state->pwm_states[i].timer,
                                  state->pwm_states[i].channel,
                                  state->pwm_states[i].pulse);
    }
}

/**
 * @brief Handle robot state transitions based on button inputs
 * @param state Pointer to system state
 * @param display_callback Function pointer for LCD display updates
 */
void robot_control_state_machine(const stm32_system_state_t *state, 
                               void (*display_callback)(const char*, const char*))
{
    if (!state || !display_callback) return;

    // Check brake button
    if (!(GPIOA->IDR & BRAKE_PIN)) 
    {
        robot_control_handle_brake(state);
        display_callback("     Brake", "    Activated");
    } 
    // Check drive button
    else if (!(GPIOA->IDR & DRIVE_PIN)) 
    {
        robot_control_handle_drive(state);
    } 
    // Check softstart button
    else if (!(GPIOA->IDR & SOFTSTART_PIN)) 
    {
        display_callback("    Softstart", "    Activated");
        robot_control_handle_softstart(state, 100, 10);
        robot_control_handle_drive(state);
        display_callback("Driving motor...", "");
    } 
    // Check reverse button
    else if (!(GPIOA->IDR & REVERSE_PIN)) 
    {
        display_callback("     Reverse", "");
        robot_control_handle_reverse(state);
    }
}

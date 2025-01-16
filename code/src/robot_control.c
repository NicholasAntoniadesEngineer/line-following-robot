/******************************************************************************
 * @file    robot_control.c
 * @brief   Robot control functions for line following car
 ******************************************************************************/

#include "robot_control.h"
#include "stm32_lib.h"
#include "lcd_stm32f0.h"

/**
 * @brief Initialize STM32 peripherals for the robot
 * @param state Pointer to system state
 */
void robot_control_stm32_init(const robot_control_state_t *state)
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

    // Initialize LCD and display initial message
    stm32_lib_init_lcd();
    lcd_command(CLEAR);
    lcd_putstring("SW0: Brakes");
    lcd_command(LINE_TWO);
    lcd_putstring("SW1: Reactivate");

    // Initialize interrupts
    stm32_lib_init_nvic();
}

/**
 * @brief Handle brake state
 * @param state Pointer to system state
 */
void robot_control_handle_brake(const robot_control_state_t *state)
{
    if (!state) return;

    // Stop both motors
    for (uint8_t i = 0; i < state->num_pwm_channels; i++)
    {
        stm32_lib_timer_set_compare(state->pwm_states[i].timer,
                                  state->pwm_states[i].channel,
                                  0);
    }

    // Set brake pin using state configuration
    stm32_lib_gpio_set_port(state->port_state.brake_port;, 0);
    stm32_lib_gpio_set_pin( state->port_state.brake_port;, state->port_state.brake_pin, 1);

    // Configure timers using state configuration
    stm32_lib_timer_enable(state->timer_state.control_timer);
    stm32_lib_timer_init(state->timer_state.pwm_timer, 0, 0);
    stm32_lib_timer_disable(state->timer_state.motor_timer);

    // Update LCD display
    lcd_command(CLEAR);
    lcd_putstring("     Brake");
    lcd_command(LINE_TWO);
    lcd_putstring("    Activated");
}

/**
 * @brief Handle drive state
 * @param state Pointer to system state
 */
void robot_control_handle_drive(const robot_control_state_t *state)
{
    if (!state) return;

    // Use state configuration for GPIO and timer settings
    stm32_lib_gpio_set_port(state->port_state.control_port, state->port_state.drive_pattern);
    stm32_lib_timer_enable(state->timer_state.motor_timer);

    // Update LCD display
    lcd_command(CLEAR);
    lcd_putstring("Driving motor...");
}

/**
 * @brief Handle soft start
 * @param state Pointer to system state
 * @param max_value Maximum PWM value
 * @param step_delay Delay between steps
 */
void robot_control_handle_softstart(const robot_control_state_t *state, uint32_t max_value, uint32_t step_delay)
{
    if (!state) return;

    // Update LCD display
    lcd_command(CLEAR);
    lcd_putstring("    Softstart");
    lcd_command(LINE_TWO);
    lcd_putstring("    Activated");

    // Set control pattern from state
    stm32_lib_gpio_set_port(state->port_state.control_port, state->port_state.softstart_pattern);

    // Gradually increase PWM values
    for (uint32_t i = 1; i <= max_value; i++) 
    {
        float percent = i / (float)max_value;
        uint32_t pwm_value = (percent * state->pwm_states[0].period);

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
void robot_control_handle_reverse(const robot_control_state_t *state)
{
    if (!state) return;

    // Set reverse pattern from state
    stm32_lib_gpio_set_port(state->port_state.control_port, state->port_state.reverse_pattern);
    stm32_lib_timer_enable(state->timer_state.motor_timer);

    // Update LCD display
    lcd_command(CLEAR);
    lcd_putstring("     Reverse");
}

/**
 * @brief Read sensor values and update ADC states
 * @param state Pointer to system state
 */
void robot_control_read_sensors(const robot_control_state_t *state)
{
    if (!state) return;

    // Read ADC values from IR sensors
    for (uint8_t i = 0; i < state->num_adc_channels; i++) 
    {
        stm32_lib_adc_input(state->adc_states[i].channel, state->adc_states[i].resolution);
        state->adc_states[i].current_value = stm32_lib_adc_data();
    }
}

/**
 * @brief Update motor speeds based on current ADC values
 * @param state Pointer to system state
 */
void robot_control_update_motor_speeds(const robot_control_state_t *state)
{
    if (!state) return;

    // Update PWM values based on ADC readings
    for (uint8_t i = 0; i < state->num_pwm_channels; i++) 
    {
        state->pwm_states[i].pulse = (state->adc_states[i].current_value / ((1 << state->adc_states[i].resolution) - 1)) * state->pwm_states[i].period;

        // Update timer compare values
        stm32_lib_timer_set_compare(state->pwm_states[i].timer,
                                  state->pwm_states[i].channel,
                                  state->pwm_states[i].pulse);
    }
}

/**
 * @brief Handle robot state transitions based on button inputs
 * @param state Pointer to system state
 */
void robot_control_state_machine(const robot_control_state_t *state)
{
    if (!state) return;

    // Check buttons using state configuration
    if (stm32_lib_check_button(GPIOA, state->button_state.brake_pin)) 
    {
        robot_control_handle_brake(state);
        lcd_command(CLEAR);
        lcd_putstring("     Brake");
        lcd_command(LINE_TWO);
        lcd_putstring("    Activated");
    } 
    else if (stm32_lib_check_button(GPIOA, state->button_state.drive_pin)) 
    {
        robot_control_handle_drive(state);
        lcd_command(CLEAR);
        lcd_putstring("Driving motor...");
    } 
    else if (stm32_lib_check_button(GPIOA, state->button_state.softstart_pin)) 
    {
        lcd_command(CLEAR);
        lcd_putstring("    Softstart");
        lcd_command(LINE_TWO);
        lcd_putstring("    Activated");
        robot_control_handle_softstart(state, state->softstart_state.max_value, state->softstart_state.step_delay);
        robot_control_handle_drive(state);
        lcd_command(CLEAR);
        lcd_putstring("Driving motor...");
    } 
    else if (stm32_lib_check_button(GPIOA, state->button_state.reverse_pin)) 
    {
        robot_control_handle_reverse(state);
        lcd_command(CLEAR);
        lcd_putstring("     Reverse");
    }
}

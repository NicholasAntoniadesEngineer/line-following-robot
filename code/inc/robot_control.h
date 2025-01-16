/******************************************************************************
 * @file    robot_control.h
 * @brief   Robot control functions for line following car
 ******************************************************************************/

#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include "stm32_lib.h"

/* State Structures */
typedef struct {
    uint8_t brake_pin;
    uint8_t drive_pin;
    uint8_t softstart_pin;
    uint8_t reverse_pin;
} robot_button_state_t;

typedef struct {
    uint32_t max_value;
    uint32_t step_delay;
} robot_softstart_state_t;

typedef struct {
    stm32_port_state_t port_state;
    stm32_adc_state_t *adc_states;
    uint8_t num_adc_channels;
    stm32_pwm_state_t *pwm_states;
    uint8_t num_pwm_channels;
    stm32_timer_state_t timer_state;
    robot_button_state_t button_state;
    robot_softstart_state_t softstart_state;
} robot_control_state_t;

/* Function Prototypes */
/**
 * @brief Initialize STM32 peripherals for the robot
 * @param state Pointer to system state
 */
void robot_control_system_init(const robot_control_state_t *state);
void robot_control_handle_brake(const robot_control_state_t *state);
void robot_control_handle_drive(const robot_control_state_t *state);
void robot_control_handle_softstart(const robot_control_state_t *state, uint32_t max_value, uint32_t step_delay);
void robot_control_handle_reverse(const robot_control_state_t *state);
void robot_control_read_sensors(const robot_control_state_t *state);
void robot_control_update_motor_speeds(const robot_control_state_t *state);
void robot_control_state_machine(const robot_control_state_t *state);

#endif /* ROBOT_CONTROL_H_ */ 
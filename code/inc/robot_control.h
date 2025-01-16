/******************************************************************************
 * @file    robot_control.h
 * @brief   Robot control functions for line following car
 ******************************************************************************/

#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include "stm32_lib.h"

/* Function Prototypes */
/**
 * @brief Initialize STM32 peripherals for the robot
 * @param state Pointer to system state
 */
void robot_control_stm32_init(const stm32_system_state_t *state);

void robot_control_handle_brake(const stm32_system_state_t *state);
void robot_control_handle_drive(const stm32_system_state_t *state);
void robot_control_handle_softstart(const stm32_system_state_t *state, uint32_t max_value, uint32_t step_delay);
void robot_control_handle_reverse(const stm32_system_state_t *state);
void robot_control_read_sensors(const stm32_system_state_t *state);
void robot_control_update_motor_speeds(const stm32_system_state_t *state);
void robot_control_state_machine(const stm32_system_state_t *state,
                                 void (*display_callback)(const char*, const char*));

#endif /* ROBOT_CONTROL_H_ */ 
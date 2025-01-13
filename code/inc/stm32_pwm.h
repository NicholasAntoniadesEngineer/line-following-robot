/******************************************************************************
 * @file    stm32_pwm.h
 * @brief   PWM configuration and control for STM32 - Header file
 ******************************************************************************/

#ifndef STM32_PWM_H
#define STM32_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Function Prototypes -------------------------------------------------------*/
void stm32_pwm_configure_gpio(void);
void stm32_pwm_configure_tim2(void);
void stm32_pwm_init(void);
void stm32_pwm_softstart(uint32_t max_value, uint32_t step_delay);
void stm32_pwm_handle_brake(void);
void stm32_pwm_handle_drive(void);
void stm32_pwm_handle_softstart(void);
void stm32_pwm_handle_reverse(void);
int stm32_pwm_set_speed(void);

#endif /* STM32_PWM_H */ 

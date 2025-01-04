/******************************************************************************
 * @file    PWM.h
 * @brief   PWM configuration and control for STM32 - Header file
 * @version 1.0
 ******************************************************************************/

#ifndef STM32_PWM_H
#define STM32_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Function Prototypes -------------------------------------------------------*/
void PWM_configure_GPIO(void);
void PWM_configure_TIM2(void);
void PWM_init(void);
void PWM_softstart(uint32_t max_value, uint32_t step_delay);
void PWM_handle_brake(void);
void PWM_handle_drive(void);
void PWM_handle_softstart(void);
void PWM_handle_reverse(void);
int PWM_pot_control(void);

#endif /* STM32_PWM_H */ 
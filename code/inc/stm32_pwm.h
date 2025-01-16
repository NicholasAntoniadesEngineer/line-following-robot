/******************************************************************************
 * @file    stm32_pwm.h
 * @brief   PWM configuration and control for STM32 - Header file
 ******************************************************************************/

#ifndef STM32_PWM_H
#define STM32_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* PWM State Structure */
typedef struct {
    uint32_t timer_channel;     /* Timer channel number */
    uint32_t pwm_frequency;     /* PWM frequency in Hz */
    uint32_t pwm_duty_cycle;    /* PWM duty cycle (0-100) */
    float adc_value;            /* Current ADC reading */
    uint32_t max_pwm_value;     /* Maximum PWM value */
    uint8_t adc_channel;        /* ADC channel to read from */
    uint8_t adc_resolution;     /* ADC resolution in bits */
} stm32_pwm_state_t;

/* Function Prototypes -------------------------------------------------------*/
void stm32_pwm_configure_gpio(void);
void stm32_pwm_configure_tim2(void);
void stm32_pwm_init(void);
void stm32_pwm_softstart(uint32_t max_value, uint32_t step_delay);
void stm32_pwm_handle_brake(void);
void stm32_pwm_handle_drive(void);
void stm32_pwm_handle_softstart(void);
void stm32_pwm_handle_reverse(void);

/* New modular functions */
void stm32_pwm_read_adc(stm32_pwm_state_t *state);
void stm32_pwm_update_speed(stm32_pwm_state_t *state);

#endif /* STM32_PWM_H */ 

/*
 * stm32_lib.h
 *
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

#ifndef STM32_LIB_H_
#define STM32_LIB_H_

#include <stdint.h>
#define STM32F051
#include "stm32f0xx.h"

// Pin masks
#define PA8_MASK  0x0100
#define PA9_MASK  0x0200
#define PA10_MASK 0x0400
#define PB12_MASK 0x1000
#define PB13_MASK 0x2000
#define PB14_MASK 0x4000
#define PB15_MASK 0x8000

// Signal generator state structure
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t frequency;
    uint32_t amplitude;
    uint32_t offset;
    uint32_t channel;
} stm32_sig_gen_state_t;

// UART state structure
typedef struct {
    UART_HandleTypeDef *huart;
    uint16_t rx_size;
    uint16_t tx_size;
    uint32_t uart_timeout;
} stm32_uart_state_t;

// Port state structure
typedef struct {
    pin_config_t *pins_a;
    int num_pins_a;
    pin_config_t *pins_b;
    int num_pins_b;
} stm32_port_state_t;

// ADC State Structure
typedef struct {
    uint8_t channel;            /* ADC channel number */
    uint8_t resolution;         /* ADC resolution in bits */
    float current_value;        /* Current ADC reading */
    uint32_t sample_time;       /* ADC sample time */
    uint32_t conversion_mode;   /* Single or continuous conversion */
} stm32_adc_state_t;

// PWM State Structure
typedef struct {
    TIM_HandleTypeDef *timer;   /* Timer handle */
    uint32_t channel;           /* Timer channel number */
    uint32_t frequency;         /* PWM frequency in Hz */
    uint32_t duty_cycle;        /* PWM duty cycle (0-100) */
    uint32_t period;            /* Timer period */
    uint32_t pulse;             /* Timer pulse value */
} stm32_pwm_state_t;

// System Configuration Structure
typedef struct {
    stm32_adc_state_t *adc_states;    /* Array of ADC configurations */
    uint8_t num_adc_channels;         /* Number of ADC channels to configure */
    stm32_pwm_state_t *pwm_states;    /* Array of PWM configurations */
    uint8_t num_pwm_channels;         /* Number of PWM channels to configure */
    stm32_port_state_t port_state;    /* Port configuration */
} stm32_system_config_t;

// Function prototypes
void stm32_lib_port_init(void);
void stm32_lib_init_pwm_tim(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);
void stm32_lib_init_pwm(int frequency);
void stm32_lib_init_adc(const stm32_adc_state_t *adc_states, uint8_t num_channels);
void stm32_lib_init_usart(UART_HandleTypeDef *huart);
void stm32_lib_init_usart1(void);
void stm32_lib_init_tim6(uint32_t arr, uint32_t psc);
int stm32_lib_adc_input(int pot, int resolution);
void stm32_lib_adc_awd_8bit(int ADC_channel, int ADC_Low_threshhold, int ADC_High_threshhold);
int stm32_lib_adc_awd_check(void);
int stm32_lib_adc_data(void);
void stm32_lib_delay(int time);
int stm32_lib_check_button_gpioa(int button);
int stm32_lib_check_button_gpiob(int button);
void stm32_lib_debounce(void);
void stm32_lib_tim6_set_psc(uint32_t psc);
void stm32_lib_tim6_set_arr(uint32_t arr);
void stm32_lib_ack_irq(void);
void stm32_lib_init_exti(void);
void stm32_lib_usart1_transmit(unsigned char DataToTx);
unsigned char stm32_lib_usart1_receive(void);
void stm32_lib_sig_gen_init(stm32_sig_gen_state_t *state);
void stm32_lib_port_init(stm32_uart_state_t *uart_state, const stm32_uart_state_t *config);
void stm32_lib_init(const stm32_system_config_t *config);

#endif /* STM32_LIB_H_ */ 
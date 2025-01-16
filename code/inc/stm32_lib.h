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
#include <stm32f0xx_exti.h>
#include "stm32f051x8.h"

// Pin masks
#define PA8_MASK  0x0100
#define PA9_MASK  0x0200
#define PA10_MASK 0x0400
#define PB12_MASK 0x1000
#define PB13_MASK 0x2000
#define PB14_MASK 0x4000
#define PB15_MASK 0x8000

// State structures
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t frequency;
    uint32_t amplitude;
    uint32_t offset;
    uint32_t channel;
} stm32_sig_gen_state_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint16_t rx_size;
    uint16_t tx_size;
    uint32_t uart_timeout;
} stm32_uart_state_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t mode;
    uint32_t pull;
} stm32_pin_config_t;

typedef struct {
    stm32_pin_config_t *pins;
    uint8_t num_pins;
} stm32_port_state_t;

typedef struct {
    TIM_TypeDef *timer;
    uint32_t prescaler;
    uint32_t period;
    uint32_t clock_division;
} stm32_timer_state_t;

typedef struct {
    uint8_t channel;            /* ADC channel number */
    uint8_t resolution;         /* ADC resolution in bits */
    float current_value;        /* Current ADC reading */
    uint32_t sample_time;       /* ADC sample time */
    uint32_t conversion_mode;   /* Single or continuous conversion */
} stm32_adc_state_t;

typedef struct {
    TIM_HandleTypeDef *timer;   /* Timer handle */
    uint32_t channel;           /* Timer channel number */
    uint32_t frequency;         /* PWM frequency in Hz */
    uint32_t duty_cycle;        /* PWM duty cycle (0-100) */
    uint32_t period;            /* Timer period */
    uint32_t pulse;             /* Timer pulse value */
} stm32_pwm_state_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t active_low;
    uint32_t debounce_time;
} stm32_button_state_t;

// Function prototypes

// Initialization functions
void stm32_lib_port_init(const stm32_port_state_t *port_state);
void stm32_lib_init_pwm_tim(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);
void stm32_lib_init_adc(const stm32_adc_state_t *adc_states, uint8_t num_channels);
void stm32_lib_init_usart(UART_HandleTypeDef *huart);
void stm32_lib_init_timer(TIM_TypeDef* timer, uint32_t arr, uint32_t psc);
void stm32_lib_init_lcd(void);
void stm32_lib_init_nvic(void);
void stm32_lib_init_exti(void);

// GPIO functions
void stm32_lib_gpio_set_pin(GPIO_TypeDef* gpio, uint16_t pin, uint8_t state);
void stm32_lib_gpio_set_port(GPIO_TypeDef* gpio, uint16_t value);
uint8_t stm32_lib_gpio_read_pin(GPIO_TypeDef* gpio, uint16_t pin);
uint16_t stm32_lib_gpio_read_port(GPIO_TypeDef* gpio);

// Timer functions
void stm32_lib_timer_enable(TIM_TypeDef* timer);
void stm32_lib_timer_disable(TIM_TypeDef* timer);
void stm32_lib_timer_init(TIM_TypeDef* timer, uint32_t arr, uint32_t psc);
void stm32_lib_timer_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
void stm32_lib_tim6_set_psc(uint32_t psc);
void stm32_lib_tim6_set_arr(uint32_t arr);

// ADC functions
uint32_t stm32_lib_adc_read_channel(uint8_t channel, uint8_t resolution);
void stm32_lib_adc_set_watchdog(uint8_t channel, uint32_t low_threshold, uint32_t high_threshold);
uint8_t stm32_lib_adc_check_watchdog(void);
int stm32_lib_adc_data(void);

// Button functions
uint8_t stm32_lib_button_read(const stm32_button_state_t *button);
void stm32_lib_button_debounce(uint32_t delay_ms);
int stm32_lib_check_button(GPIO_TypeDef* gpio, int button);

// UART functions
void stm32_lib_uart_init(stm32_uart_state_t *uart_state, const stm32_uart_state_t *config);
void stm32_lib_usart1_transmit(unsigned char DataToTx);
unsigned char stm32_lib_usart1_receive(void);

// Signal generator functions
void stm32_lib_sig_gen_init(stm32_sig_gen_state_t *state, const stm32_sig_gen_state_t *config);

// Crystal control functions
void stm32_lib_lock_crystal(void);
void stm32_lib_unlock_crystal(void);

// Utility functions
void stm32_lib_delay(int time);
void stm32_lib_ack_irq(void);

#endif /* STM32_LIB_H_ */ 
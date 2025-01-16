/**
  ******************************************************************************
  * @file           : stm32_bsp.h
  * @brief          : Board Support Package header file
  * @details        : Encapsulates hardware-specific functions for STM32
  * @date           : 2024
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#ifndef STM32_BSP_H_
#define STM32_BSP_H_

#define STM32F051
#include "stm32f0xx.h"
#include <stm32f0xx_exti.h>
#include "stm32f051x8.h"
#include <stdint.h>

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/
#define BSP_HMI_BUFFER_SIZE         12U
#define BSP_NUM_ADC_CHANNELS        3U
#define BSP_UART_TX_COMPLETE_FLAG   255U

// Pin masks
#define PA8_MASK  0x0100
#define PA9_MASK  0x0200
#define PA10_MASK 0x0400
#define PB12_MASK 0x1000
#define PB13_MASK 0x2000
#define PB14_MASK 0x4000
#define PB15_MASK 0x8000

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
typedef struct {
    uint8_t hmiTxBuffer[BSP_HMI_BUFFER_SIZE];
    uint8_t hmiRxBuffer[BSP_HMI_BUFFER_SIZE];
    uint32_t adcValues[BSP_NUM_ADC_CHANNELS];
    float pressure;
    uint8_t toggleValue;
    uint8_t runFlag;
    float pressure_threshold;
} bsp_state_t;

/*******************************************************************************
 * System Functions
 ******************************************************************************/
/* Initialization */
void stm32_bsp_hal_init(void);
void stm32_bsp_init_state(bsp_state_t* state);

/* System Timing */
uint32_t stm32_bsp_gettick(void);
void stm32_bsp_delay(uint32_t delay);

/* Flash Configuration */
void stm32_bsp_flash_config_latency(uint32_t latency);

/*******************************************************************************
 * Clock Management Functions
 ******************************************************************************/
void stm32_bsp_clock_enable_hse(void);
void stm32_bsp_clock_disable_hse(void);
void stm32_bsp_clock_config_pll(uint32_t source, uint32_t multiplier);
void stm32_bsp_clock_config_sysclk(uint32_t source);
void stm32_bsp_clock_wait_hse_ready(void);
void stm32_bsp_clock_wait_pll_ready(void);
void stm32_bsp_clock_wait_sysclk_switched(uint32_t source);

/*******************************************************************************
 * GPIO Functions
 ******************************************************************************/
/* Basic GPIO Operations */
void stm32_bsp_gpio_init(void);
void stm32_bsp_gpio_enable_clock(GPIO_TypeDef* gpio);
void stm32_bsp_gpio_config_pin(GPIO_TypeDef* gpio, uint8_t pin, uint32_t mode, uint32_t pull);
void stm32_bsp_gpio_writepin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState stm32_bsp_gpio_readpin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint32_t stm32_bsp_gpio_read_port(GPIO_TypeDef* gpio);
uint8_t stm32_bsp_gpio_read_pin(GPIO_TypeDef* gpio, uint16_t pin);

/* GPIO Alternate Function */
void stm32_bsp_gpio_config_alternate_function(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af_value);
void stm32_bsp_gpio_set_alternate_function(GPIO_TypeDef* gpio, uint8_t pin);

/*******************************************************************************
 * Timer Functions
 ******************************************************************************/
/* Basic Timer Operations */
void stm32_bsp_timer_enable_clock(TIM_TypeDef* timer);
void stm32_bsp_timer_disable_clock(TIM_TypeDef* timer);
void stm32_bsp_timer_enable(TIM_TypeDef* timer);
void stm32_bsp_timer_init(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period);
void stm32_bsp_timer_config_basic(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period);

/* Timer Advanced Features */
void stm32_bsp_timer_enable_interrupt(TIM_TypeDef* timer);
void stm32_bsp_timer_clear_flag(TIM_TypeDef* timer);
void stm32_bsp_timer_set_autoreload(TIM_TypeDef* timer, uint32_t value);
void stm32_bsp_timer_enable_autoreload_preload(TIM_TypeDef* timer);

/* PWM Functions */
void stm32_bsp_timer_config_pwm(TIM_TypeDef* timer, uint32_t channel, uint32_t arr, uint32_t ccr);
void stm32_bsp_timer_enable_channel(TIM_TypeDef* timer, uint32_t channel);

/*******************************************************************************
 * ADC Functions
 ******************************************************************************/
/* Basic ADC Operations */
void stm32_bsp_adc_enable_clock(ADC_TypeDef* adc);
void stm32_bsp_adc_enable(ADC_TypeDef* adc);
void stm32_bsp_adc_config(ADC_TypeDef* adc, uint32_t channel, uint32_t resolution);
void stm32_bsp_adc_calibrate(ADC_TypeDef* adc);
void stm32_bsp_adc_wait_ready(ADC_TypeDef* adc);

/* ADC Conversion */
void stm32_bsp_adc_start_conversion(ADC_TypeDef* adc);
void stm32_bsp_adc_wait_conversion(ADC_TypeDef* adc);
uint32_t stm32_bsp_adc_get_data(ADC_TypeDef* adc);

/* ADC DMA Operations */
void stm32_bsp_adc_start_dma(ADC_HandleTypeDef* hadc, uint32_t* buffer, uint32_t length);
void stm32_bsp_adc_stop_dma(ADC_HandleTypeDef* hadc);

/* ADC Analog Watchdog */
void stm32_bsp_adc_config_awd(ADC_TypeDef* adc, uint32_t channel, uint32_t high_threshold, uint32_t low_threshold);
void stm32_bsp_adc_enable_awd_interrupt(ADC_TypeDef* adc);
uint8_t stm32_bsp_adc_check_awd(ADC_TypeDef* adc);
void stm32_bsp_adc_clear_awd_flag(ADC_TypeDef* adc);

/*******************************************************************************
 * Communication Interface Functions
 ******************************************************************************/
/* USART Functions */
void stm32_bsp_usart_config(USART_TypeDef* usart, uint32_t baud_rate);
void stm32_bsp_usart_enable_clock(USART_TypeDef* usart);
void stm32_bsp_usart_transmit_char(USART_TypeDef* usart, uint8_t data);
uint8_t stm32_bsp_usart_receive_char(USART_TypeDef* usart);
void stm32_bsp_usart_wait_tx_complete(USART_TypeDef* usart);

/* I2C Functions */
void stm32_bsp_i2c_read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);
void stm32_bsp_i2c_write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);

/*******************************************************************************
 * Interrupt Functions
 ******************************************************************************/
/* NVIC Functions */
void stm32_bsp_nvic_enable_irq(IRQn_Type irq);
void stm32_bsp_nvic_disable_irq(IRQn_Type irq);
void stm32_bsp_nvic_set_priority(IRQn_Type irq, uint32_t priority);

/* EXTI Functions */
void stm32_bsp_exti_config(uint32_t exti_line, GPIO_TypeDef* gpio, uint32_t trigger);
void stm32_bsp_exti_enable_clock(void);
void stm32_bsp_exti_enable_interrupt(uint32_t exti_line);
void stm32_bsp_exti_clear_flag(uint32_t exti_line);

/*******************************************************************************
 * Peripheral Handles
 ******************************************************************************/
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#endif /* STM32_BSP_H_ */ 
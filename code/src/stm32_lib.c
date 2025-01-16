/*
 * Library.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

#include <stdint.h>
#include "stm32_lib.h"
#include "stm32_bsp.h"

/**
 * @brief Initialize port with given configuration
 * @param port_state Pointer to port state structure
 */
void stm32_lib_port_init(stm32_port_state_t *port_state) 
{
	if (!port_state) return;

	// Initialize GPIO subsystem
	stm32_bsp_gpio_init();

	// Initialize GPIOA pins
	for (int i = 0; i < port_state->num_pins_a; i++) {
		stm32_bsp_gpio_config_pin(GPIOA, 
								 port_state->pins_a[i].pin,
								 port_state->pins_a[i].mode,
								 port_state->pins_a[i].pull);
	}

	// Initialize GPIOB pins
	for (int i = 0; i < port_state->num_pins_b; i++) {
		stm32_bsp_gpio_config_pin(GPIOB,
								 port_state->pins_b[i].pin,
								 port_state->pins_b[i].mode,
								 port_state->pins_b[i].pull);
	}
}

/**
 * @brief Initialize PWM timer with given parameters
 * @param htim Timer handle
 * @param channel Timer channel
 * @param frequency PWM frequency
 */
void stm32_lib_init_pwm_tim(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency)
{
	if (!htim) return;

	// Enable timer clock
	stm32_bsp_timer_enable_clock(htim->Instance);

	// Configure timer for PWM
	stm32_bsp_timer_config_pwm(htim->Instance, channel, frequency, frequency/2);

	// Enable timer
	stm32_bsp_timer_enable(htim->Instance);
}

/**
 * @brief Initialize ADC with given states
 * @param adc_states Array of ADC state structures
 * @param num_channels Number of ADC channels to initialize
 */
void stm32_lib_init_adc(const stm32_adc_state_t *adc_states, uint8_t num_channels)
{
	if (!adc_states || num_channels == 0) return;

	// Enable ADC clock
	stm32_bsp_adc_enable_clock(ADC1);

	// Calibrate ADC
	stm32_bsp_adc_calibrate(ADC1);

	// Configure ADC channels
	uint8_t max_resolution = 0;
	for (uint8_t i = 0; i < num_channels; i++) {
		if (adc_states[i].resolution > max_resolution) {
			max_resolution = adc_states[i].resolution;
		}
		stm32_bsp_adc_config(ADC1, adc_states[i].channel, max_resolution);
	}

	// Enable ADC and wait until ready
	stm32_bsp_adc_enable(ADC1);
	stm32_bsp_adc_wait_ready(ADC1);
}

/**
 * @brief Initialize LCD
 */
void stm32_lib_init_lcd(void)
{
	// Call BSP layer for LCD initialization
	stm32_bsp_init_lcd();
}

/**
 * @brief Initialize NVIC
 */
void stm32_lib_init_nvic(void)
{
	// Call BSP layer for NVIC initialization
	stm32_bsp_init_nvic();
}

/**
 * @brief Set timer compare value
 * @param timer Timer handle
 * @param channel Timer channel
 * @param value Compare value
 */
void stm32_lib_timer_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
	if (!timer) return;

	// Call BSP layer to set timer compare value
	stm32_bsp_timer_set_compare(timer->Instance, channel, value);
}

// Initializing the PWM output
void stm32_lib_init_pwm(int frequency)
{
	// Enable timer clock
	stm32_bsp_timer_enable_clock(TIM2);

	// Configure PWM for channels 3 and 4
	stm32_bsp_timer_config_pwm(TIM2, 3, frequency, 0);      // Red LED
	stm32_bsp_timer_config_pwm(TIM2, 4, frequency, frequency/2);  // Green LED

	// Enable timer
	stm32_bsp_timer_enable(TIM2);
}

// Initializing the ADC
void stm32_lib_init_adc(const stm32_adc_state_t *adc_states, uint8_t num_channels)
{
    if (!adc_states || num_channels == 0) return;

    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Start calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while((ADC1->CR & ADC_CR_ADCAL) != 0); // Wait for calibration to complete

    // Configure ADC
    uint32_t chselr = 0;
    uint8_t max_resolution = 0;

    // Collect configuration from all channels
    for (uint8_t i = 0; i < num_channels; i++) {
        chselr |= (1 << adc_states[i].channel);
        if (adc_states[i].resolution > max_resolution) {
            max_resolution = adc_states[i].resolution;
        }
    }

    // Set resolution (use highest requested)
    ADC1->CFGR1 = ((max_resolution - 6) / 2) << 3; // Convert 8,10,12 bit to 0,1,2

    // Set channel selection register
    ADC1->CHSELR = chselr;

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while((ADC1->ISR & ADC_ISR_ADRDY) == 0); // Wait for ADC ready
}

// Initializing the ADC to a specific pin
int stm32_lib_adc_input(int input, int resolution)
{
    // Configure ADC
    stm32_bsp_adc_config(ADC1, input, resolution);
    
    // Start conversion and wait for result
    stm32_bsp_adc_start_conversion(ADC1);
    stm32_bsp_adc_wait_conversion(ADC1);
    
    // Return result
    return stm32_bsp_adc_get_data(ADC1);
}

int stm32_lib_adc_awd_check(void)
{
    return stm32_bsp_adc_check_awd(ADC1);
}

void stm32_lib_adc_awd_8bit(int ADC_channel, int ADC_Low_threshhold, int ADC_High_threshhold)
{
    // Configure analog watchdog
    stm32_bsp_adc_config_awd(ADC1, ADC_channel, ADC_High_threshhold, ADC_Low_threshhold);
    
    // Enable AWD interrupt
    stm32_bsp_adc_enable_awd_interrupt(ADC1);
}

// Fetching ADC data
int stm32_lib_adc_data(void)
{
    // Start conversion and wait for result
    stm32_bsp_adc_start_conversion(ADC1);
    stm32_bsp_adc_wait_conversion(ADC1);
    
    // Return result
    return stm32_bsp_adc_get_data(ADC1);
}

// Creating a delay by iterating through a loop
void stm32_lib_delay(int time) {
	// time given in milli seconds
	time = time * 727;
	while (time > 0) {time--;}
}

// Check for button debouncee
void stm32_lib_debounce(void) 
{
	stm32_bsp_delay(50);  // Replace busy-wait with HAL delay
}

// Enabling Timer 6 interrupt
void stm32_lib_init_tim6(uint32_t arr, uint32_t psc)
{
    // Enable and configure timer
    stm32_bsp_timer_enable_clock(TIM6);
    stm32_bsp_timer_config_basic(TIM6, psc, arr);
    stm32_bsp_timer_enable_interrupt(TIM6);
    stm32_bsp_timer_clear_flag(TIM6);
    stm32_bsp_timer_enable(TIM6);
    stm32_bsp_timer_enable_autoreload_preload(TIM6);
    
    // Enable timer interrupt in NVIC
    stm32_bsp_nvic_enable_irq(TIM6_DAC_IRQn);
}

// Acknowledging timer interrupt
void stm32_lib_ack_irq(void)
{
    stm32_bsp_timer_clear_flag(TIM6);
}

// Initializing the External Interrupt
void stm32_lib_init_exti(void)
{
    // Enable SYSCFG clock for EXTI
    stm32_bsp_exti_enable_clock();
    
    // Configure EXTI for PA1 with falling edge trigger
    stm32_bsp_exti_config(1, GPIOA, 0x2);  // 0x2 for falling edge
    
    // Enable EXTI interrupt in NVIC
    stm32_bsp_nvic_enable_irq(EXTI0_1_IRQn);
}

void stm32_lib_usart1_transmit(unsigned char DataToTx)
{
    stm32_bsp_usart_transmit_char(USART1, DataToTx);
    stm32_bsp_usart_wait_tx_complete(USART1);
}

unsigned char stm32_lib_usart1_receive(void)
{
    return stm32_bsp_usart_receive_char(USART1);
}

void stm32_lib_lock_crystal(void)
{
    // Enable and wait for HSE
    stm32_bsp_clock_enable_hse();
    stm32_bsp_clock_wait_hse_ready();
    
    // Configure flash latency
    stm32_bsp_flash_config_latency(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY);
    
    // Configure and enable PLL
    stm32_bsp_clock_config_pll(RCC_CFGR_PLLSRC_HSE_PREDIV, RCC_CFGR_PLLMUL6);
    stm32_bsp_clock_wait_pll_ready();
    
    // Switch system clock to PLL
    stm32_bsp_clock_config_sysclk(RCC_CFGR_SW_PLL);
    stm32_bsp_clock_wait_sysclk_switched(RCC_CFGR_SWS_PLL);
}

void stm32_lib_unlock_crystal(void)
{
    // Switch back to default clock source
    stm32_bsp_clock_config_sysclk(0);  // 0 for HSI
    stm32_bsp_clock_wait_sysclk_switched(0);
    
    // Disable HSE
    stm32_bsp_clock_disable_hse();
}

void stm32_lib_sig_gen_init(stm32_sig_gen_state_t *state, const stm32_sig_gen_state_t *config)
{
    if (!state || !state->timer) return;

    // Copy configuration to state
    memcpy(state, config, sizeof(stm32_sig_gen_state_t));

    // Enable timer clock
    stm32_bsp_timer_enable_clock(state->timer->Instance);

    // Configure PWM
    uint32_t ccr = (state->amplitude * state->frequency) / 100;
    stm32_bsp_timer_config_pwm(state->timer->Instance, state->channel, state->frequency, ccr);

    // Enable timer
    stm32_bsp_timer_enable(state->timer->Instance);
}

void stm32_lib_pwm(void)
{
    // Enable GPIO clock and configure pins for alternate function
    stm32_bsp_gpio_enable_clock(GPIOB);
    
    // Configure PB10 and PB11 for alternate function
    stm32_bsp_gpio_set_alternate_function(GPIOB, 10);
    stm32_bsp_gpio_set_alternate_function(GPIOB, 11);
    stm32_bsp_gpio_config_alternate_function(GPIOB, 10, 2);  // AF2
    stm32_bsp_gpio_config_alternate_function(GPIOB, 11, 2);  // AF2
    
    // Enable and configure timer
    stm32_bsp_timer_enable_clock(TIM2);
    
    // Configure PWM for channels 3 and 4
    uint32_t frequency = 48000;  // 1 KHz
    stm32_bsp_timer_config_pwm(TIM2, 3, frequency, 0);       // Red = 0%
    stm32_bsp_timer_config_pwm(TIM2, 4, frequency, 9600);    // Green = 20%
    
    // Enable timer
    stm32_bsp_timer_enable(TIM2);
}

void stm32_lib_uart_init(stm32_uart_state_t *uart_state, const stm32_uart_state_t *config) {
    if (!uart_state || !config) return;

    // Copy configuration to state
    memcpy(uart_state, config, sizeof(stm32_uart_state_t));

    // Initialize UART hardware
    stm32_bsp_uart_init(uart_state->huart, (uint32_t)uart_state->huart->Instance, uart_state->huart->Init.BaudRate);
}

// Add system initialization function
void stm32_lib_init(const stm32_system_config_t *config)
{
    if (!config) return;

    // Initialize ports
    stm32_lib_port_init(&config->port_state);

    // Initialize ADC
    stm32_lib_init_adc(config->adc_states, config->num_adc_channels);

    // Initialize PWM for each channel
    for (uint8_t i = 0; i < config->num_pwm_channels; i++) {
        stm32_lib_init_pwm_tim(config->pwm_states[i].timer, 
                              config->pwm_states[i].channel, 
                              config->pwm_states[i].frequency);
    }
}

/**
 * @brief Set specific GPIO pin state
 * @param gpio GPIO port
 * @param pin Pin number
 * @param state Pin state (0 or 1)
 */
void stm32_lib_gpio_set_pin(GPIO_TypeDef* gpio, uint16_t pin, uint8_t state)
{
    if (!gpio) return;
    
    // Use BSP function to write pin state
    stm32_bsp_gpio_writepin(gpio, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Set entire GPIO port value
 * @param gpio GPIO port
 * @param value Port value
 */
void stm32_lib_gpio_set_port(GPIO_TypeDef* gpio, uint16_t value)
{
    if (!gpio) return;
    
    // Configure all pins as outputs first
    for (int i = 0; i < 16; i++) {
        stm32_bsp_gpio_config_pin(gpio, i, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    }
    
    // Write port value
    gpio->ODR = value;  // This is okay since it's atomic
}

/**
 * @brief Enable timer
 * @param timer Timer peripheral
 */
void stm32_lib_timer_enable(TIM_TypeDef* timer)
{
    if (!timer) return;
    stm32_bsp_timer_enable_clock(timer);
}

/**
 * @brief Disable timer
 * @param timer Timer peripheral
 */
void stm32_lib_timer_disable(TIM_TypeDef* timer)
{
    if (!timer) return;
    stm32_bsp_timer_disable_clock(timer);
}

/**
 * @brief Initialize timer with given parameters
 * @param timer Timer peripheral
 * @param arr Auto-reload value
 * @param psc Prescaler value
 */
void stm32_lib_timer_init(TIM_TypeDef* timer, uint32_t arr, uint32_t psc)
{
    if (!timer) return;
    stm32_bsp_timer_init(timer, psc, arr);
}

int stm32_lib_check_button(GPIO_TypeDef* gpio, int button)
{
    if (!gpio) return 0;
    
    if (stm32_bsp_gpio_read_pin(gpio, button) == 0) {
        stm32_lib_debounce();
        return 1;
    }
    return 0;
}

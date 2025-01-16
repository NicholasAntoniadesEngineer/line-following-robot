/**
  ******************************************************************************
  * @file           : stm32_bsp.c
  * @brief          : Combined Board Support Package implementation
  * @details        : Encapsulates HAL and BSP functions for all prototypes
  * @date           : 2024
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "stm32_bsp.h"
#include "gpio.h"
#include "tim.h"
#include "dac.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "dma.h"

/* Initialization Functions */
void stm32_bsp_hal_init(void)
{
    HAL_Init();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C3_Init();
    MX_DAC_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
}

void stm32_bsp_init_state(bsp_state_t* state)
{
    /* Initialize state structure with default values */
    memset(state, 0, sizeof(bsp_state_t));
    state->pressure_threshold = 0.0f;
    state->runFlag = 0;
    state->toggleValue = 0;
}

/* GPIO Functions */
void stm32_bsp_gpio_writepin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

GPIO_PinState stm32_bsp_gpio_readpin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void stm32_bsp_gpio_init(void)
{
    // Initialize GPIO clocks and default configuration
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
}

void stm32_bsp_gpio_config_pin(GPIO_TypeDef* gpio, uint8_t pin, uint32_t mode, uint32_t pull)
{
    if (!gpio) return;
    
    gpio->MODER &= ~(0x3 << (pin * 2));
    gpio->MODER |= (mode << (pin * 2));
    gpio->PUPDR &= ~(0x3 << (pin * 2));
    gpio->PUPDR |= (pull << (pin * 2));
}

void stm32_bsp_gpio_enable_clock(GPIO_TypeDef* gpio)
{
    if (!gpio) return;
    
    if (gpio == GPIOA)
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if (gpio == GPIOB)
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

/* Timer Functions */
void stm32_bsp_tim_base_start(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Start(htim);
}

void stm32_bsp_tim_base_stop(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Stop(htim);
}

void stm32_bsp_tim_base_init(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Init(htim);
}

void stm32_bsp_tim_pwm_init(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t frequency)
{
    /* Configure timer for PWM mode */
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = frequency / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

void stm32_bsp_timer_config_pwm(TIM_TypeDef* timer, uint32_t channel, uint32_t arr, uint32_t ccr)
{
    if (!timer) return;
    
    timer->ARR = arr;
    timer->PSC = 0;
    
    switch(channel) {
        case 1:
            timer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
            timer->CCR1 = ccr;
            timer->CCER |= TIM_CCER_CC1E;
            break;
        case 2:
            timer->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
            timer->CCR2 = ccr;
            timer->CCER |= TIM_CCER_CC2E;
            break;
        case 3:
            timer->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
            timer->CCR3 = ccr;
            timer->CCER |= TIM_CCER_CC3E;
            break;
        case 4:
            timer->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
            timer->CCR4 = ccr;
            timer->CCER |= TIM_CCER_CC4E;
            break;
    }
}

void stm32_bsp_timer_enable_clock(TIM_TypeDef* timer)
{
    if (!timer) return;
    
    if (timer == TIM2)
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (timer == TIM3)
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (timer == TIM4)
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
}

void stm32_bsp_timer_enable_channel(TIM_TypeDef* timer, uint32_t channel)
{
    if (!timer) return;
    
    switch(channel) {
        case 1:
            timer->CCER |= TIM_CCER_CC1E;
            break;
        case 2:
            timer->CCER |= TIM_CCER_CC2E;
            break;
        case 3:
            timer->CCER |= TIM_CCER_CC3E;
            break;
        case 4:
            timer->CCER |= TIM_CCER_CC4E;
            break;
    }
}

void stm32_bsp_timer_enable(TIM_TypeDef* timer)
{
    if (!timer) return;
    timer->CR1 |= TIM_CR1_CEN;
}

/* UART Functions */
void stm32_bsp_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Receive(huart, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_uart_transmit_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(huart, data, size);
}

void stm32_bsp_uart_receive_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Receive_DMA(huart, data, size);
}

/* I2C Functions */
void stm32_bsp_i2c_read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Read(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_i2c_write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Write(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

/* ADC Functions */
void stm32_bsp_adc_start(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
}

void stm32_bsp_adc_stop(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop(hadc);
}

void stm32_bsp_adc_start_dma(ADC_HandleTypeDef* hadc, uint32_t* buffer, uint32_t length)
{
    HAL_ADC_Start_DMA(hadc, buffer, length);
}

void stm32_bsp_adc_stop_dma(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop_DMA(hadc);
}

uint32_t stm32_bsp_adc_readvalue(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(hadc);
}

void stm32_bsp_adc_config(ADC_TypeDef* adc, uint32_t channel, uint32_t resolution)
{
    if (!adc) return;
    
    adc->CFGR1 = ((resolution - 6) / 2) << 3;  // Convert 8,10,12 bit to 0,1,2
    adc->CHSELR = (1 << channel);
}

void stm32_bsp_adc_enable_clock(ADC_TypeDef* adc)
{
    if (!adc) return;
    
    if (adc == ADC1)
        RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
}

void stm32_bsp_adc_calibrate(ADC_TypeDef* adc)
{
    if (!adc) return;
    
    adc->CR |= ADC_CR_ADCAL;
    while((adc->CR & ADC_CR_ADCAL) != 0);  // Wait for calibration to complete
}

/* System Functions */
uint32_t stm32_bsp_gettick(void)
{
    return HAL_GetTick();
}

void stm32_bsp_delay(uint32_t delay)
{
    HAL_Delay(delay);
}

/* USART Functions */
void stm32_bsp_usart_config(USART_TypeDef* usart, uint32_t baud_rate)
{
    if (!usart) return;
    
    usart->CR1 &= ~USART_CR1_M;        // 8 data bits
    usart->CR1 &= ~USART_CR1_PCE;      // No parity
    usart->CR2 &= ~USART_CR2_STOP;     // 1 stop bit
    usart->BRR = SystemCoreClock/baud_rate;
    usart->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void stm32_bsp_usart_enable_clock(USART_TypeDef* usart)
{
    if (!usart) return;
    
    if (usart == USART1)
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    else if (usart == USART2)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

void stm32_bsp_usart_transmit_char(USART_TypeDef* usart, uint8_t data)
{
    if (!usart) return;
    usart->TDR = data;
}

uint8_t stm32_bsp_usart_receive_char(USART_TypeDef* usart)
{
    if (!usart) return 0;
    return (uint8_t)(usart->RDR & 0xFF);
}

void stm32_bsp_usart_wait_tx_complete(USART_TypeDef* usart)
{
    if (!usart) return;
    while(!(usart->ISR & USART_ISR_TC));
}

/* EXTI Functions */
void stm32_bsp_exti_config(uint32_t exti_line, GPIO_TypeDef* gpio, uint32_t trigger)
{
    if (!gpio) return;
    
    uint32_t port_index = 0;
    if (gpio == GPIOA) port_index = 0;
    else if (gpio == GPIOB) port_index = 1;
    
    uint32_t exti_index = exti_line / 4;
    uint32_t exti_offset = (exti_line % 4) * 4;
    
    SYSCFG->EXTICR[exti_index] &= ~(0xF << exti_offset);
    SYSCFG->EXTICR[exti_index] |= (port_index << exti_offset);
    
    EXTI->IMR |= (1 << exti_line);
    
    if (trigger & 0x1)  // Rising edge
        EXTI->RTSR |= (1 << exti_line);
    if (trigger & 0x2)  // Falling edge
        EXTI->FTSR |= (1 << exti_line);
}

void stm32_bsp_exti_enable_clock(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
}

void stm32_bsp_exti_enable_interrupt(uint32_t exti_line)
{
    EXTI->IMR |= (1 << exti_line);
}

void stm32_bsp_exti_clear_flag(uint32_t exti_line)
{
    EXTI->PR = (1 << exti_line);
}

/* Additional Timer Functions */
void stm32_bsp_timer_config_basic(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period)
{
    if (!timer) return;
    
    timer->PSC = prescaler;
    timer->ARR = period;
}

void stm32_bsp_timer_enable_interrupt(TIM_TypeDef* timer)
{
    if (!timer) return;
    timer->DIER |= TIM_DIER_UIE;
}

void stm32_bsp_timer_clear_flag(TIM_TypeDef* timer)
{
    if (!timer) return;
    timer->SR &= ~TIM_SR_UIF;
}

void stm32_bsp_timer_set_autoreload(TIM_TypeDef* timer, uint32_t value)
{
    if (!timer) return;
    timer->ARR = value;
}

void stm32_bsp_timer_enable_autoreload_preload(TIM_TypeDef* timer)
{
    if (!timer) return;
    timer->CR1 |= TIM_CR1_ARPE;
}

/* NVIC Functions */
void stm32_bsp_nvic_enable_irq(IRQn_Type irq)
{
    NVIC_EnableIRQ(irq);
}

void stm32_bsp_nvic_disable_irq(IRQn_Type irq)
{
    NVIC_DisableIRQ(irq);
}

void stm32_bsp_nvic_set_priority(IRQn_Type irq, uint32_t priority)
{
    NVIC_SetPriority(irq, priority);
}

/* GPIO Input Functions */
uint32_t stm32_bsp_gpio_read_port(GPIO_TypeDef* gpio)
{
    if (!gpio) return 0;
    return gpio->IDR;
}

uint8_t stm32_bsp_gpio_read_pin(GPIO_TypeDef* gpio, uint16_t pin)
{
    if (!gpio) return 0;
    return (gpio->IDR & (1 << pin)) ? 1 : 0;
}

/* Clock Configuration Functions */
void stm32_bsp_clock_enable_hse(void)
{
    RCC->CR |= RCC_CR_HSEON;
}

void stm32_bsp_clock_disable_hse(void)
{
    RCC->CR &= ~RCC_CR_HSEON;
}

void stm32_bsp_clock_config_pll(uint32_t source, uint32_t multiplier)
{
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= (multiplier | source);
    RCC->CR |= RCC_CR_PLLON;
}

void stm32_bsp_clock_config_sysclk(uint32_t source)
{
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= source;
}

void stm32_bsp_clock_wait_hse_ready(void)
{
    while(!(RCC->CR & RCC_CR_HSERDY));
}

void stm32_bsp_clock_wait_pll_ready(void)
{
    while(!(RCC->CR & RCC_CR_PLLRDY));
}

void stm32_bsp_clock_wait_sysclk_switched(uint32_t source)
{
    while((RCC->CFGR & RCC_CFGR_SWS) != source);
}

/* GPIO Alternate Function Configuration */
void stm32_bsp_gpio_config_alternate_function(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af_value)
{
    if (!gpio) return;
    
    uint8_t af_register = pin / 8;  // AFR[0] for pins 0-7, AFR[1] for pins 8-15
    uint8_t af_position = (pin % 8) * 4;
    
    gpio->AFR[af_register] &= ~(0xF << af_position);
    gpio->AFR[af_register] |= (af_value << af_position);
}

void stm32_bsp_gpio_set_alternate_function(GPIO_TypeDef* gpio, uint8_t pin)
{
    if (!gpio) return;
    
    gpio->MODER &= ~(0x3 << (pin * 2));
    gpio->MODER |= (0x2 << (pin * 2));  // Set to alternate function mode
}

/* Flash Configuration */
void stm32_bsp_flash_config_latency(uint32_t latency)
{
    FLASH->ACR = latency;
}

/* Additional ADC Functions */
void stm32_bsp_adc_start_conversion(ADC_TypeDef* adc)
{
    if (!adc) return;
    adc->CR |= ADC_CR_ADSTART;
}

uint32_t stm32_bsp_adc_get_data(ADC_TypeDef* adc)
{
    if (!adc) return 0;
    return adc->DR;
}

void stm32_bsp_adc_wait_conversion(ADC_TypeDef* adc)
{
    if (!adc) return;
    while((adc->ISR & ADC_ISR_EOC) == 0);
}

uint8_t stm32_bsp_adc_check_awd(ADC_TypeDef* adc)
{
    if (!adc) return 0;
    if ((adc->ISR & ADC_ISR_AWD) != 0) {
        adc->ISR &= ~ADC_ISR_AWD;
        return 1;
    }
    return 0;
}

void stm32_bsp_adc_clear_awd_flag(ADC_TypeDef* adc)
{
    if (!adc) return;
    adc->ISR &= ~ADC_ISR_AWD;
}

void stm32_bsp_adc_config_awd(ADC_TypeDef* adc, uint32_t channel, uint32_t high_threshold, uint32_t low_threshold)
{
    if (!adc) return;
    
    adc->CFGR1 |= ADC_CFGR1_AWDEN;   // Enable analog watchdog
    adc->CFGR1 |= ADC_CFGR1_AWDSGL;  // Single channel mode
    adc->CFGR1 |= (channel << ADC_CFGR1_AWDCH_Pos);  // Set channel
    
    adc->TR &= ~(ADC_TR_HT | ADC_TR_LT);  // Clear thresholds
    adc->TR |= (high_threshold << ADC_TR_HT_Pos);  // Set high threshold
    adc->TR |= low_threshold;  // Set low threshold
}

void stm32_bsp_adc_enable_awd_interrupt(ADC_TypeDef* adc)
{
    if (!adc) return;
    adc->IER |= ADC_IER_AWDIE;
}

void stm32_bsp_timer_disable_clock(TIM_TypeDef* timer)
{
    if (!timer) return;
    
    if (timer == TIM2)
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
    else if (timer == TIM3)
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
    else if (timer == TIM4)
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
    else if (timer == TIM14)
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
}

void stm32_bsp_timer_init(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period)
{
    if (!timer) return;
    
    // Configure timer basic parameters
    timer->PSC = prescaler;
    timer->ARR = period;
    timer->CR1 |= TIM_CR1_CEN;
}

void stm32_bsp_adc_enable(ADC_TypeDef* adc)
{
    if (!adc) return;
    adc->CR |= ADC_CR_ADEN;
}

void stm32_bsp_adc_wait_ready(ADC_TypeDef* adc)
{
    if (!adc) return;
    while((adc->ISR & ADC_ISR_ADRDY) == 0);
} 
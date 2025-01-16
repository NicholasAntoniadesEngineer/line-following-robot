/*
 * Library.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

#define STM32F051
#include "stm32f0xx.h"
#include <stdint.h>
#include "stm32_lib.h"
#include <stm32f0xx_exti.h>
#include "stm32f051x8.h"
#include "stm32_bsp.h"

/**
 * @brief Initialize port with given configuration
 * @param port_state Pointer to port state structure
 */
void stm32_lib_port_init(stm32_port_state_t *port_state) 
{
	if (!port_state) return;

	// Initialize GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	for (int i = 0; i < port_state->num_pins_a; i++) {
		GPIOA->MODER &= ~(0x3 << (port_state->pins_a[i].pin * 2));
		GPIOA->MODER |= (port_state->pins_a[i].mode << (port_state->pins_a[i].pin * 2));
		GPIOA->PUPDR &= ~(0x3 << (port_state->pins_a[i].pin * 2));
		GPIOA->PUPDR |= (port_state->pins_a[i].pull << (port_state->pins_a[i].pin * 2));
	}

	// Initialize GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	for (int i = 0; i < port_state->num_pins_b; i++) {
		GPIOB->MODER &= ~(0x3 << (port_state->pins_b[i].pin * 2));
		GPIOB->MODER |= (port_state->pins_b[i].mode << (port_state->pins_b[i].pin * 2));
		GPIOB->PUPDR &= ~(0x3 << (port_state->pins_b[i].pin * 2));
		GPIOB->PUPDR |= (port_state->pins_b[i].pull << (port_state->pins_b[i].pin * 2));
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
	if (htim->Instance == TIM2) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}

	// Configure timer for PWM
	htim->Instance->PSC = 0;
	htim->Instance->ARR = frequency;
	
	// Configure PWM mode based on channel
	switch(channel) {
		case TIM_CHANNEL_1:
			htim->Instance->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
			htim->Instance->CCER |= TIM_CCER_CC1E;
			break;
		case TIM_CHANNEL_2:
			htim->Instance->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
			htim->Instance->CCER |= TIM_CCER_CC2E;
			break;
		case TIM_CHANNEL_3:
			htim->Instance->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
			htim->Instance->CCER |= TIM_CCER_CC3E;
			break;
		case TIM_CHANNEL_4:
			htim->Instance->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
			htim->Instance->CCER |= TIM_CCER_CC4E;
			break;
	}

	// Enable timer
	htim->Instance->CR1 |= TIM_CR1_CEN;
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
	#define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)		/*Macros to be defined*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 						// Enable clock for GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	 					// Enable TIM2
	GPIOB->MODER |= GPIO_MODER_MODER11_1;					// set PB10 to AF
	GPIOB->AFR[1] |= (GPIO_AFRH_AFR11_AF2&(0b10<<12)); 		// Enable AF2 for PB10 in GPIOB AFR10
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1 	// PWM Mode 1
	TIM2->ARR = frequency; 									// Setting signal frequency
	TIM2->PSC = 0;											// Setting signal prescalar
	TIM2->CCR4 = frequency/2; 								// PWM Duty cycle based on fractions of ARR
	TIM2->CCER |= TIM_CCER_CC4E; 							// Compare 3 output enable
	//TIM2->CR1 |= TIM_CR1_CEN;    //Counter enable
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
    ADC1->CFGR1 |= (resolution << 3);	/* configure resolution,12,10,8,6 bits */
    ADC1->CHSELR = (1 <<  input);		// channel select where input ranges from ADC-in 0-13
    ADC1->CR |= (0b1 << 2); 		    // start conversion
    while ((ADC1->ISR & 0b100) == 0);	// wait for conversion to end
    return ADC1->DR;					// return value
}

int stm32_lib_adc_awd_check(void)
{
	if((ADC1->ISR & 0x80) == 1)
	{	// Check if AWD status bit is high
		ADC1 -> ISR &= ~0x80;
		return 1;
	}else{
		return 0;
	}
}

void stm32_lib_adc_awd_8bit(int ADC_channel, int ADC_Low_threshhold, int ADC_High_threshhold)
{
	// ADC_Low_threshhold = (ADC_Low_threshhold*256)/3.3;
	// ADC_High_threshhold = (ADC_High_threshhold*256)/3.3;

	ADC1 -> CFGR1 |= 1 << 22;  					// Set AWDEN bit high to enable AWD
	ADC1 -> CFGR1 |= 1 << 21;					// Set AWDSGL bit high to select signle channel.
	ADC1 -> CFGR1 |= ADC_channel << 25;			// Set AWDCH[4:0] bits to select channel.
	ADC1 -> TR |= ADC_High_threshhold << 15;	// Set ADC_HTR high threshholds.
	ADC1 -> TR |= ADC_Low_threshhold;			// Set ADC_LTR low threshhold.
	ADC1 -> IER |= 0x80;						// Set AWDIE bit in ADC_IER to enable the interrupt.
}

// Fetching ADC data
int stm32_lib_adc_data(void)
{
	ADC1 ->CR |= ADC_CR_ADSTART;		// Starts ADC conversion
	while((ADC1 -> ISR & 0b100) ==0 );	// Waits for the End of conversion flag to be set
	return ADC1 -> DR;
}

// Creating a delay by iterating through a loop
void stm32_lib_delay(int time) {
	// time given in milli seconds
	time = time * 727;
	while (time > 0) {time--;}
}

// Check for button press GPIOA
int stm32_lib_check_button_gpioa(int button) {
	if ((GPIOA->IDR & (0b1 << button)) == 0) {
		stm32_lib_debounce();
		return 1;
	} else { return 0; }
}
// Check for button press GPIOB
int stm32_lib_check_button_gpiob(int button) 
{
	if ((GPIOB->IDR & (0b1 << button)) == 0) {
		stm32_lib_debounce();
		return 1;
	} else { return 0; }
}

// Check for button debouncee
void stm32_lib_debounce(void) 
{
	int x = 36350;		// 50 milliseconds
	while (x > 0) {x--;}
}

// Enabling Timer 6 interrupt
void stm32_lib_init_tim6(uint32_t arr, uint32_t psc)
{
    RCC -> APB1ENR |= (1 << 4);		// Enable Tim6 in the peripheral clock enable register
    TIM6 -> PSC = psc;
    TIM6 -> ARR = arr;
    TIM6 -> DIER |= TIM_DIER_UIE;	// Enable Tim6 interrupt
    TIM6->SR &= ~TIM_SR_UIF;		// Clear the update flag
    TIM6 -> CR1 |= TIM_CR1_CEN;		// Enable Tim6 Counter
    TIM6 -> CR1 |= TIM_CR1_ARPE;	// Enable Tim6 Auto reload / preload
    NVIC_EnableIRQ(TIM6_DAC_IRQn); 	// Enable Tim6 interrupt in the (NVIC) Nested Vectored Interrupt Controller
}

// Acknowledging timer interrupt
void stm32_lib_ack_irq(void)
{
        TIM6->SR &= ~(1 << 0);	// Set interrupt acknowledge flag to zero (not 1)
}

// Initializing the External Interrupt
void stm32_lib_init_exti(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;		// Enables the clock for the sys config controller.
	SYSCFG->EXTICR[1] |=SYSCFG_EXTICR1_EXTI1_PA;    // Map PA1 to EXTI1.
	EXTI->IMR |=EXTI_IMR_MR1; 						// Unmasks EXTI1 in interrupt mask register
	EXTI->FTSR |= EXTI_FTSR_TR1; 					// Falling trigger enable for input line 1 (SW1)
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// Enable the interrupt in the NVIC
}


void stm32_system_state_t(void)
{
	// Must enable usart1_DE and the associated pins AF
	// Program the M bits in USARTx_CR1 to define the word length.
	//Program the number of stop bits in USARTx_CR2.
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 				// enable clock for UART1
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 					// enable clock to port a
	GPIOA->MODER |= GPIO_MODER_MODER9_1; 				// Set PA9 to AF mode
	GPIOA->MODER |= GPIO_MODER_MODER10_1;				// Set PA10 to AF mode
	GPIOA->AFR[1] = 0b00000000000000000000000100010000; // Map AF1 for PA9 and PA10
	USART1->CR1 &=~ USART_CR1_M; 						// clear M0(bit 12) of USARTx_CR1
	SystemCoreClockUpdate(); 							// define clock speed
	USART1->BRR = SystemCoreClock/115200;		     		// set baud rate to 115.2kbps for oversampling by 16
	USART1->CR1 &=~ USART_CR1_PCE;						// no parity
	USART1->CR2 &=~ USART_CR2_STOP;						// 1 stop bit
	USART1->CR1 |= USART_CR1_UE;						// Enable USART1
	USART1->CR1 |= USART_CR1_TE;						// Enable USART1_TE Set idle frame as first transmission.
	USART1->CR1 |= USART_CR1_RE;						// Enable USART1_RX
}

void stm32_lib_usart1_transmit(unsigned char DataToTx)
{
    USART1->TDR = DataToTx;                           // Write character to the USART1_TDR
    while((USART1->ISR & USART_ISR_TC) == 0);        // Wait: transmission complete
}

unsigned char stm32_lib_usart1_receive(void)
{
    unsigned char DataRx = 'z';                       // Default value
    while((USART1->ISR & USART_ISR_RXNE) == 0);      // Wait until data is received
    DataRx = USART1->RDR;                            // Store received data
    USART1->ICR = 0b111111111111111111111;          // Clear flags
    return DataRx;
}

void stm32_lib_lock_crystal(void)
{
    RCC->CR |= RCC_CR_HSEON;                         // Enable HSE
    while(!(RCC->CR & RCC_CR_HSERDY));               // Wait until HSE ready
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR |= RCC_CFGR_PLLMUL6;                  // PLLCLK = HSE * 6 = 48 MHz
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;        // Select HSE as PLL source
    RCC->CR |= RCC_CR_PLLON;                         // Enable PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));               // Wait until PLL ready
    RCC->CFGR |= RCC_CFGR_SW_PLL;                    // SYSCLK from PLL
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));          // Wait until switched
}

void stm32_lib_unlock_crystal(void)
{
    RCC->CFGR &= ~RCC_CFGR_SW;                       // Clear SYSCLK selection
    while(RCC->CFGR & RCC_CFGR_SWS_PLL);             // Wait until switched
    RCC->CR &= ~RCC_CR_HSEON;                        // Disable HSE
}

void stm32_lib_sig_gen_init(stm32_sig_gen_state_t *state, const stm32_sig_gen_state_t *config)
{
    if (!state || !state->timer) return;

    // Copy configuration to state
    memcpy(state, config, sizeof(stm32_sig_gen_state_t));

    // Enable timer clock
    if (state->timer == TIM2)
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (state->timer == TIM4)
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Configure timer for PWM
    state->timer->Instance->PSC = 0;
    state->timer->Instance->ARR = state->frequency;
    
    // Configure PWM mode based on channel
    switch(state->channel) {
        case TIM_CHANNEL_1:
            state->timer->Instance->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
            state->timer->Instance->CCR1 = (state->amplitude * state->frequency) / 100;
            state->timer->Instance->CCER |= TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            state->timer->Instance->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
            state->timer->Instance->CCR2 = (state->amplitude * state->frequency) / 100;
            state->timer->Instance->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            state->timer->Instance->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
            state->timer->Instance->CCR3 = (state->amplitude * state->frequency) / 100;
            state->timer->Instance->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            state->timer->Instance->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
            state->timer->Instance->CCR4 = (state->amplitude * state->frequency) / 100;
            state->timer->Instance->CCER |= TIM_CCER_CC4E;
            break;
    }

    // Enable timer
    state->timer->Instance->CR1 |= TIM_CR1_CEN;
}

void stm32_lib_pwm(void)
{
    /*Macros to be defined*/
    #define GPIO_AFRH_AFR10_AF2 ((uint32_t)0x00000200)
    #define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)

    //Initialising clocks and pins for PWM output
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;     // Enable clock for GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Enable TIM2
    GPIOB->MODER |= GPIO_MODER_MODER10_1; // set PB10 to AF
    GPIOB->MODER |= GPIO_MODER_MODER11_1; // set PB11 to AF

    //Choosing AF for pins, MAPPING them to TIM2 CH3 and CH4
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2&(0b10<<8)); //Enable AF2 for PB10 in GPIOB AFR10
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR11_AF2&(0b10<<12));//Enable AF2 for PB11 in GPIOB AFR11

    // specify PWM mode: OCxM bits in CCMRx. We want mode 1
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
    TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1

    //Setting signal frequency
    TIM2->ARR = 48000; // f = 1 KHz
    TIM2->PSC = 0;

    //PWM Duty cycle based on fractions of ARR
    TIM2->CCR3 = 0 * 480; // Red = 20%
    TIM2->CCR4 = 20 * 480; // Green = 90%

    // Enable output compare for CH3 and CH4
    TIM2->CCER |= TIM_CCER_CC3E; //Compare 3 output enable
    TIM2->CCER |= TIM_CCER_CC4E; //Compare 4 output enable
    TIM2->CR1 |= TIM_CR1_CEN;    //Counter enable
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
    
    if (state) {
        gpio->ODR |= (1 << pin);
    } else {
        gpio->ODR &= ~(1 << pin);
    }
}

/**
 * @brief Set entire GPIO port value
 * @param gpio GPIO port
 * @param value Port value
 */
void stm32_lib_gpio_set_port(GPIO_TypeDef* gpio, uint16_t value)
{
    if (!gpio) return;
    gpio->ODR = value;
}

/**
 * @brief Enable timer
 * @param timer Timer peripheral
 */
void stm32_lib_timer_enable(TIM_TypeDef* timer)
{
    if (!timer) return;
    
    if (timer == TIM14) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    } else if (timer == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
}

/**
 * @brief Disable timer
 * @param timer Timer peripheral
 */
void stm32_lib_timer_disable(TIM_TypeDef* timer)
{
    if (!timer) return;
    
    if (timer == TIM14) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
    } else if (timer == TIM2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
    }
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
    
    timer->ARR = arr;
    timer->PSC = psc;
    timer->CR1 |= TIM_CR1_CEN;
}

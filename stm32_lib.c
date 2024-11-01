//==========================================================================*
//	INCLUDES
//==========================================================================*
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"

//==========================================================================*
//	GLOBAL VARIABLES
//==========================================================================*
static int bcdArray[5];

//==========================================================================*
//	FUNCTION PROTOTYPES
//==========================================================================*
void init_EXTI(void);
void init_NVIC(void);
void init_Ports(void);
void init_ADC(void);
void init_Tim6(void);
void init_PWM(void);
void TIM6_DAC_IRQHandler(void);
int ADC_POT(int pot, int resolution);
int ADC_DATA(void);
void delay_ms(uint32_t counter);
void EXTI0_1_IRQHandler(void);

//==========================================================================*
//	FUNCTIONS
//==========================================================================*

// Initialize External Interrupt
void init_EXTI(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable clock for sys config controller
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // Map PA1 to EXTI1
    EXTI->IMR |= EXTI_IMR_MR1; // Unmask EXTI1 in interrupt mask register
    EXTI->FTSR |= EXTI_FTSR_TR1; // Falling trigger enable for input line 1 (SW1)
}

// Initialize Ports
void init_Ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN; // Enable clock for GPIOA and GPIOB
    GPIOB->MODER |= 0x505555; // Set PB0-7 as output mode, PB10 (red) and PB11 (green)
    GPIOA->PUPDR |= 0b01010101; // Enable pull-up resistors for SW0-3
    GPIOA->MODER &= 0xFFFFF0000; // Set PA0-3 to input mode
}

// Initialize NVIC
void init_NVIC(void) {
    NVIC_EnableIRQ(EXTI0_1_IRQn); // Enable EXTI0_1 in NVIC
}

// Initialize ADC
void init_ADC(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable ADC clock
    ADC1->CFGR1 |= 0x10; // Set ADC resolution
    ADC1->CHSELR = 0b100000; // Select channel for pot-0
    ADC1->CR |= ADC_CR_ADEN; // Enable ADC
    while ((ADC1->ISR & 0x01) == 0); // Wait for ADC ready
}

// Initialize Timer 6
void init_Tim6(void) {
    RCC->APB1ENR |= (1 << 4); // Enable TIM6 clock
    TIM6->PSC = 2829; // Set prescaler
    TIM6->ARR = 2829; // Set auto-reload register
    TIM6->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable counter and auto-reload preload
    NVIC_EnableIRQ(TIM6_IRQn); // Enable TIM6 interrupt
}

// Initialize PWM
void init_PWM(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock for GPIOB
    GPIOB->MODER |= 0b1000000000; // Set PB1 to Alternate Function mode
    GPIOB->AFR[1] |= GPIO_AFRL_AFRL1; // Set PB1 to AF1 (TIM3 CH1)

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 clock
    TIM3->ARR = 2829; // Set PWM frequency
    TIM3->CCR1 = 32768; // Set PWM duty cycle
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable TIM3 count and auto-reload preload

    TIM3->EGR |= TIM_EGR_UG | TIM_EGR_CC1G; // Re-initialize counter and generate capture/compare event
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1; // Set TIM3 to PWM mode 1
}

// Timer 6 Interrupt Handler
void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~(1 << 0); // Acknowledge interrupt request
}

// ADC Potentiometer Reading
int ADC_POT(int pot, int resolution) {
    ADC1->CFGR1 |= (resolution << 3); // Configure resolution
    ADC1->CHSELR = (1 << (5 + pot)); // Channel select (Pot0 = PA5, Pot1 = PA6)
    ADC1->CR |= (0b1 << 2); // Start conversion
    while ((ADC1->ISR & 0b100) == 0); // Wait for conversion
    return ADC1->DR; // Return value
}

// ADC Data Reading
int ADC_DATA(void) {
    ADC1->CR |= ADC_CR_ADSTART; // Start ADC conversion
    while ((ADC1->ISR & 0b100) == 0); // Wait for end of conversion
    return ADC1->DR; // Return data
}

// Millisecond Delay
void delay_ms(uint32_t delaylength) {
    int counter = delaylength * 735;
    while (counter > 0) {
        counter--;
    }
}

// External Interrupt Handler
void EXTI0_1_IRQHandler(void) {
    delay_ms(200); // Debouncing
    EXTI->PR |= EXTI_PR_PR1; // Clear interrupt pending bit
}
















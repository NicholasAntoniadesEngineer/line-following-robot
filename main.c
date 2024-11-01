/******************************************************************************
 * @file    main.c
 * @brief   Main program for STM32 line-following car
 * @version 1.0
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdint.h>

/* Macros --------------------------------------------------------------------*/
#define GPIO_AFRH_AFR10_AF2 ((uint32_t)0x00000200)
#define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)
#define GPIO_AF2 ((uint32_t)0x00000002)

/* Global Constants ----------------------------------------------------------*/
uint16_t centi_sec = 0;

/* Function Prototypes -------------------------------------------------------*/
void init_Ports(void);
void init_ADC(void);
void init_PWM(void);
int ADC_POT(int pot, int resolution);
int ADC_DATA(void);
void delay_ms(uint32_t counter);
int softstart(void);
int Pot_PWM(void);
void init_NVIC(void);
void init_TIM14(void);
void TIM14_IRQHandler(void);
void handle_brake(void);
void handle_drive(void);
void handle_softstart(void);
void handle_reverse(void);

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  Initialize GPIO ports
 */
void init_Ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 |
                     GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                     GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER12_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);
}

/**
 * @brief  Initialize ADC
 */
void init_ADC(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    ADC1->CFGR1 |= 0x10;
    ADC1->CHSELR = 0b100000;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & 0x01));
}

/**
 * @brief  Initialize PWM
 */
void init_PWM(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2 & (GPIO_AF2 << 8)) | (GPIO_AFRH_AFR11_AF2 & (GPIO_AF2 << 12));
    TIM2->ARR = 8000;
    TIM2->PSC = 0;
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief  Read ADC value from potentiometer
 * @param  pot: Potentiometer number
 * @param  resolution: ADC resolution
 * @retval ADC value
 */
int ADC_POT(int pot, int resolution) {
    ADC1->CFGR1 = (ADC1->CFGR1 & ~(0x3 << 3)) | (resolution << 3);
    ADC1->CHSELR = (1 << (5 + pot));
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & 0b100));
    return ADC1->DR;
}

/**
 * @brief  Read ADC data
 * @retval ADC data
 */
int ADC_DATA(void) {
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & 0b100));
    return ADC1->DR;
}

/**
 * @brief  Delay in milliseconds
 * @param  delaylength: Length of delay
 */
void delay_ms(uint32_t delaylength) {
    volatile int counter = delaylength * 735;
    while (counter-- > 0);
}

/**
 * @brief  Soft start function
 * @retval Status
 */
int softstart(void) {
    int i;
    for (i = 1; i < 10000; i++) {
        float percent_j = i / 10000.0;
        TIM2->CCR4 = percent_j * 80000; // j * ARR / 100
    }
    return 0;
}

/**
 * @brief  Potentiometer PWM control
 * @retval Status
 */
int Pot_PWM(void) {
    while (1) {
        float P0_val = ADC_POT(0, 8);
        float P1_val = ADC_POT(1, 8);
        TIM2->CCR3 = (P0_val / 255) * 80000;
        TIM2->CCR4 = (P1_val / 255) * 80000;

        if (!(GPIOA->IDR & (GPIO_IDR_0 | GPIO_IDR_1 | GPIO_IDR_2 | GPIO_IDR_3))) {
            return 0;
        }
    }
}

/**
 * @brief  Initialize TIM14
 */
void init_TIM14(void) {
    TIM14->PSC = 1;
    TIM14->ARR = 39999;
    TIM14->DIER |= TIM_DIER_UIE;
}

/**
 * @brief  Initialize NVIC
 */
void init_NVIC(void) {
    NVIC_EnableIRQ(TIM14_IRQn);
}

/**
 * @brief  TIM14 interrupt handler
 */
void TIM14_IRQHandler(void) {
    TIM14->SR &= ~TIM_SR_UIF;
    if (++centi_sec == 25) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
        centi_sec = 0;
    }
}

/**
 * @brief  Handle brake
 */
void handle_brake(void) {
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    GPIOB->ODR = 0;
    GPIOB->ODR |= 0b10000000;
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    init_TIM14();
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
    lcd_command(CLEAR);
    lcd_putstring("     Brake");
    lcd_command(LINE_TWO);
    lcd_putstring("    Activated");
}

/**
 * @brief  Handle drive
 */
void handle_drive(void) {
    GPIOB->ODR = 0b01;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    lcd_command(CLEAR);
    lcd_putstring("Driving motor...");
}

/**
 * @brief  Handle soft start
 */
void handle_softstart(void) {
    GPIOB->ODR = 0b101;
    lcd_command(CLEAR);
    lcd_putstring("    Softstart");
    lcd_command(LINE_TWO);
    lcd_putstring("    Activated");
    softstart();
    handle_drive();
}

/**
 * @brief  Handle reverse
 */
void handle_reverse(void) {
    GPIOB->ODR = 0;
    lcd_command(CLEAR);
    lcd_putstring("     Reverse");
}

/**
 * @brief  Main function
 * @retval int
 */
int main(void) {
    init_Ports();
    init_ADC();
    init_LCD();
    init_PWM();
    init_NVIC();

    lcd_command(CLEAR);
    lcd_putstring("SW0: Brakes");
    lcd_command(LINE_TWO);
    lcd_putstring("SW1: Reactivate");

    while (1) {
        Pot_PWM();

        if (!(GPIOA->IDR & GPIO_IDR_0)) {
            handle_brake();
        } else if (!(GPIOA->IDR & GPIO_IDR_1)) {
            handle_drive();
        } else if (!(GPIOA->IDR & GPIO_IDR_2)) {
            handle_softstart();
        } else if (!(GPIOA->IDR & GPIO_IDR_3)) {
            handle_reverse();
        }
    }
}

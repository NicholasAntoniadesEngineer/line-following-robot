/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.1.1   2017-03-29

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/
/*==========================================================================*
 * 																			*
 * 					EEE3017W: DIGITAL ELECTRONICS - 2017					*
 * 								PRACTICAL 1									*
 * 																			*
 *==========================================================================*
 *
 *	Group #:
 *
 *	Student #1:
 *	Student #2:
 *
 ****************************************************************************
 */
//==========================================================================*
//	INCLUDES
//==========================================================================*
	/*Include your header files here*/

#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
//==========================================================================*
//	MACROS
//==========================================================================*
	/*Define your macros here*/

//==========================================================================*
//	GLOBAL CONSTANTS
//==========================================================================*
	/*Define your global constants here*/

//==========================================================================*
//	GLOBAL VARIABLES
//==========================================================================*
	/*Define your global variables here*/
	static int bcdArray[5];

//==========================================================================*
//	FUNCTION PROTOTYPES
//==========================================================================*
	/*Declare your functions here*/
	void init_EXTI(void);
	void init_NVIC(void);
	void init_Ports(void);
	void init_ADC(void);
	void init_Tim6(void);
	void init_PWM(void);
	void TIM6_DAC_IRQHandler(void);
	int ADC_POT(int pot, int resolution);
	int ADC_DATA(void);
	void delay_ms (uint32_t counter);
	void EXTIO_1_IRQHandler(void);

//==========================================================================*
//	FUNCTIONS
//==========================================================================*

	/*Write your function descriptions here*/
	void init_EXTI(void)
		{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; //enables the clock for the sys config controller.
		SYSCFG->EXTICR[0] |=SYSCFG_EXTICR1_EXTI1_PA; //Map PA1 to EXTI1.
		EXTI->IMR |=EXTI_IMR_MR1; //unmasks EXTI1 in interrupt mask register
		EXTI->FTSR |= EXTI_FTSR_TR1; //falling trigger enable for input line 1 (SW1)
		}
	void init_Ports(void)
	{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable clock for GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for GPIOA
	GPIOB->MODER |= 0x505555;    //set PB0-7 as output mode as well as PB10 (red) and PB11 (green)
	GPIOA->PUPDR |=0b01010101;  //enable pull up resistors for SW0-3
	GPIOA->MODER &= 0xFFFFF0000;  //set PA0-3 to input mode.
	}
	void init_NVIC(void)
		{
		NVIC_EnableIRQ(EXTI0_1_IRQn); //Enable EXTI0_1 in NVIC
		}
	void init_ADC(void)
		{
		RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; //Enabling the ADC clock in the RCC APB  periperal clock enableregister
		ADC1 -> CFGR1 |= 0x10;               //Setting the ADC resolution in the ADC configuration register 1
		ADC1->CHSELR = 0b100000;				 // Selecting the chanel for pot-0.
		ADC1 -> CR |= ADC_CR_ADEN;           //Enabling the ADC
		while((ADC1 -> ISR & 0x01) ==0 );    //Waiting for the ADRDY pin to let us know the ADC is ready

		}
void init_Tim6(void)
	{
	RCC->APB1ENR |= (1 << 4);
	TIM6 -> PSC = 2829;
	TIM6 -> ARR = 2829;
	TIM6 -> DIER |= TIM_DIER_UIE;
	TIM6 -> CR1 |= TIM_CR1_CEN;
	TIM6-> CR1 |= TIM_CR1_ARPE;
	NVIC_EnableIRQ(TIM6_IRQn);
	}

void init_PWM(void){
	//Set PB4 to be in Alternate function 1, which is TIM3 CH1
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable clock for GPIOB
	GPIOB->MODER |= 0b1000000000;	//Setting PB1 to Alternate Function mode
	GPIOB->AFR[1] |= GPIO_AFRL_AFRL1;		//Setting PB1 to AF1, which is TIM3 CH1

	//Enabling PWM mode of TIM3 by setting it to Output Compare mode
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;				//Enabling clock for TIM3
	TIM3 -> ARR = 2829;						//Setting PWM frequency
	TIM3 -> CCR1 = 32768; 					//Setting PWM duty cycle
	TIM3 -> CR1 |= TIM_CR1_CEN;				//Enabling TIM3 count
	TIM3 -> CR1 |= TIM_CR1_ARPE;			//Setting auto reload preload enabled

	TIM3 -> EGR |= TIM_EGR_UG;				//Re-initialize counter
	TIM3 -> EGR |= TIM_EGR_CC1G;			//Generate a capture and compare event on channel 1
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_1;		//Setting TIM3 to PWM mode 1
}


	void TIM6_DAC_IRQHandler(void)
		{
	    TIM6->SR &=~(1<<0); //acknowledge interupt request.
		}
	int ADC_POT(int pot, int resolution){
	    /* configure resolution,
	    0: 12 bits
	    1: 10 bits
	    2: 8 bits
	    3: 6 bits */
	    ADC1->CFGR1 |= (resolution << 3);
	    // channel select where Pot0 = PA5, Pot1 = PA6
	    ADC1->CHSELR = (1 << (5 + pot));
	    // start conversion
	    ADC1->CR |= (0b1 << 2);
	    // wait for conversion
	    while ((ADC1->ISR & 0b100) == 0);
	    //return value
	    return ADC1->DR;
	}
	int ADC_DATA(void)
		{
		ADC1 ->CR |= ADC_CR_ADSTART;		// Starts ADC conversion
		while((ADC1 -> ISR & 0b100) ==0 );	// Waits for the End of conversion flag to be set
		return ADC1 -> DR;
		}
	void delay_ms (uint32_t delaylength)
	{
		int counter = delaylength*735;
		while(counter>0)
		{
			counter--;
		}
	}
	void EXTI0_1_IRQHandler(void)
		{
		delay_ms(200);									//debouncing
		EXTI -> PR |= EXTI_PR_PR1; 						//clear the interrupt pending bit
		}
//==========================================================================*

/****************************************************************************
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void){

	init_NVIC();
	init_ADC();
	initTim6();
	init_PWM();

  while (1)
  {
	  GPIOB -> ODR = ADC_POT(1,10);
	  delay_ms(50);
	}
}
















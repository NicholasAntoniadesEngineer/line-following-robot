
//==========================================================================*
//	INCLUDES
//==========================================================================*

#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdint.h>

//==========================================================================*
//	MACROS
//==========================================================================*

#define GPIO_AFRH_AFR10_AF2 ((uint32_t)0x00000200)
#define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)
#define GPIO_AF2 ((uint32_t)0x00000002)
//==========================================================================*
//	GLOBAL CONSTANTS
//==========================================================================*


//==========================================================================*
//	GLOBAL VARIABLES
//==========================================================================*

int last_state = 0;

//==========================================================================*
//	FUNCTION PROTOTYPES
//==========================================================================*

	void init_Ports(void);
	void init_ADC(void);
	void init_PWM(void);
	int ADC_POT(int pot, int resolution);
	int ADC_DATA(void);
	void delay_ms (uint32_t counter);
	int softstart(void);
	void init_NVIC();
	void init_TIM14();
	void TIM14_IRQHandler();

//==========================================================================*
//	FUNCTIONS
//==========================================================================*
	void init_Ports(void)
	{
		RCC-> AHBENR |= RCC_AHBENR_GPIOBEN;
		GPIOB-> MODER |= GPIO_MODER_MODER0_0;
		GPIOB-> MODER |= GPIO_MODER_MODER1_0;
		GPIOB-> MODER |= GPIO_MODER_MODER2_0;
		GPIOB-> MODER |= GPIO_MODER_MODER3_0;
		GPIOB-> MODER |= GPIO_MODER_MODER4_0;
		GPIOB-> MODER |= GPIO_MODER_MODER5_0;
		GPIOB-> MODER |= GPIO_MODER_MODER6_0;
		GPIOB-> MODER |= GPIO_MODER_MODER7_0;
		GPIOB-> MODER |= GPIO_MODER_MODER12_0;


		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		GPIOA->MODER &=~ GPIO_MODER_MODER0;
		GPIOA->MODER &=~ GPIO_MODER_MODER1;
		GPIOA->MODER &=~ GPIO_MODER_MODER2;
		GPIOA->MODER &=~ GPIO_MODER_MODER3;
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0;
	}
	void init_ADC(void)
		{
		RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; //Enabling the ADC clock in the RCC APB  periperal clock enableregister
		ADC1 -> CFGR1 |= 0x10;               //Setting the ADC resolution in the ADC configuration register 1
		ADC1->CHSELR = 0b100000;	     // Selecting the chanel for pot-0.
		ADC1 -> CR |= ADC_CR_ADEN;           //Enabling the ADC
		while((ADC1 -> ISR & 0x01) ==0 );    //Waiting for the ADRDY pin to let us know the ADC is ready.
		}
	void init_PWM(void){
		/*Macros to be defined*/
		//#define GPIO_AFRH_AFR10_AF2 ((uint32_t)0x00000200)
		//#define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)


		//Initialising clocks and pins for PWM output
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 	  // Enable clock for GPIOB
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	  // Enable TIM2
		GPIOB->MODER |= GPIO_MODER_MODER10_1; // set PB10 to AF
		GPIOB->MODER |= GPIO_MODER_MODER11_1; // set PB11 to AF

		//Choosing AF for pins, MAPPING them to TIM2 CH3 and CH4
		GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2&(GPIO_AF2<<8)); //Enable AF2 for PB10 in GPIOB AFR10
		GPIOB->AFR[1] |= (GPIO_AFRH_AFR11_AF2&(GPIO_AF2<<12));//Enable AF2 for PB11 in GPIOB AFR11

		//Setting signal frequency
		TIM2->ARR = 8000; // ARR = 48000 for f =1kHz
		TIM2->PSC = 0;

		// specify PWM mode: OCxM bits in CCMRx. We want mode 1
		TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
		TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1

		//PWM Duty cycle based on fractions of ARR
		TIM2->CCR3 = 0;
		TIM2->CCR4 = 0;

		// Enable output compare Sfor CH3 and CH4
		TIM2->CCER |= TIM_CCER_CC3E; //Compare 3 output enable
		TIM2->CCER |= TIM_CCER_CC4E; //Compare 4 output enable
		TIM2->CR1 |= TIM_CR1_CEN;    //Counter enable
		}
	int ADC_POT(int pot, int resolution){
	    /* configure resolution,
	    0: 12 bits
	    1: 10 bits
	    2: 8 bits
	    3: 6 bits */
	    ADC1->CFGR1 |= (resolution << 3);	// configure resolution,
	    ADC1->CHSELR = (1 << (5 + pot));	// channel select where Pot0 = PA5, Pot1 = PA6
	    ADC1->CR |= (0b1 << 2); 		// start conversion
	    while ((ADC1->ISR & 0b100) == 0);	// wait for conversion
	    return ADC1->DR;//return value
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

	int softstart(void)
	{
		//Soft start for PWM on PB10 and PB11
		//Linear increase of PWM from 0->100%

		/*Neccesary initializations*/
		//init_PWM();

		int i;
		int j;
		float percent_j;

		TIM2->CCR4 = 0;
		TIM2->CCR3 = 0;

		while(1){
		  for(i = 1; i<10000;i++){
			  percent_j = (i/10000.0);
			  j = percent_j*100;
			  //TIM2->CCR3 = j*800;
			  TIM2->CCR4 = j*800;		//j*ARR/100
		  }
		  return 0;
		}
	 }


	int Pot_PWM(void){

		int P0;
		int P1;
		float P0_val;
		float P1_val;
		float percentage_P0;
		float percentage_P1;

		while(1){
			P0_val = ADC_POT(0,8);
			P1_val = ADC_POT(1,8);
			percentage_P0 = P0_val/255;
			percentage_P1 = P1_val/255;
			P0 = percentage_P0*100;
			P1 = percentage_P1*100;

			//At P0 = 1OO, P0*48 = ARR value, this is 100% duty cylce.
			//48 will change according to ARR value
			TIM2->CCR3 = P0 * 800;
			TIM2->CCR4 = P1 * 800;

			if ((GPIOA->IDR & GPIO_IDR_0)==0)
			{
				return 0;
			}
			if ((GPIOA->IDR & GPIO_IDR_1)==0)
			{
				return 0;
			}
			if ((GPIOA->IDR & GPIO_IDR_2)==0)
			{
				return 0;
			}
			if((GPIOA->IDR & GPIO_IDR_3)==0){
				return 0;
			}
		}

	}

	// Initialise TIM14
		void init_TIM14(){
			//RCC->APB1ENR|=RCC_APB1ENR_TIM14EN;		// Enable the TIM14 clock
			TIM14->PSC= 1;							// For f=8MHz PSC must =1
			TIM14->ARR= 39999;						// For f=8MHz ARR must = 39999
			TIM14->DIER|=TIM_DIER_UIE;				// Enable TIM14 interrupt
		}

		// Initialises the NVIC
		void init_NVIC(){
			NVIC_EnableIRQ(TIM14_IRQn);				// Unmask the TIM14 interrupt in NVIC
		}

		// Initialise and set the TIM14 handler
		void TIM14_IRQHandler(){
			TIM14->SR &= ~TIM_SR_UIF;						// Clear update flag

			}



//==========================================================================*

/**************************
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void){
	init_Ports();		// Initialise ports
	init_ADC();			// Initialise ADC
	init_LCD();
	init_PWM();			// Initialise peripherals for PWM
	init_NVIC();		// initialise the NNIC

	// Send direction signal by setting D0 high
	//GPIOB->ODR |= 0b01;

	//Display message
	lcd_command(CLEAR);
	lcd_putstring("SW0: Brakes");
	lcd_command(LINE_TWO);
	lcd_putstring("SW1: Reactivate");

	while (1)
	  {
		 Pot_PWM();
		// Call function to create PWM

		 //Braking sequence
		 if((GPIOA->IDR & GPIO_IDR_0)==0){
		// If PA0 pressed
		 TIM2->CCR3 = 0;
		 TIM2->CCR4 = 0;
		 GPIOB->ODR = 0;
		// set D0 low
		 GPIOB->ODR |=0b10000000;
		 RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		// Enable the TIM14 clock
		 init_TIM14();
		// Start the timer
		 RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
		// Disable TIM2 for PWM

		 //Display LCD
		 lcd_command(CLEAR);
		 lcd_putstring("     Brake");
		 lcd_command(LINE_TWO);
		 lcd_putstring("    Activated");
		 }

		 //To reactivate the pulse, press PA1
		 if((GPIOA->IDR & GPIO_IDR_1)==0){
		 GPIOB->ODR=0b01;
		// Set D0 high
		 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		// Enable TIM2 for PWM

		 //Display LCD
		 lcd_command(CLEAR);
		 lcd_putstring("Driving motor...");
		 }

		 if((GPIOA->IDR & GPIO_IDR_2)==0){
		 GPIOB->ODR=0b101;
		// Softstart LED
		 lcd_command(CLEAR);
		 lcd_putstring("    Softstart");
		 lcd_command(LINE_TWO);
		 lcd_putstring("    Activated");
		 softstart();
		// calls softstart function
		 GPIOB->ODR = 0b01;
		 lcd_command(CLEAR);
		 lcd_putstring("Driving motor...");

		 }

		 if((GPIOA->IDR & GPIO_IDR_3)==0){
			 if (last_state == 1){
				 GPIOB->ODR = 0b0;
				 lcd_command(CLEAR);
				 lcd_putstring("     Foreward");
				 last_state = 0;
			 }
			 else if(last_state == 0){
				 GPIOB->ODR = 0b1;
				 lcd_command(CLEAR);
				 lcd_putstring("     Reverse");
				 last_state = 1;
			 }
		 }
	  }


		}


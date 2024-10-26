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

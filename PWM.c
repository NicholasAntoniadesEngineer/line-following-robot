void PWM_configure_GPIO(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock for GPIOB
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // Set PB10 and PB11 to AF
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10_AF2 & (0b10 << 8)) | (GPIO_AFRH_AFR11_AF2 & (0b10 << 12)); // Enable AF2 for PB10 and PB11
}

void PWM_configure_TIM2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1 for CH3 and CH4
    TIM2->ARR = 48000; // Set auto-reload register for 1 KHz frequency
    TIM2->PSC = 0; // Set prescaler
    TIM2->CCR3 = 0 * 480; // Set duty cycle for CH3
    TIM2->CCR4 = 20 * 480; // Set duty cycle for CH4
    TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable output compare for CH3 and CH4
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
}

void PWM_init(void) {
    configure_GPIO_for_PWM();
    configure_TIM2_for_PWM();
}

void PWM_softstart(uint32_t max_value, uint32_t step_delay) {
    // Soft start for PWM on PB10 and PB11
    // Linear increase of PWM from 0 to 100%

    // Initialize PWM if necessary
    // init_PWM();

    float percent_j;
    uint32_t j;

    TIM2->CCR4 = 0;
    TIM2->CCR3 = 0;

    for (uint32_t i = 1; i <= max_value; i++) {
        percent_j = (i / (float)max_value);
        j = percent_j * 100;
        TIM2->CCR3 = j * 480;
        TIM2->CCR4 = j * 480;

        // Delay to control the speed of the soft start
        delay_ms(step_delay);
    }
}

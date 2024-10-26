	void softstart(void)
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

	  while (1)
	  {
		  for(i = 1; i<40000;i++){
			  percent_j = (i/40000.0);
			  j = percent_j*100;
			  TIM2->CCR3 = j*480;
			  TIM2->CCR4 = j*480;
		  }
			/*i = 0;
			percent_j = 0;
			j = 0;
			TIM2->CCR4 = 0;
			TIM2->CCR3 = 0;*/
	 }
	 }

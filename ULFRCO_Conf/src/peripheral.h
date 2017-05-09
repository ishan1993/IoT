/*
 * Code consists Built in functions referred to from Silicon Labs Leopard Gecko Software Documentation
 * they are copy righted by Silicon Labs’ in 2015 and Silicon Labs’ grants
 * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,
 * and redistribute it freely subject that the origins is not misrepresented, altered source version must be
 * plainly marked, and this notice cannot be altered or removed from any source distribution.
 * ********************************************************************************
 * * @section License
 * * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 * *******************************************************************************
*/


#ifndef SRC_PERIPHERAL_H
#define SRC_PERIPHERAL_H_

/*************************************************************/
/* Routine to calibrate ULFRCO			                     */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - error in ULFRCO						 */
/*************************************************************/
float ulfrco_calibration(void);


/*************************************************************/
/* Routine to Enable and configure clocks                    */
/* Input Variables - None									 */
/* Global Variables - EM_State is used to decide lowest		 */
/* 			          allowed energy mode					 */
/* Return Variables - None									 */
/* Prescaling logic written based on example from 			 */
/* Professor Keith Graham's example code 					 */
/*************************************************************/
void CMU_Setup(void);


/*************************************************************/
/* Routine to Enable and configure LETIMER                   */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LETIMER_Setup(void);


/*************************************************************/
/* Routine to Enable and configure ACMP                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void ACMP_Setup(void);


/*************************************************************/
/* Routine to Enable and configure GPIO                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void GPIO_Setup(void);


/*************************************************************/
/* Routine to turn on LED                                    */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_On(void);


/*************************************************************/
/* Routine to turn off LED                                   */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_Off(void);


/*************************************************************/
/* Routine to toggle LED                                     */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_Toggle(void);


/* Variable to track LED's state */
int LED = 0;

/* ACMP configuration constant table for LED ON. */
static const ACMP_Init_TypeDef initACMP_on =
{
	.fullBias = false,                 /* fullBias */
	.halfBias = true,                  /* halfBias */
	.biasProg =  0x0,                  /* biasProg */
	.interruptOnFallingEdge =  false,  /* interrupt on rising edge */
	.interruptOnRisingEdge =  false,   /* interrupt on falling edge */
	.warmTime = acmpWarmTime256,       /* 256 cycle warmup to be safe */
	.hysteresisLevel = acmpHysteresisLevel4, /* hysteresis level 4 */
	.inactiveValue = false,            /* inactive value */
	.lowPowerReferenceEnabled = true, /* low power reference */
	.vddLevel = LIGHT_LOW_THRESH,                  /* VDD level */
	.enable = false                    /* Don't request enabling. */
};

/* ACMP configuration constant table for LED off. */
static const ACMP_Init_TypeDef initACMP_off =
{
  .fullBias = false,                 /* fullBias */
  .halfBias = true,                  /* halfBias */
  .biasProg =  0x0,                  /* biasProg */
  .interruptOnFallingEdge =  false,  /* interrupt on rising edge */
  .interruptOnRisingEdge =  false,   /* interrupt on falling edge */
  .warmTime = acmpWarmTime256,       /* 256 cycle warmup to be safe */
  .hysteresisLevel = acmpHysteresisLevel4, /* hysteresis level 4 */
  .inactiveValue = false,            /* inactive value */
  .lowPowerReferenceEnabled = true, /* low power reference */
  .vddLevel = LIGHT_HIGH_THRESH,                  /* VDD level */
  .enable = false                    /* Don't request enabling. */
};


/* Routine to calculate error in ULFRCO */
float ulfrco_calibration(void)
{
	float LFXO_count = 0;
	float ULFRCO_count = 0;

	/* Enable clocks for TIMER modules and LFXO for LETIMER */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	/* Conf for TIMER0 */
	TIMER_Init_TypeDef timer0Init =
	{
	    .enable     = true,
	    .debugRun   = false,
	    .clkSel     = timerClkSelHFPerClk,
	    .fallAction = timerInputActionNone,
	    .riseAction = timerInputActionNone,
	    .mode       = timerModeUp,
	    .dmaClrAct  = false,
	    .quadModeX4 = false,
	    .oneShot    = false,
	    .sync       = true,
	};

	/* Conf for TIMER1 */
	TIMER_Init_TypeDef timer1Init =
	{
		.enable     = true,
		.debugRun   = false,
		.clkSel     = timerClkSelCascade,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = true,
	};

	/* Conf for LETIMER0 */
	LETIMER_Init_TypeDef letimerInit =
	{
		.enable         = true,                   /* Start counting when init completed. */
		.debugRun       = false,                  /* Counter shall not keep running during debug halt. */
		.rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
		.rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
		.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
		.bufTop         = true,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
		.out0Pol        = 0,                      /* Idle value for output 0. */
		.out1Pol        = 0,                      /* Idle value for output 1. */
		.ufoa0          = letimerUFOANone,         /* PWM output on output 0 */
		.ufoa1          = letimerUFOANone,        /* No output on output 1*/
		.repMode        = letimerRepeatOneshot       /* Count until stopped */
	};

	/* Initialize count for timers to 0 */
	TIMER_CounterSet(TIMER0,0);
	TIMER_CounterSet(TIMER1,0);

	/* Initialize LETIMER top value to 32769 */
	LETIMER_CompareSet(LETIMER0,0,32769);
	LETIMER_RepeatSet(LETIMER0,0,1);
	LETIMER_RepeatSet(LETIMER0,1,1);

	/* Initialize and start TIMERs */
	TIMER_Init(TIMER0, &timer0Init);
	TIMER_Init(TIMER1, &timer1Init);

	/* Initialize and start LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Poll LETIMER till 0 and get the count */
	while(LETIMER_CounterGet(LETIMER0) != 1);
	LFXO_count = ((float)TIMER_CounterGet(TIMER0)) + (((float)TIMER_CounterGet(TIMER1))*((float)TIMER_TopGet(TIMER0)));

	/* Reset timers */
	TIMER_Reset(TIMER0);
	TIMER_Reset(TIMER1);
	LETIMER_Reset(LETIMER0);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* change LF clock to ULFRCO from LFXO */
	CMU_ClockEnable(cmuClock_CORELE, false);
	CMU_ClockEnable(cmuClock_LETIMER0, false);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	/* configure start values for TIMERs and LETIMER */
	TIMER_CounterSet(TIMER0,0);
	TIMER_CounterSet(TIMER1,0);
	LETIMER_CompareSet(LETIMER0,0,1001);
	LETIMER_RepeatSet(LETIMER0,0,1);
	LETIMER_RepeatSet(LETIMER0,1,1);

	/* Start timers again */
	TIMER_Init(TIMER0, &timer0Init);
	TIMER_Init(TIMER1, &timer1Init);
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Poll LETIMER till 0 and get the count*/
	while(LETIMER_CounterGet(LETIMER0) != 1);
	ULFRCO_count = ((float)TIMER_CounterGet(TIMER0)) + (((float)TIMER_CounterGet(TIMER1))*((float)TIMER_TopGet(TIMER0)));

	/*reset and disable timers */
	TIMER_Reset(TIMER0);
	TIMER_Reset(TIMER1);
	LETIMER_Reset(LETIMER0);
	CMU_ClockEnable(cmuClock_CORELE, false);
	CMU_ClockEnable(cmuClock_LETIMER0, false);
	CMU_ClockEnable(cmuClock_HFPER, false);
	CMU_ClockEnable(cmuClock_TIMER0, false);
	CMU_ClockEnable(cmuClock_TIMER1, false);

	/*calculate the error */
	ULFRCO_CAL = LFXO_count / ULFRCO_count;
	return(ULFRCO_CAL);
}

/* Enable necessary clocks */
inline void CMU_Setup(void)
{
	CMU_HFRCOBandSet(HF_FREQ);		// Set HF clock to 7 MHz band

	/* Check if ULFRCO calibration flag is set and if so do the calibration */
	if(CALIBRATION)
	{
		ulfrco_calibration();
	}
	else
	{
		ULFRCO_CAL = 1;
	}

	if(EM_State < EM3)
	{
		CAL = 1;
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

		/* Prescale LFXO if necessary */
		int Desired_Period = ((int)(PERIOD+1)) * ((int)PERIOD+1) * (CMU_ClockFreqGet(cmuClock_LFA));
		int temp = Desired_Period / 1;
		int Prescaled_two_power = 1;
		while (temp > TIMER_TOP)
		{
			prescaler++;
			Prescaled_two_power = Prescaled_two_power * 2;
			temp = Desired_Period / Prescaled_two_power;
		}
		CMU->LFAPRESC0 |= (prescaler<<LFAPRESC0_SHIFT);
		prescaler = Prescaled_two_power;
	}
	else
	{
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
		CAL = ULFRCO_CAL;
		prescaler = 1;
	}
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ACMP0, true);

	return;
}

/* Enable and Configure LETIMER */
inline void LETIMER_Setup(void)
{
	Set_Sleep_Mode(EM_State);
	float freq = ((float)CMU_ClockFreqGet(cmuClock_LFA)) * CAL;
	int COMP0 = PERIOD * ((int)freq) / prescaler;
	int COMP1 = ON_TIME * ((int)freq) / prescaler;

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
		.enable         = true,                   /* Start counting when init completed. */
		.debugRun       = false,                  /* Counter shall not keep running during debug halt. */
		.rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
		.rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
		.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
		.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
		.out0Pol        = 0,                      /* Idle value for output 0. */
		.out1Pol        = 0,                      /* Idle value for output 1. */
		.ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
		.ufoa1          = letimerUFOANone,        /* No output on output 1*/
		.repMode        = letimerRepeatFree       /* Count until stopped */
	};

	/* COMP0 contains period and COMP1 contains the active period */
	LETIMER_CompareSet(LETIMER0,0,COMP0);
	LETIMER_CompareSet(LETIMER0,1,COMP1);

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Enable interrupts */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);
}

/* LETIMER IRQ Handler */
void LETIMER0_IRQHandler(void)
{
	INT_Disable();
	if(LETIMER_IntGetEnabled(LETIMER0) == LETIMER_IF_COMP1)
	{
		ACMP_Enable(ACMP0);
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
		GPIO_PinOutSet(LIGHT_EXCITE_PORT,LIGHT_PIN);
		while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));
		//Set_Sleep_Mode(EM1);
	}
	else
	{
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
		if(ACMP0->STATUS & ACMP_STATUS_ACMPOUT)
		{
			if(LED)
			{
				LED_Off();
				ACMP_Disable(ACMP0);
				ACMP_Init(ACMP0, &initACMP_on);
				ACMP_GPIOSetup(ACMP0, 0, false, false);
				ACMP_ChannelSet(ACMP0, ACMP_LIGHT_CHANNEL,ACMP_VDD_CHANNEL);

				LED = 0;
			}
			else
			{
				LED_On();
				ACMP_Disable(ACMP0);
				ACMP_Init(ACMP0, &initACMP_off);
				ACMP_GPIOSetup(ACMP0, 0, false, false);
				ACMP_ChannelSet(ACMP0, ACMP_VDD_CHANNEL, ACMP_LIGHT_CHANNEL);
				LED = 1;
			}
		}
		GPIO_PinOutClear(LIGHT_EXCITE_PORT,LIGHT_PIN);
		//Clear_Sleep_Mode(EM1);
	}
	INT_Enable();
}

/* Configure ACMP */
void ACMP_Setup(void)
{
  /* Configure ACMP. */
  ACMP_Init(ACMP0, &initACMP_on);
  /* Disable ACMP0 out to a pin. */
  ACMP_GPIOSetup(ACMP0, 0, false, false);
  ACMP_ChannelSet(ACMP0, ACMP_LIGHT_CHANNEL,ACMP_VDD_CHANNEL);

}

/* Enable and Configure GPIO */
inline void GPIO_Setup(void)
{
	/* enable GPIO E pin 2 (LED-0) as output */
	GPIO_PinModeSet(LED_PORT,LED_PIN, gpioModePushPull,0);
	/* Light sensor config */
	GPIO_PinModeSet(LIGHT_EXCITE_PORT,LIGHT_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LIGHT_INPUT_PORT,LIGHT_PIN, gpioModeInput,0);
}



/* Turn on LED */
inline void LED_On(void)
{
		GPIO_PinOutSet(LED_PORT,LED_PIN);
}

/* Turn off LED */
inline void LED_Off(void)
{
		GPIO_PinOutClear(LED_PORT,LED_PIN);
}

/* Toggle LED */
inline void LED_Toggle(void)
{
	GPIO_PinOutToggle(LED_PORT,LED_PIN);
}

#endif

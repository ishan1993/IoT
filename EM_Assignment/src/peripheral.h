#ifndef SRC_PERIPHERAL_H
#define SRC_PERIPHERAL_H_

/*************************************************************/
/* Routine to Enable and configure clocks                    */
/* Input Variables - None									 */
/* Global Variables - EM_State is used to decide lowest		 */
/* 			          allowed energy mode					 */
/* Return Variables - None									 */
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





/* Enable necessary clocks */
inline void CMU_Setup(void)
{
	CMU_HFRCOBandSet(7);		// Set HF clock to 7 MHz band
	if(EM_State < EM3)
	{
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	}
	else
	{
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	}
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	return;
}

/* Enable and Configure LETIMER */
inline void LETIMER_Setup(void)
{
	int freq = CMU_ClockFreqGet(cmuClock_LFA);
	int COMP0 = PERIOD * freq;
	int COMP1 = ON_TIME * freq;

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
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
		LED_On();
	}
	else
	{
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
		LED_Off();
	}
	INT_Enable();
	//LED_Toggle();
}

/* Enable and Configure GPIO */
inline void GPIO_Setup(void)
{
	/* enable GPIO E pin 2 (LED-0) as output */
	GPIO_PinModeSet(LED_PORT,LED_PIN, gpioModePushPull,0);
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

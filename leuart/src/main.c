/*************************************************************/
/* Author - Ishan Shah					                     */
/* Date - 03/20/2017     									 */
/* Course - IoT         									 */
/* Assignment - LEUART/BLE					   				 */
/*************************************************************/



/* Including general libraries. */
#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"
#include "em_common.h"
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

/* Including peripheral/module specific libraries. */
#include "em_emu.h"						// Energy management
#include "em_cmu.h"						// Clock management
#include "em_letimer.h"					// LETIMER
#include "em_timer.h"					// TIMER
#include "em_acmp.h"					// ACMP
#include "em_adc.h"						// ADC
#include "em_dma.h"						// DMA
#include "dmactrl.c"
#include "em_i2c.h"						// i2c
#include "em_leuart.h"

/* Macros and constants */
#define EM2_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM2
#define EM3_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM3
#define EM_SET EM1						// Lowest energy mode available for LETIMER
#define PERIOD 3.75						// Time Period for LETIMER
#define ON_TIME 0.004					// LED On time
#define LED_PORT gpioPortE				// LED GPIO port
#define LED_PIN	2						// LED0 Pin number
#define LED1_PIN 3						// LED1 Pin number
#define LIGHT_EXCITE_PORT gpioPortD 	// Light sensor excite port
#define LIGHT_INPUT_PORT gpioPortC		// Light sensor
#define LIGHT_PIN 6						// Light sensor pin
#define CALIBRATION 1					// Enable/Disable ULFRCO Calibration (1-Enable, 0-Disable)
#define LIGHT_LOW_THRESH 2				// Threshold for detecting darkness
#define LIGHT_HIGH_THRESH 61			// Threshold for detecting brightness
#define ACMP_LIGHT_CHANNEL acmpChannel6	// ACMP channel for light sensor
#define ACMP_VDD_CHANNEL acmpChannelVDD	// ACMP channel for scaled VDD
#define LFAPRESC0_SHIFT 8				// Bit shift for LFAPRESC0 configuration of prescaler
#define TIMER_TOP 65535					// TOP value for a 16 bit timer count register
#define HF_FREQ 14						// Frequency for the HF clock
#define ADC_DMA_ENABLE 1				// Determine whether to use DMA with ADC or not (1-Enable, 0-Disable)
#define ADC_EM 1						// Energy mode for ADC blocking
#define TEMP_UPPER_LIMIT 35				// Upper limit for temperature sensor
#define TEMP_LOWER_LIMIT 30				// lower limit for temperature sensor
#define CHANNEL_ADC_0 1					// ADC channel
#define NO_OF_ADCSAMPLES 500			// Number of ADC samples
#define ADC_Singleinit_Input    adcSingleInpTemp	// ADC configs
#define ADC_Singleinit_Ref      adcRef1V25
#define ADC_Singleinit_Resol    adcRes12Bit
#define ADC_Singleinit_AcqTime  adcAcqTime2
#define ADC_Select_HFPER_Freq   0
#define ADC_PRESCALAR 12
#define Dma_ChannelCfgSelect DMAREQ_ADC0_SINGLE	// DMA Configs
#define Dma_DestCfg_dstInc  dmaDataInc2			// DMA destination config
#define Dma_DestCfg_srcInc  dmaDataIncNone
#define Dma_DestCfg_size    dmaDataSize2
#define Dma_DestCfg_arbRate dmaArbitrate1
#define PASSIVE_LIGHT_ENABLE 1			// Enable/Disable on-board passive light sensor
#define ACTIVE_LIGHT_ENABLE 0			// Enable/Disable external light sensor
#define I2C_EM 1
// Used for i2c
#define I2C1_PORT   gpioPortC
#define I2C1_INTERRUPT_PORT gpioPortD
#define I2C1_SCL     5
#define I2C1_SDA     4
#define I2C1_InterruptPin  1

#define I2C1_VDD     0
#define SLAVE_ADDRESS 0x39
#define I2C_rx_buffer_size 10
#define TSL2561_UpperLimit 0x0800
#define TSL2561_LowerLimit 0x000f

// ADDRESS FOR REGISTERS ON TSL2651
#define TSL2561_CONTROL_REG          0x80
#define TSL2561_TIMING_REG			 0x81
#define TSL2561_THRESLowLow_REG      0x82
#define TSL2561_THRESLowhigh_REG     0x83
#define TSL2561_THRESHighLow_REG     0x84
#define TSL2561_THRESHighHigh_REG    0x85
#define TSL2561_INTERRUPT_REG        0x86

#define TSL2561_DATA0Low			 0x8c
#define TSL2561_DATA0High			 0x8d
#define TSL2561_DATA1Low			 0x8e
#define TSL2561_DATA1High			 0x8f

#define LEUART_EM EM2
#define LEUART0_GPIO    gpioPortD
#define LEUART0_TX_PIN  4
#define LEUART0_RX_PIN  5
#define CIRC_BUFFER_SIZE 20
#define LEUART_BAUD 9600
#define LEUART_STOP_BITS leuartStopbits1
#define LEUART_PARITY_BITS leuartNoParity
#define LEUART0_CHANNEL0 0
#define CHANNEL_NO 2
DMA_CB_TypeDef callBack[CHANNEL_NO];

// Parameters for DMA Channel config for LEUART0
#define LEUART0_Dma_ChannelCfgSelect DMAREQ_LEUART0_TXBL

// Parameters for DMA destination config
#define LEUART0_Dma_DestCfg_dstInc  dmaDataIncNone
#define LEUART0_Dma_DestCfg_srcInc  dmaDataInc1
#define LEUART0_Dma_DestCfg_size    dmaDataSize1
#define LEUART0_Dma_DestCfg_arbRate dmaArbitrate1
#define CBUF_SIZE 20

// Parameters for DMA Channel config for ADC0
#define Dma_ChannelCfgSelect DMAREQ_ADC0_SINGLE

// Parameters for DMA destination config
#define Dma_DestCfg_dstInc  dmaDataInc2
#define Dma_DestCfg_srcInc  dmaDataIncNone
#define Dma_DestCfg_size    dmaDataSize2
#define Dma_DestCfg_arbRate dmaArbitrate1

typedef struct cbuffer
{
    uint8_t* start;
    uint8_t* end;
    uint8_t* head;
    uint8_t* tail;
	uint8_t size;
    uint8_t count;
} cbuffer;


/* Enumerated data-type to list Energy modes. */
typedef enum Energy_Modes
{
	EM0,
	EM1,
	EM2,
	EM3
}Energy_Modes_Enum;

/* Global Variables */
Energy_Modes_Enum EM_State = EM_SET;	// Global flab for lowest allowed state
int sleep_block_counter[EM_SET+1];		// State blocking flags array
float ULFRCO_CAL = 0;					// Calibration ratio for ULFRCO
float CAL = 1;							// Generic Calibration
int prescaler = 0;						// Prescaler for LFXO
volatile int16_t ramBufferAdcData[NO_OF_ADCSAMPLES]; // Buffer for ADC samples
float avrgTemp = 20;      				//Making sure the LED 1 is turned off initially
volatile int led_status;
uint8_t ADC_High;
uint8_t ADC_Low;
uint8_t LED = 0;
uint8_t TempSend;
cbuffer tx_buffer;


/*************************************************************/
/* Routine to put device in a predefined sleep mode          */
/* Input Variables - None									 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Sleep routine written based on example from 				 */
/* silicon labs' example code 								 */
/*************************************************************/
void sleep(void);


/*************************************************************/
/* Routine to set a block for a sleep mode                   */
/* Input Variables - EM_State_local which is the mode needs	 */
/*					 to be blocked							 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Block Sleep routine written based on example from 		 */
/* silicon labs' example code 	 							 */
/*************************************************************/
void Set_Sleep_Mode(Energy_Modes_Enum EM_State_local);


/*************************************************************/
/* Routine to unblock a sleep mode    		                 */
/* Input Variables - EM_State_local which is the mode needs	 */
/*					 to be blocked							 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Unblock Sleep routine written based on example from 		 */
/* silicon labs' example code 								 */
/*************************************************************/
void Clear_Sleep_Mode(Energy_Modes_Enum EM_State_local);

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
/* Routine to Enable and configure ADC                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void ADC_Setup(void);


/*************************************************************/
/* Callback routine for DMA's ADC channel                    */
/* Input Variables - channel number, priority, user			 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void ADCdmaTransferDone(unsigned int channel, bool primary, void *user);


/*************************************************************/
/* Routine to convert ADC sample value into Celsius          */
/* Input Variables - ADC sample value						 */
/* Global Variables - None		 							 */
/* Return Variables - Temperature sample in Celsius			 */
/* Sleep routine written based on example from 				 */
/* silicon labs' example code 								 */
/*************************************************************/
float convertCelsius(int32_t adcSample);


/*************************************************************/
/* Routine to Enable and configure DMA                       */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void DMA_Setup(void);


/*************************************************************/
/* Routine to Enable and configure GPIO                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void GPIO_Setup(void);


/*************************************************************/
/* Routine to turn on LED0                                   */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_On(void);


/*************************************************************/
/* Routine to turn off LED0                                  */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_Off(void);


/*************************************************************/
/* Routine to turn on LED1                                   */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED1_On(void);


/*************************************************************/
/* Routine to turn off LED1                                  */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED1_Off(void);


/*************************************************************/
/* Routine to control LED1 according to temperature          */
/* Input Variables - None									 */
/* Global Variables - Average temperature					 */
/* Return Variables - None									 */
/*************************************************************/
void TempLed1_Control(void);


/*************************************************************/
/* Routine to toggle LED                                     */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LED_Toggle(void);


/*************************************************************/
/* Routine to Enable and configure I2C                       */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void I2C_Setup();


/*************************************************************/
/* Routine to Enable and configure External light sensor     */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void Enable_TSL2651();


/*************************************************************/
/* Routine to Disable External light sensor   			     */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void Disable_TSL2651();


/*************************************************************/
/* Routine to send a buffer via LEUART		   			     */
/* Input Variables - buffer									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void SendBufSamB11();


/*************************************************************/
/* Routine to Enable and configure leuart                    */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void leuart_Setup(void);


/*************************************************************/
/* Routine to initialize a circular buffer instance          */
/* Input Variables - Pointer to buffer and size needed		 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void cbuffer_init(cbuffer* buff, uint8_t buff_size);


/*************************************************************/
/* Routine to add one element to the buffer		             */
/* Input Variables - Pointer to buffer, element to be added  */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void cbuffer_add(cbuffer* buff, uint8_t* elem);


/*************************************************************/
/* Routine to remove one element to the buffer		         */
/* Input Variables - Pointer to buffer, element to be added  */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void cbuffer_remove(cbuffer* buff, uint8_t* elem);

void LEUART0dmaTransferDone(unsigned int channel, bool primary, void *user)
{
	(void) channel;
	(void) primary;
	(void) user;

	/* Disable DMA wake-up from LEUART0 TX */
	LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;

}

void dma_leuart0_init()
{
	//CMU_ClockEnable(cmuClock_DMA, true); // may be required

	//DMA_Init_TypeDef dmaInit;
	/* Initializing the DMA */

	/*HPROT signal state when accessing the primary/alternate
	 * descriptors. Normally set to 0 if protection is not an issue.*/

	/*
	 *  *   This function will reset and prepare the DMA controller for use. Although
 *   it may be used several times, it is normally only used during system
 *   init. If reused during normal operation, notice that any ongoing DMA
 *   transfers will be aborted. When completed, the DMA controller is in
 *   an enabled state*/
//	dmaInit.hprot        = 0;
//	dmaInit.controlBlock = dmaControlBlock;// giving the parameters for control block
//	DMA_Init(&dmaInit);

	//  Setting up call-back function
	callBack[LEUART0_CHANNEL0].cbFunc  = LEUART0dmaTransferDone;   // Associate call back function here
	callBack[LEUART0_CHANNEL0].userPtr = NULL;                 // User defined pointer to provide with call back function
	callBack[LEUART0_CHANNEL0].primary = true;

	DMA_CfgChannel_TypeDef  dmachnlCfg;
	// Setting up channel
	dmachnlCfg.highPri   = false;                 // Default priority
	dmachnlCfg.enableInt = true;                  // interrupt shall be enabled for channel
	dmachnlCfg.select    = LEUART0_Dma_ChannelCfgSelect;  // DMA channel select for ADC0_SINGLE
	dmachnlCfg.cb        = &callBack[LEUART0_CHANNEL0];               // Associating the call back function with DMA channel

	DMA_CfgChannel(LEUART0_CHANNEL0, &dmachnlCfg);
	DMA_CfgDescr_TypeDef    dmadescrCfg;

	// Setting up channel descriptor
	dmadescrCfg.dstInc  = LEUART0_Dma_DestCfg_dstInc;      // destination address: 2 bytes
	dmadescrCfg.srcInc  = LEUART0_Dma_DestCfg_srcInc;      // Source address:
	dmadescrCfg.size    = LEUART0_Dma_DestCfg_size;        // data size: 2 bytes
	dmadescrCfg.arbRate = LEUART0_Dma_DestCfg_arbRate;     // arbitrate:  set to zero
	dmadescrCfg.hprot   = 0;
	DMA_CfgDescr(LEUART0_CHANNEL0, true, &dmadescrCfg);

	/* Set new DMA destination address directly in the DMA descriptor */
	dmaControlBlock->DSTEND = &LEUART0->TXDATA;
}

int main(void)
 {
  /* Chip errata */
  CHIP_Init();

  CMU_Setup();
  LETIMER_Setup();
  GPIO_Setup();
  ACMP_Setup();
  ADC_Setup();
  I2C_Setup();
  leuart_Setup();
  dma_leuart0_init();
  cbuffer_init(&tx_buffer,(uint8_t)CBUF_SIZE);

  /* Sleep till IRQ */
  while (1)
  {
  	sleep();
  }
}


/* Routine to put device in a predefined sleep mode          */
inline void sleep(void)
{
       if(sleep_block_counter[EM0] > 0)
       {
              return;
       }
       else if(sleep_block_counter[EM1] > 0)
       {
    	   EMU_EnterEM1();
       }
       else if(sleep_block_counter[EM2] > 0)
       {
    	   EMU_EnterEM2(EM2_Restore);
       }
       else if(sleep_block_counter[EM3] > 0)
       {
    	   EMU_EnterEM3(EM3_Restore);
       }
       else
       {
    	   EMU_EnterEM3(EM3_Restore);
       }
       return;
}

/* Block sleep mode */
inline void Set_Sleep_Mode(Energy_Modes_Enum EM_State_local)
{
	INT_Disable();
	sleep_block_counter[EM_State_local]++;
	INT_Enable();
}

/* Unblock sleep mode */
inline void Clear_Sleep_Mode(Energy_Modes_Enum EM_State_local)
{
	INT_Disable();
	sleep_block_counter[EM_State_local]--;
	INT_Enable();
}

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
		.ufoa0          = letimerUFOANone,         /* PWM output on output 0 */
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
	if(PASSIVE_LIGHT_ENABLE)
	{
		LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
	}

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);
}

/* LETIMER IRQ Handler */
uint32_t adcSampleCount = 0;
int External_Sensor_Operation = 0;
void LETIMER0_IRQHandler(void)
{
	INT_Disable();
	if(LETIMER_IntGetEnabled(LETIMER0) == LETIMER_IF_COMP1)
	{
		if(PASSIVE_LIGHT_ENABLE)
		{
			ACMP_Enable(ACMP0);
			LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
			GPIO_PinOutSet(LIGHT_EXCITE_PORT,LIGHT_PIN);

			while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));
		}
	}
	else
	{
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

		if(ACTIVE_LIGHT_ENABLE)
		{
			switch (External_Sensor_Operation)
			{
				case 0:
					//blockSleepMode(I2C_EM);
					Enable_TSL2651();
					External_Sensor_Operation = 1;
					break;
				case 1:

					External_Sensor_Operation = 2;
					break;
				case 2:
					Disable_TSL2651();
					External_Sensor_Operation = 0;
					//unblockSleepMode(I2C_EM);
					break;
			}

		}

		if(PASSIVE_LIGHT_ENABLE)
		{
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
		}


		uint32_t adcSampleCount = 0;
		if(ADC_DMA_ENABLE)
			{
				Set_Sleep_Mode(ADC_EM);
				// Implementing the call back routine
				DMA_ActivateBasic(CHANNEL_ADC_0,  // DMA channel 0 to activate DMA cycle for.
								true,             // primary descriptor ??
								false,            // The burst feature is only used on peripherals supporting DMA bursts
								(void *)ramBufferAdcData, // dst: Address to start location to transfer data to
								(void *)&(ADC0->SINGLEDATA),// src: Address to start location to transfer data from
								NO_OF_ADCSAMPLES - 1); // Number of DMA transfer elements (minus 1)
				ADC_Start(ADC0, adcStartSingle);
			}
			else
			{
				Set_Sleep_Mode(ADC_EM);
				int calcTemp = 0;
				while(adcSampleCount < NO_OF_ADCSAMPLES)
				{
					ADC_Start(ADC0, adcStartSingle);
					while(ADC0->STATUS & ADC_STATUS_SINGLEACT);

					calcTemp += ADC_DataSingleGet(ADC0);
					adcSampleCount++;

				}

				Clear_Sleep_Mode(ADC_EM);               // Unblocking ADC from EM1
				ADC0->CMD |= ADC_CMD_SINGLESTOP;        // Stopping the ADC0
				avrgTemp = convertCelsius((float)calcTemp/NO_OF_ADCSAMPLES);  // Temp in centigrade scale
			}
			Set_Sleep_Mode(LEUART_EM);
			LED |= led_status;
			TempSend = (uint8_t)avrgTemp;
			cbuffer_add(&tx_buffer, (uint8_t*) &TempSend);
			cbuffer_add(&tx_buffer, (uint8_t*) &LED);
			CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
			//SendBufSamB11();
			DMA_ActivateBasic(LEUART0_CHANNEL0,
											true,             // primary descriptor ??
											false,            // The burst feature is only used on peripherals supporting DMA bursts
											(void *)&LEUART0->TXDATA, // dst: Address to start location to transfer data to
											(void *)&LED,// src: Address to start location to transfer data from
											1);
			TempLed1_Control();      // LED 1 Control
	}
	INT_Enable();
}

/* Configure ACMP */
void ACMP_Setup(void)
{
	if(PASSIVE_LIGHT_ENABLE)
	{
	  /* Configure ACMP. */
	  ACMP_Init(ACMP0, &initACMP_on);
	  /* Disable ACMP0 out to a pin. */
	  ACMP_GPIOSetup(ACMP0, 0, false, false);
	  ACMP_ChannelSet(ACMP0, ACMP_LIGHT_CHANNEL,ACMP_VDD_CHANNEL);
	}

}

/* Configure ADC */
void ADC_Setup(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);      // Enables the clock to ADC0 peripheral
	ADC_Init_TypeDef       adc0_init       = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef adc0_singleInit = ADC_INITSINGLE_DEFAULT;
	adc0_init.timebase = ADC_TimebaseCalc(ADC_Select_HFPER_Freq); // calculating the time base in accordance with HFPERCLK
	adc0_init.prescale = ADC_PRESCALAR; // Set AdcFreq
	ADC_Init(ADC0, &adc0_init);

	adc0_singleInit.input = ADC_Singleinit_Input;      // Single Init input as temp sensor
	adc0_singleInit.reference = ADC_Singleinit_Ref;    // Reference: 1.25 VDD
	adc0_singleInit.resolution = ADC_Singleinit_Resol; // Resolution: 12 bit
	adc0_singleInit.acqTime = ADC_Singleinit_AcqTime;  // Acquisition time


	if(ADC_DMA_ENABLE)
	{
		DMA_Setup();
		adc0_singleInit.rep = true;
	}
	else
	{
		adc0_singleInit.rep = false;                        // Rep: false
	}

	ADC_InitSingle(ADC0, &adc0_singleInit);
}

/* Callback routine for DMA's ADC channel */
DMA_CB_TypeDef ADC_cb;
void ADCdmaTransferDone(unsigned int channel, bool primary, void *user)
{
	int calcTemp = 0;
	Clear_Sleep_Mode(ADC_EM);               // Unblocking ADC from EM1
	ADC0->CMD |= ADC_CMD_SINGLESTOP;        // Stopping the ADC0
	for (uint16_t i =0; i<NO_OF_ADCSAMPLES;i++)
	{
		calcTemp += ramBufferAdcData[i];    // Calculating the average of the ADC samples
	}
	avrgTemp = convertCelsius((float)calcTemp/NO_OF_ADCSAMPLES);  // Temp in centigrade scale
 }

/* Convert ADC sample to Celsius */
float convertCelsius(int32_t adcSample)
{
	float temperature;
	float cal_temp_0 = (float) ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)>>_DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float) ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>>_DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

	float t_grad = -6.27;
	temperature = (cal_temp_0 - ((cal_value_0-adcSample)/t_grad));

	return temperature;
}

/* Configure DMA */
void DMA_Setup(void)
{
	CMU_ClockEnable(cmuClock_DMA, true);
		DMA_Init_TypeDef dmaInit;
		/* Initializing the DMA */

		/*HPROT signal state when accessing the primary/alternate
		 * descriptors. Normally set to 0 if protection is not an issue.*/
		dmaInit.hprot        = 0;
		dmaInit.controlBlock = dmaControlBlock;// giving the parameters for control block
		DMA_Init(&dmaInit);

		//  Setting up call-back function
		callBack[CHANNEL_ADC_0].cbFunc  = ADCdmaTransferDone;   // Associate call back function here
		callBack[CHANNEL_ADC_0].userPtr = NULL;                 // User defined pointer to provide with call back function
		callBack[CHANNEL_ADC_0].primary = true;

		DMA_CfgChannel_TypeDef  dmachnlCfg;
		// Setting up channel
		dmachnlCfg.highPri   = false;                 // Default priority
		dmachnlCfg.enableInt = true;                  // interrupt shall be enabled for channel
		dmachnlCfg.select    = Dma_ChannelCfgSelect;  // DMA channel select for ADC0_SINGLE
		dmachnlCfg.cb        = &callBack[CHANNEL_ADC_0];               // Associating the call back function with DMA channel

		DMA_CfgChannel(CHANNEL_ADC_0, &dmachnlCfg);
		DMA_CfgDescr_TypeDef    dmadescrCfg;

		// Setting up channel descriptor
		dmadescrCfg.dstInc  = Dma_DestCfg_dstInc;      // destination address: 2 bytes
		dmadescrCfg.srcInc  = Dma_DestCfg_srcInc;      // Source address:
		dmadescrCfg.size    = Dma_DestCfg_size;        // data size: 2 bytes
		dmadescrCfg.arbRate = Dma_DestCfg_arbRate;     // arbitrate:  set to zero
		dmadescrCfg.hprot   = 0;
		DMA_CfgDescr(CHANNEL_ADC_0, true, &dmadescrCfg);

}

/* Enable and Configure GPIO */
inline void GPIO_Setup(void)
{
	/* enable GPIO E pin 2 (LED-0) as output */
	GPIO_PinModeSet(LED_PORT,LED_PIN, gpioModePushPull,0);
	/* enable GPIO E pin 3 (LED-1) as output */
	GPIO_PinModeSet(LED_PORT,LED1_PIN, gpioModePushPull,0);
	/* Light sensor config */
	GPIO_PinModeSet(LIGHT_EXCITE_PORT,LIGHT_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LIGHT_INPUT_PORT,LIGHT_PIN, gpioModeInput,0);

	// Open-drain output with filter and pullup
	GPIO_PinModeSet(I2C1_PORT,I2C1_SCL,gpioModeWiredAndPullUpFilter,1);

	// Initializing (Port C pin 4) as GPIO gpioModeWiredAndPullUpFilter
	// Open-drain output with filter and pullup
	GPIO_PinModeSet(I2C1_PORT,I2C1_SDA,gpioModeWiredAndPullUpFilter,1);

	// Setting up PC0 to indicate transfer direction
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_VDD, gpioModePushPull, 0);

	// Setting up PC3 to indicate interrupt direction
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_InterruptPin, gpioModeInput, 0);
}



/* Turn on LED0 */
inline void LED_On(void)
{
		GPIO_PinOutSet(LED_PORT,LED_PIN);
}

/* Turn off LED0 */
inline void LED_Off(void)
{
		GPIO_PinOutClear(LED_PORT,LED_PIN);
}

/* Turn on LED1 */
void LED1_On(void)
{
		GPIO_PinOutSet(LED_PORT,LED1_PIN);
}

/* Turn off LED1 */
void LED1_Off(void)
{
		GPIO_PinOutClear(LED_PORT,LED1_PIN);
}

/* Control LED1 using average temperature */
void TempLed1_Control(void)
{
	if (avrgTemp > TEMP_UPPER_LIMIT || avrgTemp < TEMP_LOWER_LIMIT)
	{
		LED1_On();
	}

	else
	{
		LED1_Off();
	}
}

/* Toggle LED */
inline void LED_Toggle(void)
{
	GPIO_PinOutToggle(LED_PORT,LED_PIN);
}


I2C_TransferReturn_TypeDef I2C_Status;     // To know the status of the I2C1
uint8_t I2C_tx_buffer1[] = {0x00};         // Buffer for address in readByte function
uint8_t I2C_tx_buffer_size1 = 1;	       // Size of readByte buffer
uint8_t I2C_rx_buffer[I2C_rx_buffer_size]; // receive buffer
void i2c_WriteByte(uint8_t I2C_tx_buffer[], uint8_t size)
{
	I2C_TransferSeq_TypeDef Writeseq;
	Writeseq.addr = SLAVE_ADDRESS << 1;
	Writeseq.flags = I2C_FLAG_WRITE;

	Writeseq.buf[0].data = I2C_tx_buffer;
	Writeseq.buf[0].len = size;

	Writeseq.buf[1].data = I2C_rx_buffer;
	Writeseq.buf[1].len = I2C_rx_buffer_size;

	I2C_TransferInit(I2C1, &Writeseq);
	while(I2C_Transfer(I2C1) == i2cTransferInProgress);
}
uint8_t i2c_ReadByte()
{
	I2C_TransferSeq_TypeDef Readseq;
	// Sending the address
	Readseq.addr = SLAVE_ADDRESS << 1;
	Readseq.flags = I2C_FLAG_WRITE_READ;  // this flag corresponds to restart event

	// sends the command
	Readseq.buf[0].data = I2C_tx_buffer1;
	Readseq.buf[0].len = 1;

	// stores the received value
	Readseq.buf[1].data = I2C_rx_buffer;
	Readseq.buf[1].len = I2C_rx_buffer_size;

	// checking for status
	I2C_Status = I2C_TransferInit(I2C1, &Readseq);
	while(I2C_Status != i2cTransferDone)
	{
		if (I2C_Status != i2cTransferInProgress)
		{
			break;
		}
		I2C_Status = I2C_Transfer(I2C1);
	}

	return I2C_rx_buffer[0];
}


void Enable_TSL2651()
{
	uint8_t I2C_tx_buffer[2]; // I2C_tx_buffer[0]:command I2C_tx_buffer[1]:value to write
	GPIO_PinModeSet(I2C1_PORT, I2C1_SCL, gpioModeWiredAndPullUpFilter, 0);
	GPIO_PinModeSet(I2C1_PORT, I2C1_SDA, gpioModeWiredAndPullUpFilter, 0);
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_VDD, gpioModePushPull, 0);
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_InterruptPin, gpioModeInput, 0);
	GPIO_PinOutSet(I2C1_INTERRUPT_PORT, I2C1_VDD);  // Setting the vcc

	// providing a delay to facilitate setup time
	for (uint16_t i = 0; i<500; i++);
	for(int i=0;i<9;i++)
	{
		GPIO_PinOutClear(I2C1_PORT, I2C1_SCL);
		GPIO_PinOutSet(I2C1_PORT, I2C1_SDA);
	}

	GPIO_PinOutSet(I2C1_INTERRUPT_PORT, I2C1_InterruptPin);
	GPIO_PinOutSet(I2C1_PORT, I2C1_SCL);
	GPIO_PinOutSet(I2C1_PORT, I2C1_SDA);

	// Writing 00 to control register
	I2C_tx_buffer[0] = TSL2561_CONTROL_REG;
	I2C_tx_buffer[1] = 0x00;
	i2c_WriteByte(I2C_tx_buffer, 2);

	// Writing 0x03 (power on) to control register
	I2C_tx_buffer[0] = TSL2561_CONTROL_REG;
	I2C_tx_buffer[1] = 0x03;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x01 for 101ms (power on) to Timing register
	I2C_tx_buffer[0] = TSL2561_TIMING_REG;
	I2C_tx_buffer[1] = 0x01;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x0f in TSL2561_THRESLowLow_REG
	I2C_tx_buffer[0] = TSL2561_THRESLowLow_REG;
	I2C_tx_buffer[1] = 0x0f;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x00 in TSL2561_THRESLowhigh_REG
	I2C_tx_buffer[0] = TSL2561_THRESLowhigh_REG;
	I2C_tx_buffer[1] = 0x00;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x00 in TSL2561_THRESHighLow_REG
	I2C_tx_buffer[0] = TSL2561_THRESHighLow_REG;
	I2C_tx_buffer[1] = 0x00;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x08 in TSL2561_THRESHighHigh_REG
	I2C_tx_buffer[0] = TSL2561_THRESHighHigh_REG;
	I2C_tx_buffer[1] = 0x08;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Writing 0x14 in TSL2561_INTERRUPT_REG
	I2C_tx_buffer[0] = TSL2561_INTERRUPT_REG;
	I2C_tx_buffer[1] = 0x14;
	i2c_WriteByte(I2C_tx_buffer,2);

	// Enables the interrupt
	GPIO_IntConfig(I2C1_INTERRUPT_PORT, I2C1_InterruptPin, false, true, true);   // falling edge
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}


void Disable_TSL2651()
{
	uint8_t I2C_tx_buffer[2];

	// power of the device
	I2C_tx_buffer[0] = TSL2561_CONTROL_REG;
	I2C_tx_buffer[1] = 0x00;
	i2c_WriteByte(I2C_tx_buffer, 2);

	// Disable and clear the interrupts
	GPIO_IntConfig(I2C1_INTERRUPT_PORT, I2C1_InterruptPin, false, true, false);   // rising or falling (rising for now)
	GPIO_IntClear(GPIO_IntGet());  // Disabling the interrupts
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	GPIO_DriveModeSet(I2C1_INTERRUPT_PORT, gpioDriveModeLowest);

	// Clearing the power supply to tsl2561
	GPIO_PinModeSet(I2C1_PORT, I2C1_SCL, gpioModeDisabled, 0);
	GPIO_PinModeSet(I2C1_PORT, I2C1_SDA, gpioModeDisabled, 0);
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_VDD, gpioModeDisabled, 0);
	GPIO_PinModeSet(I2C1_INTERRUPT_PORT, I2C1_InterruptPin, gpioModeDisabled, 0);

	GPIO_PinOutClear(I2C1_INTERRUPT_PORT, I2C1_VDD);
}




size_t event_check = 0;
void GPIO_ODD_IRQHandler()
{
	INT_Disable();
	GPIO_IntClear(GPIO_IntGet());
	if ((event_check & 0x01) == 0)
	{
		uint8_t I2C_tx_buffer[1];
		//uint8_t I2C_tx_buffer1[2];

		I2C_tx_buffer[0] = 0xC0;
		i2c_WriteByte(I2C_tx_buffer, 1);

		I2C_tx_buffer1[0] = TSL2561_DATA0Low;
		ADC_Low = i2c_ReadByte();

		I2C_tx_buffer1[0] = TSL2561_DATA0High;
		ADC_High = i2c_ReadByte();

		uint16_t ADC_Final = ADC_High<<8 | ADC_Low;

		if (ADC_Final < TSL2561_LowerLimit)
		{
			LED_On();
			led_status = 0x01;
		}

		else if (ADC_Final > TSL2561_UpperLimit)
		{
			LED_Off();
			led_status = 0x00;
		}
	}
	event_check++;
	INT_Enable();
}

void I2C_Setup()
{
	if(ACTIVE_LIGHT_ENABLE)
	{
		CMU_ClockEnable(cmuClock_I2C1, true);
		I2C_Init_TypeDef i2cInit =

				{                                                                         \
				  true,                    /* Enable when init done */                    \
				  true,                    /* Set to master mode */                       \
				  0,                       /* Use currently configured reference clock */ \
				  I2C_FREQ_STANDARD_MAX,   /* Set to standard rate assuring being */      \
										   /* within I2C spec */                          \
				  i2cClockHLRStandard      /* Set to use 4:4 low/high duty cycle */       \
				};

		I2C1->ROUTE = I2C_ROUTE_SDAPEN |   // /**< SDA Pin Enable */
					  I2C_ROUTE_SCLPEN; //& ~(_I2C_ROUTE_LOCATION_LOC0);
		I2C_Init(I2C1, &i2cInit);

		// Exit the busy state
		if (I2C1->STATE & I2C_STATE_BUSY){
			I2C1->CMD = I2C_CMD_ABORT;
		}


		I2C1->SADDR = SLAVE_ADDRESS << 1; // Giving the slave address
		// Enableing auto acknoledgement, slave control and autosn
		I2C1->CTRL |= (I2C_CTRL_AUTOACK | I2C_CTRL_SLAVE | I2C_CTRL_AUTOSN);
	}

}


void SendBufSamB11()
{
	uint8_t LEUART_BYTE;
	cbuffer_remove(&tx_buffer, &LEUART_BYTE);
	LEUART_Tx(LEUART0, LEUART_BYTE);
	cbuffer_remove(&tx_buffer, &LEUART_BYTE);
	for(int i=0;i<1000;i++);
	LEUART_Tx(LEUART0, LEUART_BYTE);
	 while (!(LEUART0->STATUS & LEUART_STATUS_TXBL));
	Clear_Sleep_Mode(LEUART_EM);
}

void LEUART0_IRQHandler()
{
	int leuart0_getinterr = LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0, leuart0_getinterr);

}

void leuart_Setup()
{

	CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART0 clock */
	LEUART_Reset(LEUART0);

	LEUART_Init_TypeDef leuart0Init =
	{
	  .enable   = leuartDisable,       /* Activate data reception on LEUn_TX pin. */
	  .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
	  .baudrate = LEUART_BAUD,                 /* Baudrate  */
	  .databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */
	  .parity   = LEUART_PARITY_BITS,       /* No parity bits in use */
	  .stopbits = LEUART_STOP_BITS,      /* Setting the number of stop bits in a frame to 2 bitperiods */
	};

	LEUART_Init(LEUART0, &leuart0Init);
	LEUART_IntClear(LEUART0, _LEUART_IF_MASK);

	LEUART0->ROUTE = LEUART_ROUTE_RXPEN |LEUART_ROUTE_TXPEN| LEUART_ROUTE_LOCATION_LOC0;

	GPIO_PinModeSet(LEUART0_GPIO,LEUART0_TX_PIN,gpioModePushPull,1);
	GPIO_PinModeSet(LEUART0_GPIO,LEUART0_RX_PIN,gpioModePushPull,1);

	LEUART0->IEN |= LEUART_IEN_TXC;
	LEUART_Enable(LEUART0, leuartEnable);
	NVIC_EnableIRQ(LEUART0_IRQn);
}


void cbuffer_init(cbuffer* buff, uint8_t buff_size)
{
    if(buff_size == 0)
    {
    	buff_size = 1;
    }
    buff->start = (uint8_t*)(malloc(buff_size));
    buff->end = buff->start + buff_size - 1;
    buff->head = buff->start;
    buff->tail = buff->start;
    buff->size = buff_size;
    buff->count = 0;
    return;
}

void cbuffer_add(cbuffer* buff, uint8_t* elem)
{
    if(buff->count == buff->size)
    	buff->count = 0;
        //return;
    if(buff->head == buff->end)
    {
        memcpy(buff->head, elem, sizeof(uint8_t));
        buff->head = buff->start;
    }
    else
    {
        memcpy(buff->head++, elem, sizeof(uint8_t));
    }
    buff->count++;
    return;
}


void cbuffer_remove(cbuffer* buff, uint8_t* elem)
{
    if(buff->count == 0)
    	return;
    if(buff->tail == buff-> end)
    {
        memcpy(elem, buff->tail, sizeof(uint8_t));
        buff->tail = buff->start;
    }
    else
    {
        memcpy(elem, buff->tail++, sizeof(uint8_t));
    }
    buff->count--;
    return;
}

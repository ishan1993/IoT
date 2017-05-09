/*************************************************************/
/* Author - Ishan Shah					                     */
/* Date - 02/08/2017     									 */
/* Course - IoT         									 */
/* Assignment - ADC_DMA					     				 */
/*************************************************************/



/* Including general libraries. */
#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"
#include "em_common.h"

/* Including peripheral/module specific libraries. */
#include "em_emu.h"						// Energy management
#include "em_cmu.h"						// Clock management
#include "em_letimer.h"					// LETIMER
#include "em_timer.h"					// TIMER
#include "em_acmp.h"					// ACMP
#include "em_adc.h"						// ADC
#include "em_dma.h"						// DMA
#include "dmactrl.c"

/* Macros and constants */
#define EM2_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM2
#define EM3_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM3
#define EM_SET EM3						// Lowest energy mode available for LETIMER
#define PERIOD 2						// Time Period for LETIMER
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
#define HF_FREQ 7						// Frequency for the HF clock
#define ADC_DMA_ENABLE 1				// Determine whether to use DMA with ADC or not (1-Enable, 0-Disable)
#define ADC_EM 1						// Energy mode for ADC blocking
#define TEMP_UPPER_LIMIT 35				// Upper limit for temperature sensor
#define TEMP_LOWER_LIMIT 15				// lower limit for temperature sensor
#define CHANNEL_ADC_0 0					// ADC channel
#define NO_OF_ADCSAMPLES 750			// Number of ADC samples
#define ADC_Singleinit_Input    adcSingleInpTemp	// ADC configs
#define ADC_Singleinit_Ref      adcRef1V25
#define ADC_Singleinit_Resol    adcRes12Bit
#define ADC_Singleinit_AcqTime  adcAcqTime1
#define ADCFreq                 249000
#define ADC_Select_HFPER_Freq   0
#define Dma_ChannelCfgSelect DMAREQ_ADC0_SINGLE	// DMA Configs
#define Dma_DestCfg_dstInc  dmaDataInc2			// DMA destination config
#define Dma_DestCfg_srcInc  dmaDataIncNone
#define Dma_DestCfg_size    dmaDataSize2
#define Dma_DestCfg_arbRate dmaArbitrate1

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

/* Local user defined header files */
#include "sleep.h"
#include "peripheral.h"


int main(void)
{
  /* Chip errata */
  CHIP_Init();
  CMU_Setup();
  LETIMER_Setup();
  GPIO_Setup();
  ACMP_Setup();
  ADC_Setup();

  /* Sleep till IRQ */
  while (1)
  {
  	sleep();
  }
}

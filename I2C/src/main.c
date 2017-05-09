/*************************************************************/
/* Author - Ishan Shah					                     */
/* Date - 02/24/2017     									 */
/* Course - IoT         									 */
/* Assignment - I2C						     				 */
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
#include "em_i2c.h"						// i2c

/* Macros and constants */
#define EM2_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM2
#define EM3_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM3
#define EM_SET EM2						// Lowest energy mode available for LETIMER
#define PERIOD 3						// Time Period for LETIMER
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
#define TEMP_LOWER_LIMIT 15				// lower limit for temperature sensor
#define CHANNEL_ADC_0 0					// ADC channel
#define NO_OF_ADCSAMPLES 1000			// Number of ADC samples
#define ADC_Singleinit_Input    adcSingleInpTemp	// ADC configs
#define ADC_Singleinit_Ref      adcRef1V25
#define ADC_Singleinit_Resol    adcRes12Bit
#define ADC_Singleinit_AcqTime  adcAcqTime2
#define ADCFreq                 40000
#define ADC_Select_HFPER_Freq   0
#define ADC_PRESCALAR 25
#define Dma_ChannelCfgSelect DMAREQ_ADC0_SINGLE	// DMA Configs
#define Dma_DestCfg_dstInc  dmaDataInc2			// DMA destination config
#define Dma_DestCfg_srcInc  dmaDataIncNone
#define Dma_DestCfg_size    dmaDataSize2
#define Dma_DestCfg_arbRate dmaArbitrate1
#define PASSIVE_LIGHT_ENABLE 0			// Enable/Disable on-board passive light sensor
#define ACTIVE_LIGHT_ENABLE 1			// Enable/Disable external light sensor
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
#define TSL2561_UpperLimit 0x0100
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
  I2C_Setup();

  /* Sleep till IRQ */
  while (1)
  {
  	sleep();
  }
}

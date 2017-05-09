/*************************************************************/
/* Author - Ishan Shah					                     */
/* Date - 02/08/2017     									 */
/* Course - IoT         									 */
/* Assignment - Self-calibrating ULFRCO	     				 */
/*************************************************************/



/* Including general libraries. */
#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"

/* Including peripheral/module specific libraries. */
#include "em_emu.h"						// Energy management
#include "em_cmu.h"						// Clock management
#include "em_letimer.h"					// LETIMER
#include "em_timer.h"					// TIMER
#include "em_acmp.h"					// ACMP

/* Macros and constants */
#define EM2_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM2
#define EM3_Restore true				// Restore value for deciding whether to restore clock configurations after waking up from EM3
#define EM_SET EM3						// Lowest energy mode available for LETIMER
#define PERIOD 2.5						// Time Period for LETIMER
#define ON_TIME 0.004					// LED On time
#define LED_PORT gpioPortE				// LED GPIO port
#define LED_PIN	2						// LED Pin number
#define LIGHT_EXCITE_PORT gpioPortD 	// Light sensor excite port
#define LIGHT_INPUT_PORT gpioPortC		// Light sensor
#define LIGHT_PIN 6						// Light sensor pin
#define CALIBRATION 1					// Enable/Disable ULFRCO Calibration
#define LIGHT_LOW_THRESH 2				// Threshold for detecting darkness
#define LIGHT_HIGH_THRESH 61			// Threshold for detecting brightness
#define ACMP_LIGHT_CHANNEL acmpChannel6	// ACMP channel for light sensor
#define ACMP_VDD_CHANNEL acmpChannelVDD	// ACMP channel for scaled VDD
#define LFAPRESC0_SHIFT 8				// Bit shift for LFAPRESC0 configuration of prescaler
#define TIMER_TOP 65535					// TOP value for a 16 bit timer count register
#define HF_FREQ 7						// Frequency for the HF clock

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

  /* Sleep till IRQ */
  while (1)
  {
  	sleep();
  }
}

/*************************************************************/
/* Author - Ishan Shah					                     */
/* Date - 02/01/2017     									 */
/* Course - IoT         									 */
/* Assignment - Managing Energy Modes 	     				 */
/*************************************************************/


/* Including general libraries. */
#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"

/* Including peripheral/module specific libraries. */
#include "em_emu.h"				// Energy management
#include "em_cmu.h"				// Clock management
#include "em_letimer.h"			// LETIMER
#include "em_prs.h"				// Peripheral Reflex System

/* Macros and constants */
#define EM2_Restore true		// Restore value for deciding whether to restore clock configurations after waking up from EM2
#define EM3_Restore true		// Restore value for deciding whether to restore clock configurations after waking up from EM3
#define EM_SET EM3				// Lowest energy mode available for LETIMER
#define PERIOD 1.75				// Time Period for LETIMER
#define ON_TIME 0.03			// LED On time
#define LED_PORT gpioPortE		// LED GPIO port
#define LED_PIN	2				// LED Pin number

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

/* Local user defined header files */
#include "peripheral.h"
#include "sleep.h"

int main(void)
{
	/* Chip Initialization */
	CHIP_Init();

	CMU_Setup();
	LETIMER_Setup();
	GPIO_Setup();
	Set_Sleep_Mode(EM_State);

	/* Sleep till IRQ */
	while (1)
	{
		sleep();
	}
}

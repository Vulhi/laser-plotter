/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usb/user_vcom.h"
#include "Controller.h"
#include "debugTools/RunningTime.h"

//#define JUST_TESTING

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
	Chip_PININT_Init(LPC_GPIO_PIN_INT);
	Chip_SWM_Init();
	Chip_MRT_Init();
	RunningTime::setup();

	NVIC_EnableIRQ(MRT_IRQn);
	NVIC_SetPriority(MRT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
	ITM_init();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

int main(void)
{
	prvSetupHardware();
	new Controller();
	xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE*4, nullptr, (tskIDLE_PRIORITY + 1UL), nullptr);

	vTaskStartScheduler();

	return 1;
}


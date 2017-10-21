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
#include "semphr.h"
#include "LimitSwitch.h"

#include "PWMController.h"
#include "Stepper.h"

#include "ITM_write.h"
#include "RunningTime.h"

#include "user_vcom.h"

#include "Parser.h"

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

void PWMTest(void* pPWM){
	PWMController& pwm = *static_cast<PWMController*>(pPWM);
	bool rising = true;
	double cycle = 50.0;
	while(true){
		if(rising){
			if(cycle < 100){
				++cycle;
			} else {
				rising = false;
			}
		} else {
			if(cycle > 0){
				--cycle;
			} else {
				rising = true;
			}
		}
		pwm.setDutycycleL(cycle);
		vTaskDelay(20);
	}
}

void StepperTest(void* pStepper){
	auto sendOK = [](){
		static uint8_t ok[5] = {'O', 'K', '\r', '\n', '\0'};
		USB_send(ok, 5);
	};

	LimitSwitch<0> lsY1(0, 29);
	LimitSwitch<1> lsY2(0, 9);
	LimitSwitch<2> lsX1(1, 3);
	LimitSwitch<3> lsX2(0, 0);
	Parser parser;

	Stepper stepperX(
			0, 27, // Motor drive pin&port
			0, 28, // Motor direction pin&port
			0, // MRT channel (0 or 1)
			lsX2, lsX1); // LimitSwitch_Base& front / back

	Stepper stepperY(
			0, 24,
			1, 0,
			1,
			lsY2, lsY1);

	PWMController pen(LPC_SCT0);

	/* Period of the counter needs to stay below 65535,
	 * so setting the prescaler to 12 results in nice 60000 (timer ticks)
	 * if using bidirectional mode. See PWMController::periodFromFrequencyL().
	 * */
	pen.initCounterL(50, 5, true, 12);
	pen.setOutputL(0, 10, 0, true);
	pen.startCounterL();

	pen.setDutycycleL(5);
	stepperY.setRate(4000, true);
	stepperX.setRate(4000, true);

	vTaskDelay(500);
	while(true){
		Command cmd = parser.getCommand();
		switch(cmd.code){
		case CODES::G1:{
			uint32_t currentPosX = stepperX.getSteps()/2;
			uint32_t currentPosY = stepperY.getSteps()/2;
			uint32_t targetPosX = cmd.x*100;
			uint32_t targetPosY = cmd.y*100;
			bool directionX = true;
			bool directionY = true;
			uint32_t stepsX;
			uint32_t stepsY;
			if(currentPosX < targetPosX){
				stepsX = targetPosX-currentPosX;
				directionX = false;
			} else stepsX = currentPosX-targetPosX;

			if(currentPosY < targetPosY){
				stepsY = targetPosY-currentPosY;
				directionY = false;
			} else stepsY = currentPosY-targetPosY;

			stepperX.setDirection(directionX);
			stepperY.setDirection(directionY);
			if(stepsX > stepsY){
				stepperX.setRate(3000, true);
				stepperY.setRate(Stepper::getSpeedForShorterAxle(stepsY, stepsX, stepperX.getCurrentRate()));
			} else {
				stepperY.setRate(3000, true);
				stepperX.setRate(Stepper::getSpeedForShorterAxle(stepsX, stepsY, stepperY.getCurrentRate()));
			}
			stepperX.runForSteps(stepsX);
			stepperY.runForSteps(stepsY);
			ITM_write("Waiting for steppers");
			Stepper::waitForAllSteppers();
			sendOK();
			ITM_write("OK\r\n");
			break;
		}
		case CODES::G28:
			stepperX.goHome();
			stepperY.goHome();
			Stepper::waitForAllSteppers();
			sendOK();
			break;
		case CODES::M1:
		{
			uint16_t dutycycle = cmd.x/(2.55*20)+5;
			pen.setDutycycleL(dutycycle); // Dutycycle 5 - 10%
			sendOK();
		}
			break;
		case CODES::M4:
			//laser
			sendOK();
			break;
		case CODES::M10:
#ifndef JUST_TESTING
			stepperX.calibrate();
			stepperY.calibrate();
			Stepper::waitForAllSteppers();
#endif
			sendOK();
			break;
		case CODES::E:
			break;
		}
	}
}

int main(void)
{
	prvSetupHardware();
	RunningTime::setup();

	NVIC_EnableIRQ(MRT_IRQn);
	NVIC_SetPriority(MRT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);

	xTaskCreate(StepperTest, "stepperTest", configMINIMAL_STACK_SIZE*14, nullptr, (tskIDLE_PRIORITY + 2UL), nullptr);
	xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE*4, nullptr, (tskIDLE_PRIORITY + 1UL), nullptr);

	vTaskStartScheduler();

	return 1;
}


/*
 * Controller.cpp
 *
 *  Created on: 21.10.2017
 *      Author: Ville
 */

#include "Controller.h"

//#define JUST_TESTING

Controller::Controller() :
Task("Controller", configMINIMAL_STACK_SIZE*14, (tskIDLE_PRIORITY + 2UL)),
xStepper(0, 27, 0, 28, 0),
yStepper(0, 24, 1, 0, 1),
lsY1(0, 29, yStepper, false),
lsY2(0, 9, yStepper, true),
lsX1(1, 3, xStepper, false),
lsX2(0, 0, xStepper, true),
pen(LPC_SCT0),
laser(LPC_SCT0)
{
	pen.initCounterL(50, 5, true, 12);
	pen.setOutputL(0, 10, 0, true);
	pen.startCounterL();

//	TODO: laser init

}

void Controller::_task(){
	vTaskDelay(200);
	while(true){
		Command cmd = parser.getCommand();
		switch(cmd.code){
		case CODES::G1:{
			uint32_t currentPosX = xStepper.getSteps()/2;
			uint32_t currentPosY = yStepper.getSteps()/2;
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

			xStepper.setDirection(directionX);
			yStepper.setDirection(directionY);
			if(stepsX > stepsY){
				xStepper.setRate(3000, true);
				yStepper.setRate(Stepper::getSpeedForShorterAxle(stepsY, stepsX, xStepper.getCurrentRate()));
			} else {
				yStepper.setRate(3000, true);
				xStepper.setRate(Stepper::getSpeedForShorterAxle(stepsX, stepsY, yStepper.getCurrentRate()));
			}
			xStepper.runForSteps(stepsX);
			yStepper.runForSteps(stepsY);
			ITM_write("Waiting for steppers");
			Stepper::waitForAllSteppers();
			sendOK();
			ITM_write("OK\r\n");
			break;
		}
		case CODES::G28:
			xStepper.goHome();
			yStepper.goHome();
			Stepper::waitForAllSteppers();
			sendOK();
			break;
		case CODES::M1:
		{
			/*
			 * cmd.x is angle from 0 to 180 */
			uint16_t dutycycle = cmd.x/36 + 5;
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
			xStepper.calibrate();
			yStepper.calibrate();
			Stepper::waitForAllSteppers();
#endif
			sendOK();
			break;
		case CODES::E:
			sendOK(); // Try to continue.
			break;
		}
	}
}

void Controller::sendOK(){
	static uint8_t ok[5] = {'O', 'K', '\r', '\n', '\0'};
	USB_send(ok, 5);
}


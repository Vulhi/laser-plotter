/*
 * Controller.cpp
 *
 *  Created on: 21.10.2017
 *      Author: Ville
 */

#include "Controller.h"
#include "debugTools/itoa.h"
//#define JUST_TESTING

const uint32_t Controller::LONG_AXLE_MIN_RATE = 2000;
const uint32_t Controller::MAX_RATE = 5000;
const uint32_t Controller::LONG_AXLE_ACCELERATION_MILLISTEPS = 20000;

const uint32_t Controller::MIN_STEPS_FROM_MAX_TO_MIN_RATE = Stepper::getStepsRequiredToAccelerate(
		Controller::LONG_AXLE_ACCELERATION_MILLISTEPS,
		Controller::MAX_RATE,
		Controller::LONG_AXLE_MIN_RATE);

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

	laser.initCounterH(10000, 0, true, 1);
	laser.setOutputH(0, 12, 1, true);
	laser.startCounterH();
}

void Controller::_task(){
	vTaskDelay(200);
	while(true){
		char dbg[32];
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

			uint32_t stepsX_start;
			uint32_t stepsY_start;

			bool accelerate = false;

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
//#ifdef OLD
//			if(stepsX > stepsY){
//				xStepper.setRate(3000, true);
//				yStepper.setRate(Stepper::getRateForShorterAxle(stepsY, stepsX, xStepper.getCurrentRate()), true);
//			} else {
//				yStepper.setRate(3000, true);
//				xStepper.setRate(Stepper::getRateForShorterAxle(stepsX, stepsY, yStepper.getCurrentRate()), true);
//			}
//#endif
			if(stepsX >= stepsY){
				// Calculate and set initial speed
				xStepper.setRate(LONG_AXLE_MIN_RATE, true);
				uint32_t yRate = Stepper::getRateForShorterAxle(stepsY, stepsX, LONG_AXLE_MIN_RATE);
				ITM_write(itoa(yRate, dbg, 10));
				yStepper.setRate(yRate, true);
				// Calculate and set acceleration
				xStepper.setAccelerationStepSize(LONG_AXLE_ACCELERATION_MILLISTEPS);
				uint32_t yAcc = Stepper::getAccelerationForShorterAxle(stepsY, stepsX, LONG_AXLE_ACCELERATION_MILLISTEPS);
				ITM_write(itoa(yAcc, dbg, 10));
				yStepper.setAccelerationStepSize(yAcc);
				// Set target speed
				if(MIN_STEPS_FROM_MAX_TO_MIN_RATE*2 <= stepsX){
					xStepper.setRate(MAX_RATE);
					yStepper.setRate(Stepper::getRateForShorterAxle(stepsY, stepsX, MAX_RATE));
					accelerate = true;
				}
			} else {
				yStepper.setRate(LONG_AXLE_MIN_RATE, true);
				uint32_t xRate = Stepper::getRateForShorterAxle(stepsX, stepsY, LONG_AXLE_MIN_RATE);
				ITM_write(itoa(xRate, dbg, 10));
				xStepper.setRate(xRate, true);
				yStepper.setAccelerationStepSize(LONG_AXLE_ACCELERATION_MILLISTEPS);
				uint32_t xAcc = Stepper::getAccelerationForShorterAxle(stepsX, stepsY, LONG_AXLE_ACCELERATION_MILLISTEPS);
				ITM_write(itoa(xAcc, dbg, 10));
				xStepper.setAccelerationStepSize(xAcc);

				if(MIN_STEPS_FROM_MAX_TO_MIN_RATE*2 <= stepsY){
					yStepper.setRate(MAX_RATE);
					xStepper.setRate(Stepper::getRateForShorterAxle(stepsX, stepsY, MAX_RATE));
					accelerate = true;
				}
			}


//			if(accelerate){
//				if(stepsX >= stepsY){
//					stepsX_start = stepsX-MIN_STEPS_FROM_MAX_TO_MIN_RATE;
//					stepsY_start = stepsY-((MIN_STEPS_FROM_MAX_TO_MIN_RATE*stepsY)/stepsX);
//					xStepper.runForSteps(stepsX_start);
//					yStepper.runForSteps(stepsY_start);
//				} else {
//					stepsY_start = stepsY-MIN_STEPS_FROM_MAX_TO_MIN_RATE;
//					stepsX_start = stepsX-((MIN_STEPS_FROM_MAX_TO_MIN_RATE*stepsX)/stepsY);
//					yStepper.runForSteps(stepsY_start);
//					xStepper.runForSteps(stepsX_start);
//				}
//				Stepper::waitForAllSteppers();
//				xStepper.setRate(0);
//				yStepper.setRate(0);
//				xStepper.runForSteps(stepsX-stepsX_start);
//				yStepper.runForSteps(stepsY-stepsY_start);
//			}
//			else {
				xStepper.runForSteps(stepsX);
				yStepper.runForSteps(stepsY);
//			}
			Stepper::waitForAllSteppers();
			sendOK();
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
			double dutycycle = cmd.x/36 + 5;
			pen.setDutycycleL(dutycycle); // Dutycycle 5 - 10%
			sendOK();
		}
		break;
		case CODES::M4:
			laser.setDutycycleH(cmd.x/3.0); // Diving by 3 results in max 85% dutycycle
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


/*
 * Controller.h
 *
 *  Created on: 21.10.2017
 *      Author: Ville
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "aTask.h"
#include "debugTools/ITM_write.h"
#include "LimitSwitch.h"
#include "Stepper.h"
#include "PWMController.h"
#include "Parser.h"

class Controller : public Task {
public:
	Controller();
private:
	Stepper xStepper;
	Stepper yStepper;
	LimitSwitch<0> lsY1;
	LimitSwitch<1> lsY2;
	LimitSwitch<2> lsX1;
	LimitSwitch<3> lsX2;
	PWMController pen;
	PWMController laser;
	Parser parser;

	const static uint32_t LONG_AXLE_MIN_RATE;
	const static uint32_t MAX_RATE;
	const static uint32_t LONG_AXLE_ACCELERATION_MILLISTEPS;

	void _task() override;
	void sendOK();
};

#endif /* CONTROLLER_H_ */

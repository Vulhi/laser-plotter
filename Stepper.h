/*
 * Stepper.h
 *
 *  Created on: 9.10.2017
 *      Author: Ville
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "aTask.h"

#include "DigitalIoPin.h"
#include "LimitSwitch.h"

#define MAX_RATE 3000
#define ACCELERATION_STEP_TIME_MS 1.0
#define ACCELERATION_STEP_SIZE 1


class Stepper /*: public Task*/ {
public:
	class StepControl {
	public:
		StepControl(uint8_t stepPort, uint8_t stepPin, uint8_t MRT_channel, Stepper* stepper);
		void setInterval(uint32_t rate);
		void start();
		void stop();
		void pulse();
		void setStepsToRun(uint32_t steps);
		void MRT_callback();
	private:
		Stepper* _stepper;
		LPC_MRT_CH_T* stepCtrlMRT_CH;
		uint32_t currentInterval;
		uint32_t stepsToRun;
		DigitalIoPin pin;
		bool _pulse;
	};


	Stepper(//const char* taskname, uint16_t stacksize, UBaseType_t priority,
			uint8_t stepPort, uint8_t stepPin, uint8_t dirPort, uint8_t dirPin, uint8_t MRT_channel,
			const LimitSwitch_Base& front, const LimitSwitch_Base& back);
	void toggleDirection();
	void runForSteps(uint32_t steps);
	SemaphoreHandle_t getStepsDoneSemaphore();
	bool getDirection() const;
	void setDirection(bool dir);
	void setSpeed(uint16_t speed);
	uint16_t getCurrentSpeed() const;
	void setStop(bool stop);
	void zeroSteps();
	StepControl* getStepControl();
	uint32_t getSteps() const;
	uint32_t getStepsRequiredToAccelerate() const;
	void MRT_callback();
	static Stepper* getStepperByChannel(uint8_t channel);
private:
	static Stepper* stepperByChannel[2];
	void _accelerate();
	//void _task() override;
	LPC_MRT_CH_T* accelMRT_CH;
	EventGroupHandle_t go;
	uint16_t targetSpeed;
	uint16_t currentSpeed;
	DigitalIoPin dirControl;
	StepControl stepControl;
	const LimitSwitch_Base& limitFront;
	const LimitSwitch_Base& limitBack;
	uint32_t steps;
	bool stop;

	/* false: going forward, true going forward */
	bool direction;
	SemaphoreHandle_t stepsDone;
};

#endif /* STEPPER_H_ */
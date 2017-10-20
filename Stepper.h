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
#include <cmath>

/* StepControl is the actual control of steps and Stepper does acceleration and provides the API for controlling */
class Stepper : private Task {
public:
	class StepControl {
	public:
		StepControl(uint8_t stepPort, uint8_t stepPin, uint8_t MRT_channel, Stepper* stepper);
		void setInterval(uint32_t rate);
		void start(portBASE_TYPE* pxHigherPriorityWoken);
		void stop();
		void pulse();
		void setStepsToRun(uint32_t steps);
		void MRT_callback(portBASE_TYPE* pxHigherPriorityWoken);
	private:
		Stepper* _stepper;
		LPC_MRT_CH_T* stepCtrlMRT_CH;
		uint32_t currentInterval;
		uint32_t stepsToRun;
		DigitalIoPin pin;
		bool _pulse;
	};
	const static uint32_t ACCELERATION_STEP_SIZE;
	const static double ACCELERATION_STEP_TIME_MS;
	const static uint32_t ACCELERATION;

	Stepper(uint8_t stepPort, uint8_t stepPin,
			uint8_t dirPort, uint8_t dirPin,
			uint8_t MRT_channel,
			const LimitSwitch_Base& front, const LimitSwitch_Base& back);
	void calibrate();
	void toggleDirection();
	void goHome();
	void runForSteps(uint32_t steps);
	bool getDirection() const;
	void setDirection(bool dir);
	void setRate(uint32_t rate, bool instant = false);
	uint16_t getCurrentRate() const;
	void setStop(bool stop);
	void zeroSteps();
	StepControl* getStepControl();
	uint32_t getSteps() const;
	uint32_t getStepsRequiredToAccelerate() const;
	uint32_t getRateAchievable(uint32_t steps, bool max = true);
	static uint32_t getSpeedForShorterAxle(uint32_t stepsShort, uint32_t stepsLong, uint32_t speedLong);
	void MRT_callback(portBASE_TYPE* pxHigherPriorityWoken);
	static Stepper* getStepperByChannel(uint8_t channel);
	static void waitForAllSteppers();
private:
	typedef void (Stepper::*actionFunc)(uint32_t);
	struct action {
		actionFunc f;
		uint32_t param;
	};
	void _task() override;
	static Stepper* stepperByChannel[2];
	void _accelerate();
	void _runForSteps(uint32_t steps);
	void _calibrate(uint32_t nothing = 0);
	LPC_MRT_CH_T* accelMRT_CH;
	volatile uint32_t currentRate;
	uint32_t targetRate;
	DigitalIoPin dirControl;
	StepControl stepControl;
	const LimitSwitch_Base& limitFront;
	const LimitSwitch_Base& limitBack;
	uint32_t currentSteps;
	uint32_t maxSteps;
	uint8_t channel;
	bool stop;

	/* false: going forward, true going forward */
	bool direction;
	static EventGroupHandle_t done;
	SemaphoreHandle_t _doneInternal;
	QueueHandle_t actionQueue;
};

#endif /* STEPPER_H_ */

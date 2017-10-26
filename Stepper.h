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
		uint32_t halfStepsToRun;
		DigitalIoPin pin;
		bool _pulse;
	};
	const static double ACCELERATION_STEP_TIME_MS;
	uint32_t getAcceleration() const;
	void setAccelerationStepSize(uint32_t milliSteps);

	Stepper(uint8_t stepPort, uint8_t stepPin,
			uint8_t dirPort, uint8_t dirPin,
			uint8_t MRT_channel);
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
	static uint32_t getStepsRequiredToAccelerate(uint32_t accelerationMillistepSize, uint32_t fromRate, uint32_t toRate);
	uint32_t getRateAchievable(uint32_t steps, bool max = true);
	static uint32_t getRateForShorterAxle(uint32_t stepsShort, uint32_t stepsLong, uint32_t rateLong);
	inline static uint32_t getAccelerationForShorterAxle(uint32_t stepsShort, uint32_t stepsLong, uint32_t accelLong) {
		return getRateForShorterAxle(stepsShort, stepsLong, accelLong);
	}
	void MRT_callback(portBASE_TYPE* pxHigherPriorityWoken);
	static Stepper* getStepperByChannel(uint8_t channel);
	static void waitForAllSteppers();
private:
	typedef void (Stepper::*actionFunc)(uint32_t);
	struct Action {
		actionFunc function;
		uint32_t parameter;
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
	uint32_t currentHalfSteps;
	uint32_t maxSteps;
	uint8_t channel;
	bool stop;
	uint32_t _accelerationMilliStepSize;

	/* false: going forward, true going forward */
	bool direction;
	static EventGroupHandle_t eventGroup;
	SemaphoreHandle_t _doneInternal;
	QueueHandle_t actionQueue;
};

#endif /* STEPPER_H_ */

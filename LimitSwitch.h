/*
 * LimitSwitch.h
 *
 *  Created on: 7.9.2017
 *      Author: Ville
 */

#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include "InterruptedInputPin.h"
#include "FreeRTOS/FreeRTOS.h"
#include "Stepper.h"
#include "event_groups.h"

#include "debugTools/ITM_write.h"

/* Anything non-templated should be here */
class LimitSwitch_Base {
public:
	LimitSwitch_Base(int port, int pin, int channel, Stepper& stepper, bool max);
	inline static EventGroupHandle_t getEventGroup(){
		return eventGroup;
	}
	inline bool isEventBitSet() const {
		return (xEventGroupGetBits(eventGroup) >> _channel) & 1;
	}
protected:
	/* Each LimitSwitch will have invidual bit in event group
	 * indicating if switch is pressed or not (1 or 0)
	 */
	static EventGroupHandle_t eventGroup;
	InterruptedInputPin pinControl;
	int _channel;
	Stepper& stepper;
	bool max;
};

/* Only one limit switch per channel */
template <int channel>
class LimitSwitch : public LimitSwitch_Base {
public:
	LimitSwitch(int port, int pin, Stepper& stepper, bool max);
private:
	/* Each LimitSwitch will have its own channel so we can declare everything here static */
	inline static LimitSwitch* getLimitSwitch(){
		return thisPtr;
	}
	static LimitSwitch* thisPtr;
	static void IRQHandler(portBASE_TYPE* pxHigherPriorityWoken);
};

#include "LimitSwitch.tcc"
#endif /* LIMITSWITCH_H_ */

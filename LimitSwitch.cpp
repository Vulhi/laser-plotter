#include "LimitSwitch.h"

EventGroupHandle_t LimitSwitch_Base::eventGroup = xEventGroupCreate();

LimitSwitch_Base::LimitSwitch_Base(int port, int pin, int channel, Stepper& stepper_, bool max_)
: pinControl(port, pin, true, true, channel, true, channel), stepper(stepper_) {
	_channel = channel;
	max = max_;
}

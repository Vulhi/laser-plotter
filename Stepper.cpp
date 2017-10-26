/*
 * Stepper.cpp
 *
 *  Created on: 9.10.2017
 *      Author: Ville
 */

#include "Stepper.h"
#include "LimitSwitch.h"

#define DEBUG_TOOLS

#ifdef DEBUG_TOOLS
#include "debugTools/ITM_write.h"
#include "debugTools/RunningTime.h"
#include "debugTools/itoa.h"
#endif

//const uint32_t Stepper::ACCELERATION_STEP_SIZE = 8;
const double Stepper::ACCELERATION_STEP_TIME_MS = 1.0;
//const uint32_t Stepper::ACCELERATION = (ACCELERATION_STEP_SIZE*1000 / ACCELERATION_STEP_TIME_MS);

EventGroupHandle_t Stepper::eventGroup = xEventGroupCreate();
Stepper* Stepper::stepperByChannel[2];

/* Call Chip_MRT_Init() before calling this constructor
 * Call NVIC_EnableIRQ(MRT_IRQn) after creating both Stepper objects
 * Use either 0 or 1 for MRT_channel! Stepper uses two channels per stepper. (0 and 2) or (1 and 3)
 * */
Stepper::Stepper(uint8_t stepPort, uint8_t stepPin,
		uint8_t dirPort, uint8_t dirPin,
		uint8_t MRT_channel) :
  Task("Stepper", configMINIMAL_STACK_SIZE*4, (tskIDLE_PRIORITY + 2UL)),
  accelMRT_CH(LPC_MRT_CH(MRT_channel+2)),
  dirControl(dirPort, dirPin, DigitalIoPin::output, false),
  stepControl(stepPort, stepPin, MRT_channel, this)
{
	direction = false;
	stop = false;
	currentHalfSteps = 100000; // Something big to allow calibration to not underflow when reversing.
	targetRate = 0;
	currentRate = 0;
	channel = MRT_channel;
	stepperByChannel[MRT_channel] = this;
	actionQueue = xQueueCreate(4, sizeof(Action));
	_accelerationMilliStepSize = 8000;

	_doneInternal = xSemaphoreCreateBinary();
	// set maxSteps to something ridicilously big to allow calibration to touch the limit switches
	maxSteps = 1000000;
}

void Stepper::toggleDirection() {
	direction = !direction;
	dirControl.write(direction);
}

bool Stepper::getDirection() const {
	return direction;
}

void Stepper::setStop(bool stop){
	this->stop = stop;
}

void Stepper::setRate(uint32_t rate, bool instant) {
	targetRate = rate;
	if(instant)
		currentRate = rate;
}


uint16_t Stepper::getCurrentRate() const{
	return currentRate;
}

Stepper* Stepper::getStepperByChannel(uint8_t channel) {
	return stepperByChannel[channel];
}

Stepper::StepControl* Stepper::getStepControl() {
	return &stepControl;
}

void Stepper::MRT_callback(portBASE_TYPE* pxHigherPriorityWoken) {
	_accelerate();
}

void Stepper::calibrate() {
	Action a = {&Stepper::_calibrate, 0};
	xQueueSend(actionQueue, &a, portMAX_DELAY);
}

void Stepper::waitForAllSteppers() {
	xEventGroupWaitBits(eventGroup, 3, pdTRUE, pdTRUE, portMAX_DELAY);
}

void Stepper::_accelerate(){
	if(currentRate < targetRate){
		currentRate += _accelerationMilliStepSize/1000;
		ITM_write("Accel+\r\n");
	} else if(targetRate < currentRate){
		currentRate -= _accelerationMilliStepSize/1000;
		ITM_write("Accel-\r\n");
	} else {
		accelMRT_CH->INTVAL = 0 | (1 << 31);
		accelMRT_CH->CTRL &= ~1;
	}

	stepControl.setInterval(currentRate);
}

void Stepper::setDirection(bool dir) {
	direction = dir;
	dirControl.write(dir);
}

uint32_t Stepper::getSteps() const {
	return currentHalfSteps;
}


void Stepper::runForSteps(uint32_t steps){
	Action a = {&Stepper::_runForSteps, steps};
	xQueueSend(actionQueue, &a, portMAX_DELAY);
}

uint32_t Stepper::getStepsRequiredToAccelerate() const {
	uint32_t result = currentRate < targetRate ?
			(targetRate*targetRate - currentRate*currentRate) / (getAcceleration()*2) + 0.5: // +0.5 is for rounding
			(currentRate*currentRate - targetRate*targetRate) / (getAcceleration()*2) + 0.5;
	return result;
}

uint32_t Stepper::getRateAchievable(uint32_t steps, bool max) {
	uint32_t result = max ?
			sqrt(2*getAcceleration()*steps - currentRate*currentRate) + 0.5:
			sqrt(-2*getAcceleration()*steps + currentRate*currentRate) + 0.5;
	return result;
}

void Stepper::zeroSteps() {
	currentHalfSteps = 0;
}

void Stepper::_task() {
	while(true){
		Action a;
		xQueueReceive(actionQueue, &a, portMAX_DELAY);
		(this->*(a.function))(a.parameter);
		xEventGroupSetBits(eventGroup, (1 << channel));
	}
}

Stepper::StepControl::StepControl(uint8_t stepPort, uint8_t stepPin, uint8_t MRT_channel, Stepper* stepper)
: _stepper(stepper), pin(stepPort, stepPin, DigitalIoPin::output, false) {
	stepCtrlMRT_CH = LPC_MRT_CH(MRT_channel);
}


void Stepper::StepControl::pulse(){
	pin.write(_pulse);
	_pulse = !_pulse;
}

void Stepper::StepControl::setInterval(uint32_t rate){
	if(rate > 0)
		currentInterval = Chip_Clock_GetSystemClockRate() / (rate * 2); // Two interrupts = one step
	else
		currentInterval = 0;
}

void Stepper::StepControl::start(portBASE_TYPE* pxHigherPriorityWoken){
	stepCtrlMRT_CH->INTVAL = currentInterval;
	if( currentInterval == 0 ){
		if(pxHigherPriorityWoken != nullptr)
			xSemaphoreGiveFromISR(_stepper->_doneInternal, pxHigherPriorityWoken);
		else xSemaphoreGive(_stepper->_doneInternal);
		stop();
		return;
	}
	stepCtrlMRT_CH->CTRL |= 1; // Enable interrupt
}

void Stepper::StepControl::stop() {
	stepCtrlMRT_CH->INTVAL = 0 | (1 << 31); // Stop immediatly
	stepCtrlMRT_CH->CTRL &= ~1; // Disable interrupt
}

void Stepper::StepControl::setStepsToRun(uint32_t steps) {
	if(steps > 0)
		halfStepsToRun = (2*steps) - 1;
	else
		halfStepsToRun = 0;
}

void Stepper::StepControl::MRT_callback(portBASE_TYPE* pxHigherPriorityWoken){
	if(halfStepsToRun != 0){
		bool maxed = !_stepper->direction ? _stepper->currentHalfSteps == _stepper->maxSteps : _stepper->currentHalfSteps == 0;

		if(!(_stepper->stop || maxed))
		{
			pulse();
			start(pxHigherPriorityWoken);
			--halfStepsToRun;
			_stepper->currentHalfSteps = !_stepper->direction ? _stepper->currentHalfSteps+1 : _stepper->currentHalfSteps-1;
		} else {
			stop();
			xSemaphoreGiveFromISR(_stepper->_doneInternal, pxHigherPriorityWoken);
		}
	} else {
		stop();
		xSemaphoreGiveFromISR(_stepper->_doneInternal, pxHigherPriorityWoken);
	}
}

extern "C" void MRT_IRQHandler(void) {
	uint32_t interruptPending = Chip_MRT_GetIntPending();
	Chip_MRT_ClearIntPending(interruptPending);
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	if(interruptPending & MRTn_INTFLAG(0)){
		Stepper::StepControl* stepControl = Stepper::getStepperByChannel(0)->getStepControl();
		stepControl->MRT_callback(&pxHigherPriorityTaskWoken);
	}

	if(interruptPending & MRTn_INTFLAG(1)){
		Stepper::StepControl* stepControl = Stepper::getStepperByChannel(1)->getStepControl();
		stepControl->MRT_callback(&pxHigherPriorityTaskWoken);
	}

	if(interruptPending & MRTn_INTFLAG(2)){
		Stepper* stepper = Stepper::getStepperByChannel(0);
		stepper->MRT_callback(&pxHigherPriorityTaskWoken);
	}

	if(interruptPending & MRTn_INTFLAG(3)){
		Stepper* stepper = Stepper::getStepperByChannel(1);
		stepper->MRT_callback(&pxHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
}

void Stepper::goHome() {
	setDirection(true);
	runForSteps(currentHalfSteps/2);
}

void Stepper::_runForSteps(uint32_t steps) {
//	xEventGroupSync(eventGroup, (1 << channel+2), (1 << 2) | (1 << 3), portMAX_DELAY); // Sync the stepping between motors. Didn't help
	if(steps > 0){
		accelMRT_CH->INTVAL = (Chip_Clock_GetSystemClockRate()*1000) / ACCELERATION_STEP_TIME_MS;
		accelMRT_CH->CTRL |= 1;
		stepControl.setStepsToRun(steps);
		stepControl.start(nullptr);
		xSemaphoreTake(_doneInternal, portMAX_DELAY);
	}
}

uint32_t Stepper::getRateForShorterAxle(uint32_t stepsShort,
		uint32_t stepsLong, uint32_t speedLong) {
	uint32_t speed = (stepsShort*speedLong)/stepsLong;

	return speed;
}

uint32_t Stepper::getAcceleration() const {
	return (_accelerationMilliStepSize / ACCELERATION_STEP_TIME_MS);
}

void Stepper::setAccelerationStepSize(uint32_t milliSteps){
	_accelerationMilliStepSize = milliSteps;
}

uint32_t Stepper::getStepsRequiredToAccelerate(
		uint32_t accelerationMillistepSize, uint32_t fromRate, uint32_t toRate) {
	uint32_t result = fromRate < toRate ?
			(toRate*toRate - fromRate*fromRate) / ((accelerationMillistepSize/ACCELERATION_STEP_TIME_MS)*2) + 0.5: // +0.5 is for rounding
			(fromRate*fromRate - toRate*toRate) / ((accelerationMillistepSize/ACCELERATION_STEP_TIME_MS)*2) + 0.5;
	return result;
}

void Stepper::_calibrate(uint32_t nothing) {
	setDirection(true); // Go forward first
	setRate(2000, true);
	_runForSteps(UINT32_MAX); // Run until we hit a switch
	setStop(false); // Clear stop flag

	toggleDirection();
	_runForSteps(150); // Back off from the limit switch

	zeroSteps();
	_runForSteps(UINT32_MAX); // Run to the other switch.
	setStop(false);

	toggleDirection();
	_runForSteps(150); // Back off from the limit switch
	maxSteps = currentHalfSteps;

	_runForSteps(currentHalfSteps/2);
	toggleDirection();
}

/*
 * PWMController.cpp
 *
 *  Created on: 5.10.2017
 *      Author: Ville
 */

#include "PWMController.h"

// Call constructors for L and H before calling initCounter{L,H} if used seperately
PWMController::PWMController(LPC_SCT_T* pSCTimer) {
	timer = pSCTimer;
	Chip_SCT_Init(pSCTimer);
	timer->CONFIG |= (1 << 17) | (1 << 18); // Set autolimit for both L and H counters on
	timer->EVENT[0].STATE = 0xFFFF; // Events 0 and 1 happen in all states.
	timer->EVENT[1].STATE = 0xFFFF;
}

void PWMController::initCounterL(uint16_t freq, double dutycycle,
		bool bidirectional, uint8_t prescaler) {
	L.freq = freq;
	L.dutycycle = dutycycle;
	L.bidirectional = bidirectional;
	L.prescaler = prescaler;
	if(prescaler > 1){
		timer->CTRL_L |= ((prescaler - 1) << 5) | (1 << 3); // Set prescaler and clear the count
	}

	timer->MATCHREL[0].L = periodFromFrequencyL(freq) - 1;
	timer->MATCHREL[1].L = (periodFromFrequencyL(freq) - 2) * (dutycycle/100.0);
	if(bidirectional){
		timer->CTRL_L |= (1 << 4); // Bidirectional
	}
	timer->EVENT[0].CTRL = 1 | (1 << 12); // Match 1L
}

void PWMController::initCounterH(uint16_t freq, double dutycycle,
		bool bidirectional, uint8_t prescaler) {
	H.freq = freq;
	H.dutycycle = dutycycle;
	H.bidirectional = bidirectional;
	H.prescaler = prescaler;
	if(prescaler > 1){
		timer->CTRL_H |= ((prescaler - 1) << 5) | (1 << 3);
	}

	timer->MATCHREL[0].H = periodFromFrequencyH(freq);
	timer->MATCHREL[1].H = periodFromFrequencyH(freq) * (dutycycle/100.0);
	if(bidirectional){
		timer->CTRL_H |= (1 << 4);
	}
	timer->EVENT[1].CTRL = 1 | (1 << 12) | (1 << 4); // Match 1H
}

void PWMController::startCounterL() {
	timer->CTRL_L &= ~(1 << 2);
}

void PWMController::startCounterH() {
	timer->CTRL_H &= ~(1 << 2);
}

void PWMController::stopCounterL() {
	timer->CTRL_L |= (1 << 2);
}

void PWMController::stopCounterH() {
	timer->CTRL_H |= (1 << 2);
}


uint16_t PWMController::periodFromFrequencyL(uint16_t freq) {
	return L.bidirectional ? ((SystemCoreClock / L.prescaler) / (freq * 2)) : ((SystemCoreClock / L.prescaler) / freq);
}

uint16_t PWMController::periodFromFrequencyH(uint16_t freq) {
	return H.bidirectional ? ((SystemCoreClock / H.prescaler) / (freq * 2)) : ((SystemCoreClock / H.prescaler) / freq);
}

void PWMController::setDutycycleL(double dutycycle) {
	L.dutycycle = dutycycle;
	timer->MATCHREL[1].L = (periodFromFrequencyL(L.freq) - 2) * (dutycycle / 100.0);
}

void PWMController::setDutycycleH(double dutycycle) {
	H.dutycycle = dutycycle;
	timer->MATCHREL[1].H = (periodFromFrequencyH(H.freq) - 2) * (dutycycle / 100.0);

}

double PWMController::getDutycycleL() {
	return L.dutycycle;
}

double PWMController::getDutycycleH() {
	return H.dutycycle;
}

void PWMController::setFrequencyL(uint16_t freq) {
	L.freq = freq;
	timer->MATCHREL[0].L = periodFromFrequencyL(L.freq) - 1;
	// Keep the dutycycle the same even if we change frequency
	timer->MATCHREL[1].L = (periodFromFrequencyL(L.freq) - 2) * (H.dutycycle / 100.0);
}

void PWMController::setFrequencyH(uint16_t freq) {
	H.freq = freq;
	timer->MATCHREL[0].H = periodFromFrequencyH(H.freq) - 1;
	timer->MATCHREL[1].H = (periodFromFrequencyH(H.freq) - 2) * (H.dutycycle / 100.0);
}

uint16_t PWMController::getFrequencyL() {
	return L.freq;
}

uint16_t PWMController::getFrequencyH() {
	return H.freq;
}
/* Outputs 3-7 are fixed pin outputs
 * Small SCTimers have only 6 outputs instead of 8 */
void PWMController::setOutputL(uint8_t port, uint8_t pin, uint8_t outindex,
		bool on) {
	if(L.bidirectional)
		timer->OUTPUTDIRCTRL |= (0x1 << (outindex * 2));
	timer->OUT[outindex].CLR = (1 << 0);
	if(outindex > 2){
		Chip_SWM_EnableFixedPin(getFixedSWMPin(outindex));
	} else {
		Chip_SWM_MovablePinAssign(getMovableSWMPin(outindex), (port * 32) + pin);
	}
}

/* Outputs 3-7 are fixed pin outputs
 * Small SCTimers have only 6 outputs instead of 8 */
void PWMController::setOutputH(uint8_t port, uint8_t pin, uint8_t outindex,
		bool on) {
	if(H.bidirectional)
		timer->OUTPUTDIRCTRL |= (0x2 << (outindex * 2));
	timer->OUT[outindex].CLR = (1 << 1);
	if(outindex > 2){
		Chip_SWM_EnableFixedPin(getFixedSWMPin(outindex));
	} else {
		Chip_SWM_MovablePinAssign(getMovableSWMPin(outindex), (port * 32) + pin);
	}
}

CHIP_SWM_PIN_FIXED_T PWMController::getFixedSWMPin(uint8_t outindex) {
	CHIP_SWM_PIN_FIXED_T fixed = SWM_FIXED_SCT0_OUT3;
	if(outindex > 2){
		outindex -= 3;
		if(timer == LPC_SCT0)
			fixed = static_cast<CHIP_SWM_PIN_FIXED_T>(SWM_FIXED_SCT0_OUT3 + outindex);
		else if(timer == LPC_SCT1)
			fixed = static_cast<CHIP_SWM_PIN_FIXED_T>(SWM_FIXED_SCT1_OUT3 + outindex);
		else if(timer == LPC_SCT2)
			fixed = static_cast<CHIP_SWM_PIN_FIXED_T>(SWM_FIXED_SCT2_OUT3 + outindex);
		else if(timer == LPC_SCT3)
			fixed = static_cast<CHIP_SWM_PIN_FIXED_T>(SWM_FIXED_SCT3_OUT3 + outindex);

	}
	return fixed;
}

CHIP_SWM_PIN_MOVABLE_T PWMController::getMovableSWMPin(uint8_t outindex) {
	CHIP_SWM_PIN_MOVABLE_T movable = SWM_SCT0_OUT0_O;
	if(outindex < 3){
		if(timer == LPC_SCT0)
			movable = static_cast<CHIP_SWM_PIN_MOVABLE_T>(SWM_SCT0_OUT0_O + outindex);
		else if (timer == LPC_SCT1)
			movable = static_cast<CHIP_SWM_PIN_MOVABLE_T>(SWM_SCT1_OUT0_O + outindex);
		else if (timer == LPC_SCT2)
			movable = static_cast<CHIP_SWM_PIN_MOVABLE_T>(SWM_SCT2_OUT0_O + outindex);
		else if (timer == LPC_SCT3)
			movable = static_cast<CHIP_SWM_PIN_MOVABLE_T>(SWM_SCT3_OUT0_O + outindex);

		}
		return movable;
}

/*
 * ButtonEvent.cpp
 *
 *  Created on: 7.12.2016
 *      Author: Ville
 */

#include "InterruptedInputPin.h"
#include "chip.h"

#ifdef FREERTOS
static void doNothing(portBASE_TYPE*){;};
#else
static void doNothing(){;};
#endif

InterruptedInputPin::interruptFunction InterruptedInputPin::handlers[8]
									= { doNothing, doNothing, doNothing, doNothing,
										doNothing, doNothing, doNothing, doNothing};

InterruptedInputPin::InterruptedInputPin(int port_, int pin_, bool pullup, bool risingEdgeInterrupt_, int risingEdgeInterruptChannel_, bool fallingEdgeInterrupt_, int fallingEdgeInterruptChannel_) {

	port = port_;
	pin = pin_;
	risingEdgeInterruptChannel = risingEdgeInterruptChannel_;
	fallingEdgeInterruptChannel = fallingEdgeInterruptChannel_;
	risingEdgeInterrupt = risingEdgeInterrupt_;
	fallingEdgeInterrupt = fallingEdgeInterrupt_;

	/* Set pin back to GPIO (on some boards may have been changed to something
			   else by Board_Init()) */
	if(pullup)
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_DIGMODE_EN | IOCON_MODE_INACT | IOCON_MODE_PULLUP ) );
	else
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_DIGMODE_EN | IOCON_MODE_INACT | IOCON_MODE_PULLDOWN ) );


	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
	IRQn IRQnumber;
	if(risingEdgeInterrupt){
		IRQnumber = static_cast<IRQn>(PIN_INT0_IRQn + risingEdgeInterruptChannel);
		Chip_INMUX_PinIntSel(risingEdgeInterruptChannel, port, pin);
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(risingEdgeInterruptChannel));
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(risingEdgeInterruptChannel));
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(risingEdgeInterruptChannel));
		NVIC_ClearPendingIRQ(IRQnumber);
#ifdef FREERTOS
		NVIC_SetPriority(IRQnumber, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
		NVIC_EnableIRQ(IRQnumber);
	}
	if(fallingEdgeInterrupt){
		IRQnumber = static_cast<IRQn>(PIN_INT0_IRQn + fallingEdgeInterruptChannel);
		Chip_INMUX_PinIntSel(fallingEdgeInterruptChannel, port, pin);
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(fallingEdgeInterruptChannel));
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(fallingEdgeInterruptChannel));
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(fallingEdgeInterruptChannel));
		NVIC_ClearPendingIRQ(IRQnumber);
#ifdef FREERTOS
		NVIC_SetPriority(IRQnumber, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
		NVIC_EnableIRQ(IRQnumber);
	}
}

InterruptedInputPin::~InterruptedInputPin() {
	if(fallingEdgeInterrupt)
		NVIC_DisableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + fallingEdgeInterruptChannel));
	if(risingEdgeInterrupt)
		NVIC_DisableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + risingEdgeInterruptChannel));
}

bool InterruptedInputPin::hasFallingEdgeInterruptEnabled() const {
	return fallingEdgeInterrupt;
}

int InterruptedInputPin::getFallingEdgeInterruptChannel() const {
	return fallingEdgeInterruptChannel;
}

int InterruptedInputPin::getPin() const {
	return pin;
}

int InterruptedInputPin::getPort() const {
	return port;
}

bool InterruptedInputPin::hasRisingEdgeInterruptEnabled() const {
	return risingEdgeInterrupt;
}

int InterruptedInputPin::getRisingEdgeInterruptChannel() const {
	return risingEdgeInterruptChannel;
}

void InterruptedInputPin::setRisingEdgeInterrupt(bool set) {
	if(!set){
		risingEdgeInterrupt = false;
		NVIC_DisableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + risingEdgeInterruptChannel));
	} else {
		risingEdgeInterrupt = true;
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(risingEdgeInterruptChannel));
		NVIC_EnableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + risingEdgeInterruptChannel));
	}
}

void InterruptedInputPin::setFallingEdgeInterrupt(bool set) {
	if(!set){
		fallingEdgeInterrupt = false;
		NVIC_DisableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + fallingEdgeInterruptChannel));
	} else {
		fallingEdgeInterrupt = true;
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(fallingEdgeInterruptChannel));
		NVIC_EnableIRQ(static_cast<IRQn>(PIN_INT0_IRQn + fallingEdgeInterruptChannel));
	}
}

void InterruptedInputPin::setInterruptHandler(uint8_t channel, interruptFunction f) {
	handlers[channel] = f;
}

InterruptedInputPin::interruptFunction InterruptedInputPin::getInterruptHandler(uint8_t channel){
	return handlers[channel];
}

#ifndef FREERTOS
extern "C" {
extern void PIN_INT0_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(0)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
}
extern void PIN_INT1_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(1)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
}
extern void PIN_INT2_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(2)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
}
extern void PIN_INT3_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(3)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
}
extern void PIN_INT4_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(4)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(4));
}
extern void PIN_INT5_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(5)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
}
extern void PIN_INT6_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(6)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
}
extern void PIN_INT7_IRQHandler(void){
	InterruptedInputPin::getInterruptHandler(7)();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
}
}

#else
extern "C" {
extern void PIN_INT0_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(0)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT1_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(1)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT2_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(2)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT3_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(3)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT4_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(4)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(4));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT5_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(5)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT6_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(6)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
extern void PIN_INT7_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	InterruptedInputPin::getInterruptHandler(7)(&higherPriorityTaskWoken);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
}
#endif

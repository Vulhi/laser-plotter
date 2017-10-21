template <int channel>
LimitSwitch<channel>* LimitSwitch<channel>::thisPtr;


/* Remember to call Chip_PININT_Init(LPC_GPIO_PIN_INT) before calling this */
template <int channel>
LimitSwitch<channel>::LimitSwitch(int port, int pin, Stepper& stepper, bool max)
: LimitSwitch_Base(port, pin, channel, stepper, max) {
	pinControl.setInterruptHandler(channel, IRQHandler);
}

template <int channel>
void LimitSwitch<channel>::IRQHandler(portBASE_TYPE* pxHigherPriorityTaskWoken){
	static TickType_t time = 0;
	static bool lastInterruptWasFall = false;
	if(xTaskGetTickCountFromISR() - time > 10){
		if(
			(((LPC_GPIO_PIN_INT->FALL) >> channel) & 1)
			&& !lastInterruptWasFall
			&& (thisPtr->stepper.getDirection() == thisPtr->max)
			) { // Buttons go low when pressed.
			thisPtr->stepper.setStop(true);
			ITM_write("Bit set\r\n");
			lastInterruptWasFall = true;
		} else if (
			lastInterruptWasFall
			&& (thisPtr->stepper.getDirection() == thisPtr->max)
			) {
			thisPtr->stepper.setStop(false);
			ITM_write("Bit cleared\r\n");
			lastInterruptWasFall = false;
		}
		time = xTaskGetTickCountFromISR();
	}
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(channel));
}

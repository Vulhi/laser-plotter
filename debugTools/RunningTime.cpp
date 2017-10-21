/*
 * RunningTime.cpp
 *
 *  Created on: 10.10.2017
 *      Author: Ville
 */

#include "RunningTime.h"

#include "board.h"
#include "chip.h"

void RunningTime::setup(){
	Chip_SCT_Init(LPC_SCT2);
	LPC_SCT2->CONFIG |= 1 << 0;
	LPC_SCT2->CTRL_L |= ((SystemCoreClock / 1000000-1) << 5);
}

void RunningTime::start() {
	LPC_SCT2->CTRL_L |= (1 << 3);
	LPC_SCT2->CTRL_L &= ~(1 << 2);
}

void RunningTime::stop() {
	LPC_SCT2->CTRL_L |= (1 << 2);
}

uint32_t RunningTime::getTime() {
	return LPC_SCT2->COUNT_U;
}

/*
 * RunningTime.h
 *
 *  Created on: 10.10.2017
 *      Author: Ville
 */

#ifndef DEBUGTOOLS_RUNNINGTIME_H_
#define DEBUGTOOLS_RUNNINGTIME_H_
#include <cstdint>

class RunningTime {
public:
	static void setup();
	static void start();
	static void stop();
	static uint32_t getTime();
};

#endif /* DEBUGTOOLS_RUNNINGTIME_H_ */

/*
 * Parser.cpp
 *
 *  Created on: 20.10.2017
 *      Author: Ville
 */

#include "Parser.h"

#include "debugTools/ITM_write.h"

Command Parser::getCommand() {
	uint32_t len = USB_receive(reinterpret_cast<uint8_t*>(recvBuffer), RCV_BUFSIZE);
	recvBuffer[len] = '\0';
	ITM_write(recvBuffer);
	char code[5];
	char params[27];
	int angle, intensity;
	double x;
	double y;
	sscanf(recvBuffer, "%s %[^\t\n]", code, params);
	if (strcmp(code, "G1") == 0) {
		sscanf(params, "X%lf Y%lf", &x,&y);
		return Command {CODES::G1, x, y};
	} else if (strcmp(code, "M1") == 0) {
		sscanf(params, "%d", &angle);
		return Command {CODES::M1, angle};
	} else if (strcmp(code, "M4") == 0) {
		sscanf(params, "%d", &intensity);
		return Command {CODES::M4, intensity};
	} else if (strcmp(code, "M10") == 0) {
		return Command {CODES::M10};
	} else if (strcmp(code, "G28") == 0) {
		return Command {CODES::G28};
	} else {
		return Command {CODES::E};
	}
}

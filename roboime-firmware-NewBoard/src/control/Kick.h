/*
 * Kick.h
 *
 *  Created on: 21 Jun 2019
 *      Author: user
 */

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <utils/time_functions.h>
#include "GPIO.h"
#ifndef SRC_CONTROL_KICK_H_
#define SRC_CONTROL_KICK_H_

class Kick {
public:
	Kick(GPIO *pin, uint32_t power);
	void set_kick_power(uint32_t power);
	void kick_cmd(uint32_t power);
private:
	GPIO *kickPin;
	uint32_t kickPower;
};

#endif /* SRC_CONTROL_KICK_H_ */

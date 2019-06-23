/*
 * Drible.h
 *
 *  Created on: Aug 24, 2016
 *      Author: OniasC
 */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "Pwm.h"
#ifndef DRIBLE_H_
#define DRIBLE_H_
class Drible{
public:
	Pwm* Drible_Pwm;
	Drible();
	void Set_Vel(uint16_t value);
};
#endif /* DRIBLE_H_ */

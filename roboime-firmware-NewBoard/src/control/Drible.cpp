/*
 * Drible.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: OniasC
 */
#include <Drible.h>
#include "Pwm.h"
Drible::Drible()
{
	Drible_Pwm = new Pwm(GPIOE, GPIO_Pin_5, TIM9, GPIO_PinSource5, GPIO_AF_TIM9, 1, false);
	Drible_Pwm->set_DutyCycle(0);
}
void Drible::Set_Vel(uint16_t value){
	Drible_Pwm->set_DutyCycle(value);
}





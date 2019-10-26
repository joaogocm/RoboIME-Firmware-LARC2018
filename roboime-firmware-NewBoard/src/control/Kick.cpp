/*
 * Kick.cpp
 *
 *  Created on: 21 Jun 2019
 *      Author: user
 */

#include <Kick.h>

Kick::Kick(GPIO *pin, uint32_t power) {
	kickPin = pin;
	kickPower = power;
	GPIO chargePin(GPIOD, GPIO_Pin_8);
}

void Kick::set_kick_power(uint32_t power){
	kickPower = power;
}

void Kick::charge_rst(){
	chargePin->Reset();
}

void Kick::charge_set(){
	chargePin->Set();
}

void Kick::kick_cmd(uint32_t power){
	set_kick_power(power);
	kickPin->Set();
	delay_ticks((uint32_t) (power*611)); //611 = Gustavo's magic number
	kickPin->Reset();
	this->charge_rst();
}

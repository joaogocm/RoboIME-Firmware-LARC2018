/*
 * Motor.cpp
 *
 *  Created on: Mar 12, 2016
 *      Author: lenovoi7
 */
#include "Motor.h"

/*
 * Usou-se ziegler-nichols frequency response method
 * Kc=3.2
 * Tc =0.7s
 */
float Motor::cp=8000.0f;
float Motor::ci=10.0f;
float Motor::cd=1000.0f;


Motor::Motor(Pwm *A_High,
		GPIO *A_Low,
		Pwm *B_High,
		GPIO *B_Low,
		Encoder *Enc,
		Timer_Time2 *MTimer)
{
	Motor_A_High = A_High;
	Motor_A_Low = A_Low;
	Motor_B_High = B_High;
	Motor_B_Low = B_Low;
	Motor_Enc = Enc;
	Motor_Time = MTimer;
	last_vel_answer = 0;
}
void Motor::Control_Pos(float  hold_position){
	uint32_t position;
	int16_t answer;
	position = Motor_Enc->get_position();
	answer = this->Pos_Calc_Answer(position, hold_position);	//this eh opcional por estar
																//pos_calc_answer estar dentro
																//do objeto
	this->SetDutyCycle(answer);
	return;
};

//retorna o deslocamento do eixo do motor, unidade: divisões de encoder
int16_t Motor::Get_Desloc(){
	return Motor_Enc->get_position()-last_position;
}

void Motor::get_motor_speed(){
	//position é medida em divisões de encoder (ou 1/400 de volta)
	int16_t position = (int16_t)Motor_Enc->get_position();
	Motor_Enc->set_position(20000);

	int16_t distance=position-20000;

	float speed=(float)distance*CONVERSION; //converte da unidade da roda para m/s (vel do centro da roda)
	                                     //talvez seja melhor converter de m/s pra unidade da roda
	real_wheel_speed=speed;
};

//será chamada pelo handler da interrupção gerada pelo TIM6(a cada 1 ms)
//hold_speed é a velocidade da RODA em m/s
void Motor::control_motor_speed(float desired_speed){
	this->get_motor_speed();

	error = desired_speed-real_wheel_speed;
	ierror = 0;
	for(int j = 18; j > 0; j--){
		last_error[j+1]=last_error[j];
		ierror = ierror + last_error[j+1];
	}
	last_error[0]=error;
	ierror = ierror + last_error[0];
	//if(ierror > 1000) ierror = 1000;
	//if(ierror < -1000) ierror = -1000;

	derror=error-last_error[1];

	float out=cp*error + ci * ierror + cd * derror;


	dutycycle=out;
	if(dutycycle>1000) dutycycle=1000;
	if(dutycycle<-1000) dutycycle=-1000;
	SetDutyCycle(dutycycle);

};

void Motor::SetDutyCycle(int16_t answer)
{
	if (answer > 0)
	{
		if (answer>1000)
		{
			answer=1000;
		}
		/**** ROBO NOVO: TROCAR ESTE BLOCO COM O PRÓXIMO ****/
		Motor_B_Low->Reset();
		Motor_A_High->set_DutyCycle(0);
		while(Motor_B_Low->Status());
		Motor_B_High->set_DutyCycle(answer);
		Motor_A_Low->Set();
		/****************************************************/
	}
	else
	{
		answer=-answer;
		if(answer>1000)
		{
			answer=1000;
		}
		/**** ROBO NOVO: TROCAR ESTE BLOCO COM O ANTERIOR ****/
		Motor_A_Low->Reset();
		Motor_B_High->set_DutyCycle(0);
		while(Motor_A_Low->Status());
		Motor_A_High->set_DutyCycle(answer);
		Motor_B_Low->Set();
		/*****************************************************/
	}
	return;
}

void Motor::SetPID(float p, float i, float d) {
	cp=p;
	ci=i;
	cd=d;
}

void Motor::GetPID(float c[]){
	c[0]=cp;
	c[1]=ci;
	c[2]=cd;
}

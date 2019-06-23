/*
 * Robo.h
 *
 *  Created on: Jul 9, 2016
 *      Author: lenovoi7
 */
#ifndef ROBO_H_
#define ROBO_H_

#include <Drible.h>
#include <cstring>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "GPIO.h"
#include "Pwm.h"
#include "Encoder.h"
#include "TimerTime2.h"
#include "Motor.h"
#include "adc.h"
#include "Switch.h"
#include "Kick.h"

#include "proto/grSim_Commands.pb.h"
#include "proto/pb_decode.h"
#include "proto/pb_encode.h"
#include "radio/commands.h"
#include <radio/bsp.h>

#include <utils/time_functions.h>


class Robo {
public:
	Robo(Motor *roboMotor0, Motor *roboMotor1, Motor *roboMotor2, Motor *roboMotor3, NRF24L01P *mynrf24, uint8_t ID, adc *sensorAdc, Drible *roboDrible, Kick *roboHighKick, Kick *roboLowKick, bool testmode=1);

	Drible *motorDrible;

	Kick *high_kick;
	Kick *low_kick;
	void high_kick_cmd(float power);
	void low_kick_cmd(float power);

    adc *roboAdc;
    float vBat;
    int nVerifyPacket;
    int nPacketReceived;
    NRF24L01P *_nrf24;

    /********************************    CONTROLE   ********************************/
	int pos[4];
	float speed[4];	//velocidades desejadas para cada motor
	float real_wheel_speed[4];	//armazenará as velocidades medidas (m/s) das RODAS
    Motor *motors[4];

    void get_wheel_speed(); //armazena as velocidades lineares das RODAS em *real_wheel_speed

    void set_robo_speed(float v_r, float v_t, float w); //converte as velocidades v_r, v_t e wR desejadas em speed[4]
    void set_robo_speed(float *v);

    void control_speed(); //deve ser deletado no futuro
	void control_robo_speed(float v_r, float v_t, float w); //realiza controle PID das velocidades do robo
	void control_robo_speed(float *v);

	float robo_pid(float cp, float ci, float cd, float error, float ierror, float derror, float *last_error); //malha de controle do PID

    void set_motor_speed(uint8_t motnr, float vel);
    void set_motor_speed();

	void control_pos();
	/*******************************************************************************/

    bool _testmode;
    bool InTestMode(){return _testmode;};
    void SetTestMode(bool testmode) {_testmode=testmode;}
    uint8_t GetId(){return _id;}
    void IncId();//foi adicionado na tentativa de logica do botão
    void ZeraId();//foi adicionado na tentativa de logica do botão
    void interrupt_control();
    void interruptReceive();
    void interruptTestMode();
    void processPacket();
   	void interruptTransmitter();
   	void init();
   	void interruptAckPayload();

   	bool stepbit;
   	bool printI;
    bool controlbit;
    bool printv;
   	uint8_t channel;
   	uint64_t address;
   	int16_t dutycycles[4];

   	uint32_t last_kick_time=0;

    uint32_t last_packet_ms = 0;
    grSim_Robot_Command robotcmd;
    grSim_Robot_Command robotcmd_test;
protected:
    uint8_t _id;
private:
    float error_r, error_t, error_w; //armazena os erros de v_r, v_t e wR para o algoritmo de pid
    float ierror_r, ierror_t, ierror_w;
    float derror_r, derror_t, derror_w;
    float last_error_r[20], last_error_t[20], last_error_w[20];
};

extern Robo robo;
#endif /* ROBO_H_ */

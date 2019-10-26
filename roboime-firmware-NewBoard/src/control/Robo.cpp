/*
 * Robo.cpp
 *
 *  Created on: Mar 12, 2016
 *      Author: lenovoi7
 */

#include "Robo.h"
#include "pins.h"
#define sin_phi 0.50
#define cos_phi 0.866
#define sin_theta 0.707
#define cos_theta 0.707
#define R 0.075 //Raio do robo = 9cm

/*#define alfa 1
#define Massa 3
#define Raio 0.09*/

Robo::Robo(Motor *roboMotor0, Motor *roboMotor1, Motor *roboMotor2, Motor *roboMotor3, NRF24L01P *mynrf24, uint8_t ID, adc *sensorAdc, Drible *roboDrible, Kick *roboHighKick, Kick *roboLowKick, bool testmode):
	//uint8_t bitStatus2 = 0;
	_nrf24(mynrf24),
	_testmode(testmode),
	printv(false)
{
	high_kick=roboHighKick;
	low_kick=roboLowKick;
	motorDrible=roboDrible;
	motors[0]=roboMotor0;
	motors[1]=roboMotor1;
	motors[2]=roboMotor2;
	motors[3]=roboMotor3;
	roboAdc = sensorAdc;
	roboAdc->ADC_Config();
	//_id=ID;
	//_id=0;
	//chute_alto = new GPIO(GPIOB, GPIO_Pin_0);
	//chute_baixo = new GPIO(GPIOD, GPIO_Pin_10);
	/* inicio de tentativa de logica para o botão*/
	/*uint8_t bitStatus2 = (uint8_t)Bit_RESET;
	uint8_t bitStatus=ID_Button.Read();
	if(bitStatus2!=bitStatus){
		if(bitStatus==(uint8_t)Bit_SET){
			//char *buffer = "voce apertou o botao\n";
			//CDC_Transmit_FS((uint8_t *) buffer, 22);
			if(_id<15){
				_id=_id+1;
			}
			else{
				_id=0;
			}
		}
		else{
			//char *buffer = "voce soltou o botao\n";
			//CDC_Transmit_FS((uint8_t *) buffer, 21);
		}
		bitStatus2=ID_Button.Read();
	}*/
   // osDelay(100);
    /*fim de tentativa de logica para o botão*/
	//_id=0;
	//channel=43;
	channel=107;
	//channel=117;
	address=0xE7E7E7E700;
	last_packet_ms = 0;
}

void Robo::init(){
	_nrf24->Init();
	_nrf24->Config();

	_nrf24->StartRX_ESB(channel, address + GetId(), 32, 1);
	_nrf24->TxPackage_ESB(channel, address + GetId(), 0,(uint8_t*) "TESTE", 5);
	while(_nrf24->Busy()){
		_nrf24->InterruptCallback();
	}
	_nrf24->StartRX_ESB(channel, address + GetId(), 32, 1);
}

void Robo::ball_detection(){
	last_ball_detection_time=GetLocalTime();
}

void Robo::high_kick_cmd(float power){
	if((GetLocalTime()-last_kick_time)>700){
			if(GetLocalTime()-last_ball_detection_time<200){
				low_kick->kick_cmd((uint32_t) power);
				last_kick_time = GetLocalTime();
				last_ball_detection_time = GetLocalTime();
			}
		}
}

void Robo::low_kick_cmd(float power){
	if((GetLocalTime()-last_kick_time)>700){
		if(GetLocalTime()-last_ball_detection_time<200){
			high_kick->kick_cmd((uint32_t) power);
			last_kick_time = GetLocalTime();
			last_ball_detection_time = GetLocalTime();
		}
	}
}

void Robo::control_pos(){
	for(int i=0; i<4; i++){
		motors[i]->Control_Pos(pos[i]);
	}
}

//armazena as velocidades lineares dos centros das RODAS em *real_wheel_speed
void Robo::get_wheel_speed(){
	for(int i=0; i<4; i++){
		real_wheel_speed[i]=motors[i]->real_wheel_speed;
	}
}

void Robo::control_robo_speed(float v_r, float v_t, float w){
	get_wheel_speed();

	//error=V(intel)-(D+)*(real_wheel_speed):
	error_r=v_r-(-0.4142*real_wheel_speed[0]+0.4142*real_wheel_speed[1]-0.4142*real_wheel_speed[2]+0.4142*real_wheel_speed[3]);
	error_t=v_t-(0.3464*real_wheel_speed[0]+0.2828*real_wheel_speed[1]-0.3464*real_wheel_speed[2]-0.2828*real_wheel_speed[3]);
	error_w=w-(0.2929*real_wheel_speed[0]+0.2071*real_wheel_speed[1]+0.2929*real_wheel_speed[2]+0.2071*real_wheel_speed[3])/R;

	float pid_r, pid_t, pid_w; //armazena v_r, v_t e wR calculados pelo PID

	pid_r=robo_pid(0, 0, 0, error_r, ierror_r, derror_r, last_error_r);
	pid_t=robo_pid(0, 0, 0, error_t, ierror_t, derror_t, last_error_t);
	pid_w=robo_pid(0, 0, 0, error_w, ierror_w, derror_w, last_error_w);

	robo.set_robo_speed(v_r+pid_r, v_t+pid_t, w+pid_w); //atualiza os valores desejados de velocidade para cada motor após o PID
}

float Robo::robo_pid(float cp, float ci, float cd, float error, float ierror, float derror, float *last_error){

	//implementar PID para o robo ~usando metodo tustin de discretização~

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

	float out=cp*error + ci*ierror + cd*derror;

	return out;
}

//*******  REVISAR CÓDIGO ABAIXO SOBRE CHECAGEM DE DESLIZAMENTO  ******//
void Robo::control_speed(){
  vBat = 4.3*roboAdc->adc_getConversion();

  float v0= motors[0]->real_wheel_speed;
  float v1= motors[1]->real_wheel_speed;
  float v2= motors[2]->real_wheel_speed;
  float v3= motors[3]->real_wheel_speed;

  float proj=-0.5477*v0+0.4472*v1+0.5477*v2-0.4472*v3;
  //vBat=7;
  if(vBat>6.2){
    if(proj<1 && proj>-1){
    	for(int i=0; i<4; i++){
    		motors[i]->control_motor_speed(speed[i]); //manda a velocidade speed[i] pro motor[i] na unidade m/s
    	}
    }
    else {
    	//nem o valor de alfa nem a massa interferem no espaço nulo.
    	/*speed[0]=v0*0.8+0.2*v2+0.245*(v1-v3);
    	speed[2]=v2*0.8+0.2*v0-0.245*(v1-v3);
    	speed[3]=v3*0.7+0.245*(v2-v0)+0.3*v1;
    	speed[1]=v1*0.7-0.245*(v2-v0)+0.3*v3;*/
    	speed[0]=v0-(-0.5477*v0+0.4472*v1+0.5477*v2-0.4472*v3)*(-0.5477);
    	speed[1]=v1-(-0.5477*v0+0.4472*v1+0.5477*v2-0.4472*v3)*(0.4472);
    	speed[2]=v2-(-0.5477*v0+0.4472*v1+0.5477*v2-0.4472*v3)*(0.5477);
    	speed[3]=v3-(-0.5477*v0+0.4472*v1+0.5477*v2-0.4472*v3)*(-0.4472);
    	for(int i=0; i<4; i++){
     		motors[i]->control_motor_speed(speed[i]); //manda a velocidade speed[i] pro motor[i] na unidade m/s
     	}
    }
  }
  else{
    for(int i=0; i<4; i++){
	  motors[i]->SetDutyCycle(0);
    }
  }
}

//recebe as velocidades radial, tangente em m/s e w em rad/s
//grava em speed[] os valores em m/s da velocidade DAS RODAS
void Robo::set_robo_speed(float v_r, float v_t, float w){

	speed[0] = v_t*cos_phi - v_r*sin_phi + w*R;
	speed[1] = v_t*cos_theta + v_r*sin_theta + w*R;
	speed[2] = -v_t*cos_phi - v_r*sin_phi + w*R;
	speed[3] = -v_t*cos_theta + v_r*sin_theta + w*R;

	//speed[] está em m/s. Cuidado para manter a mesma unidade qnd passar pros motores
}

void Robo::set_robo_speed(float *v){
	speed[0] = v[0];
	speed[1] = v[1];
	speed[2] = v[2];
	speed[3] = v[3];
}

void Robo::set_motor_speed(uint8_t motnr, float vel){
	if(motnr<4){
		speed[motnr]=vel;
	}
}

void Robo::set_motor_speed(){
	for(int i=0; i<4; i++){
 		motors[i]->control_motor_speed(speed[i]); //manda a velocidade speed[i] pro motor[i] na unidade m/s
 	}
}

void Robo::interrupt_control(){
	get_wheel_speed(); //update real_wheel_speed com as velocidades medidas
	if(controlbit&&(!stepbit)){
		robo.set_motor_speed();
	}
	if(!controlbit){
		for(int j=0; j<4; j++){
			if(!stepbit)motors[j]->SetDutyCycle(0);
			motors[j]->get_motor_speed();
		}
	}
	if (printv){
		robotcmd_test.kickspeedx = robo.motors[0]->real_wheel_speed;
		robotcmd_test.kickspeedz = robo.motors[1]->real_wheel_speed;
		robotcmd_test.veltangent = robo.motors[2]->real_wheel_speed;
		robotcmd_test.velnormal = robo.motors[3]->real_wheel_speed;
		robotcmd_test.velangular=(float)GetLocalTime();
		robotcmd_test.id=robo.dutycycles[0];
		robotcmd_test.spinner=true;
		uint8_t buffer[32];
		pb_ostream_t ostream=pb_ostream_from_buffer(buffer, sizeof(buffer));
		pb_encode(&ostream, grSim_Robot_Command_fields, &robotcmd_test);//escreve em ostream os dados de robotcmd_test

		_usbserialbuffer2.In((uint8_t*)buffer, (uint16_t) ostream.bytes_written);
	}
	if ((!printv)&&(printI)){
		robotcmd_test.velangular=(float)GetLocalTime();
		robotcmd_test.spinner=false;
		uint8_t buffer[32];
		pb_ostream_t ostream=pb_ostream_from_buffer(buffer, sizeof(buffer));
		pb_encode(&ostream, grSim_Robot_Command_fields, &robotcmd_test);

		_usbserialbuffer2.In((uint8_t*)buffer, (uint16_t) ostream.bytes_written);
	}
}

void Robo::interruptReceive(){
    bool status=0;
	if(_nrf24->RxSize()){
		_nrf24->StartRX_ESB(channel, address + robo.GetId(), 32, 1);
		uint8_t rxsize=_nrf24->RxSize();
		if(rxsize>32) rxsize=32;
		uint8_t buffer[32];
		_nrf24->RxData(buffer, rxsize);
		//usb_device_class_cdc_vcp.SendData((uint8_t*)buffer, rxsize);
		pb_istream_t istream=pb_istream_from_buffer(buffer, rxsize);
		status=pb_decode(&istream, grSim_Robot_Command_fields, &robotcmd);
		last_packet_ms = GetLocalTime();
		controlbit = true;
	}
	if(status){
		if(robotcmd.id==GetId()){
			processPacket();
		}
	}
	if((GetLocalTime()-last_packet_ms)>100){
		controlbit = false;
	}
}

void Robo::interruptTestMode(){
	cmdline.In(_usbserialbuffer);
	cmdline.Out(_usbserialbuffer);
	if(_usbserialbuffer.Ocupied()){
		//usb_device_class_cdc_vcp.SendData(_usbserialbuffer);
	}
	if(_usbserialbuffer2.Ocupied()){
		usb_device_class_cdc_vcp.SendData(_usbserialbuffer2);
	}
	//robo.controlbit = true;
}

//IO_Pin_STM32 CE(IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_8, GPIO_PuPd_UP, GPIO_OType_PP); //PD8->CHARGE ENABLE
void Robo::processPacket(){
	robo.control_robo_speed(robotcmd.veltangent, robotcmd.velnormal, robotcmd.velangular);
	if(robotcmd.kickspeedx!=0){
		robo.low_kick_cmd(robotcmd.kickspeedx);
		//CE.Reset();
	}
	if(robotcmd.kickspeedz!=0){
		robo.high_kick_cmd(robotcmd.kickspeedz);
		//CE.Reset();
	}

	if(robotcmd.spinner)
		robo.motorDrible->Set_Vel(800);
	//robo.drible->Set_Vel(100);
	else if(!robotcmd.spinner)
		robo.motorDrible->Set_Vel(0);
}

void Robo::interruptTransmitter(){
    bool status=0;
	uint8_t buffer[32];
	uint8_t size=_usbserialbuffer.Out(buffer, 32);//escreve em buffer o que recebeu
	pb_istream_t istream = pb_istream_from_buffer(buffer,size);
	status=pb_decode(&istream, grSim_Robot_Command_fields, &robotcmd);//preenche robotcmd
	if(status){//caso haja sucesso na decodificação
		uint8_t robotid=robotcmd.id;//extrai o  id do pacote
		uint8_t buffer[32];
		pb_ostream_t ostream=pb_ostream_from_buffer(buffer, sizeof(buffer));
		pb_encode(&ostream, grSim_Robot_Command_fields, &robotcmd);//escreve em ostream os dados de robotcmd
		uint8_t size=ostream.bytes_written;
		_nrf24->TxPackage_ESB(channel, address | robotid, 1, buffer, size);//aqui foi um lugar que foi coloca 1 no campo "no_ack" para resolver o problema da comunicação
		while(_nrf24->Busy()){
			_nrf24->InterruptCallback();
		}
	}
	else {
		_usbserialbuffer.Clear();
	}
}
void Robo::interruptAckPayload(){
	char ackBuffer[20];
	int ackSize=sprintf(ackBuffer, "test \n");
	_nrf24->write_ack_payload((uint8_t *) ackBuffer, ackSize);
}

void Robo::IncId(){
	_id=_id+1;
	_nrf24->StartRX_ESB(channel, address + GetId(), 32, 1);
}
void Robo::ZeraId(){
	_id=0;
	_nrf24->StartRX_ESB(channel, address + GetId(), 32, 1);
}
void Robo::SetId(int id){
	_id=id;
	_nrf24->StartRX_ESB(channel, address + id, 32, 1);
}

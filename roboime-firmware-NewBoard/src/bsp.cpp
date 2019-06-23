#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <radio/bsp.h>
#include <radio/serialnumber.h>
#include <hal_stm32/interrupt_stm32.h>
#include <radio/version.h>
#include <list>
#include <control/Robo.h>
#include <control/Switch.h>
#include "TimerTime.h"
extern "C"{
	#include "usb_dcd_int.h"
	#include "usb_hcd_int.h"
}
LED led_a(new IO_Pin_STM32 (IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_12));// led verde transmissão
LED led_b(new IO_Pin_STM32 (IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_13));// led laranja erro na transmissão
LED led_c(new IO_Pin_STM32 (IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_14));// led vermelho erro de comunicação
LED led_d(new IO_Pin_STM32 (IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_15));//led azul, comunicação

IO_Pin_STM32 ID_Button(IO_Pin::IO_Pin_Mode_IN, GPIOE, GPIO_Pin_2, GPIO_PuPd_UP);

IO_Pin_STM32 USB_DP(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_12, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_OTG_FS);
IO_Pin_STM32 USB_DM(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_11, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_OTG_FS);

//USB_DEVICE_CLASS_CDC_RNDIS usb_device_class_cdc_rndis(1);

std::list<USB_DEVICE_CLASS*> USB_DEVICE_CLASS::_usb_device_classes_list;
USB_DEVICE_CLASS_CDC_VCP usb_device_class_cdc_vcp({"RoboIME Serial Port"},1);
USB_STM32 usb(0x29BC, 0x2000, "IME", "RoboIME", SerialNumberGetHexaString());

IO_Pin_STM32 SPI1_SCK_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_5, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI1);
IO_Pin_STM32 SPI1_MISO_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_6, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI1);
IO_Pin_STM32 SPI1_MOSI_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_7, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI1);

IO_Pin_STM32 NRF24_SS_PIN(IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_0, GPIO_PuPd_NOPULL, GPIO_OType_PP);
IO_Pin_STM32 NRF24_CE_PIN(IO_Pin::IO_Pin_Mode_OUT, GPIOC, GPIO_Pin_12, GPIO_PuPd_NOPULL, GPIO_OType_PP);
IO_Pin_STM32 NRF24_IRQN_PIN(IO_Pin::IO_Pin_Mode_IN, GPIOC, GPIO_Pin_5, GPIO_PuPd_UP, GPIO_OType_OD);

SPI_STM32 spi_nrf(SPI1, NRF24_SS_PIN, SPI_BaudRatePrescaler_32);

NRF24L01P nrf24(spi_nrf, NRF24_SS_PIN, NRF24_CE_PIN, NRF24_IRQN_PIN);

IO_Pin_STM32 SPI2_SCK_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOB, GPIO_Pin_13, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI2);//ok
IO_Pin_STM32 SPI2_MISO_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOB, GPIO_Pin_14, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI2);//ok
IO_Pin_STM32 SPI2_MOSI_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, /*GPIOB*/GPIOE, /*GPIO_Pin_15*/GPIO_Pin_5, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_SPI2);//ok
IO_Pin_STM32 SDCARD_SS_PIN(IO_Pin::IO_Pin_Mode_OUT, GPIOD, GPIO_Pin_3, GPIO_PuPd_UP, GPIO_OType_PP);//ok
IO_Pin_STM32 MPU9250_SS_PIN(IO_Pin::IO_Pin_Mode_OUT, GPIOC, GPIO_Pin_10, GPIO_PuPd_UP, GPIO_OType_PP);//ok

SPI_STM32 spi_mpu(SPI2, MPU9250_SS_PIN, SPI_BaudRatePrescaler_128);
SPI_STM32 spi_sdcard(SPI2, SDCARD_SS_PIN, SPI_BaudRatePrescaler_128);

IO_Pin_STM32 LIS3DSH_CSN(IO_Pin::IO_Pin_Mode_IN, GPIOE, GPIO_Pin_3, GPIO_PuPd_NOPULL, GPIO_OType_OD);//não foi declarado na pinagem da placa mãe nova

IO_Pin_STM32 I2C_A_SDA_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOB, GPIO_Pin_9, GPIO_PuPd_NOPULL, GPIO_OType_OD, GPIO_AF_I2C1);//ok
IO_Pin_STM32 I2C_A_SCL_PIN(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOB, GPIO_Pin_8, GPIO_PuPd_UP, GPIO_OType_OD, GPIO_AF_I2C1);//ok
I2C_STM32 i2c_a(I2C_A_SDA_PIN, I2C_A_SCL_PIN, I2C1, 100000, 0x4000);

//INTERRUPT_STM32 nrf24_irqn_exti_interrupt(NRF24_IRQN_PIN.GetIRQChannel(), 0x0C, 0x0C, DISABLE);
Timer_Time2 robo_timer;

//MOTOR 0:
Pwm ahpwm0(GPIOC, GPIO_Pin_9, TIM8, GPIO_PinSource9, GPIO_AF_TIM8, 4, false); //M0_MAH
GPIO algpio0(GPIOD, GPIO_Pin_7);											  //M0_MAL (CHECAR PINO)
Pwm bhpwm0(GPIOC, GPIO_Pin_7, TIM8, GPIO_PinSource7, GPIO_AF_TIM8, 2, false); //M0_MBH
GPIO blgpio0(GPIOC, GPIO_Pin_13);											  //M0_MBL
Encoder encoder0(GPIOB, GPIOB, GPIO_Pin_4, GPIO_Pin_5, TIM3, GPIO_PinSource4, GPIO_PinSource5, GPIO_AF_TIM3);
Motor motor0(&ahpwm0, &algpio0, &bhpwm0, &blgpio0, &encoder0, &robo_timer);

//MOTOR 1:
Pwm ahpwm1(GPIOA, GPIO_Pin_8, TIM1, GPIO_PinSource8, GPIO_AF_TIM1, 1, false); //M1_MAH
GPIO algpio1(GPIOE, GPIO_Pin_6);											  //M1_MAL
Pwm bhpwm1(GPIOC, GPIO_Pin_8, TIM8, GPIO_PinSource8, GPIO_AF_TIM8, 3, false); //M1_MBH
GPIO blgpio1(GPIOE, GPIO_Pin_4);											  //M1_MBL
Encoder encoder1(GPIOA, GPIOB, GPIO_Pin_15, GPIO_Pin_3, TIM2, GPIO_PinSource15, GPIO_PinSource3, GPIO_AF_TIM2);
Motor motor1(&ahpwm1, &algpio1, &bhpwm1, &blgpio1, &encoder1, &robo_timer);

//MOTOR 2:
Pwm ahpwm2(GPIOC, GPIO_Pin_6, TIM8, GPIO_PinSource6, GPIO_AF_TIM8, 1, false);   //M2_MAH
GPIO algpio2(GPIOC, GPIO_Pin_2);											    //M2_MAL
Pwm bhpwm2(GPIOE, GPIO_Pin_11, TIM1, GPIO_PinSource11, GPIO_AF_TIM1, 2, false); //M2_MBH
GPIO blgpio2(GPIOB, GPIO_Pin_1);												//M2_MBL
Encoder encoder2(GPIOA, GPIOA, GPIO_Pin_0, GPIO_Pin_1, TIM5, GPIO_PinSource0, GPIO_PinSource1, GPIO_AF_TIM5);
Motor motor2(&ahpwm2, &algpio2, &bhpwm2, &blgpio2, &encoder2, &robo_timer);

//MOTOR 3:
Pwm ahpwm3(GPIOE, GPIO_Pin_14, TIM1, GPIO_PinSource14, GPIO_AF_TIM1, 4, false); //M3_MAH
GPIO algpio3(GPIOB, GPIO_Pin_12);												//M3_MAL
Pwm bhpwm3(GPIOE, GPIO_Pin_13, TIM1, GPIO_PinSource13, GPIO_AF_TIM1, 3, false); //M3_MBH
GPIO blgpio3(GPIOB, GPIO_Pin_11);												//M3_MBL
Encoder encoder3(GPIOB, GPIOB, GPIO_Pin_6, GPIO_Pin_7, TIM4, GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_TIM4);
Motor motor3(&ahpwm3, &algpio3, &bhpwm3, &blgpio3, &encoder3, &robo_timer);

//CHUTE:
GPIO high_kick_gpio(GPIOB, GPIO_Pin_0); //CHUTE ALTO->PB0
GPIO low_kick_gpio(GPIOD, GPIO_Pin_10); //CHUTE_BAIXO->PD10

Kick high_kick_pin(&high_kick_gpio, 100);
Kick low_kick_pin(&low_kick_gpio, 100);

adc sensorAdc; //adc ainda não implementado.

Drible drible;

uint8_t ID = 0;
//uint8_t ID = ID_Button.Read();
Robo robo(&motor0, &motor1, &motor2, &motor3, &nrf24, ID, &sensorAdc, &drible, &low_kick_pin, &high_kick_pin, false);

INTERRUPT_STM32 timer_robot(TIM6_DAC_IRQn, 0x0C, 0x0C, ENABLE);
CircularBuffer<uint8_t> _usbserialbuffer(0,2048);
CircularBuffer<uint8_t> _usbserialbuffer2(0,2048);
Timer_Time robo_irq_timer;

extern "C" void EXTI9_5_IRQHandler(){
	if(EXTI_GetITStatus(EXTI_Line5)){
		EXTI_ClearITPendingBit(EXTI_Line5);
		//nrf24.InterruptCallback();
	}
}

extern "C" void TIM6_DAC_IRQHandler(){
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)){
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		robo.interrupt_control();
	}
}

INTERRUPT_STM32 usb_otg_fs_interrupt(OTG_FS_IRQn, 0x0D, 0x0D, ENABLE);

extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern "C" void OTG_FS_IRQHandler(void){
	USBD_OTG_ISR_Handler (&USB_OTG_dev);
	USBH_OTG_ISR_Handler (&USB_OTG_dev);
}
extern "C" void OTG_FS_WKUP_IRQHandler(void){
	if(USB_OTG_dev.cfg.low_power){
		*(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
		SystemInit();
		USB_OTG_UngateClock(&USB_OTG_dev);
	}
	EXTI_ClearITPendingBit(EXTI_Line18);
}

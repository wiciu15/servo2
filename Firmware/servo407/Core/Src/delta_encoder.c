/*
 * delta_encoder.c
 *
 *  Created on: 24 sty 2023
 *      Author: Wiktor
 */

#include "delta_encoder.h"
#include "cmsis_os.h"
#include "inverter.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

delta_encoder_data_t delta_encoder_data={
		.encoder_command=0x32,
		.encoder_state=encoder_eeprom_reading,
		.encoder_resolution=640000
};

void delta_encoder_init(void){
	HAL_UART_DeInit(&huart1);
	huart1.Init.BaudRate=2000000;
	HAL_UART_Init(&huart1);
}

void delta_encoder_read_position(){
	if(USART_fast_transmit_RS485(&huart1, delta_encoder_data.encoder_command)!=HAL_OK){inverter_error_trip(internal_software);}
	if(HAL_UART_Receive_DMA(&huart1, delta_encoder_data.motor_response, 5)!=HAL_OK){//start listening for response, it will be automatically copied by DMA after reception
		delta_encoder_data.communication_error_count++;
		//if(delta_encoder_data.communication_error_count>10){inverter_error_trip(encoder_error_communication);}
	}else{delta_encoder_data.communication_error_count=0;}

	uint8_t xor_cheksum=0;
		for(uint8_t i=0;i<4;i++){
			xor_cheksum^=delta_encoder_data.motor_response[i];
		}
		if(xor_cheksum!=delta_encoder_data.motor_response[4]){
			delta_encoder_data.checksum_error_count++;
			//if(delta_encoder_data.checksum_error_count>100){inverter_error_trip(encoder_error_communication);}
		}
		delta_encoder_data.encoder_position=delta_encoder_data.motor_response[1]>>3 | delta_encoder_data.motor_response[2]<<5 | delta_encoder_data.motor_response[3]<<13;
		delta_encoder_data.last_encoder_hall_sensors=delta_encoder_data.encoder_hall_sensors;
		delta_encoder_data.encoder_hall_sensors=delta_encoder_data.motor_response[0]>>1 & 0b00000111; //bits 3-1 represents motor hall sensors WVU
		delta_encoder_data.encoder_hall_sensors_decimal=(delta_encoder_data.encoder_hall_sensors&0x01)+((delta_encoder_data.encoder_hall_sensors>>1 & 0x01)*10)+(delta_encoder_data.encoder_hall_sensors>>2 )*100;
		if((((delta_encoder_data.encoder_hall_sensors&0x01)^(delta_encoder_data.last_encoder_hall_sensors&0x01))!=0)&&((delta_encoder_data.encoder_hall_sensors&0b00000110)==0b00000100)){ //if hall U changed while W active and V inactive set electric angle to 0
			delta_encoder_data.position_offset=delta_encoder_data.encoder_position;
		}
		delta_encoder_data.encoder_position_indexed=delta_encoder_data.encoder_position-delta_encoder_data.position_offset;
}

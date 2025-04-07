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
#include <math.h>

extern UART_HandleTypeDef huart1;

delta_encoder_data_t delta_encoder_data={
		.encoder_command=0x32,
		.encoder_state=encoder_eeprom_reading,
		.encoder_resolution=640000
};

void delta_encoder_init(void){
	HAL_UART_DeInit(&huart1);
	set_encoder_frequency(8000);
	huart1.Init.BaudRate=2000000;
	HAL_UART_Init(&huart1);
}

void delta_encoder_read_position(){
	uint8_t xor_cheksum=0;
	if(USART_fast_transmit_RS485(&huart1, delta_encoder_data.encoder_command)!=HAL_OK){inverter_error_trip(internal_software);}
	if(HAL_UART_Receive_DMA(&huart1, delta_encoder_data.motor_response, 5)!=HAL_OK){//start listening for response, it will be automatically copied by DMA after reception
		delta_encoder_data.communication_error_count++;
		xor_cheksum=1; //corrupt the checksum on purpose when problem with tx/rx
		delta_encoder_data.rxOkCount=0;
		if(delta_encoder_data.communication_error_count>10){inverter_error_trip(encoder_error_communication);}
	}else{delta_encoder_data.communication_error_count=0;}


	for(uint8_t i=0;i<5;i++){
		xor_cheksum^=delta_encoder_data.motor_response[i];
	}
	if(xor_cheksum!=0x32){
		delta_encoder_data.checksum_error_count++;
		delta_encoder_data.rxOkCount=0;
		if(delta_encoder_data.checksum_error_count>10){inverter_error_trip(encoder_error_communication);}
		if(delta_encoder_data.last_encoder_position+=delta_encoder_data.speed);
		if(delta_encoder_data.last_encoder_position>delta_encoder_data.encoder_resolution){
			if(delta_encoder_data.speed>=0)delta_encoder_data.last_encoder_position-=delta_encoder_data.encoder_resolution;
			else delta_encoder_data.last_encoder_position+=delta_encoder_data.encoder_resolution;
		}
	}else{
		//@TODO: rather than indexing on U hall sensor, calculate appropriate offset on any hall change
		delta_encoder_data.encoder_position=delta_encoder_data.motor_response[1]>>3 | delta_encoder_data.motor_response[2]<<5 | delta_encoder_data.motor_response[3]<<13;

		if(delta_encoder_data.encoder_state == encoder_eeprom_reading){ //at first valid response chenge state to ok and set last position to not trigger acceleration check at startup
			delta_encoder_data.encoder_state=encoder_ok;
			delta_encoder_data.last_encoder_position = delta_encoder_data.encoder_position;
		}
		int32_t speed = (int32_t)delta_encoder_data.encoder_position - (int32_t)delta_encoder_data.last_encoder_position;

		if(speed < (-((int32_t)delta_encoder_data.encoder_resolution)/2))speed= ((int32_t)delta_encoder_data.encoder_resolution+speed); //positive speed correction on overflow
		if(speed > ((int32_t)delta_encoder_data.encoder_resolution/2))speed= -((int32_t)delta_encoder_data.encoder_resolution-speed);//negative speed correction on overflow
		delta_encoder_data.speed = speed;
		delta_encoder_data.last_encoder_position=delta_encoder_data.encoder_position;

		int32_t accel = delta_encoder_data.speed - delta_encoder_data.last_speed;
		if(accel > delta_encoder_data.max_accel)delta_encoder_data.max_accel = accel;
		else if(-accel > delta_encoder_data.max_accel)delta_encoder_data.max_accel = -accel;
		delta_encoder_data.accel = accel;
		if(delta_encoder_data.accel > 1000)delta_encoder_data.excessive_acceleration_error_count++;
		delta_encoder_data.last_speed=delta_encoder_data.speed;


		delta_encoder_data.last_encoder_hall_sensors=delta_encoder_data.encoder_hall_sensors;
		delta_encoder_data.encoder_hall_sensors=delta_encoder_data.motor_response[0]>>1 & 0b00000111; //bits 3-1 represents motor hall sensors WVU
		delta_encoder_data.encoder_hall_sensors_decimal=(delta_encoder_data.encoder_hall_sensors&0x01)+((delta_encoder_data.encoder_hall_sensors>>1 & 0x01)*10)+(delta_encoder_data.encoder_hall_sensors>>2 )*100;
		if((((delta_encoder_data.encoder_hall_sensors&0x01)^(delta_encoder_data.last_encoder_hall_sensors&0x01))!=0)&&((delta_encoder_data.encoder_hall_sensors&0b00000110)==0b00000100)&&delta_encoder_data.position_offset==0){ //if hall U changed while W active and V inactive set electric angle to 0
			//@TODO: for the first hall sensor edge not always hits 0 deg, needs more investigation
			//if(delta_encoder_data.position_offset==0)
			delta_encoder_data.position_offset=delta_encoder_data.encoder_position;
		}
		if(delta_encoder_data.position_offset!=0){
			delta_encoder_data.encoder_position_indexed=delta_encoder_data.encoder_position-delta_encoder_data.position_offset;
			if(delta_encoder_data.encoder_position_indexed>delta_encoder_data.encoder_resolution){//value underflow detection
				delta_encoder_data.encoder_position_indexed+=delta_encoder_data.encoder_resolution;
			}
			inverter.encoder_raw_position=delta_encoder_data.encoder_hall_sensors_decimal;
		}
		inverter.rotor_electric_angle=(((fmodf(delta_encoder_data.encoder_position_indexed, delta_encoder_data.encoder_resolution/(float)parameter_set.motor_pole_pairs))/(delta_encoder_data.encoder_resolution/(float)parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;
		if(inverter.rotor_electric_angle>=_2_PI){inverter.rotor_electric_angle-=_2_PI;}
		if(inverter.rotor_electric_angle<0){inverter.rotor_electric_angle+=_2_PI;}
		delta_encoder_data.rxOkCount++;
		if(delta_encoder_data.rxOkCount>1000){
			delta_encoder_data.rxOkCount=0;
			delta_encoder_data.communication_error_count=0;
			delta_encoder_data.checksum_error_count=0;
		}
	}
}

/*
 * tamagawa_encoder.c
 *
 *  Created on: 12 wrz 2022
 *      Author: Wiktor
 */
#include "tamagawa_encoder.h"
#include "inverter.h"
#include <string.h>
#include "cmsis_os.h"
#include "mitsubishi_encoder.h"
#include <math.h>


extern UART_HandleTypeDef huart1;
//uint8_t UART2_RX_raw[10];
tamagawa_encoder_data_t tamagawa_encoder_data={
		.checksum_error_count=0,
		.encoder_command=0x1A,
		//command 0x02 gives position[2,3,4] and cheksum[5], 0x1A gives two positions, 0xC2 resets one-turn data, 0x92 gives two null bytes and checksum
		//five bits of command, commands are listed in https://www.ti.com/lit/ug/tidue74d/tidue74d.pdf + 010 as sink
		.encoder_position=0,
		.encoder_state=encoder_eeprom_reading,
		.excessive_acceleration_error_count=0,
		.last_encoder_position=0,
		.motor_eeprom_request={0},
		.motor_data_response_packet={0},
		.motor_eeprom={0},
		.speed=0,
		.communication_error_count=0
};

void tamagawa_encoder_request_position(void){
	if(USART_fast_transmit_RS485(&huart1, tamagawa_encoder_data.encoder_command)!=HAL_OK){inverter_error_trip(internal_software);}
		if(HAL_UART_Receive_DMA(&huart1, tamagawa_encoder_data.motor_data_response_packet, 11)!=HAL_OK){//start listening for response, it will be automatically copied by DMA after reception
			tamagawa_encoder_data.communication_error_count++;
			if(tamagawa_encoder_data.communication_error_count>10){inverter_error_trip(encoder_error_communication);}
		}else{tamagawa_encoder_data.communication_error_count=0;}
}
void tamagawa_encoder_process_position(void){
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<10;i++){
		xor_cheksum^=tamagawa_encoder_data.motor_data_response_packet[i];
	}
	if(xor_cheksum!=tamagawa_encoder_data.motor_data_response_packet[10]){
		tamagawa_encoder_data.checksum_error_count++;
		if(tamagawa_encoder_data.checksum_error_count>100){inverter_error_trip(encoder_error_communication);}
	}
	else{ //calculate position and speed from received earlier data
		//tamagawa_encoder_data.encoder_state=encoder_ok;
		tamagawa_encoder_data.last_encoder_position=tamagawa_encoder_data.encoder_position;
		tamagawa_encoder_data.encoder_position=tamagawa_encoder_data.motor_data_response_packet[2] | tamagawa_encoder_data.motor_data_response_packet[3]<<8 | tamagawa_encoder_data.motor_data_response_packet[4]<<16;
		tamagawa_encoder_data.speed = tamagawa_encoder_data.last_encoder_position-tamagawa_encoder_data.encoder_position;
		inverter.encoder_raw_position=tamagawa_encoder_data.encoder_position>>1;
		inverter.rotor_electric_angle=(((fmodf(tamagawa_encoder_data.encoder_position, 131072.0f/(float)parameter_set.motor_pole_pairs))/(131072.0f/(float)parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;
		if(inverter.rotor_electric_angle>=_2_PI){inverter.rotor_electric_angle-=_2_PI;}
		if(inverter.rotor_electric_angle<0){inverter.rotor_electric_angle+=_2_PI;}
		//mechanical encoder defect detection not tested
		if(((tamagawa_encoder_data.speed>2000) && (tamagawa_encoder_data.speed<129000))|| ((tamagawa_encoder_data.speed<(-2000)) && (tamagawa_encoder_data.speed>(-129000)))){
			tamagawa_encoder_data.excessive_acceleration_error_count++;
			if(tamagawa_encoder_data.excessive_acceleration_error_count>2){tamagawa_encoder_data.encoder_state=encoder_error_acceleration;}
		}
	}
}

HAL_StatusTypeDef tamagawa_encoder_read_eeprom(uint8_t address, uint8_t * receivedByte){

	uint8_t received_data [4];
	HAL_StatusTypeDef status=HAL_OK;
	tamagawa_encoder_data.motor_eeprom_request [0] = 0xEA;
	tamagawa_encoder_data.motor_eeprom_request [1] = address;
	tamagawa_encoder_data.motor_eeprom_request [2] = tamagawa_encoder_data.motor_eeprom_request [0] ^tamagawa_encoder_data.motor_eeprom_request [1];
	status = HAL_UART_Receive_DMA(&huart1, received_data, 4);
	if(status!=HAL_OK){
		tamagawa_encoder_data.communication_error_count++;
		if(tamagawa_encoder_data.communication_error_count>10){tamagawa_encoder_data.encoder_state=encoder_error_no_communication;inverter_error_trip(encoder_error_communication);}
	}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 1);
	if(HAL_UART_Transmit(&huart1, tamagawa_encoder_data.motor_eeprom_request, 3, 1)){inverter_error_trip(internal_software);}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 0);
	osDelay(1);
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<3;i++){
		xor_cheksum^=received_data[i];
	}
	if(xor_cheksum!=received_data[3]){status=HAL_ERROR;tamagawa_encoder_data.checksum_error_count++;}
	*receivedByte=received_data[2];


	return(status);
}

HAL_StatusTypeDef tamagawa_encoder_read_id(void){

	uint8_t received_data [4];
	HAL_StatusTypeDef status=HAL_OK;
	status = HAL_UART_Receive_DMA(&huart1, received_data, 4);
	if(status!=HAL_OK){
		tamagawa_encoder_data.communication_error_count++;
		if(tamagawa_encoder_data.communication_error_count>10){tamagawa_encoder_data.encoder_state=encoder_error_no_communication;inverter_error_trip(encoder_error_communication);}
	}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 1);
	uint8_t command=0x92;
	if(HAL_UART_Transmit(&huart1, &command, 1, 1)){inverter_error_trip(internal_software);}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 0);
	osDelay(1);
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<3;i++){
		xor_cheksum^=received_data[i];
	}
	if(xor_cheksum!=received_data[3]){status=HAL_ERROR;tamagawa_encoder_data.checksum_error_count++;}else{
	tamagawa_encoder_data.encoder_id[0]=received_data[1];
	tamagawa_encoder_data.encoder_id[1]=received_data[2];}

	return(status);
}

HAL_StatusTypeDef tamagawa_encoder_write_eeprom(uint8_t address, uint8_t data){
	uint8_t data_to_send [4];
	uint8_t received_data[4];
	HAL_StatusTypeDef status = HAL_OK;
	if(address>80){status = HAL_ERROR;}else{
		data_to_send[0]=0x32;
		data_to_send[1]=address;
		data_to_send[2]=data;
		data_to_send[3]=data_to_send[0]^data_to_send[1]^data_to_send[2];
		status = HAL_UART_Receive_DMA(&huart1, received_data, 4);
		if(status!=HAL_OK){
			tamagawa_encoder_data.communication_error_count++;
			if(tamagawa_encoder_data.communication_error_count>10){tamagawa_encoder_data.encoder_state=encoder_error_no_communication;inverter_error_trip(encoder_error_communication);}
		}else{
			HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 1);
			if(HAL_UART_Transmit(&huart1, data_to_send, 4, 1)){inverter_error_trip(internal_software);}
			HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 0);
		}
		osDelay(1);
		if(received_data[1]!=address ||received_data[2]!=data || received_data[3]!=(received_data[0]^received_data[1]^received_data[2])){
			status=HAL_ERROR;
		}
	}
	return status;

}

void tamagawa_encoder_motor_identification(){
	tamagawa_encoder_read_id();
	for(uint8_t i=0;i<80;i++){
		uint8_t received_data=0;
		if(tamagawa_encoder_read_eeprom(i,&received_data)!=HAL_OK){i--;if(tamagawa_encoder_data.communication_error_count + tamagawa_encoder_data.checksum_error_count>9 ){break;}}else{
			tamagawa_encoder_data.motor_eeprom[i]=received_data;}
	}
	if(tamagawa_encoder_data.communication_error_count + tamagawa_encoder_data.checksum_error_count>9){inverter_error_trip(encoder_error_communication);}else{
	tamagawa_encoder_data.encoder_state=encoder_ok;
	//if(inverter.error==encoder_error_communication){inverter.error=no_error;} //used if hotplug is allowed
	tamagawa_encoder_data.encoder_command=0x1A;}
}


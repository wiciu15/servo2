/*
 * panasonic_encoder.c
 *
 *  Created on: 5 lip 2023
 *      Author: Wiktor
 */


#include "panasonic_encoder.h"
#include "inverter.h"
#include <string.h>
#include "cmsis_os.h"
#include "mitsubishi_encoder.h"
#include "math.h"

extern UART_HandleTypeDef huart1;
//uint8_t UART2_RX_raw[10];
panasonic_encoder_data_t panasonic_encoder_data={
		.checksum_error_count=0,
		.encoder_command=0x00,
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

void panasonic_encoder_read_position(void){
	if(USART_fast_transmit_RS485(&huart1, panasonic_encoder_data.encoder_command)!=HAL_OK){inverter_error_trip(internal_software);}
	if(HAL_UART_Receive_DMA(&huart1, panasonic_encoder_data.motor_data_response_packet, 9)!=HAL_OK){//start listening for response, it will be automatically copied by DMA after reception
		panasonic_encoder_data.communication_error_count++;
		if(panasonic_encoder_data.communication_error_count>10){memset(&panasonic_encoder_data,0,sizeof(panasonic_encoder_data_t));inverter_error_trip(encoder_error_communication);}
	}else{panasonic_encoder_data.communication_error_count=0;}

	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<8;i++){
		xor_cheksum^=panasonic_encoder_data.motor_data_response_packet[i];
	}
	if(xor_cheksum!=panasonic_encoder_data.motor_data_response_packet[8]){
		panasonic_encoder_data.checksum_error_count++;
		if(panasonic_encoder_data.checksum_error_count>100){
			memset(&panasonic_encoder_data,0,sizeof(panasonic_encoder_data_t));inverter_error_trip(encoder_error_communication);
		}
	}
	else{ //calculate position and speed from received earlier data
		panasonic_encoder_data.last_encoder_position=panasonic_encoder_data.encoder_position;
		panasonic_encoder_data.encoder_position=(panasonic_encoder_data.motor_data_response_packet[2] | panasonic_encoder_data.motor_data_response_packet[3]<<8 | panasonic_encoder_data.motor_data_response_packet[4]<<16);
		panasonic_encoder_data.speed = panasonic_encoder_data.last_encoder_position-panasonic_encoder_data.encoder_position;
		inverter.encoder_raw_position=panasonic_encoder_data.encoder_position>>4;
		inverter.rotor_electric_angle=(((fmodf(panasonic_encoder_data.encoder_position, 1048576.0f/(float)parameter_set.motor_pole_pairs))/(1048576.0f/(float)parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;
		if(inverter.rotor_electric_angle>=_2_PI){inverter.rotor_electric_angle-=_2_PI;}
		if(inverter.rotor_electric_angle<0){inverter.rotor_electric_angle+=_2_PI;}
		//mechanical encoder defect detection not tested
		if(((panasonic_encoder_data.speed>20000) && (panasonic_encoder_data.speed<800000))|| ((panasonic_encoder_data.speed<(-20000)) && (panasonic_encoder_data.speed>(-800000)))){
			panasonic_encoder_data.excessive_acceleration_error_count++;
			if(panasonic_encoder_data.excessive_acceleration_error_count>2){panasonic_encoder_data.encoder_state=encoder_error_acceleration;}
		}
	}
}

HAL_StatusTypeDef panasonic_encoder_read_eeprom(uint8_t address, uint8_t * receivedByte){

	uint8_t received_data [4];
	HAL_StatusTypeDef status=HAL_OK;
	panasonic_encoder_data.motor_eeprom_request [0] = 0xEA;
	panasonic_encoder_data.motor_eeprom_request [1] = address;
	panasonic_encoder_data.motor_eeprom_request [2] = panasonic_encoder_data.motor_eeprom_request [0] ^panasonic_encoder_data.motor_eeprom_request [1];
	status = HAL_UART_Receive_DMA(&huart1, received_data, 4);
	if(status!=HAL_OK){
		panasonic_encoder_data.communication_error_count++;
		if(panasonic_encoder_data.communication_error_count>10){panasonic_encoder_data.encoder_state=encoder_error_no_communication;memset(&panasonic_encoder_data,0,sizeof(panasonic_encoder_data_t));inverter_error_trip(encoder_error_communication);}
	}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 1);
	if(HAL_UART_Transmit(&huart1, panasonic_encoder_data.motor_eeprom_request, 3, 1)){inverter_error_trip(internal_software);}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 0);
	osDelay(1);
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<3;i++){
		xor_cheksum^=received_data[i];
	}
	if(xor_cheksum!=received_data[3]){status=HAL_ERROR;panasonic_encoder_data.checksum_error_count++;}
	*receivedByte=received_data[2];


	return(status);
}

HAL_StatusTypeDef panasonic_encoder_read_id(void){

	uint8_t received_data [9];
	HAL_StatusTypeDef status=HAL_OK;
	status = HAL_UART_Receive_DMA(&huart1, received_data, 9);
	if(status!=HAL_OK){
		panasonic_encoder_data.communication_error_count++;
		if(panasonic_encoder_data.communication_error_count>10){panasonic_encoder_data.encoder_state=encoder_error_no_communication;memset(&panasonic_encoder_data,0,sizeof(panasonic_encoder_data_t));inverter_error_trip(encoder_error_communication);}
	}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 1);
	uint8_t command=0x52;
	if(HAL_UART_Transmit(&huart1, &command, 1, 1)){inverter_error_trip(internal_software);}
	HAL_GPIO_WritePin(ENCODER_DE_GPIO_Port,ENCODER_DE_Pin, 0);
	osDelay(1);
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<8;i++){
		xor_cheksum^=received_data[i];
	}
	if(xor_cheksum!=received_data[8]){status=HAL_ERROR;panasonic_encoder_data.checksum_error_count++;}else{
		for(uint8_t i=0;i<6;i++){
			panasonic_encoder_data.encoder_id[i]=received_data[i];
		}
	}
	return(status);
}

HAL_StatusTypeDef panasonic_encoder_write_eeprom(uint8_t address, uint8_t data){
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
			panasonic_encoder_data.communication_error_count++;
			if(panasonic_encoder_data.communication_error_count>10){panasonic_encoder_data.encoder_state=encoder_error_no_communication;memset(&panasonic_encoder_data,0,sizeof(panasonic_encoder_data_t));inverter_error_trip(encoder_error_communication);}
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

void panasonic_encoder_motor_identification(){
	panasonic_encoder_read_id();
	for(uint8_t i=0;i<80;i++){
		uint8_t received_data=0;
		if(panasonic_encoder_read_eeprom(i,&received_data)!=HAL_OK){i--;if(panasonic_encoder_data.communication_error_count>9){break;}}else{
			panasonic_encoder_data.motor_eeprom[i]=received_data;}
	}
	if(panasonic_encoder_data.communication_error_count>9){inverter_error_trip(encoder_error_communication);}else{
	panasonic_encoder_data.encoder_state=encoder_ok;
	if(inverter.error==encoder_error_communication){inverter.error=no_error;}
	panasonic_encoder_data.encoder_command=0x2A;}
}


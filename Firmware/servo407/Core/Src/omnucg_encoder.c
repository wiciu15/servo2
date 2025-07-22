#include "omnucg_encoder.h"
#include "inverter.h"
#include <string.h>
#include "cmsis_os.h"
#include "mitsubishi_encoder.h"
#include "math.h"

extern UART_HandleTypeDef huart1;
//uint8_t UART2_RX_raw[10];
omnucg_encoder_data_t omnucg_encoder_data={
		.checksum_error_count=0,
		.encoder_command=0x00,
		.encoder_position=0,
		.encoder_state=encoder_eeprom_reading,
		.excessive_acceleration_error_count=0,
		.last_encoder_position=0,
		.speed=0,
		.communication_error_count=0
};

void omnucg_encoder_read_position(void){
	omnucg_encoder_data.encoder_command = 0x2A;
	if(USART_fast_transmit_RS485(&huart1, omnucg_encoder_data.encoder_command)!=HAL_OK){inverter_error_trip(internal_software);}
	if(HAL_UART_Receive_DMA(&huart1, omnucg_encoder_data.motor_data_response_packet, 9)!=HAL_OK){//start listening for response, it will be automatically copied by DMA after reception
		omnucg_encoder_data.communication_error_count++;
		if(omnucg_encoder_data.communication_error_count>10){memset(&omnucg_encoder_data,0,sizeof(omnucg_encoder_data_t));inverter_error_trip(encoder_error_communication);}
	}else{omnucg_encoder_data.communication_error_count=0;}

	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<8;i++){
		xor_cheksum^=omnucg_encoder_data.motor_data_response_packet[i];
	}
	if(xor_cheksum!=omnucg_encoder_data.motor_data_response_packet[8]){
		omnucg_encoder_data.checksum_error_count++;
		if(omnucg_encoder_data.checksum_error_count>100){
			memset(&omnucg_encoder_data,0,sizeof(omnucg_encoder_data_t));inverter_error_trip(encoder_error_communication);
		}
	}
	else{ //calculate position and speed from received earlier data
		omnucg_encoder_data.last_encoder_position=omnucg_encoder_data.encoder_position;
		omnucg_encoder_data.hall=omnucg_encoder_data.motor_data_response_packet[1];
		if(omnucg_encoder_data.hall != omnucg_encoder_data.last_hall){
			uint8_t hall = omnucg_encoder_data.hall & 0x07; //mask 3 lowest bits
			switch (hall){
			case 4: inverter.rotor_electric_angle=0.0f;break; //0deg
			case 5: inverter.rotor_electric_angle=1.047f;break; //60deg
			case 1: inverter.rotor_electric_angle=2.094f;break; //120deg
			case 3: inverter.rotor_electric_angle=3.1415f;break; //180deg
			case 2: inverter.rotor_electric_angle=4.1887f;break; //240deg
			case 6: inverter.rotor_electric_angle=5.236f;break; //300deg
			}
		}
		omnucg_encoder_data.last_hall = omnucg_encoder_data.hall;

		if(!bitcheck(omnucg_encoder_data.hall,7)){ //encoder is indexed when bit7=0 and byte 2 and 3 in response is absolute position
			omnucg_encoder_data.encoder_position=(omnucg_encoder_data.motor_data_response_packet[2] | omnucg_encoder_data.motor_data_response_packet[3]<<8);
			omnucg_encoder_data.speed = omnucg_encoder_data.last_encoder_position-omnucg_encoder_data.encoder_position;

			inverter.encoder_raw_position=(uint16_t)omnucg_encoder_data.encoder_position;
			if(parameter_set.encoder_polarity==0)inverter.rotor_electric_angle=(((fmodf(omnucg_encoder_data.encoder_position, 10000.0f/(float)parameter_set.motor_pole_pairs))/(10000.0f/(float)parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;
			else inverter.rotor_electric_angle=(((fmodf((10000.0f-omnucg_encoder_data.encoder_position), 10000.0f/(float)parameter_set.motor_pole_pairs))/(10000.0f/(float)parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;

			if(inverter.rotor_electric_angle>=_2_PI){inverter.rotor_electric_angle-=_2_PI;}
			if(inverter.rotor_electric_angle<0){inverter.rotor_electric_angle+=_2_PI;}
			//mechanical encoder defect detection not tested
			if(((omnucg_encoder_data.speed>125) && (omnucg_encoder_data.speed<9875))|| ((omnucg_encoder_data.speed<(-125)) && (omnucg_encoder_data.speed>(-9875)))){
				omnucg_encoder_data.excessive_acceleration_error_count++;
				if(omnucg_encoder_data.excessive_acceleration_error_count>3){omnucg_encoder_data.encoder_state=encoder_error_acceleration;}
			}
		}
	}
}

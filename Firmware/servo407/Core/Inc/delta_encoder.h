/*
 * delta_encoder.h
 *
 *  Created on: 24 sty 2023
 *      Author: Wiktor
 */

#ifndef INC_DELTA_ENCODER_H_
#define INC_DELTA_ENCODER_H_

#include "main.h"
#include "mitsubishi_encoder.h"

typedef struct _delta_encoder_data_t {
	encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t encoder_position_indexed;
	uint32_t position_offset;
	float motor_electric_angle;
	uint32_t last_encoder_position;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t communication_error_count;
	uint8_t motor_eeprom[20];
	uint8_t motor_response[5];
	uint8_t encoder_command;
	uint8_t encoder_hall_sensors;
	uint8_t last_encoder_hall_sensors;
	uint8_t encoder_hall_sensors_decimal;
	uint32_t encoder_resolution;
}delta_encoder_data_t;
extern delta_encoder_data_t delta_encoder_data;

HAL_StatusTypeDef USART_fast_transmit_RS485(UART_HandleTypeDef * huart, uint8_t byte_to_send);
void delta_encoder_init(void);
void delta_encoder_read_position(void);

#endif /* INC_DELTA_ENCODER_H_ */

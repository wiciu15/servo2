/*
 * panasonic_encoder.h
 *
 *  Created on: 5 lip 2023
 *      Author: Wiktor
 */

#ifndef INC_PANASONIC_ENCODER_H_
#define INC_PANASONIC_ENCODER_H_

#include "main.h"
#include "mitsubishi_encoder.h"

typedef struct _panasonic_encoder_data_t {
	encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t communication_error_count;
	uint8_t motor_data_response_packet[9];//full response to command
	uint8_t encoder_id[7];
	uint8_t motor_eeprom[80];
	uint8_t motor_eeprom_request[3];
	uint8_t encoder_command;
	int32_t speed;
}panasonic_encoder_data_t;
extern panasonic_encoder_data_t panasonic_encoder_data;

void panasonic_encoder_read_position(void);
HAL_StatusTypeDef panasonic_encoder_read_eeprom(uint8_t address,uint8_t * receivedByte);
HAL_StatusTypeDef panasonic_encoder_write_eeprom(uint8_t address, uint8_t data);
void panasonic_encoder_motor_identification();
HAL_StatusTypeDef panasonic_encoder_read_id(void);

#endif /* INC_PANASONIC_ENCODER_H_ */

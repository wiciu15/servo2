/*
 * omnucg_encoder.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Wiktor
 */

#ifndef INC_OMNUCG_ENCODER_H_
#define INC_OMNUCG_ENCODER_H_

#include "main.h"
#include "mitsubishi_encoder.h"

typedef struct _omnucg_encoder_data_t {
	encoder_state_t encoder_state;
	uint8_t hall; //2nd byte is a status word bit 7 is index status, lowest bits are hall sensors dec 4-5-1-3-2-6 when rotating CW front of motor(stator angle 0->360), at -U+VW hall = 4
	uint8_t last_hall;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t communication_error_count;
	uint8_t motor_data_response_packet[9];//full response to command
	uint8_t encoder_command;
	int32_t speed;
}omnucg_encoder_data_t;
extern omnucg_encoder_data_t omnucg_encoder_data;

void omnucg_encoder_read_position(void);

#endif /* INC_OMNUCG_ENCODER_H_ */

/*
 * abz_encoder.h
 *
 *  Created on: Jun 11, 2023
 *      Author: Wiktor
 */

#ifndef INC_ABZ_ENCODER_H_
#define INC_ABZ_ENCODER_H_

#include "main.h"

typedef enum{abz_encoder_not_marked,abz_encoder_ok,abz_encoder_acceleration_error}abz_encoder_state_t;
typedef struct _abz_encoder_data_t {
	abz_encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint8_t excessive_acceleration_error_count;
}abz_encoder_data_t;

extern abz_encoder_data_t abz_encoder_data;


void abz_encoder_update_counter_on_marker(void);
void abz_encoder_calculate_abs_position(void);

#endif /* INC_ABZ_ENCODER_H_ */

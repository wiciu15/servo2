/*
 * axis.h
 *
 *  Created on: 16 lis 2023
 *      Author: Wiktor
 */

#ifndef INC_AXIS_H_
#define INC_AXIS_H_

#include "inverter.h"

typedef struct _axis_t{
	int32_t axis_position_change_raw; //value holding change in position that has not been recalculated to user unit and updated in axis position
	float actual_position;
	int32_t target_position;
	int32_t error_position;
	int32_t next_target_position;
	float unit_factor; //multiply encoder pulses by this to get position in user unit
	PID_t position_controller_data;
}axis_t;

extern axis_t axis;

void axis_update_controller_data(void);
void update_axis_position(int32_t position_change_since_last_reading);
float axis_positioning_loop (void);

#endif /* INC_AXIS_H_ */

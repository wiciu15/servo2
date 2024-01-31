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
	int64_t actual_position_raw;
	float actual_position;
	int32_t target_position;
	int32_t last_step_pulses;
	int32_t target_position_from_tg;//target position after applying target generator ramps to it
	float error_position;
	int32_t next_target_position;
	float max_tg_increment_positive;
	float max_tg_increment_negative;
	float tg_increment;
	float tg_accel;
	float tg_braking_distance;
	float unit_factor; //multiply encoder pulses by this to get position in user unit
	PID_t position_controller_data;
}axis_t;

extern axis_t axis;

void axis_update_controller_data(void);
void update_axis_position(int32_t position_change_since_last_reading);
void update_step_input_pulses();
int32_t axis_position_trajectory_generator(int32_t actual_demand_position,int32_t target_position);
float axis_positioning_loop (void);

#endif /* INC_AXIS_H_ */

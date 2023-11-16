/*
 * axis.c
 *
 *  Created on: 16 lis 2023
 *      Author: Wiktor
 */


#include "axis.h"

axis_t axis={
		.actual_position=0.0f,
		.axis_position_change_raw=0.0f,
		.unit_factor=0.01f,
		.position_controller_data={
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				(1.0f/DEFAULT_CTRL_LOOP_FREQ)*10.0f,
				0.0f,0.0f,0.0f,0.0f
		}
};

void axis_update_controller_data(){
	if(parameter_set.position_factor_denominator==0){parameter_set.position_factor_denominator=1;}
	if(parameter_set.position_factor_numerator==0){parameter_set.position_factor_numerator=1;}
	axis.unit_factor=(float)parameter_set.position_factor_numerator/(float)parameter_set.position_factor_denominator;

	axis.position_controller_data.antiwindup_limit=parameter_set.speed_limit_positive;
	axis.position_controller_data.output_limit=parameter_set.speed_limit_positive;
	axis.position_controller_data.proportional_gain=parameter_set.position_controller_proportional_gain/axis.unit_factor;
	axis.position_controller_data.integral_gain=parameter_set.position_controller_integral_gain/axis.unit_factor;
}
/**
 * @brief  This function updates axis position
 * @param encoder pulses by which position changed since last update
 */
void update_axis_position(int32_t position_change_since_last_reading){
	int32_t increment = position_change_since_last_reading;
	//encoder overflow
	if(position_change_since_last_reading>32000){
		increment-=65535;
	}
	if(position_change_since_last_reading<-32000){
		increment+=65535;
	}
	axis.actual_position -=(float)increment*axis.unit_factor;
}
/**
 * @brief  PI positioning loop
 * @retval speed demand in RPM
 */
float axis_positioning_loop (void){
	float speed_demand = PI_control(&axis.position_controller_data,(float)(axis.target_position-axis.actual_position));
	return speed_demand;
}

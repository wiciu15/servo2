/*
 * pid.c
 *
 *  Created on: 5 cze 2023
 *      Author: Wiktor
 */

#include "pid.h"

float PI_control(PID_t  *pid_data, float error){
	float proportional = pid_data->proportional_gain * error;
	float integral = pid_data->last_integral + (pid_data->integral_gain*pid_data->sampling_time*0.5f*(error+pid_data->last_error));
	//antiwindup
	if(integral>pid_data->antiwindup_limit){integral=pid_data->antiwindup_limit;}
	if(integral<(pid_data->antiwindup_limit*-1)){integral=(pid_data->antiwindup_limit)*(-1.0f);}

	float output = proportional+integral;
	if(output>pid_data->output_limit){output=pid_data->output_limit;}
	if(output<((pid_data->output_limit)*(-1.0f))){output=(pid_data->output_limit)*(-1.0f);}

	pid_data->last_error=error;
	pid_data->last_integral=integral;
	pid_data->last_output=output;
	return output;
}

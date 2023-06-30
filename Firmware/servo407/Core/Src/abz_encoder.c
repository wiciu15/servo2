/*
 * abz_encoder.c
 *
 *  Created on: Jun 11, 2023
 *      Author: Wiktor
 */

#include "abz_encoder.h"
#include "inverter.h"

extern TIM_HandleTypeDef htim3;

abz_encoder_data_t abz_encoder_data={
		.encoder_state=abz_encoder_not_marked,
		.encoder_position=0,
		.excessive_acceleration_error_count=0,
		.last_encoder_position=0
};
/**
 * @brief  When marker(Z) signal active zero out encoder counter to create one-turn-absolute value usable for motor commutation
 * @retval null
 */
void abz_encoder_update_counter_on_marker(void){
	TIM3->CNT=parameter_set.encoder_resolution; //"align" actual postion in counter with marker
	if(abz_encoder_data.encoder_state==abz_encoder_not_marked && htim3.ChannelState[0]==HAL_TIM_CHANNEL_STATE_BUSY){abz_encoder_data.encoder_state=abz_encoder_ok;}
}

void abz_encoder_calculate_abs_position(void){
	//@TODO: add option to flip direction of encoder
	if(abz_encoder_data.encoder_state!=abz_encoder_not_marked){
		abz_encoder_data.encoder_position=TIM3->CNT-parameter_set.encoder_resolution;
		if(abz_encoder_data.encoder_position>parameter_set.encoder_resolution){abz_encoder_data.encoder_position+=parameter_set.encoder_resolution;}
		if(parameter_set.encoder_polarity){
			abz_encoder_data.encoder_position=parameter_set.encoder_resolution-abz_encoder_data.encoder_position;
		}
		//to get electric angle in radians calculate fraction of 1 electric revolution, then multiply by 2PI and apply correction from parameter set
		inverter.rotor_electric_angle=(((float)(abz_encoder_data.encoder_position % (parameter_set.encoder_resolution/parameter_set.motor_pole_pairs))/(float)(parameter_set.encoder_resolution/parameter_set.motor_pole_pairs))*_2_PI)-parameter_set.encoder_electric_angle_correction;  //calculate rotor electric angle
		inverter.encoder_raw_position=abz_encoder_data.encoder_position;
		//@TODO:implement excessive acceleration detection to trip inverter on faulty encoder signal
	}
}

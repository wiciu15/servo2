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
	if(abz_encoder_data.encoder_state!=abz_encoder_not_marked){
		TIM3->CNT=0x7FFF; //"align" half of timers counter to marker signal
	}
	if(abz_encoder_data.encoder_state==abz_encoder_not_marked && htim3.ChannelState[0]==HAL_TIM_CHANNEL_STATE_BUSY){
		abz_encoder_data.encoder_state=abz_encoder_ok;
		TIM3->CNT=0x7FFF; //"align" half of timers counter to marker signal}
	}
}

void abz_encoder_calculate_abs_position(void){
	if(abz_encoder_data.encoder_state!=abz_encoder_not_marked){
		int32_t difference=TIM3->CNT-0x7FFF;
		if(difference<0){difference=parameter_set.encoder_resolution-(-difference);}
		if(difference>parameter_set.encoder_resolution){difference=difference%parameter_set.encoder_resolution;}
		abz_encoder_data.encoder_position=difference;
		if(parameter_set.encoder_polarity){
			abz_encoder_data.encoder_position=parameter_set.encoder_resolution-abz_encoder_data.encoder_position;
		}
		//to get electric angle in radians calculate fraction of 1 electric revolution, then multiply by 2PI and apply correction from parameter set
		inverter.rotor_electric_angle=(((float)(abz_encoder_data.encoder_position % (parameter_set.encoder_resolution/parameter_set.motor_pole_pairs))/(float)(parameter_set.encoder_resolution/parameter_set.motor_pole_pairs))*_2_PI)+parameter_set.encoder_electric_angle_correction;  //calculate rotor electric angle
		if(inverter.rotor_electric_angle>=_2_PI){inverter.rotor_electric_angle-=_2_PI;}
		if(inverter.rotor_electric_angle<0){inverter.rotor_electric_angle+=_2_PI;}
		inverter.encoder_raw_position=abz_encoder_data.encoder_position;
		//@TODO:implement excessive acceleration detection to trip inverter on faulty encoder signal
	}
}

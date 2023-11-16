/*
 * parameter_set.h
 *
 *  Created on: 5 cze 2023
 *      Author: Wiktor
 */

#ifndef INC_PARAMETER_SET_H_
#define INC_PARAMETER_SET_H_

#include "main.h"
#include "inverter.h"

typedef enum {no_feedback,abz_encoder,mitsubishi_encoder,tamagawa_encoder,delta_encoder,panasonic_minas_encoder}motor_feedback_type_t;
typedef enum _operation_mode_t {
	manual=-5,
	u_f=-4,
	open_loop_current=-1,
	sensorless_speed=-3,
	sensorless_torque=-2,
	foc_torque=4,
	foc_speed=3,
	foc_position_profile=1,
	foc_position_interpolated=7,
	homing=6
}operation_mode_t;

typedef struct _parameter_set_t{
	uint32_t XOR_checksum;
	uint16_t software_version; //software version on which parameter set was created

	operation_mode_t control_mode;

	float motor_max_current; //current for motor overcurrent trip
	float motor_nominal_current; //current for torque calculation
	uint8_t motor_pole_pairs;  //needed for actual electric angle calculation in FOC, and accurate speed setpoint in manual mode
	float motor_voltage;
	float motor_max_torque;
	float motor_nominal_torque;
	float motor_nominal_speed;
	float motor_base_frequency;
	float motor_max_speed;
	float motor_rs; //resistance of single coil in Ohm
	float motor_ls; //inductance of single coil in H
	float motor_K; //@TODO: convert from v/kRPM to Vs/rad during read/write
	float motor_inertia; // inertia in kgm2

	motor_feedback_type_t motor_feedback_type;  //type of feedback
	float encoder_electric_angle_correction;  //correction of difference in electric angle calculated from encoder and electric angle of stator current. needed to achieve 90/-90 torque angle
	uint16_t encoder_resolution;
	uint8_t encoder_polarity; //flip direction of encoder

	float torque_current_ctrl_proportional_gain;
	float torque_current_ctrl_integral_gain;
	float field_current_ctrl_proportional_gain;
	float field_current_ctrl_integral_gain;

	float speed_filter_ts; //measured speed-low pass filter time constant for speed regulator
	float speed_controller_proportional_gain;
	float speed_controller_integral_gain;
	float speed_controller_output_torque_limit; //limit torque, Id is the output so the calcualtion is needed to convert N/m to A
	float speed_controller_integral_limit; //same conversion like output torque limit

	float speed_limit_positive;
	float speed_limit_negative;
	float acceleration_ramp_s; //unit: s/1000RPM
	float deceleration_ramp_s;

	uint16_t position_factor_numerator;
	uint16_t position_factor_denominator;
	float position_controller_proportional_gain;
	float position_controller_integral_gain;
}parameter_set_t;


#endif /* INC_PARAMETER_SET_H_ */

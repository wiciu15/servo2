/*
 * inverter.h
 *
 *  Created on: Mar 21, 2023
 *      Author: Wiktor
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#define INVERTER_OVERCURRENT_TRIP_LEVEL 10.0f  //overcurrrent trip setting level in Amperes
#define ADC_SAMPLES_PER_AMP 65.0f //number of ADC samples read for 1A of phase current
#define CURRENT_RMS_SAMPLING_COUNT 500 //(2*pwm frequency)/this define=rms current sampling frequency, for 500 = 32 calculations per second
#define MOTOR_CTRL_LOOP_FREQ 8000.0f

#define _2_PI 6.283185f
#define _PI 3.141592f
#define _PI_3 1.0472f
#define _SQRT3 1.73205f

#include "pid.h"
#include "parameter_set.h"
#include "mitsubishi_encoder.h"

//list of possible inverter errors that need to inhibit output and trip the inverter
typedef enum {no_error,
	undervoltage_condition,//condition that clears ittself after voltage comes back ok
	undervoltage, //trip if supply was too low when running
	overvoltage,
	shortcircuit,
	inverter_overcurrent,
	inverter_overtemperature,
	motor_overcurrent,
	encoder_error_communication,
	encoder_error_mechanical,
	overspeed,
	adc_no_communication,
	internal_software,
	external_comm
}inverter_error_t;
typedef enum {stop,run,inhibit,trip}inverter_state_t;
typedef enum {manual,u_f,open_loop_current,foc}control_mode_t;
typedef struct _output_voltage_vector_t{
	float U_Alpha;/*!< stator electric angle in radians, values over 6,28 (2*PI) are not allowed and will result in error trip */
	float U_Beta;/*!< stator voltage vector lenght in volts */
}output_voltage_vector_t;
typedef struct _HOT_ADC_t{
	uint8_t HOT_ADC_rx_buffer[2]; //buffer for response of ADC on the HOT side (measures DClink and igbt temp)
	uint8_t HOT_ADC_tx_buffer[4]; //send byte 0 and 1 to ask for dc voltage, send byte 2 and 3 to ask for igbt temp measurement
	uint32_t DCVolt_sum; //sum to calculate average
	uint32_t IGBTtemp_sum;
	uint8_t measurement_loop_iteration; //number of measurement loop iterations,divide by 2 to get number of measurements
}HOT_ADC_t;
typedef struct _RMS_current_t{
	uint16_t rms_count;
	float I_U_square_sum;
	float I_V_square_sum;
	float I_W_square_sum;
}RMS_current_t;

typedef struct _inverter_t {
uint32_t error;
inverter_state_t state;
control_mode_t control_mode;
uint16_t duty_cycle_limit;
output_voltage_vector_t output_voltage_vector;
float stator_electric_angle;
float rotor_electric_angle;
float last_rotor_electric_angle;
float rotor_speed;
float last_rotor_speed;
float filtered_rotor_speed;
float torque_angle;
float output_voltage;
float stator_field_speed;
float DCbus_voltage;
float IGBT_temp;
uint16_t zerocurrent_ADC_samples_U; //number of ADC samples when output is off and current is 0
uint16_t zerocurrent_ADC_samples_V; //number of ADC samples when output is off and current is 0
uint16_t output_current_adc_buffer [10]; //buffer for ADC samples
HOT_ADC_t HOT_ADC;
float DCbus_volts_for_sample;
float igbt_overtemperature_limit;
float undervoltage_limit;
float overvoltage_limit;

uint16_t encoder_raw_position;
uint8_t speed_measurement_loop_i;
float I_U;
float I_V;
float I_W;
float I_alpha;
float I_beta;
float I_d; //field producing current
float I_q; //torque producing current
float I_d_filtered;
float I_q_filtered;
float I_d_last; //values from previous control loop iteration
float I_q_last;
RMS_current_t RMS_current;
float I_RMS;
float U_q;
float U_d;
float U_U;
float U_V;
float U_W;
float torque_current_setpoint;
float field_current_setpoint;
float speed_setpoint;
PID_t id_current_controller_data;
PID_t iq_current_controller_data;
PID_t speed_controller_data;
}inverter_t;

extern inverter_t inverter;
extern parameter_set_t parameter_set;

void inverter_setup(void);
void inverter_enable(void);
void inverter_disable(void);
void inverter_error_trip(uint8_t error_number);
HAL_StatusTypeDef HOT_ADC_read(void);
void HOT_ADC_RX_Cplt(void);
void HOT_ADC_calculate_avg(void);
void clarke_transform(float I_U,float I_V,float * I_alpha,float * I_beta);
void park_transform(float I_alpha,float I_beta,float angle,float * I_d,float * I_q);
void inv_park_transform(float U_d,float U_q, float angle, float * U_alpha, float * U_beta);
float LowPassFilter(float Tf,float actual_measurement, float * last_filtered_value);

void output_sine_pwm(output_voltage_vector_t voltage_vector);
void output_svpwm(output_voltage_vector_t voltage_vector);
void DCBus_voltage_check(void);
void RMS_current_calculation_loop(void);
void motor_control_loop(void);
void motor_control_loop_slow(void);

#endif /* INC_INVERTER_H_ */

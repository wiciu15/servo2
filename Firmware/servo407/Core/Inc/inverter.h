/*
 * inverter.h
 *
 *  Created on: Mar 21, 2023
 *      Author: Wiktor
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#define SOFTWARE_VERSION 201

#define INVERTER_OVERCURRENT_TRIP_LEVEL 10.0f  //overcurrrent trip setting level in Amperes
#define ADC_SAMPLES_PER_AMP 60.0f //number of ADC samples read for 1A of phase current
#define CURRENT_RMS_SAMPLING_COUNT 500 //(2*pwm frequency)/this define=rms current sampling frequency, for 500 = 32 calculations per second
#define DEFAULT_CTRL_LOOP_FREQ 8000

#define _2_PI 6.283185f
#define _PI 3.141592f
#define _PI_3 1.0472f
#define _SQRT3 1.73205f
#define _SQRT2 1.414213f

#define bitset(byte,nbit)   ((byte) |=  (1UL<<(nbit)))
#define bitclear(byte,nbit) ((byte) &= ~(1UL<<(nbit)))
#define bitflip(byte,nbit)  ((byte) ^=  (1UL<<(nbit)))
#define bitcheck(byte,nbit) ((byte) &   (1<<(nbit)))


#include "pid.h"
#include "eeprom.h"
#include "mitsubishi_encoder.h"
#include "tamagawa_encoder.h"
#include "panasonic_encoder.h"
#include "abz_encoder.h"
#include "delta_encoder.h"
#include "parameter_set.h"
#include "parameter_list.h"

//list of possible inverter errors that need to inhibit output and trip the inverter

typedef enum {no_error=0x0000,
	undervoltage, //trip if supply was too low when running
	overvoltage,
	shortcircuit, //hardware overcurrent detection fault pin on igbt
	inverter_overcurrent,
	inverter_overtemperature,
	motor_overcurrent,
	encoder_error_communication,
	encoder_error_mechanical,
	overspeed,
	adc_no_communication,
	internal_software,
	external_comm,
	softstart_failure,
	eeprom_error,
	inverter_error_count
}inverter_error_t;


typedef struct _inverter_error_object_t{
	uint8_t error_number;
	uint16_t error_number_cia402;
	uint8_t error_name[14];
	uint8_t error_desc[60];
}inverter_error_object_t;
typedef struct _inverter_error_history_buffer_t{
	uint16_t error_num;
	uint32_t error_time;
}inverter_error_history_buffer_t;
typedef struct _inverter_error_history_t{
	uint8_t latest_record_index;
	inverter_error_history_buffer_t error_buffer[9];
}inverter_error_history_t;
typedef enum {not_ready_to_switch_on,switch_on_disabled,ready_to_switch_on,switched_on,operation_enabled,quickstop_active,fault_reaction,faulted}inverter_state_t;


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

typedef struct _ramp_generator_t{
	enum {linear_ramp,stype_ramp}ramp_type;
	float previous_output;
	float incr_per_second_positive;
	float incr_per_second_negative;
	float sampling_time;
	float positive_limit;
	float negative_limit;
}ramp_generator_t;

typedef struct _inverter_t {
uint8_t constant_values_update_needed;
float control_loop_freq;
inverter_error_t error;
inverter_state_t state;
operation_mode_t control_mode;
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
float output_power_active;
float output_power_apparent;
float stator_field_speed;
float DCbus_voltage;
uint8_t softstart_finished;
uint16_t RAW_DCBUS;
float IGBT_temp;
uint16_t zerocurrent_ADC_samples_U; //number of ADC samples when output is off and current is 0
uint16_t zerocurrent_ADC_samples_V; //number of ADC samples when output is off and current is 0
uint16_t output_current_adc_buffer [10]; //buffer for ADC samples
HOT_ADC_t HOT_ADC;
float DCbus_volts_for_sample;
float igbt_overtemperature_limit;
float undervoltage_limit;
float overvoltage_limit;
uint16_t chopper_duty_cycle;

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
float speed_setpoint_after_rg;
ramp_generator_t speed_ramp_generator_data;
PID_t id_current_controller_data;
PID_t iq_current_controller_data;
PID_t speed_controller_data;
}inverter_t;

extern inverter_t inverter;
extern parameter_set_t parameter_set;
const extern inverter_error_object_t inverter_error_object[];
extern inverter_error_history_t inverter_error_history;

uint16_t get_CIA402_error_number(inverter_error_t error_num);
void inverter_setup(void);
void set_ctrl_loop_frequency(uint16_t frequency);
void inverter_enable(void);
void inverter_disable(void);
void inverter_error_trip(uint8_t error_number);
HAL_StatusTypeDef inverter_error_reset(void);

uint8_t isInverter_running(void);
void update_constant_values(void);
HAL_StatusTypeDef HOT_ADC_read(void);
void HOT_ADC_RX_Cplt(void);
void HOT_ADC_calculate_avg(void);
void clarke_transform(float I_U,float I_V,float * I_alpha,float * I_beta);
void park_transform(float I_alpha,float I_beta,float angle,float * I_d,float * I_q);
void inv_park_transform(float U_d,float U_q, float angle, float * U_alpha, float * U_beta);
float LowPassFilter(float Tf,float actual_measurement, float * last_filtered_value);
float LowPassFilterA(float Tf,float Ts,float actual_measurement, float * last_filtered_value);
float ramp_generator(ramp_generator_t * ramp_generator_data,float input);

void output_sine_pwm(output_voltage_vector_t voltage_vector);
void output_svpwm(output_voltage_vector_t voltage_vector);
void DCBus_voltage_check(void);
void RMS_current_calculation_loop(void);
void motor_control_loop(void);
void motor_control_loop_slow(void);

#endif /* INC_INVERTER_H_ */

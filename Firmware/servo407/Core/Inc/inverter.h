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

#define _PI_3 1.0472f
#define _SQRT3 1.73205f

//list of possible inverter errors that need to inhibit output and trip the inverter
typedef enum {no_error,
	undervoltage_condition,
	undervoltage,
	overvoltage,
	shortcircuit,
	inverter_overcurrent,
	motor_overcurrent,
	encoder_error_communication,
	encoder_error_mechanical,
	internal_software,
	external_comm
}inverter_error_t;
typedef enum {stop,run,inhibit,trip}inverter_state_t;
typedef enum {manual,u_f,open_loop_current,foc}control_mode_t;
typedef struct _output_voltage_vector_t{
	float U_Alpha;/*!< stator electric angle in radians, values over 6,28 (2*PI) are not allowed and will result in error trip */
	float U_Beta;/*!< stator voltage vector lenght in volts */
}output_voltage_vector_t;

typedef struct _inverter_t {
inverter_error_t error;
inverter_state_t state;
control_mode_t control_mode;
uint16_t duty_cycle_limit;
output_voltage_vector_t output_voltage_vector;
float DCbus_voltage;
uint16_t zerocurrent_ADC_samples_U; //number of ADC samples when output is off and current is 0
uint16_t zerocurrent_ADC_samples_V; //number of ADC samples when output is off and current is 0
uint16_t output_current_adc_buffer [10]; //buffer for ADC samples
float I_U;
float I_V;
float I_W;
float U_U;
float U_V;
float U_W;
}inverter_t;

extern inverter_t inverter;

void inverter_setup(void);
void inverter_enable(void);
void inverter_disable(void);
void inverter_error_trip(uint8_t error);
void output_sine_pwm(output_voltage_vector_t voltage_vector);
void output_svpwm(output_voltage_vector_t voltage_vector);

#endif /* INC_INVERTER_H_ */

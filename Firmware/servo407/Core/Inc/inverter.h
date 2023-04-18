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
enum inverter_error_t {no_error,
	undervoltage_condition,//condition that clears ittself after voltage comes back ok
	undervoltage, //trip if supply was too low when running
	overvoltage,
	shortcircuit,
	inverter_overcurrent,
	inverter_overtemperature,
	motor_overcurrent,
	encoder_error_communication,
	encoder_error_mechanical,
	internal_software,
	external_comm
};
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

typedef struct _inverter_t {
uint32_t error;
inverter_state_t state;
control_mode_t control_mode;
uint16_t duty_cycle_limit;
output_voltage_vector_t output_voltage_vector;
float DCbus_voltage;
float IGBT_temp;
uint16_t zerocurrent_ADC_samples_U; //number of ADC samples when output is off and current is 0
uint16_t zerocurrent_ADC_samples_V; //number of ADC samples when output is off and current is 0
uint16_t output_current_adc_buffer [10]; //buffer for ADC samples
HOT_ADC_t HOT_ADC;
float DCbus_volts_for_sample;
float igbt_overtemperature_limit;
float undervoltage_limit;
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
void inverter_error_trip(uint8_t error_number);
void HOT_ADC_read(void);
void HOT_ADC_RX_Cplt(void);
void HOT_ADC_calculate_avg(void);
void output_sine_pwm(output_voltage_vector_t voltage_vector);
void output_svpwm(output_voltage_vector_t voltage_vector);

#endif /* INC_INVERTER_H_ */

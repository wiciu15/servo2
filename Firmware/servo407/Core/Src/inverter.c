/*
 * inverter.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Wiktor
 */

#include "main.h"
#include "inverter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"
#include "cmsis_os.h"
#include "arm_math.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi2;

extern osTimerId_t timerSoftstartHandle;

//DEFAULT PARAMETER SET FROM DRIVE ROM used to set default parameter set
parameter_set_t parameter_set={
		.software_version=SOFTWARE_VERSION,
		.control_mode=u_f,
		.motor_max_current=9.5f, //14.3 according to datasheet
		.motor_nominal_current=6.8f,
		.motor_pole_pairs=5, //4 for abb motor 5 for bch and mitsubishi hf-kn43
		.motor_max_voltage=170.0f,
		.motor_max_torque=7.17f,
		.motor_nominal_torque=2.39f,
		.motor_nominal_speed=2000.0f,
		.motor_base_frequency=200.0f,
		.motor_max_speed=3000.0f,
		.motor_rs=0.25f,
		.motor_ls=0.002f, //winding inductance in H
		.motor_K=0.18f,  //electical constant in V/(rad/s*pole_pairs) 1000RPM=104.719rad/s
		.motor_feedback_type=abz_encoder,
		.encoder_electric_angle_correction=0.0f, //-90 for abb BSM, 0 for bch, 0 for abb esm18, 60 for hf-kn43
		.encoder_resolution=4000,
		.encoder_polarity=1,


		.current_filter_ts=0.0001f,
		.torque_current_ctrl_proportional_gain=11.0f, //gain in V/A
		.torque_current_ctrl_integral_gain=1000.0f,
		.field_current_ctrl_proportional_gain=11.0f,
		.field_current_ctrl_integral_gain=1000.0f,

		.speed_filter_ts=0.002f,
		.speed_controller_proportional_gain=0.008f,
		.speed_controller_integral_gain=0.4f,
		.speed_controller_output_torque_limit=1.0f, //limit torque, Iq is the output so the calcualtion is needed to convert N/m to A
		.speed_controller_integral_limit=1.0f, //1.0 is for example, valid iq current gets copied from motor max current

		.speed_limit_positive=3000.0f,
		.speed_limit_negative=-3000.0f,
		.acceleration_ramp_s=0.1f, //unit: s/1000RPM
		.deceleration_ramp_s=0.1f

};

inverter_t inverter={
		.constant_values_update_needed=1,
		.control_loop_freq=8000,
		.error=no_error,
		.state=not_ready_to_switch_on,
		.control_mode=manual,
		.duty_cycle_limit=10498, //max value you can write to timer compare register
		.output_voltage_vector={
				.U_Alpha=0.0f,
				.U_Beta=0.0f
		},
		.stator_electric_angle=0.0f,
		.rotor_electric_angle=0.0f,
		.last_rotor_electric_angle=0.0f,
		.rotor_speed=0.0f,
		.last_rotor_speed=0.0f,
		.filtered_rotor_speed=0.0f,
		.torque_angle=0.0f,
		.output_voltage=0.0f,
		.output_power_active=0.0f,
		.output_power_apparent=0.0f,
		.stator_field_speed=0.0f,
		.DCbus_voltage=0.0f,
		.softstart_finished=0,
		.IGBT_temp=20.0f,
		.zerocurrent_ADC_samples_U=0,
		.zerocurrent_ADC_samples_V=0,
		.output_current_adc_buffer={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
		.HOT_ADC={
				.HOT_ADC_tx_buffer={0x60,0,0x70,0},
				.HOT_ADC_rx_buffer={0x00,0x00},
				.DCVolt_sum=0,
				.IGBTtemp_sum=0,
				.measurement_loop_iteration=0
		},
		.DCbus_volts_for_sample=0.421f,
		.igbt_overtemperature_limit=65.0f,
		.undervoltage_limit=230.0f,
		.overvoltage_limit=390.0f,
		.chopper_duty_cycle=0,
		.encoder_raw_position=0,
		.speed_measurement_loop_i=0,
		.I_U=0.0f,
		.I_V=0.0f,
		.I_W=0.0f,
		.I_alpha=0.0f,
		.I_beta=0.0f,
		.I_d=0.0f,
		.I_q=0.0f,
		.I_d_filtered=0.0f,
		.I_q_filtered=0.0f,
		.I_d_last=0.0f, //values from previous control loop iteration
		.I_q_last=0.0f,
		.RMS_current={
				.rms_count=0,
				.I_U_square_sum=0.0f,
				.I_V_square_sum=0.0f,
				.I_W_square_sum=0.0f
		},
		.I_RMS=0.0f,
		.U_q=0.0f,
		.U_d=0.0f,
		.U_U=0.0f,
		.U_V=0.0f,
		.U_W=0.0f,

		.torque_current_setpoint=0.0f,
		.field_current_setpoint=0.0f,
		.speed_setpoint=0.0f,
		.speed_ramp_generator_data={
				.ramp_type=linear_ramp,
				.previous_output=0.0f,
					.incr_per_second_positive=0.0f,
					.incr_per_second_negative=0.0f,
					.sampling_time=1.0f/(DEFAULT_CTRL_LOOP_FREQ/10),
					.positive_limit=990.0f,
					.negative_limit=-990.0f
		},

		.id_current_controller_data = {
				0.0f,
				0.0f,
				0.0f,0.0f,
				1.0f/DEFAULT_CTRL_LOOP_FREQ,
				0.0f,0.0f,0.0f,0.0f
		},
		.iq_current_controller_data = {
				0.0f,
				0.0f,
				0.0f,0.0f,
				1.0f/DEFAULT_CTRL_LOOP_FREQ,
				0.0f,0.0f,0.0f,0.0f
		},
		.speed_controller_data = {
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				(1.0f/DEFAULT_CTRL_LOOP_FREQ)*10.0f,
				0.0f,0.0f,0.0f,0.0f
		}
};
float U_sat=2.0f;

const inverter_error_object_t inverter_error_object[]={
		{.error_number=0,.error_number_cia402=0x0000,.error_name="No error",.error_desc="No error"},
		{.error_number=undervoltage,.error_number_cia402=0x3220,.error_name="Undervoltage",.error_desc="DC link undervoltage"},
		{.error_number=overvoltage,.error_number_cia402=0x3210,.error_name="Overvoltage",.error_desc="DC link overvoltage"},
		{.error_number=shortcircuit,.error_number_cia402=0x2320,.error_name="Short circuit",.error_desc="Output short circuit"},
		{.error_number=inverter_overcurrent,.error_number_cia402=0x2310,.error_name="Overcurrent1",.error_desc="Inverter overload"},
		{.error_number=inverter_overtemperature,.error_number_cia402=0x4310,.error_name="IGBT overtemp",.error_desc="Drive IGBT too hot"},
		{.error_number=motor_overcurrent,.error_number_cia402=0x2311,.error_name="Overcurrent2",.error_desc="Motor overload"},
		{.error_number=encoder_error_communication,.error_number_cia402=0x7380,.error_name="Encoder comm",.error_desc="Encoder no communication"},
		{.error_number=encoder_error_mechanical,.error_number_cia402=0x7381,.error_name="Encoder oper",.error_desc="Encoder position deviation too big"},
		{.error_number=overspeed,.error_number_cia402=0x7180,.error_name="Overspeed",.error_desc="Motor overspeed"},
		{.error_number=adc_no_communication,.error_number_cia402=0x7280,.error_name="ADC error",.error_desc="Power board ADC no communication"},
		{.error_number=internal_software,.error_number_cia402=0x6100,.error_name="Int software",.error_desc="Internal software error"},
		{.error_number=external_comm,.error_number_cia402=0x9000,.error_name="External error",.error_desc="Error triggered by external source"},
		{.error_number=softstart_failure,.error_number_cia402=0x5441,.error_name="Softstart fail",.error_desc="Softstart failed"},
		{.error_number=eeprom_error,.error_number_cia402=0x5530,.error_name="EEPROM error",.error_desc="Data in EEPROM incorrect"}
};

inverter_error_history_t inverter_error_history;

uint16_t get_CIA402_error_number(inverter_error_t error_num){
	uint16_t cia402_error_number=0xFFFF;
	for(uint8_t i=0;i<(sizeof(inverter_error_object)/sizeof(inverter_error_object_t));i++){
		if(inverter_error_object[i].error_number==error_num){cia402_error_number=inverter_error_object[i].error_number_cia402;break;}
	}
	return cia402_error_number;
}

/**
 * @brief  Set up inverter for operation
 * @retval null
 */
void inverter_setup(void){
	//@TODO: implement writing default parameter set to eeprom
	read_parameter_set_from_eeprom();
	osDelay(200);
	set_ctrl_loop_frequency(DEFAULT_CTRL_LOOP_FREQ);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)inverter.output_current_adc_buffer, 10);//start current reading
	if(parameter_set.motor_feedback_type!=no_feedback){ //enable encoder power supply
		HAL_GPIO_WritePin(ENC_ENABLE_GPIO_Port, ENC_ENABLE_Pin, 1);
		osDelay(500);
	}
	if(parameter_set.motor_feedback_type==panasonic_minas_encoder && panasonic_encoder_data.encoder_state==encoder_eeprom_reading){panasonic_encoder_motor_identification();}
	if(parameter_set.motor_feedback_type==tamagawa_encoder && tamagawa_encoder_data.encoder_state==encoder_eeprom_reading){tamagawa_encoder_motor_identification();	}
	if(parameter_set.motor_feedback_type == mitsubishi_encoder && mitsubishi_encoder_data.encoder_state==encoder_eeprom_reading){mitsubishi_motor_identification();}
	if(parameter_set.motor_feedback_type == abz_encoder){HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);} //enable abz encoder inputs
	if(parameter_set.motor_feedback_type == delta_encoder && delta_encoder_data.encoder_state== encoder_eeprom_reading){delta_encoder_init();}
	HAL_TIM_Base_Start_IT(&htim5); //start main motor control loop
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //start braking chopper
	inverter.state=switch_on_disabled;

}

void set_ctrl_loop_frequency(uint16_t frequency){
	inverter.control_loop_freq=frequency;
	TIM5->ARR=(84000000/(uint32_t)inverter.control_loop_freq)-1;
	inverter.id_current_controller_data.sampling_time=1.0f/inverter.control_loop_freq;
	inverter.iq_current_controller_data.sampling_time=1.0f/inverter.control_loop_freq;
	inverter.speed_controller_data.sampling_time=(1.0f/inverter.control_loop_freq)*10.0f;
	inverter.speed_ramp_generator_data.sampling_time=1.0f/(frequency/10.0f);
}
/**
  * @brief  Enable PWM output of the inverter
  * @retval null
  */
void inverter_enable(){
	if(inverter.error==no_error){
		inverter.state=operation_enabled;
		htim1.Instance->CCR1=inverter.duty_cycle_limit/2;
		htim1.Instance->CCR2=inverter.duty_cycle_limit/2;
		htim1.Instance->CCR3=inverter.duty_cycle_limit/2;
		//HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	}
}

/**
 * @brief  Disable PWM output of the inverter
 * @retval null
 */
void inverter_disable(){
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

	inverter.output_voltage_vector.U_Alpha=0.0f;
	inverter.output_voltage_vector.U_Beta=0.0f;

	inverter.id_current_controller_data.last_error=0.0f;
	inverter.id_current_controller_data.last_integral=0.0f;
	inverter.id_current_controller_data.last_output=0.0f;

	inverter.iq_current_controller_data.last_error=0.0f;
	inverter.iq_current_controller_data.last_integral=0.0f;
	inverter.iq_current_controller_data.last_output=0.0f;

	inverter.speed_controller_data.last_error=0.0f;
	inverter.speed_controller_data.last_integral=0.0f;
	inverter.speed_controller_data.last_output=0.0f;

	inverter.speed_setpoint_after_rg=0.0f;
	//inverter.speed_setpoint=0.0f;
	inverter.speed_ramp_generator_data.previous_output=0.0f;

	if(!(inverter.control_mode==foc_torque || inverter.control_mode==sensorless_torque || inverter.control_mode==open_loop_current)){//zero out torque setpoints in position/speed modes only
	inverter.torque_current_setpoint=0.0f;
	inverter.field_current_setpoint=0.0f;
	}
	//HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 0);
}

/**
  * @brief  Trips the inverter with appropriate error
  * @param  error to set, DO NOT write no_error(0) here
  * @retval null
  */
void inverter_error_trip(uint16_t error_number){
	inverter_disable();
	inverter.state=faulted;
	inverter.error=error_number;
	//log error to history
	uint8_t index = inverter_error_history.latest_record_index+1;
	if(index>8)index=0;

	inverter_error_history.error_buffer[index].error_num=get_CIA402_error_number(error_number);
	inverter_error_history.latest_record_index=index;
	//@TODO: write history buffer to eeprom in OS task, to not stay in this function for too long
}

HAL_StatusTypeDef inverter_error_reset(void){
	//non-resettable errors
	if(
			inverter.error!=internal_software &&
			inverter.error!=eeprom_error
	){inverter.error=no_error;inverter.state=switch_on_disabled;return HAL_OK;}else{return HAL_ERROR;}
}

uint8_t isInverter_running(void){
	if(htim1.ChannelState[0]==HAL_TIM_CHANNEL_STATE_BUSY){return 1;}else{return 0;}
}

void update_constant_values(void){
	inverter.control_mode=parameter_set.control_mode;

	inverter.id_current_controller_data.proportional_gain=parameter_set.field_current_ctrl_proportional_gain;
	inverter.id_current_controller_data.integral_gain=parameter_set.field_current_ctrl_integral_gain;


	inverter.iq_current_controller_data.proportional_gain=parameter_set.torque_current_ctrl_proportional_gain;
	inverter.iq_current_controller_data.integral_gain=parameter_set.torque_current_ctrl_integral_gain;


	inverter.speed_controller_data.proportional_gain=parameter_set.speed_controller_proportional_gain;
	inverter.speed_controller_data.integral_gain=parameter_set.speed_controller_integral_gain;
	inverter.speed_controller_data.antiwindup_limit=parameter_set.motor_max_current;
	inverter.speed_controller_data.output_limit=parameter_set.motor_max_current;

	inverter.speed_ramp_generator_data.positive_limit=parameter_set.speed_limit_positive;
	inverter.speed_ramp_generator_data.negative_limit=parameter_set.speed_limit_negative;
	if(parameter_set.acceleration_ramp_s!=0.0f){inverter.speed_ramp_generator_data.incr_per_second_positive=1000.0f/parameter_set.acceleration_ramp_s;}else{inverter.speed_ramp_generator_data.incr_per_second_positive=100000.0f;}
	if(parameter_set.deceleration_ramp_s!=0.0f){inverter.speed_ramp_generator_data.incr_per_second_negative=1000.0f/parameter_set.deceleration_ramp_s;}else{inverter.speed_ramp_generator_data.incr_per_second_negative=100000.0f;}

	inverter.constant_values_update_needed=0;
}

HAL_StatusTypeDef HOT_ADC_read(){
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, 0);
	HAL_StatusTypeDef  status;
	if(inverter.HOT_ADC.measurement_loop_iteration%2==0){
	status=HAL_SPI_TransmitReceive_DMA(&hspi2, inverter.HOT_ADC.HOT_ADC_tx_buffer, inverter.HOT_ADC.HOT_ADC_rx_buffer, 2);
	}else{
		status=HAL_SPI_TransmitReceive_DMA(&hspi2, inverter.HOT_ADC.HOT_ADC_tx_buffer+2, inverter.HOT_ADC.HOT_ADC_rx_buffer, 2);
	}
	return status;
}

void HOT_ADC_RX_Cplt(){
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, 1);
	uint16_t ADC_reading=inverter.HOT_ADC.HOT_ADC_rx_buffer[1]+((inverter.HOT_ADC.HOT_ADC_rx_buffer[0]&0b00000111)<<8);
	if(inverter.HOT_ADC.measurement_loop_iteration%2==0){
		inverter.HOT_ADC.DCVolt_sum+=ADC_reading;
		inverter.RAW_DCBUS=ADC_reading;
	}else{
		inverter.HOT_ADC.IGBTtemp_sum+=ADC_reading;
	}
	inverter.HOT_ADC.measurement_loop_iteration++;
	if(inverter.HOT_ADC.measurement_loop_iteration>=8){
		HOT_ADC_calculate_avg();
	}
	//this method provides too little delay between transactions and probably caused hot adc communication errors in open loop vector/foc mode
	//communication with hot adc is now done synchronously in main motor control loop 8khz
	//if(HOT_ADC_read()!=HAL_OK){
	//	inverter_error_trip(adc_no_communication);
	//}
}

void HOT_ADC_calculate_avg(){
	float avg_dcbus=(inverter.HOT_ADC.DCVolt_sum/(inverter.HOT_ADC.measurement_loop_iteration/2))*inverter.DCbus_volts_for_sample;
	//@TODO: there is an interference which causes noise on hot ADC readings each 700ms, filter is not an elegant solution, proper fix in hardware needed
	inverter.DCbus_voltage=LowPassFilterA(0.03,0.001, avg_dcbus, &inverter.DCbus_voltage);
	float U_thermistor=((inverter.HOT_ADC.IGBTtemp_sum/(inverter.HOT_ADC.measurement_loop_iteration/2))/1024.0f)*5.0f;
	float R_thermistor=22000*(1.0f/((5.0f/(U_thermistor))-1.0f));
	float temp_reading=(1.0f/((logf(R_thermistor/85000.0f)/(4092.0f))+(1.0f/298.15f)))-273.15f;
	inverter.IGBT_temp=LowPassFilterA(0.7, 0.001, temp_reading, &inverter.IGBT_temp);
	inverter.HOT_ADC.measurement_loop_iteration=0;
	inverter.HOT_ADC.DCVolt_sum=0;
	inverter.HOT_ADC.IGBTtemp_sum=0;
}

void clarke_transform(float I_U,float I_V,float * I_alpha,float * I_beta){
	* I_alpha=I_U;
	* I_beta=(0.5773502f * I_U) + (1.1547005f * I_V);
}
void park_transform(float I_alpha,float I_beta,float angle,float * I_d,float * I_q){
	float pSinVal,pCosVal;
	pSinVal = arm_sin_f32(angle);
	pCosVal = arm_cos_f32(angle);
	*I_d = (I_alpha * pCosVal) + (I_beta * pSinVal);
	*I_q = (I_alpha * pSinVal *(-1)) + (I_beta * pCosVal);
}


void inv_park_transform(float U_d,float U_q, float angle, float * U_alpha, float * U_beta){
	float pSinVal,pCosVal;
	pSinVal = arm_sin_f32(angle);
	pCosVal = arm_cos_f32(angle);
	*U_alpha= (U_d * pCosVal) - (U_q * pSinVal);
	*U_beta = (U_d * pSinVal) + (U_q * pCosVal);
}

//Tf - filter time constant in seconds
float LowPassFilter(float Tf,float actual_measurement, float * last_filtered_value){
	float alpha = Tf/(Tf + (1.0f/inverter.control_loop_freq)); //works if called synchronously with motor control loop
	float filtered_value = (alpha*(*last_filtered_value)) + ((1.0f - alpha)*actual_measurement);
	*last_filtered_value = filtered_value;
	return filtered_value;
}

//Tf - filter time constant in seconds, Ts - sampling interval
float LowPassFilterA(float Tf,float Ts,float actual_measurement, float * last_filtered_value){
	float alpha = Tf/(Tf + Ts);
	float filtered_value = (alpha*(*last_filtered_value)) + ((1.0f - alpha)*actual_measurement);
	*last_filtered_value = filtered_value;
	return filtered_value;
}

float ramp_generator(ramp_generator_t * ramp_generator_data,float input){
	float output=ramp_generator_data->previous_output;
	if(fabsf(output-input)>=0.001){ //input changed
		if(ramp_generator_data->ramp_type==linear_ramp){
			float increment=0.0f; //value to increment in current cycle of control loop
			//positive ramp
			if(input>output){
				increment=ramp_generator_data->incr_per_second_positive*ramp_generator_data->sampling_time;
				if(input-output>=increment){output=ramp_generator_data->previous_output+increment;}
				else{output=ramp_generator_data->previous_output+(input-output);}
			}
			//negative ramp
			if(input<output){
				increment=ramp_generator_data->incr_per_second_negative*ramp_generator_data->sampling_time;
				if(input-output<=increment){output=ramp_generator_data->previous_output-increment;}
				else{output=ramp_generator_data->previous_output+(input-output);}
			}
		}
	}
	else{//do nothing if output finished ramping
		output=input;
	}

	if(output>ramp_generator_data->positive_limit)output=ramp_generator_data->positive_limit;
	if(output<ramp_generator_data->negative_limit)output=ramp_generator_data->negative_limit;
	ramp_generator_data->previous_output=output;
	return output;
}
/**
 * @brief Limits
 */
float constrainf(float number, float min, float max){
	if(number<min){return min;}
	else if(number>min && number<max){return number;}
	else if(number>max){return max;}
	else{return number;}
}

/**
  * @brief  Synthesize output voltage with PWM using sinusoidal PWM algorythm
  * @param  vector of the output voltage
  * @retval null
  */
void output_sine_pwm(output_voltage_vector_t voltage_vector){
	//float cos_u = 0;
	//float cos_v = 0;
	//float cos_w = 0;
	//if(voltage_vector.angle>6.28f){ //some software error occured, trip inverter
	//	inverter_error_trip(internal_software);
	//}else{
		//prevent writing duty cycle over 100% to the timer when voltage higher than inverter DC-bus

		inverter.U_U = (voltage_vector.U_Alpha);
		inverter.U_V = ((-0.5f *voltage_vector.U_Alpha)+(0.86602*voltage_vector.U_Beta));
		inverter.U_W = ((-0.5f *voltage_vector.U_Alpha)-(0.86602*voltage_vector.U_Beta));
		//saturation voltage compensation

		/*if(inverter.U_U>0.0f){inverter.U_U+=U_sat;}
		if(inverter.U_U<0.0f){inverter.U_U-=U_sat;}
		if(inverter.U_V>0.0f){inverter.U_V+=U_sat;}
		if(inverter.U_V<0.0f){inverter.U_V-=U_sat;}
		if(inverter.U_W>0.0f){inverter.U_W+=U_sat;}
		if(inverter.U_W<0.0f){inverter.U_W-=U_sat;}
		 */
		if(inverter.I_U>=0.0f){inverter.U_U+=U_sat;}
		if(inverter.I_U<0.0f){inverter.U_U-=U_sat;}
		if(inverter.I_V>=0.0f){inverter.U_V+=U_sat;}
		if(inverter.I_V<0.0f){inverter.U_V-=U_sat;}
		if(inverter.I_W>=0.0f){inverter.U_W+=U_sat;}
		if(inverter.I_W<0.0f){inverter.U_W-=U_sat;}

		float U_U=((inverter.U_U/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;
		float U_V=((inverter.U_V/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;
		float U_W=((inverter.U_W/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;

		if(U_U>inverter.duty_cycle_limit){U_U=inverter.duty_cycle_limit;}
		if(U_V>inverter.duty_cycle_limit){U_V=inverter.duty_cycle_limit;}
		if(U_W>inverter.duty_cycle_limit){U_W=inverter.duty_cycle_limit;}

		TIM1->CCR1=(uint16_t)U_U;
		TIM1->CCR2=(uint16_t)U_V;
		TIM1->CCR3=(uint16_t)U_W;
	//}
}

/**
  * @brief  Synthesize output voltage with PWM using space vector PWM algorythm, used version from SimpleFOC project
  * @param  vector of the output voltage
  * @retval null
  */
void output_svpwm(output_voltage_vector_t voltage_vector){
	//get angle of voltage vector
	float angle_el = atan2f(-voltage_vector.U_Beta,-voltage_vector.U_Alpha)+_PI+0.00001f;
	//calculate lenght(voltage) in timer units
	float voltage_vector_len=(inverter.output_voltage/inverter.DCbus_voltage)*inverter.duty_cycle_limit/2.0f;
	//get sector of voltage
	uint8_t sector = floor(angle_el / _PI_3) + 1;
	// calculate T1 and T2 times in timer units
	float T1 = _SQRT3*arm_sin_f32(sector*_PI_3 - angle_el) * voltage_vector_len;
	float T2 = _SQRT3*arm_sin_f32(angle_el - (sector-1.0f)*_PI_3) * voltage_vector_len;
	float T0 = inverter.duty_cycle_limit - T1 - T2; // modulation_centered around driver->voltage_limit/2
	// calculate the duty cycles of each PWM driver

	switch(sector){
	case 1:
		inverter.U_U = T1 + T2 + T0/2;
		inverter.U_V = T2 + T0/2;
		inverter.U_W = T0/2;
		break;
	case 2:
		inverter.U_U = T1 +  T0/2;
		inverter.U_V = T1 + T2 + T0/2;
		inverter.U_W = T0/2;
		break;
	case 3:
		inverter.U_U = T0/2;
		inverter.U_V = T1 + T2 + T0/2;
		inverter.U_W = T2 + T0/2;
		break;
	case 4:
		inverter.U_U = T0/2;
		inverter.U_V = T1+ T0/2;
		inverter.U_W = T1 + T2 + T0/2;
		break;
	case 5:
		inverter.U_U = T2 + T0/2;
		inverter.U_V = T0/2;
		inverter.U_W = T1 + T2 + T0/2;
		break;
	case 6:
		inverter.U_U = T1 + T2 + T0/2;
		inverter.U_V = T0/2;
		inverter.U_W = T1 + T0/2;
		break;
	default:
		// possible error state
		inverter.U_U = 0;
		inverter.U_V = 0;
		inverter.U_W = 0;
	}
	//dead time and forward drop compensation
	float offset=(U_sat/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f);
	if(inverter.I_U>-0.05f){inverter.U_U+=offset/**constrainf(inverter.I_U/0.03f,0.0f,1.0f)*/;}
	if(inverter.I_U<0.05f){inverter.U_U-=offset/**constrainf(-inverter.I_U/0.03f,0.0f,1.0f)*/;}
	if(inverter.I_V>-0.05f){inverter.U_V+=offset/**constrainf(inverter.I_V/0.03f,0.0f,1.0f)*/;}
	if(inverter.I_V<0.05f){inverter.U_V-=offset/**constrainf(-inverter.I_V/0.03f,0.0f,1.0f)*/;}
	if(inverter.I_W>-0.05f){inverter.U_W+=offset/**constrainf(inverter.I_W/0.03f,0.0f,1.0f)*/;}
	if(inverter.I_W<0.05f){inverter.U_W-=offset/**constrainf(-inverter.I_W/0.03f,0.0f,1.0f)*/;}

	if(inverter.U_U>inverter.duty_cycle_limit){inverter.U_U=inverter.duty_cycle_limit;}
	if(inverter.U_V>inverter.duty_cycle_limit){inverter.U_V=inverter.duty_cycle_limit;}
	if(inverter.U_W>inverter.duty_cycle_limit){inverter.U_W=inverter.duty_cycle_limit;}
	if(inverter.U_U<0){inverter.U_U=0;}
	if(inverter.U_V<0){inverter.U_V=0;}
	if(inverter.U_W<0){inverter.U_W=0;}



	TIM1->CCR1=(uint16_t)inverter.U_U;
	TIM1->CCR2=(uint16_t)inverter.U_V;
	TIM1->CCR3=(uint16_t)inverter.U_W;
}


/**
 * @brief  This function when ran in loop will integrate 3 phase currents to get average RMS value
 * @param  null
 * @retval null
 */
void RMS_current_calculation_loop(void){
	inverter.RMS_current.rms_count++;
	//integrate phase currents
	inverter.RMS_current.I_U_square_sum+=(inverter.I_U*inverter.I_U);
	inverter.RMS_current.I_V_square_sum+=(inverter.I_V*inverter.I_V);
	inverter.RMS_current.I_W_square_sum+=(inverter.I_W*inverter.I_W);
	//calculate RMS values and average of 3 phases
	if(inverter.RMS_current.rms_count>CURRENT_RMS_SAMPLING_COUNT){
		//float I_U_RMS=sqrtf(inverter.RMS_current.I_U_square_sum/(float)inverter.RMS_current.rms_count);
		//float I_V_RMS=sqrtf(inverter.RMS_current.I_V_square_sum/(float)inverter.RMS_current.rms_count);
		////float I_W_RMS=sqrtf(inverter.RMS_current.I_W_square_sum/(float)inverter.RMS_current.rms_count);
		//inverter.I_RMS=(I_U_RMS+I_V_RMS+I_W_RMS)/3.0f;
		inverter.I_RMS=sqrtf(((inverter.RMS_current.I_U_square_sum+inverter.RMS_current.I_V_square_sum+inverter.RMS_current.I_W_square_sum)/3.0f)/(float)inverter.RMS_current.rms_count);
		inverter.RMS_current.rms_count=0;inverter.RMS_current.I_U_square_sum=0.0f;inverter.RMS_current.I_V_square_sum=0.0f;inverter.RMS_current.I_W_square_sum=0.0f;}
}

/**
  * @brief  Slow control loop running every n cycles of main motor control loop. Used for speed and position controllers and ramp generators
  * @param  null
  * @retval null
  */
void motor_control_loop_slow(void){
	//speed controller
	if((inverter.control_mode==foc_speed || inverter.control_mode==sensorless_speed) && inverter.state==operation_enabled){
		inverter.speed_setpoint_after_rg=ramp_generator(&inverter.speed_ramp_generator_data, inverter.speed_setpoint);
		inverter.torque_current_setpoint = PI_control(&inverter.speed_controller_data, inverter.speed_setpoint_after_rg-inverter.filtered_rotor_speed);
	}

}

/**
  * @brief  This function inhibits or trips the inverter in case of undervoltage, also controls softstart relay
  * @param  null
  * @retval null
  */
void DCBus_voltage_check(void){
	if((inverter.DCbus_voltage>=inverter.undervoltage_limit+10.0f)&& !osTimerIsRunning(timerSoftstartHandle)&&(inverter.state==switch_on_disabled || inverter.state==faulted)){
		//start timer to delay softstart relay and inverter readiness
		osStatus_t status = osTimerStart(timerSoftstartHandle, 1000);
		if(status!=0){inverter_error_trip(softstart_failure);}
	}
}

/**
 * @brief  Main motor control loop
 * @param  null
 * @retval null
 */
void motor_control_loop(void){
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,1);

	//check if any parameters have been
	if(inverter.constant_values_update_needed){update_constant_values();}
	inverter.id_current_controller_data.antiwindup_limit=inverter.DCbus_voltage;
	inverter.id_current_controller_data.output_limit=inverter.DCbus_voltage;
	inverter.iq_current_controller_data.antiwindup_limit=inverter.DCbus_voltage;
	inverter.iq_current_controller_data.output_limit=inverter.DCbus_voltage;

	//check if everything is fine with HOT ADC and reset it if not
	if(HOT_ADC_read()!=HAL_OK){
		inverter_error_trip(adc_no_communication);
	}
	if(hspi2.ErrorCode!=0){
		inverter_error_trip(adc_no_communication);
		HAL_SPI_DeInit(&hspi2);
		HAL_SPI_Init(&hspi2);
		HOT_ADC_read();
	}
	//check if softstart relay is energized, due to software timer failure it can be not energized while inverter is in ready state
	if(!HAL_GPIO_ReadPin(SOFTSTART_GPIO_Port, SOFTSTART_Pin) && (inverter.state==switched_on||inverter.state==operation_enabled)){
		inverter_error_trip(softstart_failure);
	}

	//check if DC bus volatge is appropriate
	if(inverter.DCbus_voltage<inverter.undervoltage_limit && isInverter_running()){inverter_error_trip(undervoltage);HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 0);inverter.softstart_finished=0;}
	if(inverter.DCbus_voltage<inverter.undervoltage_limit && (!isInverter_running())){HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 0);inverter.softstart_finished=0;if(inverter.state!=switch_on_disabled && inverter.state!=not_ready_to_switch_on && inverter.state!=faulted){inverter_error_trip(undervoltage);}}
	if(inverter.DCbus_voltage>inverter.overvoltage_limit){inverter_error_trip(overvoltage);	}

	//brake chopper control
	if(inverter.DCbus_voltage>inverter.overvoltage_limit-30.0f){
		//@TODO: increase 6000 to 10499 to dissipate full 390 volts on chopper resistor
		inverter.chopper_duty_cycle=(uint16_t)(((inverter.DCbus_voltage-(inverter.overvoltage_limit-20.0f))/20.0f)*6000);
		if(inverter.chopper_duty_cycle>6000)inverter.chopper_duty_cycle=6000;
		//@TODO: integrate duty cycle to calculate thermals of braking resistor
		TIM2->CCR1=inverter.chopper_duty_cycle;
	}else{
		TIM2->CCR1=0;
	}

	//check IGBT temperature
	if(inverter.IGBT_temp>inverter.igbt_overtemperature_limit){inverter_error_trip(inverter_overtemperature);}
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,0);
	//run RMS current calculation loop
	RMS_current_calculation_loop();
	//calculate current vector
	clarke_transform(inverter.I_U, inverter.I_V, &inverter.I_alpha, &inverter.I_beta);
	//these methods of calculating power are not very reliable or accurate
	//inverter.output_power_apparent=inverter.output_voltage*hypotf(inverter.I_alpha,inverter.I_beta);
	//inverter.output_power_active=inverter.output_voltage*inverter.I_q_filtered;
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,1);

	//calculate/get rotor electric angle from encoder
	if(parameter_set.motor_feedback_type==abz_encoder){abz_encoder_calculate_abs_position();}
	if(parameter_set.motor_feedback_type==mitsubishi_encoder){mitsubishi_encoder_process_data();}
	if(parameter_set.motor_feedback_type==tamagawa_encoder){tamagawa_encoder_process_position();}

	//calculate torque angle
	if(inverter.stator_electric_angle-inverter.rotor_electric_angle>_PI){inverter.torque_angle=(inverter.stator_electric_angle-inverter.rotor_electric_angle) - _2_PI;}
	else if(inverter.stator_electric_angle-inverter.rotor_electric_angle<(-_PI)){inverter.torque_angle=inverter.stator_electric_angle-inverter.rotor_electric_angle + _2_PI;}
	else{inverter.torque_angle=inverter.stator_electric_angle-inverter.rotor_electric_angle;}

	//calculate rotor speed
	if(inverter.speed_measurement_loop_i>=10){
		if(inverter.last_rotor_electric_angle==0.0f){inverter.last_rotor_electric_angle=inverter.rotor_electric_angle;} //this prevents overspeed detection on first slow loop execution
		float speed_calc_angle_delta=inverter.rotor_electric_angle-inverter.last_rotor_electric_angle;
		inverter.rotor_speed=((speed_calc_angle_delta)/parameter_set.motor_pole_pairs)*9.549296f*(inverter.control_loop_freq/10.0f);
		//speed(rpm) = ((x(deg)/polepairs)/360deg)/(0,002(s)/60s)
		float theoretical_encoder_speed=(_2_PI/parameter_set.motor_pole_pairs)*9.549296*(inverter.control_loop_freq/10);
		if(inverter.rotor_speed>theoretical_encoder_speed/2.0f){inverter.rotor_speed-=theoretical_encoder_speed;}if(inverter.rotor_speed<(-theoretical_encoder_speed/2.0f)){inverter.rotor_speed+=theoretical_encoder_speed;}
		inverter.last_rotor_electric_angle = inverter.rotor_electric_angle;
		motor_control_loop_slow();
		inverter.speed_measurement_loop_i=0;
	}
	inverter.speed_measurement_loop_i++;
	inverter.filtered_rotor_speed=LowPassFilter(parameter_set.speed_filter_ts,inverter.rotor_speed, &inverter.last_rotor_speed);

	//overspeed detection
	if(inverter.filtered_rotor_speed>parameter_set.motor_max_speed +400.0f || inverter.filtered_rotor_speed<(-parameter_set.motor_max_speed-400.0f)){
		if(inverter.state>0)inverter_error_trip(overspeed);
	}
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,0);
	//increment stator electric angle based on set frequency if not in foc mode
	if(inverter.control_mode==manual || inverter.control_mode==u_f || inverter.control_mode==open_loop_current ){
		if(inverter.state==operation_enabled){inverter.stator_electric_angle+=inverter.stator_field_speed;}
		if(inverter.stator_electric_angle>_2_PI){inverter.stator_electric_angle-=_2_PI;}
		if(inverter.stator_electric_angle<0.0f){inverter.stator_electric_angle+=_2_PI;}
	}


	//if in u/f mode calculate voltage for given frequency
	if(inverter.control_mode==u_f){
		inverter.output_voltage=(inverter.stator_field_speed/(parameter_set.motor_base_frequency*(_2_PI/inverter.control_loop_freq)))*parameter_set.motor_max_voltage;
	}
	//calculate field and torque currents
	if(inverter.control_mode>=1){ // foc based modes
		park_transform(inverter.I_alpha, inverter.I_beta, inverter.rotor_electric_angle, &inverter.I_d, &inverter.I_q);
	}
	else{ //custom modes
		park_transform(inverter.I_alpha, inverter.I_beta, inverter.stator_electric_angle, &inverter.I_d, &inverter.I_q);
	}
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,1);

	//low pass filter of current vector values
	inverter.I_d_filtered = LowPassFilter(parameter_set.current_filter_ts, inverter.I_d, &inverter.I_d_last);
	inverter.I_q_filtered = LowPassFilter(parameter_set.current_filter_ts, inverter.I_q, &inverter.I_q_last);


	//PI control of voltage vector in open loop
	if(inverter.control_mode==open_loop_current && isInverter_running()){
		inverter.U_q = PI_control(&inverter.iq_current_controller_data,inverter.torque_current_setpoint-inverter.I_q_filtered);
		inverter.U_d = PI_control(&inverter.id_current_controller_data,inverter.field_current_setpoint-inverter.I_d_filtered);
		inv_park_transform(inverter.U_d, inverter.U_q, inverter.stator_electric_angle, &inverter.output_voltage_vector.U_Alpha, &inverter.output_voltage_vector.U_Beta);
	}
	//PI control of voltage vector in foc modes
	if(inverter.control_mode>=1 && isInverter_running()){
		//speed limiter removes torque if motor spins too fast
		if(inverter.filtered_rotor_speed>parameter_set.motor_max_speed || inverter.filtered_rotor_speed<(-parameter_set.motor_max_speed)){
			//@TODO: implement speed limiter based on some linear decrease of torque instead of setting 0.0 setpoint - motor vibrates at speed limit
			inverter.U_q = PI_control(&inverter.iq_current_controller_data,0.0f-inverter.I_q_filtered);
			inverter.U_d = PI_control(&inverter.id_current_controller_data,inverter.field_current_setpoint-inverter.I_d_filtered);
		}else{
			inverter.U_q = PI_control(&inverter.iq_current_controller_data,inverter.torque_current_setpoint-inverter.I_q_filtered);
			inverter.U_d = PI_control(&inverter.id_current_controller_data,inverter.field_current_setpoint-inverter.I_d_filtered);
		}
		inv_park_transform(inverter.U_d, inverter.U_q,  inverter.rotor_electric_angle, &inverter.output_voltage_vector.U_Alpha, &inverter.output_voltage_vector.U_Beta);
	}

	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,0);
	//calculate voltage vectors if manual or u/f
	if(inverter.control_mode==manual || inverter.control_mode==u_f){
		inverter.output_voltage_vector.U_Alpha=arm_cos_f32(inverter.stator_electric_angle)*inverter.output_voltage;
		inverter.output_voltage_vector.U_Beta=arm_sin_f32(inverter.stator_electric_angle)*inverter.output_voltage;
	}else{
		//calculate output voltage for preview
		inverter.output_voltage=hypotf(inverter.output_voltage_vector.U_Alpha,inverter.output_voltage_vector.U_Beta);
	}
	//calculate stator field angle in foc modes to calculate torque angle correctly
	if(inverter.control_mode>=1){
		inverter.stator_electric_angle=atan2f(-inverter.output_voltage_vector.U_Beta,-inverter.output_voltage_vector.U_Alpha)+_PI+0.00000001f;
	}

	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,1);
	//synthesize PWM times from stator voltage vector
	output_svpwm(inverter.output_voltage_vector);
	//output_sine_pwm(inverter.output_voltage_vector);
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,0);

	//start data tansaction with encoder
		if(parameter_set.motor_feedback_type==tamagawa_encoder){tamagawa_encoder_request_position();}
		if(parameter_set.motor_feedback_type==panasonic_minas_encoder){panasonic_encoder_read_position();}
		if(parameter_set.motor_feedback_type==delta_encoder){delta_encoder_read_position();}

	if((parameter_set.motor_feedback_type==mitsubishi_encoder) && (mitsubishi_encoder_data.encoder_state!=encoder_error_no_communication || mitsubishi_encoder_data.encoder_state!=encoder_error_cheksum )){
		mitsubishi_encoder_send_command();
	}

	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,1);
	HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin,0);
}


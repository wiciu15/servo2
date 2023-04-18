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

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi2;

inverter_t inverter={
		.error=no_error,
		.state=stop,
		.control_mode=manual,
		.duty_cycle_limit=10498, //max value you can write to timer compare register
		.output_voltage_vector={
				.U_Alpha=0.0f,
				.U_Beta=0.0f
		},
		.DCbus_voltage=66.6f,
		.IGBT_temp=0.0f,
		.zerocurrent_ADC_samples_U=0,
		.zerocurrent_ADC_samples_V=0,
		.output_current_adc_buffer={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
		.HOT_ADC={
				.HOT_ADC_tx_buffer={0x60,0,0x70,0},
				.DCVolt_sum=0,
				.IGBTtemp_sum=0,
				.measurement_loop_iteration=0
		},
		.DCbus_volts_for_sample=0.421f,
		.igbt_overtemperature_limit=65.0f,
		.undervoltage_limit=10,
		.I_U=10.0f,
		.I_V=0.0f,
		.I_W=0.0f,
		.U_U=0.0f,
		.U_V=0.0f,
		.U_W=0.0f,
};
float U_sat=2.0f;

/**
  * @brief  Set up inverter for operation
  * @retval null
  */
void inverter_setup(void){
	HAL_ADC_Start_DMA(&hadc2, inverter.output_current_adc_buffer, 10);//start current reading
	HOT_ADC_read(); //start igbt temp and dc link reading
}
/**
  * @brief  Enable PWM output of the inverter
  * @retval null
  */
void inverter_enable(){
	if(inverter.error==no_error){
		inverter.state=run;
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
	if(inverter.state==run){inverter.state=stop;}
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	//HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 0);
}

/**
  * @brief  Trips the inverter with appropriate error
  * @param  error to set, DO NOT write no_error(0) here
  * @retval null
  */
void inverter_error_trip(uint8_t error_number){
	inverter_disable();
	if(error_number==undervoltage_condition && inverter.error==no_error){inverter.state=inhibit;}else{inverter.state=trip;}
	inverter.error|=1<<(error_number-1);
}

void HOT_ADC_read(){
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, 0);
	if(inverter.HOT_ADC.measurement_loop_iteration%2==0){
	HAL_SPI_TransmitReceive_DMA(&hspi2, inverter.HOT_ADC.HOT_ADC_tx_buffer, inverter.HOT_ADC.HOT_ADC_rx_buffer, 2);
	}else{
		HAL_SPI_TransmitReceive_DMA(&hspi2, inverter.HOT_ADC.HOT_ADC_tx_buffer+2, inverter.HOT_ADC.HOT_ADC_rx_buffer, 2);
	}
}

void HOT_ADC_RX_Cplt(){
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, 1);
	uint16_t ADC_reading=inverter.HOT_ADC.HOT_ADC_rx_buffer[1]+((inverter.HOT_ADC.HOT_ADC_rx_buffer[0]&0b00000111)<<8);
	if(inverter.HOT_ADC.measurement_loop_iteration%2==0){
		inverter.HOT_ADC.DCVolt_sum+=ADC_reading;
	}else{
		inverter.HOT_ADC.IGBTtemp_sum+=ADC_reading;
	}
	inverter.HOT_ADC.measurement_loop_iteration++;
	if(inverter.HOT_ADC.measurement_loop_iteration>=8){
		HOT_ADC_calculate_avg();
	}else{
		HOT_ADC_read();
	}
}

void HOT_ADC_calculate_avg(){
	inverter.DCbus_voltage=(inverter.HOT_ADC.DCVolt_sum/(inverter.HOT_ADC.measurement_loop_iteration/2))*inverter.DCbus_volts_for_sample;
	float U_thermistor=((inverter.HOT_ADC.IGBTtemp_sum/(inverter.HOT_ADC.measurement_loop_iteration/2))/1024.0f)*5.0f;
	float R_thermistor=22000*(1.0f/((5.0f/(U_thermistor))-1.0f));
	inverter.IGBT_temp=(1.0f/((logf(R_thermistor/85000.0f)/(4092.0f))+(1.0f/298.15f)))-273.15f;
	if(inverter.IGBT_temp>inverter.igbt_overtemperature_limit){
		inverter_error_trip(inverter_overtemperature);
	}
	inverter.HOT_ADC.measurement_loop_iteration=0;
	inverter.HOT_ADC.DCVolt_sum=0;
	inverter.HOT_ADC.IGBTtemp_sum=0;
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
		if(inverter.I_U>=0.0f){inverter.U_U-=U_sat;}
		if(inverter.I_U<0.0f){inverter.U_U+=U_sat;}
		if(inverter.I_V>=0.0f){inverter.U_V-=U_sat;}
		if(inverter.I_V<0.0f){inverter.U_V+=U_sat;}
		if(inverter.I_W>=0.0f){inverter.U_W-=U_sat;}
		if(inverter.I_W<0.0f){inverter.U_W+=U_sat;}

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
	float angle_el = atan2f(voltage_vector.U_Beta,voltage_vector.U_Alpha)+3.1415f;
	//calculate lenght(voltage) in timer units
	float voltage_vector_len=(hypotf(voltage_vector.U_Alpha,voltage_vector.U_Beta)/inverter.DCbus_voltage)*inverter.duty_cycle_limit/2.0f;
	//get sector of voltage
	uint8_t sector = floor(angle_el / _PI_3) + 1;
	      // calculate T1 and T2 times in timer units
	      float T1 = _SQRT3*sinf(sector*_PI_3 - angle_el) * voltage_vector_len;
	      float T2 = _SQRT3*sinf(angle_el - (sector-1.0f)*_PI_3) * voltage_vector_len;
	      // idle time none or split in half to get centered PWM
	      float T0 = 0; // pulled to 0 - better for low power supply voltage
	      if (1) {
	        T0 = inverter.duty_cycle_limit - T1 - T2; // modulation_centered around driver->voltage_limit/2
	      }

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
	     /*float offset=(U_sat/inverter.DCbus_voltage)*inverter.duty_cycle_limit;
	      if(inverter.I_U>=0.0f){inverter.U_U-=offset;}
	      if(inverter.I_U<0.0f){inverter.U_U+=offset;}
	      if(inverter.I_V>=0.0f){inverter.U_V-=offset;}
	      if(inverter.I_V<0.0f){inverter.U_V+=offset;}
	      if(inverter.I_W>=0.0f){inverter.U_W-=offset;}
	      if(inverter.I_W<0.0f){inverter.U_W+=offset;}*/

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


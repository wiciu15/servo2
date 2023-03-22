/*
 * inverter.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Wiktor
 */

#include "main.h"
#include "inverter.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc2;

inverter_t inverter={
		.error=no_error,
		.state=stop,
		.control_mode=manual,
		.duty_cycle_limit=5248, //max value you can write to timer compare register
		.output_voltage_vector={
				.angle=0.0f,
				.voltage=0.0f
		},
		.DCbus_voltage=30.0f,
		.output_current_adc_buffer={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
		.I_U=0.0f,
		.I_V=0.0f,
		.I_W=0.0f
};

/**
  * @brief  Set up inverter for operation
  * @retval null
  */
void inverter_setup(void){
	HAL_ADC_Start_DMA(&hadc2, inverter.output_current_adc_buffer, 16);
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
	inverter.state=stop;
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
  * @param  error to set
  * @retval null
  */
void inverter_error_trip(inverter_error_t error){
	inverter_disable();
	inverter.state=trip;
	inverter.error=error;
}

/**
  * @brief  Synthesize output voltage with PWM using sinusoidal PWM algorythm
  * @param  vector of the output voltage
  * @retval null
  */
void output_sine_pwm(output_voltage_vector_t voltage_vector){
	float cos_u = 0;
	float cos_v = 0;
	float cos_w = 0;
	if(voltage_vector.angle>6.28f){ //some software error occured, trip inverter
		inverter_error_trip(internal_software);
	}else{
		//prevent writing duty cycle over 100% to the timer when voltage higher than inverter DC-bus
		if(voltage_vector.voltage>inverter.DCbus_voltage){voltage_vector.voltage=inverter.DCbus_voltage;}
		float duty_cycle = (voltage_vector.voltage/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2);
		cos_u=cosf(voltage_vector.angle);
		cos_v=cosf(voltage_vector.angle-2.094395f);//120 deg offset
		cos_w=cosf(voltage_vector.angle+2.094395f);
		TIM1->CCR1=(inverter.duty_cycle_limit/2)+cos_u*(duty_cycle);
		TIM1->CCR2=(inverter.duty_cycle_limit/2)+cos_v*(duty_cycle);
		TIM1->CCR3=(inverter.duty_cycle_limit/2)+cos_w*(duty_cycle);
	}
}


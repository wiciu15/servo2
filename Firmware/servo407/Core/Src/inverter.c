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

inverter_t inverter={
		.error=no_error,
		.state=stop,
		.control_mode=manual,
		.duty_cycle_limit=10498, //max value you can write to timer compare register
		.output_voltage_vector={
				.U_Alpha=0.0f,
				.U_Beta=0.0f
		},
		.DCbus_voltage=20.0f,
		.zerocurrent_ADC_samples_U=0,
		.zerocurrent_ADC_samples_V=0,
		.output_current_adc_buffer={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
		.I_U=10.0f,
		.I_V=0.0f,
		.I_W=0.0f,
		.U_U=0.0f,
		.U_V=0.0f,
		.U_W=0.0f
};
float U_sat=2.1f;

/**
  * @brief  Set up inverter for operation
  * @retval null
  */
void inverter_setup(void){
	HAL_ADC_Start_DMA(&hadc2, inverter.output_current_adc_buffer, 10);
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
	//float cos_u = 0;
	//float cos_v = 0;
	//float cos_w = 0;
	//if(voltage_vector.angle>6.28f){ //some software error occured, trip inverter
	//	inverter_error_trip(internal_software);
	//}else{
		//prevent writing duty cycle over 100% to the timer when voltage higher than inverter DC-bus
		//if(voltage_vector.voltage>inverter.DCbus_voltage){voltage_vector.voltage=inverter.DCbus_voltage;}
		//float duty_cycle = (voltage_vector.voltage/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2);
		inverter.U_U = (voltage_vector.U_Alpha);
		inverter.U_V = ((-0.5f *voltage_vector.U_Alpha)+(0.86602*voltage_vector.U_Beta));
		inverter.U_W = ((-0.5f *voltage_vector.U_Alpha)-(0.86602*voltage_vector.U_Beta));
		//saturation voltage compensation

		if(inverter.U_U>0.0f){inverter.U_U+=U_sat;}
		if(inverter.U_U<0.0f){inverter.U_U-=U_sat;}
		if(inverter.U_V>0.0f){inverter.U_V+=U_sat;}
		if(inverter.U_V<0.0f){inverter.U_V-=U_sat;}
		if(inverter.U_W>0.0f){inverter.U_W+=U_sat;}
		if(inverter.U_W<0.0f){inverter.U_W-=U_sat;}
		/*if(inverter.U_U<U_sat && inverter.U_U>0.0f){inverter.U_U=U_sat;}
		if(inverter.U_U>-U_sat && inverter.U_U<0.0f){inverter.U_U=-U_sat;}
		if(inverter.U_V<U_sat && inverter.U_V>0.0f){inverter.U_V=U_sat;}
		if(inverter.U_V>-U_sat && inverter.U_V<0.0f){inverter.U_V=-U_sat;}
		if(inverter.U_W<U_sat && inverter.U_W>0.0f){inverter.U_W=U_sat;}
		if(inverter.U_W>-U_sat && inverter.U_W<0.0f){inverter.U_W=-U_sat;}*/

		float U_U=((inverter.U_U/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;
		float U_V=((inverter.U_V/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;
		float U_W=((inverter.U_W/inverter.DCbus_voltage)*(inverter.duty_cycle_limit/2.0f))+inverter.duty_cycle_limit/2.0f;

		if(U_U>inverter.duty_cycle_limit){U_U=inverter.duty_cycle_limit;}
		if(U_V>inverter.duty_cycle_limit){U_V=inverter.duty_cycle_limit;}
		if(U_W>inverter.duty_cycle_limit){U_W=inverter.duty_cycle_limit;}
		//cos_u=cosf(voltage_vector.angle);
		//cos_v=cosf(voltage_vector.angle-2.094395f);//120 deg offset
		//cos_w=cosf(voltage_vector.angle+2.094395f);
		//float U_U=(inverter.duty_cycle_limit/2)+(cos_u*duty_cycle); //centered modulation
		//float U_V=(inverter.duty_cycle_limit/2)+(cos_v*duty_cycle);
		//float U_W=(inverter.duty_cycle_limit/2)+(cos_w*duty_cycle);
		//float miniumum_duty_cycle = fminf(cos_u, fminf(cos_v,cos_w)) * duty_cycle;
		//float U_U=(cos_u*duty_cycle)-miniumum_duty_cycle;
		//float U_V=(cos_v*duty_cycle)-miniumum_duty_cycle;
		//float U_W=(cos_w*duty_cycle)-miniumum_duty_cycle;

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
	      float Ta,Tb,Tc;
	      switch(sector){
	        case 1:
	          Ta = T1 + T2 + T0/2;
	          Tb = T2 + T0/2;
	          Tc = T0/2;
	          break;
	        case 2:
	          Ta = T1 +  T0/2;
	          Tb = T1 + T2 + T0/2;
	          Tc = T0/2;
	          break;
	        case 3:
	          Ta = T0/2;
	          Tb = T1 + T2 + T0/2;
	          Tc = T2 + T0/2;
	          break;
	        case 4:
	          Ta = T0/2;
	          Tb = T1+ T0/2;
	          Tc = T1 + T2 + T0/2;
	          break;
	        case 5:
	          Ta = T2 + T0/2;
	          Tb = T0/2;
	          Tc = T1 + T2 + T0/2;
	          break;
	        case 6:
	          Ta = T1 + T2 + T0/2;
	          Tb = T0/2;
	          Tc = T1 + T0/2;
	          break;
	        default:
	         // possible error state
	          Ta = 0;
	          Tb = 0;
	          Tc = 0;
	      }
	      //write values to timer
	      TIM1->CCR1=(uint16_t)Ta;
	      TIM1->CCR2=(uint16_t)Tb;
	      TIM1->CCR3=(uint16_t)Tc;
}


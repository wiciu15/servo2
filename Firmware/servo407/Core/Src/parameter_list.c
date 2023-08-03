/*
 * parameter_list.c
 *
 *  Created on: Aug 2, 2023
 *      Author: Wiktor
 */

#include "parameter_list.h"

parameter_t parameter_list[]={
		//uint16_t numberCAN;uint16_t numberModbus;char name [12];void * ptrToActualValue;char description[40];	uint8_t WriteAllowed;uint8_t precision;	char unit [3];float defaultValue;float minValue;float maxValue;
		{.numberCAN=6040,.numberModbus=3,.name={"Status/ctrl"},.ptrToActualValue=&inverter.state,.description={"Status/control word"},.WriteAllowed=2,.precision=0,.unit={""},.defaultValue=0.0f,.minValue=0.0f,.maxValue=3.0f},
		{.numberCAN=6040,.numberModbus=6,.name={"Speed setpoint"},.ptrToActualValue=&inverter.speed_setpoint,.description={"Speed setpoint"},.WriteAllowed=2,.precision=0,.unit={"RPM"},.defaultValue=0.0f,.minValue=-6000.0f,.maxValue=6000.0f}

};

parameter_t monitor_list[]={
		{.numberCAN=6040,.numberModbus=6,.name={"Speed setpoint"},.shortName={"Set speed"},.ptrToActualValue=&inverter.speed_setpoint,.description={"Speed setpoint"},.WriteAllowed=2,.precision=0,.unit={"RPM"},.defaultValue=0.0f,.minValue=-6000.0f,.maxValue=6000.0f}
		,{.numberCAN=6040,.numberModbus=7,.name={"Torque setpoint"},.shortName={"Torque set"},.ptrToActualValue=&inverter.torque_current_setpoint,.description={"Torque setpoint"},.WriteAllowed=2,.precision=2,.unit={"A"},.defaultValue=0.0f,.minValue=-INVERTER_OVERCURRENT_TRIP_LEVEL,.maxValue=INVERTER_OVERCURRENT_TRIP_LEVEL}
		,{.numberCAN=6040,.numberModbus=8,.name={"Field setpoint"},.shortName={"Field set"},.ptrToActualValue=&inverter.field_current_setpoint,.description={"Field setpoint"},.WriteAllowed=2,.precision=2,.unit={"A"},.defaultValue=0.0f,.minValue=-INVERTER_OVERCURRENT_TRIP_LEVEL,.maxValue=INVERTER_OVERCURRENT_TRIP_LEVEL}
		,{.numberCAN=6040,.numberModbus=10,.name={"Output current"},.shortName={"I out"},.ptrToActualValue=&inverter.I_RMS,.description={"Output current"},.WriteAllowed=0,.precision=2,.unit={"A"},.defaultValue=0.0f,.minValue=-INVERTER_OVERCURRENT_TRIP_LEVEL,.maxValue=INVERTER_OVERCURRENT_TRIP_LEVEL}
		,{.numberCAN=6040,.numberModbus=11,.name={"Stator field angle"},.shortName={"Elec angle"},.ptrToActualValue=&inverter.stator_electric_angle,.description={"Stator field angle"},.WriteAllowed=0,.precision=1,.unit={"deg"},.defaultValue=0.0f,.minValue=0.0f,.maxValue=360.0f}
		,{.numberCAN=6040,.numberModbus=12,.name={"Torque angle"},.shortName={"Torq angle"},.ptrToActualValue=&inverter.torque_angle,.description={"Torque angle"},.WriteAllowed=0,.precision=1,.unit={"deg"},.defaultValue=0.0f,.minValue=-180.0f,.maxValue=180.0f}
		,{.numberCAN=6040,.numberModbus=13,.name={"Actual speed"},.shortName={"Act speed"},.ptrToActualValue=&inverter.filtered_rotor_speed,.description={"Actual speed"},.WriteAllowed=0,.precision=0,.unit={"RPM"},.defaultValue=0.0f,.minValue=-6000.0f,.maxValue=6000.0f}
		,{.numberCAN=6040,.numberModbus=14,.name={"DC bus voltage"},.shortName={"DC bus"},.ptrToActualValue=&inverter.DCbus_voltage,.description={"Voltage in intermediate circuit"},.WriteAllowed=0,.precision=0,.unit={"V"},.defaultValue=0.0f,.minValue=0.0f,.maxValue=900.0f}
		,{.numberCAN=6040,.numberModbus=17,.name={"Encoder pulses"},.shortName={"Enc pulse"},.ptrToActualValue=&inverter.encoder_raw_position,.description={"Encoder position"},.WriteAllowed=0,.precision=0,.unit={"ppr"},.defaultValue=0.0f,.minValue=0.0f,.maxValue=65535.0f}

};

uint16_t parameter_list_size=sizeof(parameter_list)/sizeof(parameter_t);
uint16_t monitor_list_size=sizeof(monitor_list)/sizeof(parameter_t);

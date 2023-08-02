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
		{.numberCAN=6040,.numberModbus=6,.name={"Speed setpoint"},.ptrToActualValue=&inverter.speed_setpoint,.description={"Speed setpoint"},.WriteAllowed=2,.precision=0,.unit={"RPM"},.defaultValue=0.0f,.minValue=-6000.0f,.maxValue=6000.0f}
		,{.numberCAN=6040,.numberModbus=7,.name={"Torque setpoint"},.ptrToActualValue=&inverter.torque_current_setpoint,.description={"Torque setpoint"},.WriteAllowed=2,.precision=2,.unit={"A"},.defaultValue=0.0f,.minValue=-INVERTER_OVERCURRENT_TRIP_LEVEL,.maxValue=INVERTER_OVERCURRENT_TRIP_LEVEL}

};

uint16_t parameter_list_size=sizeof(parameter_list)/sizeof(parameter_t);
uint16_t monitor_list_size=sizeof(monitor_list)/sizeof(parameter_t);

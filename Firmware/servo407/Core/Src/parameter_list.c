/*
 * parameter_list.c
 *
 *  Created on: Aug 2, 2023
 *      Author: Wiktor
 */

#include "parameter_list.h"
#include <string.h>

const parameter_t parameter_list[]={
		{.number=1,.ModbusAddress=1,.CANAddress=1111,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
};

const parameter_t monitor_list[]={
		{.number=1,.ModbusAddress=1,.CANAddress=1111,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},

};

uint16_t parameter_list_size=sizeof(parameter_list)/sizeof(parameter_t);
uint16_t monitor_list_size=sizeof(monitor_list)/sizeof(parameter_t);

HAL_StatusTypeDef parameter_read(parameter_t par , uint32_t * ptrToReturnValue){
	switch(par.number){
	case 1:{int16_t value =(int16_t)inverter.filtered_rotor_speed;memcpy(ptrToReturnValue,&value,2);break;}
	case 2:{float value=inverter.I_RMS;memcpy(ptrToReturnValue,&value,4);break;}
	case 3:{float value=(inverter.I_d_filtered/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 4:{float value=(inverter.I_q_filtered/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	default:{return HAL_ERROR;break;}
	}
	return HAL_OK;
}
HAL_StatusTypeDef parameter_write(parameter_t par, uint32_t * pValue){

	return HAL_OK;
}

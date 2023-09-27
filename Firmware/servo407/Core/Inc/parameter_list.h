/*
 * parameter_list.h
 *
 *  Created on: Aug 2, 2023
 *      Author: Wiktor
 */

#ifndef INC_PARAMETER_LIST_H_
#define INC_PARAMETER_LIST_H_

#include "inverter.h"

typedef struct _parameter_t{
	uint16_t number;
	uint16_t ModbusAddress;
	uint16_t CANAddress;
	char name [24];
	char shortName [8];
	char description[40];
	uint8_t WriteAllowed; //0-not allowed, 1-allowed during stop, 2-allowed
	uint8_t precision;
	char unit [3];
	enum{
		mbUINT16,
		mbINT16
	}ModbusDataType;
	enum{
		pBOOL,
		pBOOL8,
		pBOOL16,
		pHEX16,
		pHEX32,
		pUINT16,
		pINT16,
		pUINT32,
		pINT32,
		pFLOAT
	}type;
	float multiplier;
	float defaultValue;
	float minValue;
	float maxValue;
}parameter_t;

extern const parameter_t parameter_list[];
extern const parameter_t monitor_list[];
extern uint16_t parameter_list_size;
extern uint16_t monitor_list_size;

HAL_StatusTypeDef parameter_read(parameter_t par , uint32_t * ptrToReturnValue);
HAL_StatusTypeDef parameter_write(parameter_t par, uint32_t * pValue);

HAL_StatusTypeDef parameter_read_modbus(parameter_t par , void * ptrToReturnValue);
HAL_StatusTypeDef parameter_write_modbus(parameter_t par, void * pValue);

#endif /* INC_PARAMETER_LIST_H_ */

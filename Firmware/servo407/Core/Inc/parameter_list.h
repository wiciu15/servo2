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
	uint16_t numberModbus;
	uint16_t numberCAN;
	char name [18];
	void * ptrToActualValue;
	char description[40];
	uint8_t WriteAllowed;
	uint8_t precision;
	char unit [3];
	float defaultValue;
	float minValue;
	float maxValue;
}parameter_t;

extern parameter_t parameter_list[];
extern parameter_t monitor_list[];
extern uint16_t parameter_list_size;
extern uint16_t monitor_list_size;

#endif /* INC_PARAMETER_LIST_H_ */

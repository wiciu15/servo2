/*
 * comm_modbus.h
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#ifndef INC_COMM_MODBUS_H_
#define INC_COMM_MODBUS_H_

#include "modbus.h"

typedef struct _modbus_instance_t{
	uint8_t RX_FIFO [512];
	uint16_t fifo_oldpos;
	uint16_t fifo_newpos;
	uint16_t fifo_read_pos;
	uint8_t modbusReceiveBuffer[256];
	uint8_t modbusSendBuffer[256];
}modbus_instance_t;

extern modbus_instance_t modbusUSBinstance;

uint16_t modbus_protocol_read(uint32_t la);
uint16_t modbus_protocol_write(uint32_t la, uint16_t value);
int mbus_send(const mbus_t context,const uint8_t* data, const uint16_t size);
void Modbus_init(modbus_instance_t *modbus_instance);
void modbus_process_new_data_to_fifo(modbus_instance_t* modbus_instance,uint8_t *buffer,uint32_t Size);
void process_modbus_command(modbus_instance_t* modbus_instance);

#endif /* INC_COMM_MODBUS_H_ */

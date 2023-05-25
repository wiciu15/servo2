/*
 * comm_modbus.h
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#ifndef INC_COMM_MODBUS_H_
#define INC_COMM_MODBUS_H_

#include "modbus.h"

extern uint8_t UART_RX_buf[250];

uint16_t modbus_protocol_read(uint32_t la);
uint16_t modbus_protocol_write(uint32_t la, uint16_t value);
int mbus_send(const mbus_t context,const uint8_t* data, const uint16_t size);
void Modbus_init();
void modbus_process_new_data_to_fifo(uint8_t *buffer,uint32_t Size);
void process_modbus_command(void);

#endif /* INC_COMM_MODBUS_H_ */

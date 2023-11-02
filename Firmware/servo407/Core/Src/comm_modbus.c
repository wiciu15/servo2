/*
 * comm_modbus.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "comm_modbus.h"
#include "modbus.h"
#include <string.h>
#include "inverter.h"
#include "usbd_cdc_if.h"
#include "eeprom.h"
#include "cmsis_os.h"

modbus_instance_t * ptrModbusUSBinstance=NULL;
extern osThreadId_t taskModbusUSBHandle;

uint16_t modbus_protocol_read(uint32_t la){

	uint16_t local_address=la-40000;
	uint16_t response=0xFFFF;
	parameter_t * par=NULL;
	//search for chosen parameter in internal parameter list
	for(uint16_t i=0;i<parameter_list_size;i++){
		if(parameter_list[i].ModbusAddress==local_address){par=&parameter_list[i];break;}	
	}
	if(par==NULL){
		//@TODO: address does not exist in parameter list response
	}else{
		uint32_t rcv_data=0;
		parameter_read(par, &rcv_data);
		switch(par->type){
		case pFLOAT:{
			float value = 0.0f;
			memcpy(&value,&rcv_data,4);
			if(par->ModbusDataType==mbINT16){response=(int16_t)(value/par->multiplierMB);}
			if(par->ModbusDataType==mbUINT16){response=(uint16_t)(value/par->multiplierMB);}
			break;
		}
		case pBOOL16: case pUINT16:{
			uint16_t value;
			memcpy(&value,&rcv_data,2);
			response=value;
			break;
		}
		case pINT16:{
			int16_t value=0;
			memcpy(&value,&rcv_data,2);
			response=value;
			break;
		}
		default:{
			//@TODO: trip as software error
			break;
		}
		}

	}

	return (uint16_t)response;
}

uint16_t modbus_protocol_write(uint32_t la, uint16_t value)
{
	uint16_t local_address=la-40000;
	parameter_t * par=NULL;
	//search for chosen parameter in internal parameter list
	for(uint16_t i=0;i<parameter_list_size;i++){
		if(parameter_list[i].ModbusAddress==local_address){par=&parameter_list[i];break;}
	}
	if(par==NULL){
		//@TODO: address does not exist in parameter list response
	}else{
		uint32_t dataToWrite=0; //4 bytes of raw data independent of parameter type
		switch(par->type){
		case pFLOAT:{
			float valueToWrite=0.0f;
			if(par->ModbusDataType==mbINT16){int16_t signedValue = value;valueToWrite=signedValue;}
			if(par->ModbusDataType==mbUINT16){valueToWrite=value;}
			valueToWrite=valueToWrite*par->multiplierMB;
			memcpy(&dataToWrite,&valueToWrite,4);
			break;
		}
		case pBOOL16: case pUINT16:{
			dataToWrite=value;
			break;
		}
		case pINT16:{
			int16_t valueToWrite=0;
			valueToWrite=(int16_t)value;
			memcpy(&dataToWrite,&valueToWrite,2);
			break;
		}
		default:{
			//@TODO: trip as software error
			break;
		}
		}
		parameter_write(par, &dataToWrite);
	}
	return value;
}

int mbus_send(const mbus_t context,const uint8_t* data, const uint16_t size){
	UNUSED(context);
	//HAL_GPIO_WritePin(MODBUS_DE_GPIO_Port,MODBUS_DE_Pin, 1);
	//if(HAL_UART_Transmit_DMA( &huart1, (uint8_t*) data,size)==HAL_OK){
	if(CDC_Transmit_FS((uint8_t*) data, size)==HAL_OK){
		return MBUS_OK;
	}else{return MBUS_ERROR;}
}

void ModbusUSB_init(modbus_instance_t* mbus_instance){
	/* Device slave address */
	mbus_instance->mb_config.devaddr = 0x01;

	/* Just ptr on any external object, you can get it by context */
	mbus_instance->mb_config.device = (void*) 0;

	uint8_t * pmodbusSendBuffer;
	pmodbusSendBuffer=mbus_instance->modbusSendBuffer;
	mbus_instance->mb_config.sendbuf = pmodbusSendBuffer;
	mbus_instance->mb_config.sendbuf_sz = sizeof(mbus_instance->modbusSendBuffer);

	uint8_t * pmodbusRecvBuffer;
	pmodbusRecvBuffer=mbus_instance->modbusReceiveBuffer;
	mbus_instance->mb_config.recvbuf = pmodbusRecvBuffer;
	mbus_instance->mb_config.recvbuf_sz = sizeof(mbus_instance->modbusReceiveBuffer);

	/* This that function for sending some data (use sendbuf for buf) */
	mbus_instance->mb_config.send = &mbus_send;

	Modbus_Conf_t * pconf;
	pconf=&mbus_instance->mb_config;
	//User Read callback function ( read by logical address)
	pconf->read = modbus_protocol_read;

	//Write callback function
	pconf->write = modbus_protocol_write;

	//Open modbus contex
	mbus_instance->modbus = mbus_open(pconf);

	//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_RX_buf, sizeof(UART_RX_buf));
}

void modbus_process_new_data_to_fifo(modbus_instance_t* mbus_instance, uint8_t * buffer,uint32_t Size){
	if(mbus_instance!=NULL){ //break if modbus not initialized yet
		mbus_flush(mbus_instance->modbus);
		/* start the DMA again */
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) UART_RX_buf, sizeof(UART_RX_buf));
		//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

		mbus_instance->fifo_oldpos = mbus_instance->fifo_newpos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (mbus_instance->fifo_oldpos+Size > sizeof(mbus_instance->RX_FIFO)-1)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = sizeof(mbus_instance->RX_FIFO)-mbus_instance->fifo_oldpos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)mbus_instance->RX_FIFO+mbus_instance->fifo_oldpos, buffer, datatocopy);  // copy data in that remaining space

			mbus_instance->fifo_oldpos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)mbus_instance->RX_FIFO, (uint8_t*)buffer+datatocopy, (Size-datatocopy));  // copy the remaining data
			mbus_instance->fifo_newpos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)mbus_instance->RX_FIFO+mbus_instance->fifo_oldpos, buffer, Size);
			mbus_instance->fifo_newpos = Size+mbus_instance->fifo_oldpos;
		}
		vTaskNotifyGiveFromISR(taskModbusUSBHandle, NULL);
	}
}

void process_modbus_command(modbus_instance_t* mbus_instance){
	while(mbus_instance->fifo_read_pos!=mbus_instance->fifo_newpos){
		mbus_poll(mbus_instance->modbus, mbus_instance->RX_FIFO[mbus_instance->fifo_read_pos] );
		mbus_instance->fifo_read_pos++;
		if(mbus_instance->fifo_read_pos>=sizeof(mbus_instance->RX_FIFO)){
			mbus_instance->fifo_read_pos=0;
		}
	}
}

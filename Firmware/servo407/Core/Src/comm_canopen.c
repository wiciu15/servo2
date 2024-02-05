/*
 * comm_canopen.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Wiktor
 */

#include "comm_canopen.h"
#include "OD.h"


ODR_t OD_606C_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 OD_RAM.x606C_velocityActualValue=inverter.filtered_rotor_speed;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6041_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 uint16_t index=par_get_index_CAN(0x6041);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 uint32_t statusword=0;
	 parameter_read(&parameter_list[index], &statusword);
	 memcpy(&OD_RAM.x6041_statusWord,&statusword,2);
	 return OD_readOriginal(stream, buf, count, countRead);
}


OD_extension_t OD_606C_extension={.object = NULL, .read = OD_606C_read, .write = NULL};
OD_extension_t OD_6041_extension={.object = NULL, .read = OD_6041_read, .write = NULL};

ODR_t initialize_OD_extensions(){
	OD_extension_init(OD_ENTRY_H606C,&OD_606C_extension);
	OD_extension_init(OD_ENTRY_H6041_statusWord, &OD_6041_extension);
	return ODR_OK;
}

/*
 * comm_canopen.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Wiktor
 */

#include "comm_canopen.h"
#include "OD.h"


ODR_t OD_6040_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6040_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	 uint8_t index=par_get_index_CAN(0x6040);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 uint32_t data=0;
	 memcpy(&data,&OD_RAM.x6040_controlWord,2);
	 if(parameter_write(&parameter_list[index], &data)!=HAL_OK){
		 return ODR_WRITEONLY;
	 }
	 return ODR_OK;
}

ODR_t OD_6041_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 uint16_t index=par_get_index_CAN(0x6041);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 uint32_t data=0;
	 parameter_read(&parameter_list[index], &data);
	 memcpy(&OD_RAM.x6041_statusWord,&data,2);
	 return OD_readOriginal(stream, buf, count, countRead);
}
ODR_t OD_6060_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 uint8_t index=par_get_index_CAN(0x6060);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 uint32_t data=0;
	 parameter_read(&parameter_list[index], &data);
	 int16_t opmode16=0;
	 memcpy(&opmode16,&data,2);
	 int8_t opmode=(int8_t)opmode16;
	 OD_RAM.x6060_modesOfOperation=opmode;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6060_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	 uint8_t index=par_get_index_CAN(0x6060);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 int16_t opMode = (int16_t)OD_RAM.x6060_modesOfOperation;
	 uint32_t data=0;
	 memcpy(&data,&opMode,2);
	 if(parameter_write(&parameter_list[index], &data)!=HAL_OK){
		 return ODR_OK;
	 }
	 return ODR_OK;
}

ODR_t OD_6061_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 uint8_t index=par_get_index_CAN(0x6060);
	 if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	 uint32_t data=0;
	 parameter_read(&parameter_list[index], &data);
	 int16_t opmode16=0;
	 memcpy(&opmode16,&data,2);
	 int8_t opmode=(int8_t)opmode16;
	 OD_RAM.x6061_modesOfOperationDisplay=opmode;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6064_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 OD_RAM.x6064_positionActualValue=(int32_t)axis.actual_position;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_606C_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 OD_RAM.x606C_velocityActualValue=inverter.filtered_rotor_speed;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6071_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6071_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	uint8_t index=par_get_index_CAN(0x6071);
	if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	float trgtTorque = (float)OD_RAM.x6071_targetTorque/10.0f;
	uint32_t data=0;
	memcpy(&data,&trgtTorque,2);
	if(parameter_write(&parameter_list[index], &data)!=HAL_OK){
		return ODR_UNSUPP_ACCESS;
	}
	return ODR_OK;
}

ODR_t OD_6083_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	OD_RAM.x6083_profileAcceleration=1000.0f/parameter_set.acceleration_ramp_s;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6083_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	uint8_t index=par_get_index_CAN(0x6083);
	if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	float accRamp = 1000.0f/(float)OD_RAM.x6083_profileAcceleration;
	uint32_t data=0;
	memcpy(&data,&accRamp,2);
	if(parameter_write(&parameter_list[index], &data)!=HAL_OK){
		return ODR_UNSUPP_ACCESS;
	}
	return ODR_OK;
}

ODR_t OD_6084_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	OD_RAM.x6084_profileDeceleration=1000.0f/parameter_set.deceleration_ramp_s;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_6084_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	uint8_t index=par_get_index_CAN(0x6084);
	if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	float decRamp = 1000.0f/(float)OD_RAM.x6084_profileDeceleration;
	uint32_t data=0;
	memcpy(&data,&decRamp,2);
	if(parameter_write(&parameter_list[index], &data)!=HAL_OK){
		return ODR_UNSUPP_ACCESS;
	}
	return ODR_OK;
}

ODR_t OD_60FF_read(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *countRead){
	OD_RAM.x60FF_targetVelocityPv=(int32_t)inverter.speed_setpoint;
	 return OD_readOriginal(stream, buf, count, countRead);
}

ODR_t OD_60FF_write(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten){
	OD_writeOriginal(stream, buf, count, countWritten);
	uint8_t index=par_get_index_CAN(0x60FF);
	if(index==0xFFFF)return ODR_IDX_NOT_EXIST;
	float trgtVelocity = (float)OD_RAM.x60FF_targetVelocityPv;
	if(inverter.control_mode!=foc_speed){return ODR_UNSUPP_ACCESS;}
	else{
		if(trgtVelocity>parameter_list[index].maxValue)return ODR_VALUE_HIGH;
		else if(trgtVelocity<parameter_list[index].minValue)return ODR_VALUE_LOW;
		else inverter.speed_setpoint=trgtVelocity;
	}
	return ODR_OK;
}


OD_extension_t OD_6040_extension={.object = NULL, .read = OD_6040_read, .write = OD_6040_write};
OD_extension_t OD_6041_extension={.object = NULL, .read = OD_6041_read, .write = NULL};
OD_extension_t OD_6060_extension={.object = NULL, .read = OD_6060_read, .write = OD_6060_write};
OD_extension_t OD_6061_extension={.object = NULL, .read = OD_6061_read, .write = NULL};
OD_extension_t OD_6064_extension={.object = NULL, .read = OD_6064_read, .write = NULL};
OD_extension_t OD_606C_extension={.object = NULL, .read = OD_606C_read, .write = NULL};
OD_extension_t OD_6071_extension={.object = NULL, .read = OD_6071_read, .write = OD_6071_write};
OD_extension_t OD_6083_extension={.object = NULL, .read = OD_6083_read, .write = OD_6083_write};
OD_extension_t OD_6084_extension={.object = NULL, .read = OD_6084_read, .write = OD_6084_write};
OD_extension_t OD_60FF_extension={.object = NULL, .read = OD_60FF_read, .write = OD_60FF_write};


ODR_t initialize_OD_extensions(){
	OD_extension_init(OD_ENTRY_H6040_controlWord, &OD_6040_extension);
	OD_extension_init(OD_ENTRY_H6041_statusWord, &OD_6041_extension);
	OD_extension_init(OD_ENTRY_H6060_modesOfOperation, &OD_6060_extension);
	OD_extension_init(OD_ENTRY_H6061_modesOfOperationDisplay, &OD_6061_extension);
	OD_extension_init(OD_ENTRY_H6064_positionActualValue, &OD_6064_extension);
	OD_extension_init(OD_ENTRY_H606C,&OD_606C_extension);
	OD_extension_init(OD_ENTRY_H6071_targetTorque,&OD_6071_extension);
	OD_extension_init(OD_ENTRY_H6083, &OD_6083_extension);
	OD_extension_init(OD_ENTRY_H6084, &OD_6084_extension);
	OD_extension_init(OD_ENTRY_H60FF,&OD_60FF_extension);

	return ODR_OK;
}

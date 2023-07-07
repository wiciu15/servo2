#include "eeprom.h"
#include "cmsis_os.h"

eeprom_t eeprom;

void eeprom_init(I2C_HandleTypeDef* i2c_handle, uint16_t i2c_memSize) {
	eeprom.i2c_handle = i2c_handle;
	eeprom.i2c_memSize = i2c_memSize;
}

//used to write less than 16 bytes
HAL_StatusTypeDef eeprom_write(uint16_t address, uint8_t *data, uint16_t Size) {
	return HAL_I2C_Mem_Write(eeprom.i2c_handle,0xa0, address,eeprom.i2c_memSize, data, Size,10);
}

HAL_StatusTypeDef eeprom_read(uint16_t address, uint8_t *rData,	uint16_t Size) {
	return HAL_I2C_Mem_Read(eeprom.i2c_handle,0xA1, address, eeprom.i2c_memSize,rData, Size,10);
}

//used to write more than 16 bytes
HAL_StatusTypeDef eeprom_write_sequential(uint16_t page,uint8_t * data,uint16_t Size){
	HAL_StatusTypeDef status=HAL_ERROR;
	uint16_t address=page*16;
	uint16_t max_address=address+Size;
	for(uint8_t i=address;i<max_address;i+=16){
		if(i+16>max_address){
			//calculate number of bytes in last packet
			uint8_t number_of_remaining_bytes=max_address-i;
			status=eeprom_write(i, data+i-address,number_of_remaining_bytes);
		}else{
			status=eeprom_write(i, data+i-address,16);
		}
		//wait for eeprom to write before sending next packet
		osDelay(10);
	}
	return status;
}

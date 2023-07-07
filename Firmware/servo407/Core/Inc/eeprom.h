#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32f4xx_hal.h"
#include "main.h"

typedef struct {
	I2C_HandleTypeDef* i2c_handle;
	uint16_t i2c_memSize;
} eeprom_t;

extern eeprom_t eeprom;

void eeprom_init(I2C_HandleTypeDef* i2c_handle, uint16_t i2c_memSize);
HAL_StatusTypeDef eeprom_write(uint16_t address, uint8_t *data, uint16_t Size);
HAL_StatusTypeDef eeprom_read(uint16_t address, uint8_t *data,	uint16_t Size);
HAL_StatusTypeDef eeprom_write_sequential(uint16_t page,uint8_t * data,uint16_t Size);


#endif /* INC_EEPROM_H_ */

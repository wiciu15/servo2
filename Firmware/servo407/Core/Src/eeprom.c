
/**
  ******************************************************************************

  EEPROM.c Using the HAL I2C Functions
  Author:   ControllersTech
  Updated:  Feb 16, 2021

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

#include "EEPROM.h"
#include "math.h"
#include "string.h"
#include "cmsis_os.h"
#include "inverter.h"

// Define the I2C
extern I2C_HandleTypeDef hi2c1;
#define EEPROM_I2C &hi2c1

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xA0

// Define the Page Size and number of pages
#define PAGE_SIZE 64     // in Bytes
#define PAGE_NUM  256    // number of pages



/*****************************************************************************************************************************************/
uint8_t bytes_temp[4];

// function to determine the remaining bytes
uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;
	else return PAGE_SIZE-offset;
}

/* write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
HAL_StatusTypeDef EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef status=HAL_ERROR;
	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		status = HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 100);  // write the data to the EEPROM
		if(status!=HAL_OK){break;}
		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		osDelay (5);  // Write cycle delay (5ms)
	}
	return status;
}

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}


/*Write the Float/Integer values to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the float/integer value that you want to write
 */

void EEPROM_Write_NUM (uint16_t page, uint16_t offset, float data)
{

	float2Bytes(bytes_temp, data);

	EEPROM_Write(page, offset, bytes_temp, 4);
}

/* Reads the single Float/Integer values from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @returns the float/integer value
 */

float EEPROM_Read_NUM (uint16_t page, uint16_t offset)
{
	uint8_t buffer[4];

	EEPROM_Read(page, offset, buffer, 4);

	return (Bytes2float(buffer));
}

/* READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 500);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
}

/* Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 */
void EEPROM_PageErase (uint16_t page)
{
	// calculate the memory address based on the page number
	int paddrposition = log(PAGE_SIZE)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[PAGE_SIZE];
	memset(data,0xff,PAGE_SIZE);

	// write the data to the EEPROM
	HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, data, PAGE_SIZE, 50);

	osDelay (5);  // write cycle delay
}

//@TODO: add option to load default values from ROM to RAM during runtime

/* copy parameter set from RAM to EEPROM
 * and calculate XOR checksum
 */
HAL_StatusTypeDef save_parameter_set_to_eeprom(void){
	//calculate checksum of parameter set in RAM
	uint32_t xor_checksum=0;
	uint32_t * ptr_parameter_set = (uint32_t*)&parameter_set;
	for(uint8_t i=1;i<sizeof(parameter_set_t)/4;i++){
		xor_checksum= xor_checksum ^ (*(ptr_parameter_set+i));
	}
	if(xor_checksum==0x00000000 || xor_checksum==0xFFFFFFFF){
		inverter_error_trip(eeprom_error);
	}
	parameter_set.XOR_checksum=xor_checksum;
	//write from RAM to eeprom with newly calculated xor
	HAL_StatusTypeDef status =HAL_ERROR;
	status = EEPROM_Write(1, 0, (uint8_t*)&parameter_set, sizeof(parameter_set_t));
	return status;
}


HAL_StatusTypeDef read_parameter_set_from_eeprom(void){
	//read data from eeprom to buffer
	parameter_set_t eeprom_parameter_set;
	EEPROM_Read(1, 0, (uint8_t*)&eeprom_parameter_set, sizeof(parameter_set_t));
	//calculate XOR checksum of data from eeprom
	uint32_t xor_checksum=0;
	uint32_t * ptr_eeprom_parameter_set = (uint32_t*)&eeprom_parameter_set;
	//start from 1 to not calculate checksum of checksum
	for(uint8_t i=1;i<sizeof(parameter_set_t)/4;i++){
		xor_checksum= xor_checksum ^ (*(ptr_eeprom_parameter_set+i));
	}
	//@TODO: compare checksums, if 0 or FFFFFFFF the eeprom is empty if i think correctly, but this may be a result of valid data so better checksum method would be needed
	if((xor_checksum!=eeprom_parameter_set.XOR_checksum) || xor_checksum==0 || xor_checksum==0xFF){
		inverter_error_trip(eeprom_error); //@TODO: this error should be persistent
		save_parameter_set_to_eeprom(); //write default parameter set to eeprom
	}else{
	//copy data from eeprom to RAM parameter set address
	memcpy(&parameter_set,&eeprom_parameter_set,sizeof(parameter_set_t));
	}
	return HAL_OK;
}

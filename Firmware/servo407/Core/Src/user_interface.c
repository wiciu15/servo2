/*
 * user_interface.c
 *
 *  Created on: 21 cze 2023
 *      Author: Wiktor
 */

#include "user_interface.h"
#include "cmsis_os.h"

void test(){
	ssd1306_TestAll();
}

void draw(){
	ssd1306_TestRectangleFill();
	osDelay(500);
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();
}

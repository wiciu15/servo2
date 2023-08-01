/*
 * user_interface.h
 *
 *  Created on: 21 cze 2023
 *      Author: Wiktor
 */

#ifndef INC_USER_INTERFACE_H_
#define INC_USER_INTERFACE_H_

#include "ssd1306.h"
#include "ssd1306_tests.h"

#define SCREEN_REFRESH_RATE_MS 50

typedef enum {esc_key,up_key,down_key,enter_key}menu_button;


typedef struct _buttons_state_t{
	uint8_t prev_buttons_state; //previous loop buttons state to compare with actual states
	uint32_t pressed_time [4]; //array holding how long each button has been pressed so far
}buttons_state_t;


void display_init(void);
void draw(void);
void process_buttons_state(void);


#endif /* INC_USER_INTERFACE_H_ */

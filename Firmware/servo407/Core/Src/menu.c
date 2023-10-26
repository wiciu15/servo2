/*
 * menu.c
 *
 *  Created on: 1 sie 2023
 *      Author: Wiktor
 */
#include "inverter.h"
#include "user_interface.h"
#include "menu.h"
#include "ssd1306.h"
#include <string.h>

void (*key_back_func)(void) = &menu_back;
void (*key_next_func)(void) = &menu_next;
void (*key_prev_func)(void) = &menu_prev;
void (*key_ent_func)(void) = &menu_enter;
void (*refresh_func)(void) = &menu_refresh;

void menu_jog_callback(void);
void menu_monitor_callback(void);

uint8_t monitor_page=0;

menu_t main1;
	menu_t subpar1;
	menu_t subpar2;
	menu_t subpar3;
	menu_t subpar4;
menu_t main2;
	menu_t mon_menu;
menu_t main3;
	menu_t action_error_reset;
	menu_t action_jog;
	menu_t action_reboot;

menu_t main1 = {&main2,&main3,NULL,&subpar1,NULL,"PARAMETERS"};
	menu_t subpar1={&subpar2,NULL,&main1,NULL/**/,NULL/**/,"ALL CODES"};
	menu_t subpar2={&subpar3,&subpar1,&main1,NULL/**/,NULL/**/,"Motor"};
	menu_t subpar3={&subpar4,&subpar2,&main1,NULL/**/,NULL/**/,"Feedback"};
	menu_t subpar4={NULL,&subpar3,&main1,NULL/**/,NULL/**/,"Controller"};
menu_t main2 = {&main3,&main1,NULL,NULL,menu_monitor_callback,"MONITOR"};
menu_t main3 = {NULL,&main2,NULL,&action_error_reset,NULL,"ACTIONS"};
	menu_t action_error_reset={&action_jog,NULL,&main3,NULL,(void(*)(void))inverter_error_reset,"Error reset"};
	menu_t action_jog ={&action_reboot,&action_error_reset,&main3,NULL,menu_jog_callback,"JOG menu"};
	menu_t action_reboot={NULL,&action_jog,&main3,NULL,HAL_NVIC_SystemReset,"Reboot"};

menu_ctx_t menu_ctx = {
		.currentPointer = &main1,
		.menu_index=0,
		.lcd_row_pos=0
};

uint8_t menu_get_index(menu_t *q) {

	menu_t *temp;
	uint8_t i = 0;

	if (q->parent) temp = (q->parent)->child;
	else temp = &main1;

	while (temp != q) {
		temp = temp->next;
		i++;
	}

	return i;
}


uint8_t menu_get_level(menu_t *q) {

	menu_t *temp = q;
	uint8_t i = 0;

	if (!q->parent) return 0;

	while (temp->parent != NULL) {
		temp = temp->parent;
		i++;
	}

	return i;
}

void menu_refresh(void) {

	menu_t *temp;
	uint8_t i;

	if (menu_ctx.currentPointer->parent) temp = (menu_ctx.currentPointer->parent)->child;
	else temp = &main1;

	for (i = 0; i != menu_ctx.menu_index - menu_ctx.lcd_row_pos; i++) {
		temp = temp->next;
	}

	for (i = 14; i < 56; i+=11) {

		ssd1306_SetCursor(2	, i);
		if (temp == menu_ctx.currentPointer)ssd1306_WriteString(temp->name, Font_7x10,0);
		else ssd1306_WriteString(temp->name, Font_7x10,1);

		temp = temp->next;
		if (!temp) break;

	}
}

void menu_enter(void) {

	if (menu_ctx.currentPointer->menu_function) menu_ctx.currentPointer->menu_function();

	if (menu_ctx.currentPointer->child)
	{
		switch (menu_get_level(menu_ctx.currentPointer)) {
			case 0:
				menu_ctx.lcd_row_pos_level_1 = menu_ctx.lcd_row_pos;
				break;

			case 1:
				menu_ctx.lcd_row_pos_level_2 = menu_ctx.lcd_row_pos;
				break;
		}

		// switch...case can be replaced by:
		// lcd_row_pos_level[ menu_get_level(currentPointer) ] = lcd_row_pos;

		menu_ctx.menu_index = 0;
		menu_ctx.lcd_row_pos = 0;

		menu_ctx.currentPointer = menu_ctx.currentPointer->child;

	}
}

void menu_back(void) {

	if (menu_ctx.currentPointer->parent) {

		switch (menu_get_level(menu_ctx.currentPointer)) {
			case 1:
				menu_ctx.lcd_row_pos = menu_ctx.lcd_row_pos_level_1;
				break;
			case 2:
				menu_ctx.lcd_row_pos = menu_ctx.lcd_row_pos_level_2;
				break;
			}
		menu_ctx.currentPointer = menu_ctx.currentPointer->parent;
		menu_ctx.menu_index = menu_get_index(menu_ctx.currentPointer);

		menu_refresh();

	}else{
		if(inverter.state!=switch_on_disabled){inverter_disable();inverter.state=switch_on_disabled;}
		if(inverter.error!=no_error)inverter_error_reset();
	}
}

void menu_next(void) {

	if (menu_ctx.currentPointer->next)
	{
		menu_ctx.currentPointer = menu_ctx.currentPointer->next;
		menu_ctx.menu_index++;
		if (++menu_ctx.lcd_row_pos > LCD_ROWS - 1) menu_ctx.lcd_row_pos = LCD_ROWS - 1;
	}
	else
	{
		menu_ctx.menu_index = 0;
		menu_ctx.lcd_row_pos = 0;

		if (menu_ctx.currentPointer->parent) menu_ctx.currentPointer = (menu_ctx.currentPointer->parent)->child;
		else menu_ctx.currentPointer = &main1;
	}

	menu_refresh();

}

void menu_prev(void) {

	if (menu_ctx.currentPointer->prev)
	{
		menu_ctx.currentPointer = menu_ctx.currentPointer->prev;

		if (menu_ctx.menu_index)
		{
			menu_ctx.menu_index--;
			if (menu_ctx.lcd_row_pos > 0) menu_ctx.lcd_row_pos--;
		}
		else
		{
			menu_ctx.menu_index = menu_get_index(menu_ctx.currentPointer);

			if (menu_ctx.menu_index >= LCD_ROWS - 1) menu_ctx.lcd_row_pos = LCD_ROWS - 1;
			else menu_ctx.lcd_row_pos = menu_ctx.menu_index;
		}
	}

	menu_refresh();
}

void menu_jog_back(){
	inverter.speed_setpoint=0.0f;
	inverter_disable();
	key_next_func = menu_next;
	key_prev_func = menu_prev;
	key_ent_func = menu_enter;
	key_back_func = menu_back;

	refresh_func = menu_refresh;
}

void menu_jog_next(){
	inverter.speed_setpoint+=10.0f;
}

void menu_jog_prev(){
	inverter.speed_setpoint-=10.0f;
}

void menu_jog_enter(){
	if(inverter.state==switch_on_disabled){inverter_enable();inverter.state=operation_enabled;}
	else if(inverter.state==switched_on||inverter.state==operation_enabled){inverter_disable();}
}


void menu_jog_refresh(){
	if(inverter.control_mode>=1){
		if(isInverter_running() && button_state.pressed_time[enter_key]<20){inverter_disable();}
		key_next_func = menu_jog_next;
		key_prev_func = menu_jog_prev;
		key_ent_func = menu_jog_enter;
		ssd1306_SetCursor(36, 20);
		ssd1306_WriteString("JOG MENU", Font_7x10,0);
		ssd1306_SetCursor(2, 32);
		char  setpointString [24];
		sprintf(setpointString,"Set speed: %.0fRPM",inverter.speed_setpoint);
		ssd1306_WriteString(setpointString, Font_6x8, 1);
	}else{
		ssd1306_SetCursor(11, 32);
		ssd1306_WriteString("NOT IN FOC MODE", Font_7x10, 1);
		key_next_func = NULL;
		key_prev_func = NULL;
		key_ent_func = NULL;
	}
}

void menu_jog_callback(){
	key_back_func = menu_jog_back;
	refresh_func = menu_jog_refresh;
	menu_jog_refresh();
}

//this function is used to make corrections to values, account for type different than float or values depending on inverter actual state
/*float monitor_get_value(void* ptr){
	float returnValue;
	if(ptr==&inverter.speed_setpoint){
		if(inverter.control_mode!=foc)returnValue=(inverter.stator_field_speed/(_2_PI/inverter.control_loop_freq));
		else {returnValue=*(float*)ptr;}
	}
	else if(ptr==&inverter.stator_electric_angle)returnValue=(*(float*)ptr)*(180.0f/_PI);
	else if(ptr==&inverter.torque_angle)returnValue=(*(float*)ptr)*(180.0f/_PI);
	else if(ptr==&inverter.encoder_raw_position)returnValue=(float)(*(uint16_t*)ptr);
	else returnValue = *(float*)ptr;

	return returnValue;
}*/
void menu_monitor_back(){
	key_next_func = menu_next;
	key_prev_func = menu_prev;
	key_ent_func = menu_enter;
	key_back_func = menu_back;

	refresh_func = menu_refresh;
}

void menu_monitor_next(){
if(monitor_page*5<monitor_list_size)monitor_page++;
}

void menu_monitor_prev(){
if(monitor_page>0)monitor_page--;
}

void menu_monitor_enter(){

}


void menu_monitor_refresh(){
	for(uint16_t i=monitor_page*5;i<monitor_page*5+5;i++){
		if(i>=monitor_list_size)break; //check if monitor list not ended
		ssd1306_SetCursor(2, ((i%5)*9)+14);
		char shortname [10];
		memcpy(shortname,monitor_list[i].shortName,sizeof(monitor_list[i].shortName));
		ssd1306_WriteString(shortname, Font_6x8, 1);
		ssd1306_SetCursor(70, ((i%5)*9)+14);
		char  stringbuf [30];
		uint32_t rawvalue=0;
		if(parameter_read(&monitor_list[i], &rawvalue)==HAL_OK){
		switch(monitor_list[i].type){
		case pFLOAT:{
			float value;
			memcpy(&value,&rawvalue,4); //switch from 32bit data to float
			sprintf(stringbuf,"%6.*f%s",(int)monitor_list[i].precision,value,monitor_list[i].unit);
			break;
		}
		case pINT16:{
			int16_t value;
			memcpy(&value,&rawvalue,2); //switch from 32bit raw data to int16
			sprintf(stringbuf,"%6.1i%s",value,monitor_list[i].unit);
			break;
		}
		case pUINT16:{
			uint16_t value;
			memcpy(&value,&rawvalue,2); //switch from 32bit raw data to int16
			sprintf(stringbuf,"%6.1u%s",value,monitor_list[i].unit);
			break;
		}
		case pBOOL16:{
			uint16_t value;
			memcpy(&value,&rawvalue,2); //switch from 32bit raw data to int16
			sprintf(stringbuf,"%#X%s",value,monitor_list[i].unit);
			break;
		}
		}
		ssd1306_WriteString(stringbuf, Font_6x8, 1);
		}
	}
}

void menu_monitor_callback(){
	key_back_func = menu_monitor_back;
	key_next_func = menu_monitor_next;
	key_prev_func = menu_monitor_prev;
	key_ent_func = menu_monitor_enter;

	refresh_func = menu_monitor_refresh;

}

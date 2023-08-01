/*
 * menu.c
 *
 *  Created on: 1 sie 2023
 *      Author: Wiktor
 */
#include "menu.h"
#include "ssd1306.h"

menu_t main1;
	menu_t subpar1;
	menu_t subpar2;
	menu_t subpar3;
	menu_t subpar4;
menu_t main2;
	menu_t mon_menu;
menu_t main3;
	menu_t action_jog;
	menu_t action_save;
	menu_t action_reboot;

menu_t main1 = {&main2,&main3,NULL,&subpar1,NULL,"PARAMETERS"};
	menu_t subpar1={&subpar2,NULL,&main1,NULL/**/,NULL/**/,"ALL CODES"};
	menu_t subpar2={&subpar3,&subpar1,&main1,NULL/**/,NULL/**/,"Motor"};
	menu_t subpar3={&subpar4,&subpar2,&main1,NULL/**/,NULL/**/,"Feedback"};
	menu_t subpar4={NULL,&subpar3,&main1,NULL/**/,NULL/**/,"Controller"};
menu_t main2 = {&main3,&main1,NULL,NULL,NULL/**/,"MONITOR"};
menu_t main3 = {NULL,&main2,NULL,&action_jog,NULL,"ACTIONS"};
	menu_t action_jog ={&action_save,NULL,&main3,NULL,NULL/**/,"JOG menu"};
	menu_t action_save ={&action_reboot,&action_jog,&main3,NULL,NULL/**/,"Save to EEPROM"};
	menu_t action_reboot={NULL,&action_save,&main3,NULL,NULL/**/,"Reboot"};

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

	for (i = 12; i < 56; i+=11) {

		ssd1306_SetCursor(0	, i);
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

		//menu_refresh();
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

		//menu_refresh();

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

	//menu_refresh();

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

	//menu_refresh();
}



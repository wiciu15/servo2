/*
 * menu.h
 *
 *  Created on: 1 sie 2023
 *      Author: Wiktor
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "main.h"
#include "parameter_list.h"

#define LCD_ROWS 4

typedef struct _menu_t menu_t;
struct _menu_t{
	menu_t * next;
	menu_t * prev;
	menu_t * parent;
	menu_t * child;
	void (*menu_function)(void);
	char * name;
};

typedef struct _menu_ctx_t{
	menu_t *currentPointer;
	uint8_t menu_index;
	uint8_t lcd_row_pos;
	uint8_t lcd_row_pos_level_1;
	uint8_t lcd_row_pos_level_2;
}menu_ctx_t;

extern menu_ctx_t menu_ctx;

extern void (*key_back_func)(void);
extern void (*key_next_func)(void);
extern void (*key_prev_func)(void);
extern void (*key_ent_func)(void);
extern void (*refresh_func)(void);

void menu_refresh(void);
void menu_next(void);
void menu_prev(void);
void menu_enter(void);
void menu_back(void);


#endif /* INC_MENU_H_ */

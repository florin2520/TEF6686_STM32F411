/*
 * display_functions.h
 *
 *  Created on: Nov 3, 2024
 *      Author: nicaf
 */

#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#define DISPLAY_ADDRESS 0x70<<1  //
#define NR_OF_DIGITS 8
//#define SCROLL_SPEED 150  // increase the number for slow
//#define SCROLL_SPEED 80000  // increase the number for slow
//#define SCROLL_SPEED 70000  // increase the number for slow
#define SCROLL_SPEED 12   //
#define ONE_STEP 1
#define FOUR_STEP 4

#ifndef INC_DISPLAY_FUNCTIONS_H_
#define INC_DISPLAY_FUNCTIONS_H_

void clear_display(void);
void disp_vol(uint32_t vol);
void disp_freq(uint32_t freq);
void display_static_message(char *mess);
void show_on_display(char *message, uint8_t index_of_point);
uint8_t find_decimal_point_in_string(char *message);
void display_scrolling_message(char *mess);
void left_rotate_by_one(char arr[], int n);
void left_rotate(char arr[], int d, int n);
void populate_freq_array(uint32_t freq);
void populate_vol_array(uint16_t vol);
uint8_t find_3_space_in_string(char *message);
void disp_freq_animate(uint32_t freq);

#endif /* INC_DISPLAY_FUNCTIONS_H_ */

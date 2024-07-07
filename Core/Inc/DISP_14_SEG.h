/*
 * DISP_14_SEG.h
 *
 *  Created on: Nov 19, 2021
 *      Author: nicaf
 */

#ifndef INC_DISP_14_SEG_H_
#define INC_DISP_14_SEG_H_

#endif /* INC_DISP_14_SEG_H_ */

#include "stdio.h"
#include "stdint.h"
#include <stdbool.h>


#define LED_ON 1
#define LED_OFF 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

void begin_disp(uint8_t _addr);
void setBrightness(uint16_t i2c_addr, uint8_t b);
void blinkRate(uint8_t i2c_addr, uint8_t b);
void writeDisplay(uint8_t i2c_addr);
void clear_display(void);
//uint16_t displaybuffer[8];
void init(uint8_t a);

void writeDigitRaw(uint8_t n, uint16_t bitmask);
void writeDigitAscii(uint8_t n, uint8_t ascii, bool dot);


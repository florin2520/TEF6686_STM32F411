/*
 * tef6686.h
 *
 *  Created on: Jan 28, 2023
 *      Author: nicaf
 */
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef INC_TEF6686_H_
#define INC_TEF6686_H_



void Write(uint8_t *buf, uint8_t len);
void Read(uint8_t *buf, uint8_t len);
void Set_Cmd(uint8_t mdl, uint8_t cmd, int len, ...);
void Get_Cmd(uint8_t mdl, uint8_t cmd, int16_t *receive, int len);
void dsp_write_data(const uint8_t* data);
//void scan(bool continous);
void get_RDS();
void serial_hex(uint8_t val);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void setup();





#endif /* INC_TEF6686_H_ */

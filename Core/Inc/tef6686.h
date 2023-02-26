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

struct RdsInfo
{
  char programType[17];
  char programService[9];
  char radioText[65];
  bool newRadioText;
};

uint16_t seek(uint8_t up);
uint16_t tune(uint8_t up);
void rdsFormatString(char* str, uint16_t length);



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
int bitRead(char bit, uint16_t number);

uint8_t readRDS();
void getRDS(struct RdsInfo *rdsInfo);
void rdsFormatString(char* str, uint16_t length);


void show_Rds();
void showPTY();
void showPS();
void showRadioText();
bool str_cmp(char* str1, char* str2, int length);


#endif /* INC_TEF6686_H_ */

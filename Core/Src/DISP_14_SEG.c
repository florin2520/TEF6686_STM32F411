/*
 * DISP_14_SEG.c
 *
 *  Created on: Nov 19, 2021
 *      Author: nicaf
 */

#include "DISP_14_SEG.h"
#include "i2c.h"
//#include <stdint.h>
//#include "string.h"

#include "I2C_reg.h"
//extern I2C_HandleTypeDef hi2c2;
uint16_t displaybuffer[8];
//uint8_t i2c_addr;

/*
 *
 *          _____A_____
 *         |\    |    /|
 *         | \H  |J  / |
 *       F |  \  |  /K | B
 *         |   \ | /   |
 *         |_G1_\|/_G2_|
 *         |    /|\    |
 *         |   / | \   |
 *       E | L/  |  \N | C
 *         | /   |M  \ |
 *         |/____|____\| .DP
 *               D
 *
 * */


static const uint16_t alphafonttable[]  =
{
//0 DP N M L K J H G2 G1 F E D C B A
0b0000000000000001,
0b0000000000000010,
0b0000000000000100,
0b0000000000001000,
0b0000000000010000,
0b0000000000100000,
0b0000000001000000,
0b0000000010000000,
0b0000000100000000,
0b0000001000000000,
0b0000010000000000,
0b0000100000000000,
0b0001000000000000,
0b0010000000000000,
0b0100000000000000,
0b1000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0001001011001001,
0b0001010111000000,
0b0001001011111001,
0b0000000011100011,
0b0000010100110000,
0b0001001011001000,
0b0011101000000000,
0b0001011100000000,
0b0000000000000000, //
0b0000000000000110, // !
0b0000001000100000, // "
0b0001001011001110, // #
0b0001001011101101, // $
0b0000110000100100, // %
0b0010001101011101, // &
0b0000010000000000, // '
0b0010010000000000, // (
0b0000100100000000, // )
0b0011111111000000, // *
0b0001001011000000, // +
0b0000100000000000, // ,
0b0000000011000000, // -
0b0000000000000000, // .
0b0000110000000000, // /
0b0000110000111111, // 0
0b0000010000000110, // 1
0b0000000011011011, // 2
0b0000000010001111, // 3
0b0000000011100110, // 4
0b0010000001101001, // 5
0b0000000011111101, // 6
0b0000110011000001, // 7
0b0000000011111111, // 8
0b0000000011101111, // 9
0b0001001000000000, // :
0b0000101000000000, // ;
0b0010010000000000, // <
0b0000000011001000, // =
0b0000100100000000, // >
0b0001000010000011, // ?
0b0000001010111011, // @
0b0000000011110111, // A
0b0001001010001111, // B
0b0000000000111001, // C
0b0001001000001111, // D
0b0000000011111001, // E
0b0000000001110001, // F
0b0000000010111101, // G
0b0000000011110110, // H
0b0001001000000000, // I
0b0000000000011110, // J
0b0010010001110000, // K
0b0000000000111000, // L
0b0000010100110110, // M
0b0010000100110110, // N
0b0000000000111111, // O
0b0000000011110011, // P
0b0010000000111111, // Q
0b0010000011110011, // R
0b0000000011101101, // S
0b0001001000000001, // T
0b0000000000111110, // U
0b0000110000110000, // V
0b0010100000110110, // W
0b0010110100000000, // X
0b0001010100000000, // Y
0b0000110000001001, // Z
0b0000000000111001, // [
0b0010000100000000, //
0b0000000000001111, // ]
0b0000110000000011, // ^
0b0000000000001000, // _
0b0000000100000000, // `
0b0001000001011000, // a
0b0010000001111000, // b
0b0000000011011000, // c
0b0000100010001110, // d
0b0000100001011000, // e
0b0000000001110001, // f
0b0000010010001110, // g
0b0001000001110000, // h
0b0001000000000000, // i
0b0000000000001110, // j
0b0011011000000000, // k
0b0000000000110000, // l
0b0001000011010100, // m
0b0001000001010000, // n
0b0000000011011100, // o
0b0000000101110000, // p
0b0000010010000110, // q
0b0000000001010000, // r
0b0010000010001000, // s
0b0000000001111000, // t
0b0000000000011100, // u
0b0010000000000100, // v
0b0010100000010100, // w
0b0010100011000000, // x
0b0010000000001100, // y
0b0000100001001000, // z
0b0000100101001001, // {
0b0001001000000000, // |
0b0010010010001001, // }
0b0000010100100000, // ~
0b0011111111111111,

};

void setBrightness(uint16_t i2c_addr, uint8_t b)
{
  if (b > 15) b = 15;
  //Wire.beginTransmission(i2c_addr);
  //Wire.write(HT16K33_CMD_BRIGHTNESS | b);
  //Wire.endTransmission();

  I2C_2_Start ();
  I2C_2_Address (i2c_addr);
  I2C_2_Write (HT16K33_CMD_BRIGHTNESS | b);
  I2C_2_Stop ();

  //HAL_I2C_Master_Transmit(hi2c2, DevAddress, addr, 2, HAL_MAX_DELAY);
  //HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)(HT16K33_CMD_BRIGHTNESS | b), 1, 150);
}

void blinkRate(uint8_t i2c_addr, uint8_t b)
{
  //Wire.beginTransmission(i2c_addr);
  I2C_2_Start ();
  I2C_2_Address (i2c_addr);
  if (b > 3) b = 0; // turn off if not sure

  //Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
  //Wire.endTransmission();
  I2C_2_Write (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
  I2C_2_Stop ();
  //HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1)), 1, 150);
}


void begin_disp(uint8_t i2c_addr)
{
  //i2c_addr = _addr;

//  Wire.begin();
//  Wire.beginTransmission(i2c_addr);
//  Wire.write(0x21);  // turn on oscillator
//  Wire.endTransmission();
  I2C_2_Start ();
  I2C_2_Address (i2c_addr);
  I2C_2_Write (0x21);  // turn on oscillator
  I2C_2_Stop ();
	//HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)0x21, 1, 150);

  blinkRate(i2c_addr, HT16K33_BLINK_OFF);
  //blinkRate(i2c_addr, HT16K33_BLINK_2HZ);

  //setBrightness(15); // max brightness
  setBrightness(i2c_addr, 15); //
}

void writeDisplay(uint8_t i2c_addr)
{
  //Wire.beginTransmission(i2c_addr);
  //Wire.write((uint8_t)0x00); // start at address $00
  I2C_2_Start ();
  I2C_2_Address (i2c_addr);
  I2C_2_Write ((uint8_t)0x00);  // start at address $00

  for (uint8_t i=0; i<8; i++)
  {
    //Wire.write(displaybuffer[i] & 0xFF);
    //Wire.write(displaybuffer[i] >> 8);
	 I2C_2_Write (displaybuffer[i] & 0xFF);
	 I2C_2_Write (displaybuffer[i] >> 8);
  }
  //Wire.endTransmission();
  I2C_2_Stop ();

//	HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)0, 1, 150);
//    for (uint8_t i=0; i<8; i++)
//	{
//		 HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)(displaybuffer[i] & 0xFF), 1, 150);
//		 HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_addr, (uint8_t*)(displaybuffer[i] >> 8), 1, 150);
//	}

}

void clear_display(void)
{
  for (uint8_t i=0; i<8; i++)
  {
    displaybuffer[i] = 0;
  }
}


void writeDigitRaw(uint8_t n, uint16_t bitmask)
{
  displaybuffer[n] = bitmask;
}

void writeDigitAscii(uint8_t n, uint8_t a,  bool d) // d = true => punct activ
{
  //uint16_t font = pgm_read_word(alphafonttable+a);
  uint16_t font = alphafonttable[a];
  displaybuffer[n] = font;

  if (d) displaybuffer[n] |= (1<<14);
}

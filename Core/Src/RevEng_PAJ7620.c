/**
  \file RevEng_PAJ7620.cpp
  \author Aaron S. Crandall

  \version 1.5.0

  \copyright
  \parblock
  - Copyright (c) 2015 seeed technology inc.
  - Website    : www.seeed.cc
  - Author     : Wuruibin & Xiangnan
  - Modified Time: June 2015

  Additional contributions:
  - 2017 - Modified by MarcFinns to encapsulate in class without global variables  
  - 2020 - PROGMEM code adapted from Jaycar-Electronics' work  
  - 2020 - Modified by Aaron S. Crandall <crandall@gonzaga.edu>  
  - 2020 - Modified by Sean Kallaher (GitHub: skallaher) 
  
  Description: This driver class can recognize 9 gestures and output the result,
        including move up, move down, move left, move right,
        move forward, move backward, circle-clockwise,
        circle-anti (counter) clockwise, and wave.
        The driver also allows changing the sensor to 'cursor mode' where it
        tracks the closest object in view on an (X,Y) coordinate system.

  License: The MIT License (MIT)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  \endparblock

  PAJ7620U2 Sensor data sheet for reference found here:
    https://datasheetspdf.com/pdf-file/1309990/PixArt/PAJ7620U2/1

  Driver sources, latest code, and authors available at:
    https://github.com/acrandal/RevEng_PAJ7620
*/

#include "I2C_reg.h"
#include "RevEng_PAJ7620.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

/**
 * Initial device register addresses and values.
 * \note Puts device into gesture mode with various "normal" mode values.
 * \note Values taken from PixArt reference documentation v0.8 & v1.0 - see <a href="https://github.com/acrandal/RevEng_PAJ7620/wiki">wiki</a> for files
 */
#ifdef PROGMEM_COMPATIBLE
const unsigned short initRegisterArray[] PROGMEM = {
#else
const unsigned short initRegisterArray[] = {
#endif
    0xEF00,       // Bank 0
    0x4100,       // Disable interrupts for first 8 gestures
    0x4200,       // Disable wave (and other modes') interrupt(s)
    0x3707,
    0x3817,
    0x3906,
    0x4201,
    0x462D,
    0x470F,
    0x483C,
    0x4900,
    0x4A1E,
    0x4C22,
    0x5110,
    0x5E10,
    0x6027,
    0x8042,
    0x8144,
    0x8204,
    0x8B01,
    0x9006,
    0x950A,
    0x960C,
    0x9705,
    0x9A14,
    0x9C3F,
    0xA519,
    0xCC19,
    0xCD0B,
    0xCE13,
    0xCF64,
    0xD021,
    0xEF01,       // Bank 1
    0x020F,
    0x0310,
    0x0402,
    0x2501,
    0x2739,
    0x287F,
    0x2908,
    0x3EFF,
    0x5E3D,
    0x6596,       // R_IDLE_TIME LSB - Set sensor speed to 'normal speed' - 120 fps
    0x6797,
    0x69CD,
    0x6A01,
    0x6D2C,
    0x6E01,
    0x7201,
    0x7335,
    0x7400,       // Set to gesture mode
    0x7701,
    0xEF00,       // Bank 0
    0x41FF,       // Re-enable interrupts for first 8 gestures
    0x4201        // Re-enable interrupts for wave gesture
};


/** Generated size of the register set gesture mode array */
#define SET_GES_MODE_REG_ARRAY_SIZE (sizeof(setGestureModeRegisterArray)/sizeof(setGestureModeRegisterArray[0]))

/**
 * Gesture mode specific register addresses and values
 * \note Puts device into gesture mode with appropriate values.
 * \note Values taken from PixArt reference documentation v0.8 & v1.0 - see <a href="https://github.com/acrandal/RevEng_PAJ7620/wiki">wiki</a> for files
 */
#ifdef PROGMEM_COMPATIBLE
const unsigned short setGestureModeRegisterArray[] PROGMEM = {
#else
const unsigned short setGestureModeRegisterArray[] = {
#endif
    0xEF00,       // Bank 0
    0x4100,       // Disable interrupts for first 8 gestures
    0x4200,       // Disable wave (and other mode's) interrupt(s)
    0x483C,
    0x4900,
    0x5110,
    0x8320,
    0x9ff9,
    0xEF01,       // Bank 1
    0x011E,
    0x020F,
    0x0310,
    0x0402,
    0x4140,
    0x4330,
    0x6596,       // R_IDLE_TIME  - Normal mode LSB "120 fps" (supposedly)
    0x6600,
    0x6797,
    0x6801,
    0x69CD,
    0x6A01,
    0x6bb0,
    0x6c04,
    0x6D2C,
    0x6E01,
    0x7400,       // Set gesture mode
    0xEF00,       // Bank 0
    0x41FF,       // Re-enable interrupts for first 8 gestures
    0x4201        // Re-enable interrupts for wave gesture
};


/** Generated size of the register set cursor mode array */
#define SET_CURSOR_MODE_REG_ARRAY_SIZE (sizeof(setCursorModeRegisterArray)/sizeof(setCursorModeRegisterArray[0]))

/**
 * Cursor mode specific register addresses and values
 * \note Puts device into cursor mode with reasonable basic values.
 * \note Values taken from PixArt reference documentation v0.8 & v1.0 - see <a href="https://github.com/acrandal/RevEng_PAJ7620/wiki">wiki</a> for files
 */
#ifdef PROGMEM_COMPATIBLE
const unsigned short setCursorModeRegisterArray[] PROGMEM = {
#else
const unsigned short setCursorModeRegisterArray[] = {
#endif
    0xEF00,   // Set Bank 0
    0x3229,   // Default  29  [0] Cursor use top - def 1
              //              [1] Cursor Use BG Model - def 0
              //              [2] Cursor Invert Y - def 0       -- Not sure, doesn't seem to work
              //              [3] Cursor Invert X - def 1
              //              [5:4] Cursor top Ratio - def 0x2
    0x3301,   // Default  01  R_PositionFilterStartSizeTh [7:0]
    0x3400,   // Default  00  R_PositionFilterStartSizeTh [8]
    0x3501,   // Default  01  R_ProcessFilterStartSizeTh [7:0]
    0x3600,   // Default  00  R_ProcessFilterStartSizeTh [8]
    0x3703,   // Default  09  R_CursorClampLeft [4:0]
    0x381B,   // Default  15  R_CursorClampRight [4:0]
    0x3903,   // Default  0A  R_CursorClampUp [4:0]
    0x3A1B,   // Default  12  R_CursorClampDown [4:0]
    0x4100,   // Interrupt enable mask - Should be 00 (disable gestures)
              //              All gesture flags [7:0]
    0x4284,   // Interrupt enable mask - Should be 84 (0b 1000 0100)
              //              bit 0: Wave, wave mode use only
              //              bit 1: Proximity, proximity mode use only
              //              bit 2: Has Object, cursor mode use only
              //              bit 3: Wake up trigger, trigger mode use only
              //              bit 4: Confirm, confirm mode use only
              //              bit 5: Abort, confirm mode use only
              //              bit 6: N/A
              //              bit 7:No Object, cursor mode use only
    0x8B01,   // Default  10  R_Cursor_ObjectSizeTh [7:0]
    0x8C07,   // Default  07  R_PositionResolution [2:0]
    0xEF01,   // Set Bank 1
    0x0403,   // Invert X&Y Axes in lens for GUI coordinates
              //  Where (0,0) is in upper left, positive down (Y) and right (X)
    0x7403,   // Enable cursor mode 0 - gesture, 3 - cursor, 5 - proximity
    0xEF00    // Set Bank 0 (parking it)
};

    unsigned long gestureEntryTime; // User set gesture entry delay in ms (default: 0)
    unsigned long gestureExitTime;  // User set gesture exit delay in ms (default 200)


/**
 * PAJ7620 device initialization and I2C connect to default Wire bus
 * 
 * \param none
 * \return error code: 0 (false); success: return 1 (true)
 */
//uint8_t RevEng_PAJ7620::begin()
//{
//  return begin(&Wire);
//}


/**
 * PAJ7620 device initialization and I2C connect on specified Wire bus
 *
 * Override version:
 * \par
 * Takes a TwoWire pointer allowing the user to pass
 *    in a specified I2C bus for devices using alternatives to bus 0 such
 *    as: begin(&Wire1) or begin(&Wire2)
 *
 * \param chosenWireHandle A pointer to the Wire handle that should be
 *   used to communicate with the PAJ7620
 * \return error code: 0 (false); success: return 1 (true)
 */
//uint8_t begin(TwoWire *chosenWireHandle)
uint8_t beginPAJ7620()
{
  //print_serial2_message("DEBUG 1");
  // Reasonable timing delay values to make algorithm insensitive to
  //  hand entry and exit moves before and after detecting a gesture
  gestureEntryTime = 0;
  //gestureExitTime = 200;
  gestureExitTime = 200;
  //wireHandle = chosenWireHandle;      // Save selected I2C bus for our use

  //delayMicroseconds(700);	            // Wait 700us for PAJ7620U2 to stabilize
                                        // Reason: see v0.8 of 7620 documentation
  //wireHandle->begin();                // Start the I2C bus via wire library
  //HAL_Delay(1);
  uint32_t a;
  for (uint32_t i = 0; i < 500000; i++)
  {
    a = i;
    a++;
  }

  //print_serial2_message("DEBUG 2");
  /* There's two register banks (0 & 1) to be selected between.
   * BANK0 is where most data collection operations happen, so it's default.
   * Selecting the bank is done here twice for a reason. When the 7620 turns
   *  on, the I2C bus is sleeping. When you first read/write to the bus
   *  the 7620 wakes up, but it sometimes misses that first message.
   * Running the 7620 on an arduino with the USB power, a single call here
   *  usually works, but as soon as you use an external power bus it often
   *  fails to properly initialize and begin returns an error.
   */
  selectRegisterBank(BANK0);          // This is done twice on purpose
  selectRegisterBank(BANK0);          // Default operations on BANK0


//  if( !isPAJ7620UDevice() )
//  {
//    return 0;                         // Return false - wrong device found
//  }

  initializeDeviceSettings();         // Set registers up
  setGestureMode();                   // Specifically set to gesture mode

  return 1;
}


/**
 * Write memory register over I2C
 * \param i2cAddress register address
 * \param dataByte data (byte) to write
 * \return error code; success: return 0
 */
uint8_t writeRegister(uint8_t i2cAddress, uint8_t dataByte)
{
	  //uint32_t a;
//	  uint8_t buf[2];
//	  HAL_StatusTypeDef r;
//      buf[0] = dataByte;
//	  r = HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)PAJ7620_I2C_BUS_ADDR, buf, 1, 1000);
//
////	   for (uint32_t i = 0; i < 5000; i++)
////	   {
////	     a = i;
////	     a++;
////	   }
//	  return (r == HAL_OK) ? 1 : 0;




  uint8_t resultCode = 0;
//  wireHandle->beginTransmission(PAJ7620_I2C_BUS_ADDR);   // start transmission
//  wireHandle->write(i2cAddress);                         // send register address
//  wireHandle->write(dataByte);                           // send value to write
//  resultCode = wireHandle->endTransmission();            // end transmission
  //print_serial2_message("DEBUG 3"); ///last ok
  I2C_2_Start ();
  I2C_2_Address (PAJ7620_I2C_BUS_ADDR); // todo
  I2C_2_Write (i2cAddress);
  I2C_2_Write (dataByte);
  I2C_2_Stop ();

  //print_serial2_message("DEBUG 4");
  return resultCode;
}

//todo
/**
 * Read memory register over I2C
 * \param i2cAddress : register address
 * \param byteCount : quantity of bytes to read into data
 * \param data : array of uint8_t to read data into
 * \return error code; success: return 0
 */
uint8_t readRegister(uint8_t i2cAddress, uint8_t byteCount, uint8_t data[])
{
  //uint8_t result_code;
//  wireHandle->beginTransmission(PAJ7620_I2C_BUS_ADDR);
//  wireHandle->write(i2cAddress);
//  result_code = wireHandle->endTransmission();
	I2C_2_Start ();
	I2C_2_Address (PAJ7620_I2C_BUS_ADDR);
	I2C_2_Write (i2cAddress);  // adresa registrului intern
	//I2C_3_Stop ();
	//I2C_3_Address (i2cAddress);
	I2C_2_Start ();  // start
	//I2C_3_Read (PAJ7620_I2C_BUS_ADDR+0x01, data, byteCount);
	I2C_2_Read (PAJ7620_I2C_READ_BUS_ADDR, data, byteCount);
	I2C_2_Stop ();
//	  uint32_t a;
//	  for (uint32_t i = 0; i < 10000; i++)
//	  {
//	    a = i;
//	    a++;
//	  }
//  if (result_code)            //return error code - if not zero
//    { return result_code; }

  //wireHandle->requestFrom((int)PAJ7620_I2C_BUS_ADDR, (int)byteCount);

//  while (wireHandle->available())
//  {
//    *data = wireHandle->read();
//    data++;
//  }

  return 0;
}


/**
 * Read the gestures interrupt vector #0 - all gestures except wave
 * \param data : &uint8_t for storing value read
 * \return error code; success: return 0
 */
uint8_t getGesturesReg0(uint8_t data[])
  { return readRegister(PAJ7620_ADDR_GES_RESULT_0, 1, data); }


/**
 * Read the gestures interrupt vector #1 - only holds wave
 * \param data : &uint8_t for storing value read
 * \return error code; success: return 0
 */
uint8_t getGesturesReg1(uint8_t data[])
  { return readRegister(PAJ7620_ADDR_GES_RESULT_1, 1, data); }


/**
 * Select memory bank to read/write to
 * \par
 * The PAJ7620 has two memory banks. The user must select which bank to use
 * when reading and writing over I2C.
 * \note This driver defaults to operations resetting to BANK0 for general operation.
 * \param bank : \link Bank_e \endlink to select (BANK0, BANK1)
 * \return none
 */
void selectRegisterBank(Bank_e bank)
{
  if( bank == BANK0 )
    { writeRegister(PAJ7620_REGISTER_BANK_SEL, PAJ7620_BANK0); }
  else if( bank == BANK1 )
    { writeRegister(PAJ7620_REGISTER_BANK_SEL, PAJ7620_BANK1); }
}


/**
 * Reads device memory to check for the PAJ7620 hardware identifier (ID)
 * \par
 * At memory address BANK0, 0x00 the device returns 0x20.
 * At memory address BANK0, 0x01 the device returns 0x76.
 * If this is not true, a non-PAJ7620 I2C device is attached at this I2C address.
 * See: PAJ7620U2 datasheet page 24 - 5.16 Chip/Version ID
 * \param none
 * \return bool: True means it is a PAJ7620, False means failure to read or ID match
 */
bool isPAJ7620UDevice()
{
  uint8_t data0 = 0, data1 = 0;

  // Device ID is stored in BANK0
  selectRegisterBank(BANK0);

  // Read PartID LSB[7:0] from Bank0, 0x00 - Should read 0x20
  // Read PartID MSB[15:8] from Bank0, 0x01 - Should read 0x76
  readRegister(PAJ7620_ADDR_PART_ID_0, 1, &data0);
  readRegister(PAJ7620_ADDR_PART_ID_1, 1, &data1);

  // Test if part ID is corect for PAJ7620U2
  //  See: PAJ7620U2 datasheet page 24 - 5.16 Chip/Version ID
  if ( (data0 != PAJ7620_PART_ID_LSB ) || (data1 != PAJ7620_PART_ID_MSB) )
    { return false; }

  return true;
}


/**
 * Writes an array of values to the device memory
 * 
 * \par
 * Writes over I2C to the memory banks a set of default values for operation.
 * The values are taken from the PAJ7620U2 v0.8 documentation and encoded
 * in the \link initRegisterArray \endlink from the driver's header file
 * 
 * \note Expects array[] to be stored in PROGMEM if it is available on your microcontroller
 * 
 * \param array : array of const unsigned shorts - first byte is address, second byte is data
 * \param arraySize : quantity of elements in array to write
 * \return none
 */
void writeRegisterArray(const unsigned short array[], int arraySize)
{
  for (unsigned int i = 0; i < arraySize; i++)
  {
    #ifdef PROGMEM_COMPATIBLE
      uint16_t word = pgm_read_word(&array[i]);
    #else
      uint16_t word = array[i];
    #endif

    uint8_t address, value;
    address = (word & 0xFF00) >> 8;
    value = (word & 0x00FF);
    writeRegister(address, value);
  }
  selectRegisterBank(BANK0);        // Guarantee parking in BANK0
}


/**
 * Initializes registers for device to default values
 * 
 * \par
 * Writes over I2C to the memory banks a set of default values for operation.
 * The values are taken from the PAJ7620U2 v0.8 documentation and encoded
 * in the \link initRegisterArray \endlink from the driver's header file
 * 
 * \param none
 * \return none
 */
void initializeDeviceSettings()
{
  writeRegisterArray(initRegisterArray, INIT_REG_ARRAY_SIZE);
}


/**
 * Puts device into Gesture mode
 * 
 * \par
 * Initializes registers for Gesture mode and enables only the gesture interrupts
 * 
 * \param none
 * \return none
 */
void setGestureMode()
{
  writeRegisterArray(setGestureModeRegisterArray, SET_GES_MODE_REG_ARRAY_SIZE);
}


/**
 * Puts device into Cursor mode
 * 
 * \par
 * Initializes registers for Cursor mode and enables only the cursor interrupts
 * 
 * \param none
 * \return none
 */
void setCursorMode()
{
  writeRegisterArray(setCursorModeRegisterArray, SET_CURSOR_MODE_REG_ARRAY_SIZE);
}


/**
 * Gets cursor object's current X location
 * 
 * \note Only works in cursor mode
 * \param none
 * \return int : X coordinate of cursor
 */
int getCursorX()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_CURSOR_X_LOW, 1, &data0);
  readRegister(PAJ7620_ADDR_CURSOR_X_HIGH, 1, &data1);
  data1 &= 0x0F;      // Mask off high bits (unused)
  result |= data1;
  result = result << 8;
  result |= data0;

  return result;
}


/**
 * Gets cursor object's current Y location
 * 
 * \note Only works in cursor mode
 * \param none
 * \return int : Y coordinate of cursor
 */
int getCursorY()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_CURSOR_Y_LOW, 1, &data0);
  readRegister(PAJ7620_ADDR_CURSOR_Y_HIGH, 1, &data1);
  data1 &= 0x0F;      // Mask off high bits (unused)
  result |= data1;
  result = result << 8;
  result |= data0;

  return result;
}


/**
 * Returns whether an object is in view as a cursor
 * 
 * \note Only works in cursor mode
 * \param none
 * \return bool : True if object in view, False if no object in view
 */
bool isCursorInView()
{
  bool result = false;
  uint8_t data = 0x00;
  readRegister(PAJ7620_ADDR_CURSOR_INT, 1, &data);
  switch(data)
  {
    case CUR_NO_OBJECT:   result = false;   break;
    case CUR_HAS_OBJECT:  result = true;    break;
    default:              result = false;   break;
  }
  return result;
}


/**
 * Inverts the X (horizontal) axis
 * 
 * \par
 * Allows you to choose the orientation of your coordinate system.
 * In all modes, the X axis is inverted. Left becomes Right, etc.
 * For cursor mode, the X values will flip
 * 
 * \param none
 * \return none
 */
void invertXAxis()
{
  uint8_t data = 0x00;
  selectRegisterBank(BANK1);
  readRegister(PAJ7620_ADDR_LENS_ORIENTATION, 1, &data);
  data ^= 1UL << 0;               // Bit[0] controls X axis
  writeRegister(PAJ7620_ADDR_LENS_ORIENTATION, data);
  selectRegisterBank(BANK0);
}


/**
 * Inverts the Y (vertical) axis
 * 
 * \par
 * Allows you to choose the orientation of your coordinate system.
 * In all modes, the Y axis is inverted. Up becomes Down, etc.
 * For cursor mode, the Y values will flip
 * 
 * \param none
 * \return none
 */
void invertYAxis()
{
  uint8_t data = 0x00;
  selectRegisterBank(BANK1);
  readRegister(PAJ7620_ADDR_LENS_ORIENTATION, 1, &data);
  data ^= 1UL << 1;                 // Bit[1] controls Y axis
  writeRegister(PAJ7620_ADDR_LENS_ORIENTATION, data);
  selectRegisterBank(BANK0);
}


/**
 * Disables sensor for reading & interrupts
 * \note This is the light disable state, not the full I2C shutdown state
 * \param none
 * \return none
 */
void disablePAJsensor()
{
  selectRegisterBank(BANK1);
  writeRegister(PAJ7620_ADDR_OPERATION_ENABLE, PAJ7620_DISABLE);
  selectRegisterBank(BANK0);
}


/**
 * Enables sensor for reading & interrupts
 * 
 *  \param none
 *  \return none
 */
void enablePAJsensor()
{
  selectRegisterBank(BANK1);
  writeRegister(PAJ7620_ADDR_OPERATION_ENABLE, PAJ7620_ENABLE);
  selectRegisterBank(BANK0);
}

/**
 * Sets time sensor waits between getGesture call to reading gesture from sensor
 * \par
 *  This time is most important in hardware interrupt driven use of the driver.
 *  The PAJ7620's interrupt pin will raise when a gesture is first recognized.
 *  If the user is trying to move their hand to do a Backward gesture, they will
 *  first trip a lateral (up, down, left, right) gesture, which will immediately
 *  raise the interrupt.
 *  By increasing this value, the user shall have more time to reach in and complete
 *  their intended gesture before the interrupt is handled.
 * \note Default value for entry time is 0
 * \param newGestureEntryTime : milliseconds (ms) for delay
 * \return none
 */
void setGestureEntryTime(unsigned long newGestureEntryTime)
{
  gestureEntryTime = newGestureEntryTime;
}


/**
 * Sets time sensor waits during getGesture() after gesture value read
 * \par
 *  This value represents the time the user has to exit the sensor's field of view
 *  before the next gesture might be read, which is most important in the Z axis gestures
 *  (forward and backward).
 *  Setting this lower makes the driver delay less so the main program can control
 *  more of the global timing, but puts responsibility on the coder to take this higher
 *  sensitivity into account.
 * \note Default value for exit time is 200
 * \param newGestureEntryTime : milliseconds (ms) for delay
 * \return none
 */
void setGestureExitTime(unsigned long newGestureExitTime)
{
  gestureExitTime = newGestureExitTime;
}


/**
 * Set sensor to "game mode" sampling speed of 240fps
 * \note Value of 0x30 for setting comes from PixArt contact
 *
 * \param none
 * \return none
 */
void setGameSpeed()
{
  selectRegisterBank(BANK1);
  writeRegister(PAJ7620_ADDR_R_IDLE_TIME_0, PAJ7620_GAME_SPEED);
  selectRegisterBank(BANK0);
}


/**
 * Set sensor to "normal" sampling speed of 120fps
 * \note Value of 0xAC for setting comes from PixArt contact
 *
 * \param none
 * \return none
 */
void setNormalSpeed()
{
  selectRegisterBank(BANK1);
  writeRegister(PAJ7620_ADDR_R_IDLE_TIME_0, PAJ7620_NORMAL_SPEED);
  selectRegisterBank(BANK0);
}


/**
 * Clear current gesture interrupt vectors without returning gesture value
 * \note The gesture interrupt vectors are reset in hardware after any reads
 *
 * \param none
 * \return none
 */
void clearGestureInterrupts()
{
    uint8_t data = 0, data1 = 0;
    getGesturesReg0(&data);
    getGesturesReg1(&data1);
}


/**
 * Get current count of waves by user
 * \param none
 * \return int : current count of "waves" over the sensor
 */
int getWaveCount()
{
  uint8_t waveCount = 0;
  readRegister(PAJ7620_ADDR_WAVE_COUNT, 1, &waveCount);
  waveCount &= 0x0F;      // Count is [3:0] bits - values in 0..15
  return waveCount;
}


/**
 * Double check to see if user is executing a Z-axis gesture 
 * 
 * \par
 *  This is there the gestureEntryTime and gestureExitTime delays are executed
 *  to buffer high speed polling & return against human gesture speeds.
 * \param initialGesture : The gesture initially found when getGesture() was called
 * \return \link Gesture \endlink : Either the initialGesture or the updated one if the user does another one
 */
Gesture forwardBackwardGestureCheck(Gesture initialGesture)
{
  uint8_t data1 = 0;
  Gesture result = initialGesture;

  //delay(gestureEntryTime);
  HAL_Delay(gestureEntryTime);
  getGesturesReg0(&data1);
  if (data1 == GES_FORWARD_FLAG)
  {
    //delay(gestureExitTime);
	HAL_Delay(gestureExitTime);
    result = GES_FORWARD;
  }
  else if (data1 == GES_BACKWARD_FLAG)
  {
    //delay(gestureExitTime);
	HAL_Delay(gestureExitTime);
    result = GES_BACKWARD;
  }
  return result;
}


/**
 * Reads the latest gesture from the device
 * 
 * \par
 *  This is the central method for reading and calculating the main 9 gestures
 *  the PAJ7620 can recognize. It returns a Gesture enum with the read gesture,
 *  which can by GES_NONE if no gesture was currently found.
 * \note Clears interrupt vector of gestures when called
 * \param none
 * \return \link Gesture \endlink found or \link GES_NONE Gesture::GES_NONE \endlink if no gesture found
 */
Gesture readGesture()
{
  uint8_t data = 0, data1 = 0, readCode = 0;
  Gesture result = GES_NONE;

  readCode = getGesturesReg0(&data);
  if (readCode)
  {
    return GES_NONE;
  }
  else
  {
    switch (data)
    {
      case GES_RIGHT_FLAG:
        result = forwardBackwardGestureCheck(GES_RIGHT);
        break;

      case GES_LEFT_FLAG:
        result = forwardBackwardGestureCheck(GES_LEFT);
        break;

      case GES_UP_FLAG:
        result = forwardBackwardGestureCheck(GES_UP);
        break;

      case GES_DOWN_FLAG:
        result = forwardBackwardGestureCheck(GES_DOWN);
        break;

      case GES_FORWARD_FLAG:
        //delay(gestureExitTime);
        HAL_Delay(gestureExitTime);
        result = GES_FORWARD;
        break;

      case GES_BACKWARD_FLAG:
        //delay(gestureExitTime);
    	HAL_Delay(gestureExitTime);
        result = GES_BACKWARD;
        break;

      case GES_CLOCKWISE_FLAG:
        result = GES_CLOCKWISE;
        break;

      case GES_ANTI_CLOCKWISE_FLAG:
        result = GES_ANTICLOCKWISE;
        break;

      default:
        getGesturesReg1(&data1);      // Bank 1 (Reg 0x44) has wave flag
        if (data1 == GES_WAVE_FLAG)
          { result = GES_WAVE; }
        break;
    }
  }
  return result;
}


/**
 * Read object's "brightness"
 * 
 * \par
 * Objects in view have their IR reflection measured. This interface returns a
 *  measure of this brightness from 0..255
 * \return int brightness: brightness value 0..255
 */
int getObjectBrightness()
{
  uint8_t brightness = 0x00;
  readRegister(PAJ7620_ADDR_OBJECT_BRIGHTNESS, 1, &brightness);
  return brightness;
}


/**
 * Read object's size (in pixels)
 * 
 * \par
 * The sensor has a 30x30 IR LED array. This interface returns a count of
 *  how many pixels are part of the object in view being tracked.
 * \return int size: value 0..900
 */
int getObjectSize()
{
  uint8_t data0, data1 = 0x00;
  int result = 0;
  readRegister(PAJ7620_ADDR_OBJECT_SIZE_LSB, 1, &data0);
  readRegister(PAJ7620_ADDR_OBJECT_SIZE_MSB, 1, &data1);
  result = data1;
  result = result << 8;
  result |= data0;
  return result;
}


/**
 * Get how long since the object left the view in gesture mode
 * 
 * \par
 * When an object has left the sensor's view, this register starts counting up.
 *  It's counting in ticks, roughly one per 7.2ms. It maxes out at 255, which
 *  happens at about 1830ms
 * \return int : ticks value 0..255
 */
int getNoObjectCount()
{
  uint8_t data0 = 0x00;
  readRegister(PAJ7620_ADDR_NO_OBJECT_COUNT, 1, &data0);
  return (int)data0;
}


/**
 * Get how long no motion has been seen in gesture mode
 * 
 * \par
 * This counts how long it has been since motions has occurred in front of the sensor.
 * This counts even if there's an object in view when it isn't moving.
 * Eratta: This *should* return 0..255, but seems to stop at 12.
 * Each "count" is probably 7.2ms, but it's been tough to figure out.
 * \return int : ticks value 0..12
 */
int getNoMotionCount()
{
  uint8_t data0 = 0x00;
  readRegister(PAJ7620_ADDR_NO_MOTION_COUNT, 1, &data0);
  return (int)data0;
}


/**
 * Gets Gesture object's current X location
 * 
 * \note Only works in gesture mode - default coordinates are 0 on right
 * \note Range seems to be 0..3712 in default gesture mode
 * \param none
 * \return int : X coordinate of cursor
 */
int getObjectCenterX()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_OBJECT_CENTER_X_LSB, 1, &data0);
  readRegister(PAJ7620_ADDR_OBJECT_CENTER_X_MSB, 1, &data1);
  data1 &= 0x1F;      // Mask off high bits (unused)
  result |= data1;
  result = result << 8;
  result |= data0;

  return result;
}


/**
 * Gets Gesture object's current Y location
 * 
 * \note Only works in gesture mode - default coordinates are 0 on top
 * \note Range seems to be 0..3712 in default gesture mode
 * \param none
 * \return int : Y coordinate of cursor
 */
int getObjectCenterY()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_OBJECT_CENTER_Y_LSB, 1, &data0);
  readRegister(PAJ7620_ADDR_OBJECT_CENTER_Y_MSB, 1, &data1);
  data1 &= 0x1F;      // Mask off high bits (unused)
  result |= data1;
  result = result << 8;
  result |= data0;

  return result;
}


/**
 * Gets object's current X velocity's raw value
 * 
 * \note Range seems to be -63..63
 * \param none
 * \return int : X velocity -63..63
 */
int getObjectVelocityX_raw()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_OBJECT_VEL_X_LSB, 1, &data0);
  readRegister(PAJ7620_ADDR_OBJECT_VEL_X_MSB, 1, &data1);

  data0 &= 0x3F;        // Yup, see wiki for reason
  result = data0;
  if(data1) { result *= -1; }

  return result;
}


/**
 * Gets object's current Y velocity's raw value
 * 
 * \note Range seems to be -63..63
 * \param none
 * \return int : Y velocity -63..63
 */
int getObjectVelocityY_raw()
{
  int result = 0;
  uint8_t data0 = 0x00;
  uint8_t data1 = 0x00;

  readRegister(PAJ7620_ADDR_OBJECT_VEL_Y_LSB, 1, &data0);
  readRegister(PAJ7620_ADDR_OBJECT_VEL_Y_MSB, 1, &data1);
  data0 &= 0x3F;        // Yup, see wiki for reason
  result = data0;
  if(data1) { result *= -1; }

  return result;
}


/**
 * Gets object's current X velocity's value
 * 
 * \par
 * Value filtered to zero if object not in view
 * \note Range seems to be -63..63
 * \param none
 * \return int : X velocity -63..63
 */
int getObjectVelocityX()
{
  if(!isObjectInView()) {
    return 0;
  } else {
    return getObjectVelocityX_raw();
  }
}


/**
 * Gets object's current Y velocity's value
 * 
 * \par
 * Value filtered to zero if object not in view
 * \note Range seems to be -63..63
 * \param none
 * \return int : Y velocity -63..63
 */
int getObjectVelocityY()
{
  if(!isObjectInView()) {
    return 0;
  } else {
    return getObjectVelocityY_raw();
  }
}


/**
 * Gets whether an object is in view or not
 * 
 * \param none
 * \return bool : true if object in view
 */
bool isObjectInView()
{
  if(getNoObjectCount())
  {
    return false;
  }
  return true;
}


/**
 * Gets which quadrant an object is in
 * 
 * \param none
 * \return Corner : [NW, NW, SW, SE] quadrants, middle/buffer, NONE for no object in view
 */
Corner getCorner()
{
  Corner ret = CORNER_NONE;
  int object_x, object_y = 0;

  if( !isObjectInView() ) {   // Bail if no object in view
    return CORNER_NONE;
  }

  object_x = getObjectCenterX();
  object_y = getObjectCenterY();

  if( object_x < CORNERS_BUFFER_LOWER && object_y < CORNERS_BUFFER_LOWER ) {
    return CORNER_NE;
  }
  else if( object_x > CORNERS_BUFFER_UPPER && object_y < CORNERS_BUFFER_LOWER ) {
    return CORNER_NW;
  }
  else if( object_x > CORNERS_BUFFER_UPPER && object_y > CORNERS_BUFFER_UPPER ) {
    return CORNER_SW;
  }
  else if( object_x < CORNERS_BUFFER_LOWER && object_y > CORNERS_BUFFER_UPPER ) {
    return CORNER_SE;
  }
  else {
    return CORNER_MIDDLE;     // Is in view, but not fully in a corner yet
  }
}



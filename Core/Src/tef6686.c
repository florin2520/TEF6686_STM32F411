/*
 * tef6686.c
 *
 *  Created on: Jan 28, 2023
 *      Author: nicaf
 */


//TEF6686HN/V102 xdr-gtk控制程序
//作者 eggplant886
//rds add by stailus

#include "tef6686.h"
#include <stdarg.h>
#include "i2c.h"
#include "dsp_init.h"

#include "usart.h"


char str2[30];
//#define pgm_read_byte_near(address_short) __LPM((uint16_t)(address_short))
//#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

char rdsRadioText[65];

char rdsProgramService[9];

uint8_t rdsAb;
char rdsProgramType[17];

uint8_t isRdsNewRadioText;

uint16_t seekMode;
uint16_t seekStartFrequency;

uint8_t isRDSReady;
struct RdsInfo rdsInfo;


char programTypePrevious[17] = "                ";
char programServicePrevious[9];
char radioTextPrevious[65];



const char* ptyLUT[51] = {
      "      None      ",
      "      News      ",
      "  Information   ",
      "     Sports     ",
      "      Talk      ",
      "      Rock      ",
      "  Classic Rock  ",
      "   Adult Hits   ",
      "   Soft Rock    ",
      "     Top 40     ",
      "    Country     ",
      "     Oldies     ",
      "      Soft      ",
      "   Nostalgia    ",
      "      Jazz      ",
      "   Classical    ",
      "Rhythm and Blues",
      "   Soft R & B   ",
      "Foreign Language",
      "Religious Music ",
      " Religious Talk ",
      "  Personality   ",
      "     Public     ",
      "    College     ",
      " Reserved  -24- ",
      " Reserved  -25- ",
      " Reserved  -26- ",
      " Reserved  -27- ",
      " Reserved  -28- ",
      "     Weather    ",
      " Emergency Test ",
      "  !!!ALERT!!!   ",
      "Current Affairs ",
      "   Education    ",
      "     Drama      ",
      "    Cultures    ",
      "    Science     ",
      " Varied Speech  ",
      " Easy Listening ",
      " Light Classics ",
      "Serious Classics",
      "  Other Music   ",
      "    Finance     ",
      "Children's Progs",
      " Social Affairs ",
      "    Phone In    ",
      "Travel & Touring",
      "Leisure & Hobby ",
      " National Music ",
      "   Folk Music   ",
      "  Documentary   "};

/*
  by Eustake (marsel90)
  With this sketch it is recommended to use together with TEF-GTK v1.1.2
  changes:
  Sketch compatible with NXP-TEF6686 (F8602 or F8605)
  Add = MPX out mode ( DAC_Left : FM MPX (DARC) signal / DAC_Right : mono audio ) + MPX obtion in TEF-GTK
  Check signal level and 19kHz subcarrier every TIMER_INTERVAL. Now in TEF-GTK appear MO for mono and ST for stereo.
  Without IF+ and RF+ - RF Gain is 0db
  Only RF+ - RF Gain max +6db
  Only IF+ - RF Gain max +9db
  Both IF+ & RF+ - RF Gain max +15db
  ChannelEqualizer obtion - on/off use TEF-GTK button EQ
  MphSuppression iMS obtion - on/off use TEF-GTK button iMS
  +6dB analog radio sound gain - in TEF-GTK enable Show antenna input alignment - NO AM
  #
  by VoXiTPro
  changes:
  all warnings are removed  (unsigned int to signed int did go wrong)
  Filters are now switchable (AUTO is max 236khz, manual you can set 311Khz)
  Stereo on/off
  Signal measurement improvements
  Keep filter (to do) and frequency settings when switching from AM to FM / FM to AM
  Settings are now from Eustake (marsel90)
  AGC is now switchable (now for FM/AM and reversed the settings)
  Removed some settings from Eustake, FMSI stereo improvement only for tef6687 and tef6689, Softmute_mod AM only, Wavegen and I2S audio (only using internal audio)
  You can now change settings with the antenne switch ANT A = default settings, ANT B = Improved settings, ANT C = Eustake Settings
  Settings are now A,B,C from eustake. D are improved settings from prog manual.
  AGC, Deemphasis, IF+ RF+ are now in subroutines so you can set them at other moments. If you change settings the old values will be reapplied.
  IF+ RF+ works now also for AM.
  Squelch will change volume scale for AM.  if you have the right value change the define AM_VOL_SCALE too that value (now -1)
*/
/*
 If you have changed the above value squelch will be off again.
 Switching from AM to FM or from FM to AM filter settings are kept
  Thanks also to:
  - FMDXklaas for testing
  - ODJeetje
  - Eustake for default and improved settings
  and many others ...
  Tested on Arduino Nano V3.0 at 5V
*/

uint8_t buff_pos = 0;
uint32_t REG_FREQ;
uint32_t MODA_FREQ = 0;
uint32_t MODF_FREQ = 0;
uint32_t timer = 0;          // Signal level reporting timer
uint32_t timer_RDS = 0;      // RDS reporting timer
int8_t current_filter = -1;  // Current FIR filter (-1 is adaptive)
int8_t current_set = -1;     // Current FIR filter (-1 is adaptive)
int8_t forced_mono;
int16_t LevelOffset;
int16_t Level;
int16_t AudioLevel;
int16_t LevelA;
int8_t mode;
int8_t AGC_tress;
int8_t IFplus;
int8_t RFplus;
int8_t Squelch;
int8_t Setsquelch;
int16_t nDeemphasis, volume;
uint32_t freq;
int8_t Filter_AM = 16;
int8_t Filter_FM = 16;
int8_t radio_mode;
int8_t scan_mode;

/* Scan */
uint16_t scan_start = 0;
uint16_t scan_end = 0;
uint16_t scan_step = 0;
uint8_t scan_filter = 0;
uint16_t AM_start_scan = 0;
uint16_t AM_scan_end = 0;
uint16_t AM_scan_step = 0;
uint8_t AM_scan_filter = 0;

uint8_t SeekSenLevel = LOW;
/* station */
StationMemType StationRecord[MaxStationNum];

/* area config*/
#define AreaSelect Radio_EUR

#if (AreaSelect==Radio_CHN)   /*China */
const Radio_AreaConfigDef Radio_AreaConfig={
    //FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
    10800,        6500,      1620,       522,         FM_Step_100k,        FM_Step_100k,                 9,                  9};

const FreqBaundDef FreqBaundConfig[MaxBandNum]={
   /*0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW*/
    {6500,10800},{6500,10800},{6500,10800},{522,1710},{144,288},{2300,27000}};

#elif (AreaSelect==Radio_EUR)  /*Europe */
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
10800,        6500,      1602,       531,         FM_Step_100k,        FM_Step_100k,               9,                  9};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    //

#elif (AreaSelect==Radio_USA)      /*USA */
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
10790,        8750,      1710,       530,        FM_Step_100k,        FM_Step_100k,         10,                  10};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    //

#elif (AreaSelect==Radio_JPN)      /*Japan */
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
9000,        7600,      1629,       522,          FM_Step_100k,        FM_Step_100k,               9,                   9};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    //

#endif


/*check station step*/
static uint8_t CheckIfStep;

/*current radio  band*/
uint8_t Radio_CurrentBand;
/*current radio  freqency*/
uint16_t Radio_CurrentFreq;
/*current radio  station*/
uint8_t Radio_CurrentStation;

#ifdef RADIO_D3_ENABLE
eDev_Type RadioDev = Radio_Dirana3;
#elif defined RADIO_LITHIO_ENABLE
eDev_Type RadioDev = Radio_Lithio;
#elif defined RADIO_ATOMIC_ENABLE
eDev_Type RadioDev = Radio_Atomic;
#elif defined RADIO_ATOMIC2_ENABLE
eDev_Type RadioDev = Radio_Atomic2;
#elif defined RADIO_HELIO_ENABLE
eDev_Type RadioDev = Radio_Helio;
#else
eDev_Type RadioDev = Radio_Hero;
#endif

#define SERIAL_PORT_SPEED 115200
#define SERIAL_BUFFER_SIZE 16
char buff[SERIAL_BUFFER_SIZE];


#define TIMER_INTERVAL 66
#define RDS_TIMER_INTERVAL 87
#define AM_VOL_SCALE -1
uint8_t DSP_I2C = (0x64U<<1);

/* Buffer used for transmission */
uint8_t aTxBuffer[10];

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


//TODO DMA
void Write(uint8_t *buf, uint8_t len)
{
  //Wire.beginTransmission(DSP_I2C);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DSP_I2C, (uint8_t*)buf, len, 1500);

  //for (int i = 0; i < len; i++)
    //Wire.write(*buf++);
  //Wire.endTransmission();
}

//TODO DMA
void Read(uint8_t *buf, uint8_t len)
{
  //uint8_t lenrec = Wire.requestFrom(DSP_I2C, len);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DSP_I2C, (uint8_t *)buf, len, 1500);
  //for (int i = 0; i < lenrec; i++)
    //*buf++ = Wire.read();
}



void Set_Cmd(uint8_t mdl, uint8_t cmd, int len, ...)
{
  uint8_t buf[31];
  //uint16_t temp;
  uint32_t temp;
  va_list vArgs;
  va_start(vArgs, len);
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  for (uint8_t i = 0; i < len; i++)
  {
    //temp = va_arg(vArgs, uint16_t);
	//temp = va_arg(vArgs, int);
	temp = va_arg(vArgs, uint32_t);
    buf[3 + i * 2] = (uint8_t)(temp >> 8);
    buf[4 + i * 2] = (uint8_t)temp;
  }
  va_end(vArgs);
  Write(buf, len * 2 + 3);
}

void Get_Cmd(uint8_t mdl, uint8_t cmd, int16_t *receive, int len)
{
  uint8_t buf[3];
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  Write(buf, 3);
  Read((uint8_t*)receive, 2 * len);
  for (uint8_t i = 0; i < len; i++)
  {
    uint16_t newval = (uint8_t)(receive[i] >> 8) | (((uint8_t)(receive[i])) << 8);
    receive[i] = newval;
  }
}

void dsp_write_data(const uint8_t* data)
{
  uint8_t *pa = (uint8_t *)data;
  uint8_t len, first;
  uint8_t data_to_send[45];
  for (;;)
  {
	  len = pgm_read_byte(pa++);
//	  if (len == 45)  // ???
//	     break;
	  first = pgm_read_byte(pa);
    if (!len)
      break;

    if (len == 2 && first == 0xff)
    {
      int delaytime = pgm_read_byte(++pa);
      HAL_Delay(delaytime);
      pa++;
    }
    else
    {
      for (int i = 0; i < len; i++)
      {
    	  data_to_send[i] = pgm_read_byte(pa++);
      }
	  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DSP_I2C, data_to_send, len, 1500);
	  memset(data_to_send, '\0', 45);
    }
  }
}

//void scan(bool continous)
//{
//  uint32_t freq;
//  uint32_t buffer;
//
//  if ( scan_start < 6500 )
//  {
//    scan_mode = 1;
//    scan_start = AM_start_scan;
//    scan_end = AM_scan_end;
//    scan_step = AM_scan_step;
//  }
//  else
//  {
//    scan_mode = 0;
//  }
//
//  if (scan_mode == 0)
//  {
//  Set_Cmd(32, 1, 2, 1, scan_start);
//  }
//  else
//  {
//    Set_Cmd(33, 10, 2, scan_filter == -1 ? 1 : 0, pgm_read_byte_near(AMFilterMap + scan_filter));
//    Set_Cmd(33, 1, 2, 1, scan_start);
//  }
//  do
//  {
//    //Serial.print('U');
//    for (freq = scan_start; freq <= scan_end; freq += scan_step)
//    {
//      Set_Cmd( scan_mode == 0 ? 32 : 33, 1, 2, 1, freq);
//      //Serial.print(scan_mode == 0 ? freq * 10 : freq, DEC);
//      //Serial.print('=');
//      delay(10);
//
//      int16_t uQuality[4] = { 0 };
//      Get_Cmd(scan_mode == 0 ? 32 : 33, 128, uQuality, 4);
//      //Serial.print(uQuality[1] / 10, DEC);
//      //Serial.print(',');
//    }
//
//    //Serial.print('\n');
//  } while (continous && !Serial.available());
//  if (radio_mode == 0)
//  {
//    Set_Cmd( 32, 1, 2, 1, REG_FREQ / 10 );
//  }
//  else
//  {
//    Set_Cmd( 33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
//    Set_Cmd( 33, 1, 2, 1, REG_FREQ );
//  }
//}

//void get_RDS()
//{
//  int16_t uRDS_RDS[8] = {0};
//  Get_Cmd(32, 131, uRDS_RDS, 8);
//  print_serial2_message("get_RDS1");
//  if ( bitRead(uRDS_RDS[0], 15) == 1 )
//  {
//	//print_serial2_message("get_RDS if");
//    //Serial.print('P');
//	print_serial2_message("P");
//    serial_hex(uRDS_RDS[1] >> 8);
//    serial_hex(uRDS_RDS[1]);
//    //Serial.print('\n');
//    //Serial.print('R');
//    print_serial2_message("\nR");
//
//    serial_hex(uRDS_RDS[1] >> 8);
//    serial_hex(uRDS_RDS[2] >> 8);
//    serial_hex(uRDS_RDS[2]);
//    serial_hex(uRDS_RDS[3] >> 8);
//    serial_hex(uRDS_RDS[3]);
//    serial_hex(uRDS_RDS[4] >> 8); /// aici este denumirea postului R A T A
//    serial_hex(uRDS_RDS[4]);      //  aici este denumirea postului R C U L
//    serial_hex(uRDS_RDS[5] >> 8);
////    sprintf(str2, "%s\n\r", (char)uRDS_RDS);
////    HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
//    //Serial.print('\n');
//    print_serial2_message("\n");
//  }
//}

void serial_hex(uint8_t val)
{
  //Serial.print((val >> 4) & 0xF, HEX);
  //Serial.print(val & 0xF, HEX);
	//print_serial2_message_number("RDS1= ", ((val >> 4) & 0xF));
	//print_serial2_message_number("RDS2= ", (val & 0xF));

	print_serial2_message_number("RDS= ", val);
	//print_serial2_message_number("RDS2= ", (val & 0xF));
}

void Set_AGC_tresshold(uint8_t val)
{
 if (val == 0) {
  Set_Cmd(32, 11, 2, 920, 0);
  Set_Cmd(33, 11, 1, 1020);
 } else if (val == 1) {
  Set_Cmd(32, 11, 2, 890, 0);
  Set_Cmd(33, 11, 1, 990);
 } else if (val == 2) {
  Set_Cmd(32, 11, 2, 870, 0);
  Set_Cmd(33, 11, 1, 970);
 } else {
  Set_Cmd(32, 11, 2, 840, 0);
  Set_Cmd(33, 11, 1, 940);
 }
}

void Set_IF_RF(uint8_t val, uint8_t val1)
{
  if (val == 1 && val1 == 0) {
     Set_Cmd(32, 39, 1, 60);
     Set_Cmd(33, 39, 1, 60);
  } else if (val == 0 && val1 == 1) {
     Set_Cmd(32, 39, 1, 90);
     Set_Cmd(33, 39, 1, 60);
  } else if (val == 1 && val1 == 1) {
     Set_Cmd(32, 39, 1, 150);
     Set_Cmd(33, 39, 1, 120);
  } else {
     Set_Cmd(32, 39, 1, 0);
     Set_Cmd(33, 39, 1, 0);
  }
}

void Set_Deempasis(uint8_t val)
{
  if (val == 0) {
    Set_Cmd(32, 31, 1, 500); //50 us
    Set_Cmd(32, 85, 1, 0);
  } else  if (val == 1) {
    Set_Cmd(32, 31, 1, 750); //75 us
    Set_Cmd(32, 85, 1, 0);
  } else if (val == 2) {
    Set_Cmd(32, 31, 1, 0);  //0 us
    Set_Cmd(32, 85, 1, 0);
  } else if (val == 3) {
    Set_Cmd(32, 85, 1, 1); //MPX out mode ( DAC_Left : FM MPX (DARC) signal / DAC_Right : mono audio );
  }
}

void setup()
{
  //Wire.begin();
  //Serial.begin(115200);
  //delay(40);
  HAL_Delay(50);
  int16_t uState;
  Get_Cmd(64, 128, &uState, 1);
  if (uState < 2)
    dsp_write_data(DSP_INIT);
  else if (uState > 2)
  {
    Set_Cmd(64, 1, 1, 1);
  }

  HAL_Delay(20); //TODO pe analizor nu apare ??
//  if (AM_VOL_SCALE != -1)
//  {
//    int SetVolScale = map(AM_VOL_SCALE, 0, 100, -120, 60);
//    Set_Cmd(33, 80, 1, SetVolScale);
//  }
//

  // set volume
    volume = 130;
    Set_Cmd(48, 11, 1, 0);  //unmute
    int Setvolume = map(volume, 0, 100, -599, 50);
    Set_Cmd(48, 10, 1, Setvolume);

  // set freq
    //MODF_FREQ = 104500;
    //MODF_FREQ = 88700;
    //MODF_FREQ = 103600;
    //MODF_FREQ = 98000;
    //MODF_FREQ = 90600;
    //Set_Cmd(32, 1, 2, 1, MODF_FREQ / 10);

   // if (Filter_FM == 16) {Filter_FM = -1;}
   // current_filter = Filter_FM;
    //Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
   // Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word(FMFilterMap + current_filter), 1000, 1000);
}

//void loop()
//{
//  // check signal level and 19kHz subcarrier every TIMER_INTERVAL
////  if ((millis() - timer) >= TIMER_INTERVAL)
////  {
////    if (radio_mode == 0) {
////      Serial.print('S');
////      uint16_t uStatus;
////      Get_Cmd(32, 133, &uStatus, 1);
////      char stereo ;
////        if(!forced_mono)
////          stereo = (uStatus & (1 << 15)) ? 's' : 'm';
////        else
////          stereo = (uStatus & (1 << 15)) ? 'S' : 'M';
////        Serial.print(stereo);
////      int16_t uQuality[4] = { 0 };
////      if (radio_mode == 0){
////      Get_Cmd(32, 128, uQuality, 6);}
////      if (uQuality[1] > 1200){uQuality[1] = 1200;}
////      Serial.print(uQuality[1] / 10, DEC);
////      Serial.print(',');
////      if ((uQuality[2] > 1000) && (radio_mode ==0)) {uQuality[2] = 1000;}
////      Serial.print(uQuality[2] / 10, DEC);
////      Serial.print(',');
////      Serial.print(uQuality[3] / 10, DEC);
////      Serial.print('\n');
////      timer = millis();
////     }
////     else if (radio_mode == 1) {
////      Serial.print("Sm");
////      int16_t uQuality[4] = { 0 };
////      Get_Cmd(33, 128, uQuality, 4);
////      Serial.print(uQuality[1] / 10, DEC);
////      Serial.print(',');
////      if (radio_mode ==1){uQuality[2] = uQuality[2] / 5;}
////      Serial.print(uQuality[2] / 10, DEC);
////      Serial.print(',');
////      Serial.print(uQuality[3] / 10, DEC);
////      Serial.print('\n');
////      timer = millis();
////    }
////  }
//
////  if (radio_mode == 0) {
////    if ((millis() - timer_RDS) >= RDS_TIMER_INTERVAL)
////    {
////      get_RDS();
////      timer_RDS = millis();
////    }
////  }
//
//  if (Serial.available() > 0)
//  {
//    buff[buff_pos] = Serial.read();
//    if (buff[buff_pos] != '\n' && buff_pos != SERIAL_BUFFER_SIZE - 1)
//      buff_pos++;
//    else {
//      buff[buff_pos] = 0x00;
//      buff_pos = 0;
//
//      switch (buff[0])
//      {
//
//        case 'x':
//          Serial.println("OK");
//          break;
//
//        case 'Y':  // Audio volume scaler
//          volume = atoi(buff + 1);
//          if (volume == 0) {
//            Set_Cmd(48, 11, 1, 1);  //mute
//          } else {
//            Set_Cmd(48, 11, 1, 0);  //unmute
//            int Setvolume = map(volume, 0, 100, -599, 50);
//            Set_Cmd(48, 10, 1, Setvolume);
//          }
//         // Serial.print("Y");
//          //Serial.println(volume);
//          break;
//
//        case 'S':
//          if (buff[1] == 'a')
//          {
//            scan_start = (atol(buff + 2) + 5) / 10;
//            AM_start_scan = atol(buff + 2);
//          }
//          else if (buff[1] == 'b')
//          {
//            scan_end = (atol(buff + 2) + 5) / 10;
//            AM_scan_end = atol(buff + 2);
//          }
//          else if (buff[1] == 'c')
//          {
//            scan_step = (atol(buff + 2) + 5) / 10;
//            AM_scan_step = atol(buff + 2);
//          }
//          else if (buff[1] == 'f')
//          {
//            scan_filter = atol(buff + 2);
//          }
//          else if (scan_start > 0 && scan_end > 0 && scan_step > 0 && scan_filter >= 0)
//          {
//            if (buff[1] == 'm')
//              scan(true);   // Multiple (continous) scan
//            else
//              scan(false);  // Single scan
//          }
//          break;
//
//        case 'B':
//          forced_mono = atol(buff + 1);
//          if(forced_mono == 1)
//          Set_Cmd(32, 66, 1, 2, forced_mono == 1 ? 2 : 0 , 60);
//          else
//          Set_Cmd(32, 66, 1, 0, forced_mono == 1 ? 2 : 0 , 200);
//          //Serial.print('B');
//          //Serial.print(forced_mono ? '1' : '0');
//          //Serial.print('\n');
//          break;
//        case 'N':
//          break;
//
//        case 'M':
//          mode = atol(buff + 1);
//          if (mode == 0){
//            if (MODF_FREQ == 0){MODF_FREQ = 87500;}
//            Set_Cmd(32, 1, 2, 1, MODF_FREQ / 10);
//            //Serial.println("M0");
//            //Serial.print('T');
//            //Serial.println(MODF_FREQ);
//            if (Filter_AM != 16) {Filter_AM = current_filter;}
//			      if (Filter_FM == 16) {Filter_FM = -1;}
//			      current_filter = Filter_FM;
//            Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
//            radio_mode = 0;
//          } else {
//            if (MODA_FREQ == 0){MODA_FREQ = 558;}
//            Set_Cmd(33, 1, 2, 1, MODA_FREQ);
//            //Serial.println("M1");
//            //Serial.print('T');
//            //Serial.println(MODA_FREQ);
//			      Filter_FM = current_filter;
//	          if (Filter_FM != 16) {Filter_FM = current_filter;}
//   	        if (Filter_AM == 16) {Filter_AM = 1;}
//            current_filter = Filter_AM;
//            Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
//            radio_mode = 1;
//          }
//          //Serial.print('F');
//          //Serial.println(current_filter);
//          //Serial.print('V');
//          //Serial.println(LevelOffset);
//          break;
//
//         case 'T':  // Tune to a frequency (to do keep filter settings)
//          freq = atol(buff + 1);
//          REG_FREQ = freq;
//          if ((REG_FREQ >= 65000) && (REG_FREQ <= 108000)) {
//            Set_Cmd(32, 1, 2, 1, REG_FREQ / 10);
//            Serial.println("M0");
//            Serial.print('T');
//            Serial.println(REG_FREQ);
//            if (radio_mode == 1) {                         // from AM to FM then switch to adaptive filter AUTO in TEF-GTK
//              if (Filter_AM != 16) {Filter_AM = current_filter;}
//             if (Filter_FM == 16) {Filter_FM = -1;}
//              current_filter = Filter_FM;
//              Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
//            }
//            radio_mode = 0;
//            MODF_FREQ = REG_FREQ;
//          } else if ((REG_FREQ >= 144) && (REG_FREQ <= 26999)) {
//              Set_Cmd(33, 1, 2, 1, REG_FREQ);
//              //Serial.println("M1");
//              //Serial.print('T');
//              //Serial.println(REG_FREQ);
//              if (radio_mode == 0) {                       // from FM to AM then use filter 1 -> 4khz
//                if (Filter_FM != 16) {Filter_FM = current_filter;}
//                if (Filter_AM == 16) {Filter_AM = 1;}
//                current_filter = Filter_AM;
//                Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
//              }
//              radio_mode = 1;
//              MODA_FREQ = REG_FREQ;
//          }
//          //Serial.print('F');
//          //Serial.println(current_filter);
//          //Serial.print('V');
//          //Serial.println(LevelOffset);
//          break;
//
//        case 'F':  // Change FIR filters
//          current_filter = atoi(buff + 1);
//          if (radio_mode == 0) {
//            Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
//          }
//          else {
//            Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
//          }
//          //Serial.print('F');
//          //Serial.println(buff + 1);
//          break;
//
//        case 'V':
//          LevelOffset = atoi(buff + 1);
//          Level = map(LevelOffset, 0, 127, 0, 360);
//          if (radio_mode == 1) {Set_Cmd(33, 12, 1, Level);} // Set AM LNA (in TEF-GTK enable Show antenna input alignment - NO FM)
//          else
//          AudioLevel = atoi(buff + 1);
//          LevelA = map(AudioLevel, 0, 127, 0, 60);
//          if (radio_mode == 0) {Set_Cmd(32, 80, 1, LevelA);}  // Set +6dB analog radio sound gain (in TEF-GTK enable Show antenna input alignment - NO AM)
//          //Serial.print('V');
//          //Serial.println(buff + 1);
//          break;
//
//        case 'Q':  // Audio volume scaler
//          Squelch = atoi(buff + 1);
//          if (Squelch != -1 && AM_VOL_SCALE == -1) {
//            int Setsquelch = map(Squelch, 0, 100, -120, 60);
//            Set_Cmd(33, 80, 1, Setsquelch);
//            //Serial.print("Q");
//            //Serial.println(Squelch);
//          }
//          break;
//
//        case 'D':  // Change the de-emphasis
//          nDeemphasis = atoi(buff + 1);
//          //Serial.print("D");
//          Set_Deempasis(nDeemphasis);
//          //Serial.println(nDeemphasis);
//          break;
//
//        case 'G':
//          //Serial.print('G');
//          RFplus = buff[1] - 48;
//          IFplus = buff[2] - 48;
//          Set_IF_RF(RFplus,IFplus);
//          //Serial.print(RFplus);
//          //Serial.println(IFplus);
//          break;
//
//        case 'A':  // AGC  (check what levelstepping does)
//          //Serial.print('A');
//          AGC_tress = atoi(buff + 1);
//          Set_AGC_tresshold(AGC_tress);
//          //Serial.println(AGC_tress);
//          break;
//
//       case 'C':
//         //Serial.print('C');
//         if (buff[1] == '0' ) {
//         Set_Cmd(32, 22, 1, 1);  //ChannelEqualizer (EQ 1)
//         Set_Cmd(32, 20, 1, 1);  //MphSuppression (iMS 1)
//         //Serial.print("0");
//         } else if (buff[1] == '1' ) {
//          Set_Cmd(32, 22, 1, 0); //ChannelEqualizer (EQ 0)
//          //Serial.print("1");
//         }else if (buff[1] == '2' ) {
//          Set_Cmd(32, 20, 1, 0);  //MphSuppression (iMS 0)
//         //Serial.print("2");
//         }
//         //Serial.print('\n');
//         break;
//
//        case 'Z':
//          //Serial.print('Z');
//          if (buff[1] == '0') {
//              dsp_write_data(INIT_SET1);
//            //Serial.print("0");
//          } else if (buff[1] == '1') {
//              dsp_write_data(INIT_SET2);
//            //Serial.print("1");
//          } else if (buff[1] == '2') {
//              dsp_write_data(INIT_SET3);
//            //Serial.print("2");
//          } else if (buff[1] == '3') {
//              dsp_write_data(INIT_SET4);
//            //Serial.print("3");
//          } else {
//            //Serial.print("4");
//          }
//          //Serial.print('\n');
//          Set_AGC_tresshold(AGC_tress);
//          Set_IF_RF(RFplus,IFplus);
//          Set_Deempasis(nDeemphasis);
//          break;
//
//        case 'X':  // shutdown
//          Set_Cmd(64, 1, 1, 1);
//          //TWCR = 0;  // Release SDA and SCL lines used by hardware I2C
//          //Serial.println("X");
//          //delay(10);
//          HAL_Delay(10);
//          //asm("jmp 0");
//          break;
//      }
//    }
//  }
//}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int bitRead(char bit, uint16_t number)
{
    bit = 1 << bit;
    return(bit & number);
}

uint8_t readRDS()
{
	/* uRDS_RDS[0] = status
	 * uRDS_RDS[1] = A_block
	 * uRDS_RDS[2] = B_block
	 * uRDS_RDS[3] = C_block
	 * uRDS_RDS[4] = D_block
	 * uRDS_RDS[5] = dec_error error code (determined by decoder)
	 *
	 *
	 * */

	int16_t uRDS_RDS[8] = {0};
	//uint16_t uRDS_RDS[8] = {0};
	Get_Cmd(32, 131, uRDS_RDS, 8);
//
//	  if ( bitRead(uRDS_RDS[0], 15) == 1 )
//	  {
//	    //Serial.print('P');
//		print_serial2_message("P");
//	    serial_hex(uRDS_RDS[1] >> 8);
//	    serial_hex(uRDS_RDS[1]);
//	    //Serial.print('\n');
//	    //Serial.print('R');
//
//	    serial_hex(uRDS_RDS[1] >> 8);
//	    serial_hex(uRDS_RDS[2] >> 8);
//	    serial_hex(uRDS_RDS[2]);
//	    serial_hex(uRDS_RDS[3] >> 8);
//	    serial_hex(uRDS_RDS[3]);
//	    serial_hex(uRDS_RDS[4] >> 8); /// aici este denumirea postului R A T A
//	    serial_hex(uRDS_RDS[4]);      //  aici este denumirea postului R C U L
//	    serial_hex(uRDS_RDS[5] >> 8);
//	    //Serial.print('\n');
//	  }

  //char status;
  uint8_t rdsBHigh, rdsBLow, rdsCHigh, rdsCLow, rdsDHigh, isReady = 0, rdsDLow;

  uint16_t rdsStat, rdsA, rdsB, rdsC, rdsD, rdsErr;

  rdsStat = uRDS_RDS[0];
  rdsA = uRDS_RDS[1];
  rdsB = uRDS_RDS[2];
  rdsC = uRDS_RDS[3];
  rdsD = uRDS_RDS[4];
  rdsErr = uRDS_RDS[5];
  //uint16_t result = devTEF668x_Radio_Get_RDS_Data(1, &rdsStat, &rdsA, &rdsB, &rdsC, &rdsD, &rdsErr);

//  if (!(result && (rdsB != 0x0) && ((rdsStat & 0x8000) != 0x0) && ((rdsErr & 0x0a00) == 0x0)))
//  {
//    return isReady;
//  }

    if (!((rdsStat & 0x8000) != 0x0) && ((rdsErr & 0x0a00) == 0x0))
    {
    	return isReady;
    }

  rdsBHigh = (uint8_t)(rdsB >> 8);
  rdsBLow = (uint8_t)rdsB;
  rdsCHigh = (uint8_t)(rdsC >> 8);
  rdsCLow = (uint8_t)rdsC;
  rdsDHigh = (uint8_t)(rdsD >> 8);
  rdsDLow = (uint8_t)rdsD;

  uint8_t programType = ((rdsBHigh & 3) << 3) | ((rdsBLow >> 5) & 7);
  strcpy(rdsProgramType, (programType >= 0 && programType < 32) ? ptyLUT[programType] : "    PTY ERROR   ");

  uint8_t type = (rdsBHigh >> 4) & 15;
  uint8_t version = bitRead(rdsBHigh, 4);

  // Groups 0A & 0B
  // Basic tuning and switching information only
  if (type == 0) {
    uint8_t address = rdsBLow & 3;
    // Groups 0A & 0B: to extract PS segment we need blocks 1 and 3
    if (address >= 0 && address <= 3) {
      if (rdsDHigh != '\0') {
        rdsProgramService[address * 2] = rdsDHigh;
      }
      if (rdsDLow != '\0') {
        rdsProgramService[address * 2 + 1] = rdsDLow;
      }
      isReady = (address == 3) ? 1 : 0;
    }
    rdsFormatString(rdsProgramService, 8);
  }
  // Groups 2A & 2B
  // Radio Text
  else if (type == 2) {
    uint16_t addressRT = rdsBLow & 15;
    uint8_t ab = bitRead(rdsBLow, 4);
    uint8_t cr = 0;
    uint8_t len = 64;
    if (version == 0) {
      if (addressRT >= 0 && addressRT <= 15) {
        if (rdsCHigh != 0x0D) {
          rdsRadioText[addressRT*4] = rdsCHigh;
        }
        else {
          len = addressRT * 4;
          cr = 1;
        }
        if (rdsCLow != 0x0D) {
          rdsRadioText[addressRT * 4 + 1] = rdsCLow;
        }
        else {
          len = addressRT * 4 + 1;
          cr = 1;
        }
        if (rdsDHigh != 0x0D) {
          rdsRadioText[addressRT * 4 + 2] = rdsDHigh;
        }
        else {
          len = addressRT * 4 + 2;
          cr = 1;
        }
        if (rdsDLow != 0x0D) {
          rdsRadioText[addressRT * 4 + 3] = rdsDLow;
        }
        else {
          len = addressRT * 4 + 3;
          cr = 1;
        }
      }
    }
    else {
      if (addressRT >= 0 && addressRT <= 7) {
        if (rdsDHigh != '\0') {
          rdsRadioText[addressRT * 2] = rdsDHigh;
        }
        if (rdsDLow != '\0') {
          rdsRadioText[addressRT * 2 + 1] = rdsDLow;
        }
      }
    }
    if (cr) {
      for (uint8_t i = len; i < 64; i++) {
        rdsRadioText[i] = ' ';
      }
    }
    if (ab != rdsAb) {
      for (uint8_t i = 0; i < 64; i++) {
        rdsRadioText[i] = ' ';
      }
      rdsRadioText[64] = '\0';
      isRdsNewRadioText = 1;
    }
    else {
      isRdsNewRadioText = 0;
    }
    rdsAb = ab;
    rdsFormatString(rdsRadioText, 64);
  }
  return isReady;
}

void getRDS(struct RdsInfo *rdsInfo)
{
  strcpy(rdsInfo->programType, rdsProgramType);
  strcpy(rdsInfo->programService, rdsProgramService);
  strcpy(rdsInfo->radioText, rdsRadioText);
}

void rdsFormatString(char* str, uint16_t length)
{
  for (uint16_t i = 0; i < length; i++) {
    if ((str[i] != 0 && str[i] < 32) || str[i] > 126 ) {
      str[i] = ' ';
    }
  }
}


//
//void show_Rds()
//{
//  isRDSReady = readRDS();
//  getRDS(&rdsInfo);
//
//  showPTY();
//  showPS();
//  showRadioText();
//}

void showPTY()
{
  //if ((isRDSReady == 1) && !str_cmp(rdsInfo.programType, programTypePrevious, 16))
  {
    //Serial.print(rdsInfo.programType);
    strcpy(programTypePrevious, rdsInfo.programType);
    print_serial1_message("PTY: ");
    print_serial2_message(programTypePrevious);
    //Serial.println();
    //lcd.setPosition(4, 11);
    //lcd.print(rdsInfo.programType);
    //lcd.print(strcpy(programTypePrevious, rdsInfo.programType));
  }
}

void showPS()
{
  //if ((isRDSReady == 1) && (strlen(rdsInfo.programService) == 8) && !str_cmp(rdsInfo.programService, programServicePrevious, 8))
  {
    //Serial.print("-=[ ");
    //Serial.print(rdsInfo.programService);
   // Serial.print(" ]=-");
    strcpy(programServicePrevious, rdsInfo.programService);
    print_serial1_message("PS : ");
    print_serial2_message(programServicePrevious);
    //Serial.println();
    //lcd.setPosition(4, 0);
    //lcd.print(rdsInfo.programService);
    //lcd.print(strcpy(programServicePrevious, rdsInfo.programService));
  }
}

void showRadioText()
{
  //if ((isRDSReady == 1) && !str_cmp(rdsInfo.radioText, radioTextPrevious, 65))
  {
    //Serial.print(rdsInfo.radioText);
    strcpy(radioTextPrevious, rdsInfo.radioText);
    print_serial1_message("RDS: ");
    print_serial2_message(radioTextPrevious);
    //Serial.println();
//    lcd.setPosition(3, 11);
    //lcd.setPosition(3, 0);
    // lcd.print(rdsInfo.radioText);
    //lcd.print(strcpy(radioTextPrevious, rdsInfo.radioText));
  }
}

bool str_cmp(char* str1, char* str2, int length)
{
  for (int i = 0; i < length; i++) {
    if (str1[i] != str2[i]) {
      return false;
    }
  }
  return true;
}

void clear_rds_buffers(char* str1, uint8_t length)
{
	  for (int i = 0; i < length; i++)
	  {
        str1[i] = ' ';
	  }

}




// seek
uint16_t seekUp() {
	return seek(1);
}

uint16_t seekDown() {
	return seek(0);
}

uint16_t tuneUp() {
  return tune(1);
}

uint16_t tuneDown() {
  return tune(0);
}


uint16_t tune(uint8_t up) {
  Radio_ChangeFreqOneStep(up);

  Radio_SetFreq(Radio_PRESETMODE, Radio_GetCurrentBand(), Radio_GetCurrentFreq());
  Radio_ClearCurrentStation();
  return Radio_GetCurrentFreq();
}

uint16_t seek(uint8_t up) {
  uint16_t mode = 20;
  uint16_t startFrequency = Radio_GetCurrentFreq();

  while (true) {
    switch(mode){
      case 20:
        Radio_ChangeFreqOneStep(up);
        Radio_SetFreq(Radio_SEARCHMODE, Radio_GetCurrentBand(), Radio_GetCurrentFreq());

        mode = 30;
        Radio_CheckStationInit();
        Radio_ClearCurrentStation();

        break;

      case 30:
        //delay(20);
    	HAL_Delay(20);
        Radio_CheckStation();
        if (Radio_CheckStationStatus() >= NO_STATION) {
          mode = 40;
        }

        break;

      case 40:
        if (Radio_CheckStationStatus() == NO_STATION) {
          mode = (startFrequency == Radio_GetCurrentFreq()) ? 50 : 20;
        }
        else if (Radio_CheckStationStatus() == PRESENT_STATION) {
          mode = 50;
        }

        break;

      case 50:
        Radio_SetFreq(Radio_PRESETMODE, Radio_GetCurrentBand(), Radio_GetCurrentFreq());
        return Radio_GetCurrentFreq();
    }
  }
  return 0;
}

uint16_t seekSync(uint8_t up) {
  if (seekMode == 0) {
	seekMode = 20;
	seekStartFrequency = Radio_GetCurrentFreq();
  }
  switch(seekMode) {
    case 20:
	  Radio_ChangeFreqOneStep(up);
      Radio_SetFreq(Radio_SEARCHMODE, Radio_GetCurrentBand(), Radio_GetCurrentFreq());

      seekMode = 30;
      Radio_CheckStationInit();
      Radio_ClearCurrentStation();

      return 0;

    case 30:
      //delay(20);
      HAL_Delay(20);
      Radio_CheckStation();
      if (Radio_CheckStationStatus() >= NO_STATION) {
        seekMode = 40;
      }

      return 0;

    case 40:
      if (Radio_CheckStationStatus() == NO_STATION) {
        seekMode = (seekStartFrequency == Radio_GetCurrentFreq()) ? 50 : 20;
      }
      else if (Radio_CheckStationStatus() == PRESENT_STATION) {
        seekMode = 50;
      }

      return 0;

    case 50:
	  seekMode = 0;
      Radio_SetFreq(Radio_PRESETMODE, Radio_GetCurrentBand(), Radio_GetCurrentFreq());
      return 1;
  }
  return 0;
}



/*====================================================
 Function:Radio_ChangeFreqOneStep
 Input:
      UP/DOWN
 OutPut:
      Null
 Desp:
     change curren freq on step
=========================================================*/
void Radio_ChangeFreqOneStep(uint8_t UpDown )
{
	int step;

	step=Radio_GetFreqStep(Radio_CurrentBand);
	if(UpDown==1)
	{	/*increase one step*/
		Radio_CurrentFreq+=step;
		/*frequency baundary check	*/
		if(Radio_CurrentFreq>FreqBaundConfig[Radio_CurrentBand].MaxFreq)
		{
			Radio_CurrentFreq=FreqBaundConfig[Radio_CurrentBand].MinFreq;
		}
	}
	else
	{      /*decrease one step*/
		Radio_CurrentFreq-=step;
		/*frequency baundary check*/
		if(Radio_CurrentFreq<FreqBaundConfig[Radio_CurrentBand].MinFreq)
		{
		    	Radio_CurrentFreq=FreqBaundConfig[Radio_CurrentBand].MaxFreq;
		}
	}

}
/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentFreq
Input:
Output:
Description:	return current freq
------------------------------------------------------------------------*/
uint16_t Radio_GetCurrentFreq(void)
{
	 return Radio_CurrentFreq;
}
/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentBand
Input:
Output:
Description:	 return current band
------------------------------------------------------------------------*/
uint8_t Radio_GetCurrentBand(void)
{
	 return Radio_CurrentBand;
}

/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentStation
Input:
Output:
Description:	 return current station
------------------------------------------------------------------------*/
uint8_t Radio_GetCurrentStation(void)
{
	 return Radio_CurrentStation;
}
/*-----------------------------------------------------------------------
Function name:	Radio_ClearCurrentStation
Input:
Output:
Description:	 set current station
------------------------------------------------------------------------*/
void Radio_ClearCurrentStation(void)
{
	  Radio_CurrentStation=0;
}
/*-----------------------------------------------------------------------
Function name:	Radio_GetFreqStep
Input:
Output:
Description:	 Get current band freq step
------------------------------------------------------------------------*/
uint32_t Radio_GetFreqStep(uint8_t band)
{
	return (band<=FM3_BAND)? Radio_AreaConfig.FM_ManualSeekStep : Radio_AreaConfig.AM_ManualSeekStep;
}

/*--------------------------------------------------------------------
 Function:Radio_CheckStationStatus
 Input:
      Null
 OutPut:
      CheckIfStep
 Desp:
     get check station step
---------------------------------------------------------------------*/
uint8_t Radio_CheckStationStatus(void)
{
	return CheckIfStep;
}

/*--------------------------------------------------------------------
 Function:Radio_CheckStationInit
 Input:
      Null
 OutPut:
      NULL
 Desp:
      check station init
---------------------------------------------------------------------*/
void Radio_CheckStationInit(void)
{
	CheckIfStep=10;
}
/*-----------------------------------------------------------------------
Function name:	Radio_SetFreq
Input:		mode:
                              TEF663X_PRESETMODE.,TEF663X_SEARCHMODE
                              TEF663X_AFUPDATEMODE,TEF663X_JUMPMODE...
                        Freq:
Output:
Description:
------------------------------------------------------------------------*/
void Radio_SetFreq(uint8_t mode,uint8_t Band,uint16_t Freq)
{
       /*frequency baundary check*/
	if((Freq>FreqBaundConfig[Radio_CurrentBand].MaxFreq)||(Freq<FreqBaundConfig[Radio_CurrentBand].MinFreq)){
		Freq = FreqBaundConfig[Radio_CurrentBand].MinFreq;
	}
	if(Band>=MaxBandNum){
		return;
	}

//AF_UPDATE not change current info
	if(mode != Radio_AFUPDATEMODE)
	{
		Radio_CurrentBand=Band;
		Radio_CurrentFreq=Freq;
		StationRecord[Radio_CurrentBand].Freq[0]=Radio_CurrentFreq;
	}

	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		AR_TuningAction_t A2Mode;

		switch(mode){
			case Radio_PRESETMODE:
				A2Mode = eAR_TuningAction_Preset;
				break;
			case Radio_SEARCHMODE:
				A2Mode = eAR_TuningAction_Search;
				break;
			case Radio_AFUPDATEMODE:
				A2Mode = eAR_TuningAction_AF_Update;
				break;
			case Radio_JUMPMODE:
				A2Mode = eAR_TuningAction_Jump;
				break;
			case Radio_CHECKMODE:
				A2Mode = eAR_TuningAction_Check;
				break;
		}
        devTEF668x_Radio_Tune_To(TEF665X_Is_FM_Freq(Freq), (uint16_t)A2Mode, Freq);
	}
}

/*--------------------------------------------------------------------
 Function:Radio_CheckStation
 Input:
      Null
 OutPut:
      Null
 Desp:
     check station if aviable
---------------------------------------------------------------------*/
#define RADIO_CHECK_INTERVAL	5
static uint32_t Radio_Check_Timer;
void Radio_CheckStation(void)
{
	unsigned char threshold;

	uint8_t usn, wam;
	uint16_t offset;

	uint8_t fm = (Radio_CurrentBand<=FM3_BAND);
	switch(CheckIfStep)
	{
		case 10://start check init
			CheckIfStep = 20;
			break;

		case 20://Check QRS(quality of read status)
            //delay(fm ? RADIO_FM_LEVEL_AVAILABLE_TIME : RADIO_AM_LEVEL_AVAILABLE_TIME);
			HAL_Delay(fm ? RADIO_FM_LEVEL_AVAILABLE_TIME : RADIO_AM_LEVEL_AVAILABLE_TIME);
			CheckIfStep=30;  //Set to next step
			break;

		case 30://check level , AM=3 times,FM=2 times
		case 31:
		case 32:
			threshold = fm ? ((SeekSenLevel ==HIGH) ? FM_SCAN_LEVEL_HI : FM_SCAN_LEVEL)
						: ((SeekSenLevel ==HIGH) ? AM_SCAN_LEVEL_HI : AM_SCAN_LEVEL);

			if(Radio_Get_Level(fm) < threshold)
			{//exit check
				CheckIfStep=NO_STATION;
			}
			else{
				threshold = fm ? 31 : 32;
				if(++CheckIfStep > threshold)
				{
					CheckIfStep=40;
					//delay(40);//set for usn... available
					HAL_Delay(40);
				}
				else
					//delay(RADIO_CHECK_INTERVAL);//set for the next time
					HAL_Delay(RADIO_CHECK_INTERVAL);
			}
			break;

		case 40:
			CheckIfStep = NO_STATION;

			if(1 == Radio_Get_Data(fm,&usn,&wam,&offset))
			{
				if( fm? ((usn<FM_USN_DISTURBANCE)&&(wam < FM_WAM_DISTURBANCE)&&(offset < FM_FREQ_OFFSET))
									: (offset < AM_FREQ_OFFSET))

				CheckIfStep = PRESENT_STATION ;
			}
			break;

        case NO_STATION:
		break;

        case PRESENT_STATION:
		break;

        default:
		CheckIfStep = NO_STATION;
		break;
	}
}

/*-----------------------------------------------------------------------
Function name:
Input:
Output:
Description:	 check stereo indicator
------------------------------------------------------------------------*/
uint8_t Radio_CheckStereo(void)
{
	uint16_t status;
	uint8_t stereo = 0;

	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		if (1==devTEF668x_Radio_Get_Signal_Status(1, &status)) {
			stereo = ((status >> 15) & 1) ? 1 : 0;
		}
	}

	return stereo;
}

//uint8_t SeekSenLevel = LOW;
void Radio_SetSeekSenLevel(uint8_t Lev)
{
	SeekSenLevel = Lev;
}

uint8_t Radio_Is_AF_Update_Available (void)
{
	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		return devTEF668x_Radio_Is_AF_Update_Available();
	}

	return 0;
}
uint8_t Radio_Is_RDAV_Available (void)
{
	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		return devTEF668x_Radio_Is_RDAV_Available();
	}

	return 0;
}



//level detector result
//output: -200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
//return =  dBuV
uint16_t Radio_Get_Level(uint8_t fm)
{
	int16_t level;
	uint8_t status;

	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		if(1 == devTEF668x_Radio_Get_Quality_Level(fm,&status,&level))
		{
			return level;
		}
	}

	return -255;
}

// status =  0.1  32 ms
// level = -20 ... 120 dBuV
// usn = 0  100%
// wam = 0  100%
//offset = -1200  1200 (*0.1 kHz) = -120 kHz ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 kHz
//bandwidth = FM, 560  3110 [*0.1 kHz]  AM 30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80 [*0.1 kHz]
// modulation = 0  100%
static int Radio_Get_Data(uint8_t fm, uint8_t *usn, uint8_t *wam, uint16_t *offset)
{
	if(Is_Radio_Atomic2||Is_Radio_Lithio)
	{
		if(1 == devTEF668x_Radio_Get_Quality_Data (fm,usn,wam,offset))
		{
			return 1;
		}
	}

	return 0;
}









/* -----------------------------------------------------------------------------
 * Internal define:
 *-----------------------------------------------------------------------------*/

#define High_16bto8b(a)	((uint8_t)((a) >> 8))
#define Low_16bto8b(a) 	((uint8_t)(a ))

#define Convert8bto16b(a)	((uint16_t)(((uint16_t)(*(a))) << 8 |((uint16_t)(*(a+1)))))

/* -----------------------------------------------------------------------------
 * Internal Prototypes:
 *-----------------------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
* Exported functions:
* -----------------------------------------------------------------------------
*/

/****************************************************************/
/* Function Calls for Control Commands and Information Requests */
/****************************************************************/
/*
typedef struct{
	TEF668x_MODULE module;
	uint8_t cmd;
	uint16_t len;	//buffer size of all data
}TEF668x_CMD_LEN;
*/
//#define TEF668x_CMD_LEN_MAX	20
#define TEF668x_CMD_LEN_MAX	31

uint16_t devTEF668x_Set_Cmd(TEF668x_MODULE module, uint8_t cmd, uint16_t len,...)
{
	uint16_t i;
	uint8_t buf[TEF668x_CMD_LEN_MAX];
	uint16_t temp;
    va_list vArgs;

    va_start(vArgs, len);

	buf[0]= module;			//module,		FM/AM/APP
	buf[1]= cmd;		//cmd,		1,2,10,...
	buf[2]= 1;	//index, 		always 1

//fill buffer with 16bits one by one
	for(i=3;i<len;i++)
	{
		temp = va_arg(vArgs,int);	//the size only uint16_t valid for compile

		buf[i++]=High_16bto8b(temp);
		buf[i]=Low_16bto8b(temp);
	}

	va_end(vArgs);

	return Tuner_WriteBuffer(buf, len);



//	  uint8_t buf[31];
//	  //uint16_t temp;
//	  uint32_t temp;
//	  va_list vArgs;
//	  va_start(vArgs, len);
//	  buf[0] = module;
//	  buf[1] = cmd;
//	  buf[2] = 1;
//	  for (uint8_t i = 0; i < len; i++)
//	  {
//	    //temp = va_arg(vArgs, uint16_t);
//		//temp = va_arg(vArgs, int);
//		temp = va_arg(vArgs, uint32_t);
//	    buf[3 + i * 2] = (uint8_t)(temp >> 8);
//	    buf[4 + i * 2] = (uint8_t)temp;
//	  }
//	  va_end(vArgs);
//	  Write(buf, len * 2 + 3);
//	  return 0;  // no error
}


static uint16_t devTEF668x_Get_Cmd(TEF668x_MODULE module, uint8_t cmd, uint8_t *receive,uint16_t len)
{
//	uint8_t buf[3];
//
//	buf[0]= module;			//module,		FM/AM/APP
//	buf[1]= cmd;		//cmd,		1,2,10,...
//	buf[2]= 1;	//index, 		always 1
//
//	Tuner_WriteBuffer(buf, 3);
//
//	return Tuner_ReadBuffer(receive,len);

	  uint8_t buf[3];
	  buf[0] = module;
	  buf[1] = cmd;
	  buf[2] = 1;
	  Write(buf, 3);
	  Read((uint8_t*)receive, 2 * len);
	  for (uint8_t i = 0; i < len; i++)
	  {
	    uint16_t newval = (uint8_t)(receive[i] >> 8) | (((uint8_t)(receive[i])) << 8);
	    receive[i] = newval;
	  }

	  return 0;  // no error
}

/*
module 32 / 33 FM / AM
cmd 1 Tune_To mode, frequency

index
1 mode
	[ 15:0 ]
	tuning actions
	0 = no action (radio mode does not change as function of module band)
	1 = Preset Tune to new program with short mute time
	2 = Search Tune to new program and stay muted
	FM 3 = AF-Update Tune to alternative frequency, store quality
	and tune back with inaudible mute
	4 = Jump Tune to alternative frequency with short
	inaudible mute
	5 = Check Tune to alternative frequency and stay
	muted
	AM 3 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5 = reserved
	6 = reserved
	7 = End Release the mute of a Search or Check action
	(frequency is not required and ignored)
2 frequency
[ 15:0 ]
	tuning frequency
	FM 6500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 10800 65.00 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 108.00 MHz / 10 kHz step size
	AM LW 144 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 288 144 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 288 kHz / 1 kHz step size
	MW 522 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1710 522 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1710 kHz / 1 kHz step size
	SW 2300 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 27000 2.3 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 27 MHz / 1 kHz step size
*/
uint16_t devTEF668x_Radio_Tune_To (uint8_t fm,uint16_t mode,uint16_t frequency )
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Tune_To,
			(mode<=5)? 7 : 5,
			mode, frequency);
}

/*
module 32 FM
cmd 2 Set_Tune_Options afu_bw_mode, afu_bandwidth, afu_mute_time, afu_sample_time

index
1 afu_bw_mode
	[ 15:0 ]
	IF bandwidth control mode during AF_Update
	0 = fixed (default)
2 afu_bandwidth
	[ 15:0 ]
	fixed IF bandwidth during AF_Update
	560 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 3110 [*0.1 kHz] = IF bandwidth 56 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 311 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ wide
	2360 = 236 kHz (default)
3 afu_mute_time
	[ 15:0 ]
	AF_update inaudible mute slope time
	250 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [* 1 us] = 0.25 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1 ms
	1000 = 1 ms (default)
4 afu_sample_time
	[ 15:0 ]
	AF_update sampling time
	1000 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20000 [* 1 us] = 1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20 ms
	2000 = 2 ms (default)
*/
uint16_t devTEF668x_Radio_Set_Tune_Options(uint8_t fm,uint16_t afu_bw_mode,uint16_t afu_bandwidth,uint16_t afu_mute_time,uint16_t afu_sample_time)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Tune_Options,
			11,
			afu_bw_mode, afu_bandwidth,afu_mute_time,afu_sample_time);
}

/*
module 32 / 33 FM / AM
cmd 10 Set_Bandwidth FM : mode, bandwidth, control_sensitivity, low_level_sensitivity
AM : mode, bandwidth

index
1 mode
	[ 15:0 ]
	IF bandwidth control mode
	FM 0 = fixed
	1 = automatic (default)
	AM 0 = fixed (default)
2 bandwidth
	[ 15:0 ]
	fixed IF bandwidth
	FM 560 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 3110 [*0.1 kHz] = IF bandwidth 56 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 311 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ wide
	2360 = 236 kHz (default)
	AM 30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80 [*0.1 kHz] = IF bandwidth 3 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 8 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ wide
	40 = 4.0 kHz (default)
3 control_sensitivity
	[ 15:0 ]
	FM automatic IF bandwidth control sensitivity
	500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1500 [*0.1 %] = 50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 150 % relative adjacent channel sensitivity
	1000 = 100 % (default)
4 low_level_sensitivity
	[ 15:0 ]
	FM automatic IF bandwidth control sensitivity for low level conditions
	500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1500 [*0.1 %] = 50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 150 % relative adjacent channel sensitivity
	1000 = 100 % (default)
*/
uint16_t devTEF668x_Radio_Set_Bandwidth(uint8_t fm,uint16_t mode,uint16_t bandwidth,uint16_t control_sensitivity,uint16_t low_level_sensitivity)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Bandwidth,
			fm? 11: 7,
			mode,bandwidth,control_sensitivity,low_level_sensitivity);
}

/*
module 32 / 33 FM / AM
cmd 11 Set_RFAGC start, extension

index
1 start
	[ 15:0 ]
	RF AGC start
	FM 830 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 920 [*0.1 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V) = 83 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 92 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V
	920 = 92 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (default)
	AM 940 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 1030 (*0.1 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V) = 94 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 103 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V
	1000 = 100 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (default)
2 extension
	[ 15:0 ]
	RF AGC step extension
	FM 0 = integrated steps only (default)
	1 = AGC step extension from control output (GPIO feature ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½AGCÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	AM reserved
*/
uint16_t devTEF668x_Radio_Set_RFAGC(uint8_t fm,uint16_t start,uint16_t extension)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_RFAGC,
			fm? 7: 5,
			start,extension);
}

/*
module 33 AM
cmd 12 Set_Antenna attenuation

1 attenuation
	[ 15:0 ]
	LNA gain reduction
	0 / 60 / 120 / 180 / 240 / 300 / 360 (*0.1 dB) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 36 dB antenna
	attenuation (6 dB step size)
	0 = no attenuation (default)
*/
uint16_t devTEF668x_Radio_Set_Antenna(uint8_t fm,uint16_t attenuation)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Antenna,
			5,
			attenuation);
}

/*
module 32 FM
cmd 20 Set_MphSuppression mode

index
1 mode
	[ 15:0 ]
	FM multipath suppression
	0 = off (default)
	1 = on
*/
uint16_t devTEF668x_Radio_Set_MphSuppression(uint8_t fm,uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_MphSuppression,
			5,
			mode);
}
/*
FM cmd 22 Set_ChannelEqualizer
Optional use of the FM channel equalizer.
module   32  FM
cmd  22  Set_ChannelEqualizer  mode
index  1  mode
[ 15:0 ]
FM channel equalizer
0 = off (default)
1 = on
*/
uint16_t devTEF668x_Radio_Set_ChannelEqualizer(uint8_t fm,uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF668X_Cmd_Set_ChannelEqualizer,
			5,
			mode);
}
/*
module    32  FM
cmd  32  Set_StereoImprovement  mode
index  1  mode
[ 15:0 ]
FM stereo extended weak signal handling
0 = stereo high blend (default)
1 = FMSI stereo band blend system
*/
uint16_t devTEF668x_Radio_Set_StereoImprovement(uint8_t fm,uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF668X_Cmd_Set_StereoImprovement,
			5,
			mode);
}
/*
cmd  70  Set_StHiBlend_Time  slow_attack, slow_decay, fast_attack, fast_decay
index  1  slow_attack
[ 15:0 ]
slow attack time of weak signal handling
60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 (ms) = 60 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2 s slow attack time
500 = 500 ms (default)
2  slow_decay
[ 15:0 ]
slow decay time of weak signal handling
120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12500 (ms) = 120 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12.5 s slow attack time
2000 = 2 s (default)
3  fast_attack
[ 15:0 ]
fast attack time of weak signal handling
10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1200 (*0.1 ms) = 1 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 ms fast attack time
20 = 2 ms (default)
4  fast_decay
[ 15:0 ]
fast decay time of weak signal handling
20 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 ( *0.1 ms) = 2 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms fast attack time
20 = 2 ms (default)

*/
uint16_t devTEF668x_Radio_Set_StHiBlend_Time(uint8_t fm,uint16_t slow_attack,uint16_t slow_decay,uint16_t fast_attack,uint16_t fast_decay)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF668X_Cmd_Set_StHiBlend_Time,
			11,
			slow_attack,slow_decay,fast_attack,fast_decay);
}
/*
module 32 / 33 FM / AM
cmd 23 Set_NoiseBlanker mode, sensitivity

index
1 mode
	[ 15:0 ]
	noise blanker
	0 = off
	1 = on (default)
2 sensitivity
	[ 15:0 ]
	trigger sensitivity
	500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1500 [*0.1 %] = 50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 150 % relative trigger sensitivity
	1000 = 100 % (default)
*/
uint16_t devTEF668x_Radio_Set_NoiseBlanker(uint8_t fm,uint16_t mode,uint16_t sensitivity)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_NoiseBlanker,
			7,
			mode,sensitivity);
}
/*
module 33 AM
cmd 24 Set_NoiseBlanker_Audio mode, sensitivity

index
1 mode
	[ 15:0 ]
	AM audio noise blanker (audio frequency detection)
	0 = off
	1 = on (default)
2 sensitivity
	[ 15:0 ]
	AM audio noise blanker trigger sensitivity
	500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1500 [*0.1 %] = 50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 150 % relative trigger sensitivity
	1000 = 100 % (default)
*/
uint16_t devTEF668x_Radio_Set_NoiseBlanker_Audio(uint8_t fm,uint16_t mode,uint16_t sensitivity)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_NoiseBlanker_Audio,
			7,
			mode,sensitivity);
}
/*
module 32 / 33 FM / AM
cmd 30 Set_DigitalRadio mode

index
1 mode
	[ 15:0 ]
	digital radio
	0 = off (default)
	1 = on
*/
uint16_t devTEF668x_Radio_Set_DigitalRadio(uint8_t fm,uint16_t mode)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_DigitalRadio,
			5,
			mode);
}
/*
module 32 FM
cmd 31 Set_Deemphasis timeconstant

index
1 timeconstant
	[ 15:0 ]
	deemphasis time constant
	0 = off; for evaluation purposes only
	500 = 50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½s deemphasis (default)
	750 = 75 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½s deemphasis
*/
uint16_t devTEF668x_Radio_Set_Deemphasis(uint8_t fm,uint16_t timeconstant)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Deemphasis,
			5,
			timeconstant);
}

/*
module 32 / 33 FM / AM
cmd 38 Set_LevelStep step1, step2, step3, step4, step5, step6, step7

index 1 step1
	[ 15:0 ]
	level offset for an AGC step from 0 to 1
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-20 = -2 dB (FM default) / -10 = -1 dB (AM default)
2 step2
	[ 15:0 ]
	level offset for an AGC step from 1 to 2
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-30 = -3 dB (FM default) / -20 = -2 dB (AM default)
3 step3
	[ 15:0 ]
	level offset for an AGC step from 2 to 3
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-40 = -4 dB (FM default) / -30 = -3 dB (AM default)
4 step4
	[ 15:0 ]
	level offset for an AGC step from 3 to 4
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-50 = -5 dB (FM default) / -40 = -4 dB (AM default)
5 step5
	[ 15:0 ]
	level offset for an AGC step from 4 to 5
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-60 = -6 dB (FM default) / -50 = -5 dB (AM default)
6 step6
	[ 15:0 ]
	level offset for an AGC step from 5 to 6
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-60 = -6 dB (default)
7 step7
	[ 15:0 ]
	level offset for an AGC step from 6 to 7 (or higher)
	-60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-60 = -6 dB (default)
*/
uint16_t devTEF668x_Radio_Set_LevelStep(uint8_t fm,uint16_t step1,uint16_t step2,uint16_t step3,uint16_t step4,uint16_t step5,uint16_t step6,uint16_t step7)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_LevelStep,
			17,
			(uint16_t)step1,(uint16_t)step2,(uint16_t)step3,(uint16_t)step4,(uint16_t)step5,(uint16_t)step6,(uint16_t)step7);
}

/*
module 32 / 33 FM / AM
cmd 39 Set_LevelOffset offset

1 offset
	[ 15:0 ] signed
	level offset
	-480 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +150 (*0.1 dB) = -48 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +15 dB
	0 = 0 dB (default)
*/
uint16_t devTEF668x_Radio_Set_LevelOffset(uint8_t fm,uint16_t offset)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_LevelOffset,
			5,
			(uint16_t)offset);
}

/*
module 32 / 33 FM / AM
cmd 40 Set_Softmute_Time slow_attack, slow_decay, fast_attack, fast_decay

1 slow_attack
	[ 15:0 ]
	slow attack time of weak signal handling
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 (ms) = 60 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2 s slow attack time
	120 = 120 ms (default)
2 slow_decay
	[ 15:0 ]
	slow decay time of weak signal handling
	120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12500 (ms) = 120 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12.5 s slow attack time
	500 = 500 ms (default)
3 fast_attack
	[ 15:0 ]
	fast attack time of weak signal handling
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1200 (*0.1 ms) = 1 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 ms fast attack time
	20 = 2 ms (FM default) / 120 = 12 ms (AM default)
4 fast_decay
	[ 15:0 ]
	fast decay time of weak signal handling
	20 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 ( *0.1 ms) = 2 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms fast attack time
	20 = 2 ms (FM default) / 500 = 50 ms (AM default)
*/
uint16_t devTEF668x_Radio_Set_Softmute_Time(uint8_t fm,uint16_t slow_attack,uint16_t slow_decay,uint16_t fast_attack,uint16_t fast_decay)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Softmute_Time,
			11,
			slow_attack,slow_decay,fast_attack,fast_decay);
}

/*
module 32 / 33 FM / AM
cmd 42 Set_Softmute_Level mode, start, slope

1 mode
	[ 15:0 ]
	timer selection
	0 = off (only for evaluation)
	1 = fast timer control
	2 = slow timer control (default)
	3 = dual timer control; combined fast and slow timer control
2 start
	[ 15:0 ] 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 500 [*0.1 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V] = control when level falls below 0 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 50 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V
	150 = 15 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (FM default) / 280 = 28 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (AM default)
3 slope
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 300 [*0.1 dB] = control over level range of 6 dB ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 30 dB
	220 = 22 dB (default) / 250 = 25 dB (default)
*/
uint16_t devTEF668x_Radio_Set_Softmute_Level(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Softmute_Level,
			9,
			mode,start,slope);
}

/*
module 32 FM
cmd 43 Set_Softmute_Noise mode, start, slope

1 mode
	[ 15:0 ]
	timer selection
	0 = off (default)
	1 = fast timer control
	2 = slow timer control
	3 = dual timer control; combined fast and slow timer control
2 start
	[ 15:0 ]
	FM weak signal handling noise start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when noise above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of USN detector
	500 = 50% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling noise range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of USN detector
	1000 = 100% (default)
*/
uint16_t devTEF668x_Radio_Set_Softmute_Noise(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Softmute_Noise,
			9,
			mode,start,slope);
}

/*
module 32 FM
cmd 44 Set_Softmute_Mph mode, start, slope

1 mode
	[ 15:0 ]
	timer selection
	0 = off (default)
	1 = fast timer control
	2 = slow timer control
	3 = dual timer control; combined fast and slow timer control
2 start
	[ 15:0 ]
	FM weak signal handling multipath start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when mph above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of WAM detector
	500 = 50% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling multipath range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of WAM detector
	1000 = 100% (default)
*/
uint16_t devTEF668x_Radio_Set_Softmute_Mph(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Softmute_Mph,
			9,
			mode,start,slope);
}

/*
module 32 / 33 FM / AM
cmd 45 Set_Softmute_Max mode, limit

index
1 mode
	[ 15:0 ]
	weak signal handling (dynamic control)
	0 = off (for evaluation only)
	1 = on; maximum dynamic control defined by limit parameter (default)
2 limit
	[ 15:0 ]
	softmute dynamic attenuation limit
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 400 [*0.1 dB] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 40 dB softmute maximum attenuation
	200 = 20 dB (FM default) / 250 = 25 dB (AM default)

*/
uint16_t devTEF668x_Radio_Set_Softmute_Max(uint8_t fm,uint16_t mode,uint16_t limit)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Softmute_Max,
			7,
			mode,limit);
}

/*
module 32 / 33 FM / AM
cmd 50 Set_Highcut_Time slow_attack, slow_decay, fast_attack, fast_decay

index
1 slow_attack
	[ 15:0 ]
	slow attack time of weak signal handling
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 (ms) = 60 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2 s slow attack time
	500 = 500 ms (default)
2 slow_decay
	[ 15:0 ]
	slow decay time of weak signal handling
	120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12500 (ms) = 120 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12.5 s slow attack time
	2000 = 2 s (default)
3 fast_attack
	[ 15:0 ]
	fast attack time of weak signal handling
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1200 (*0.1 ms) = 1 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 ms fast attack time
	20 = 2 ms (FM default) / 120 = 12 ms (AM default)
4 fast_decay
	[ 15:0 ]
	fast decay time of weak signal handling
	20 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 ( *0.1 ms) = 2 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms fast attack time
	20 = 2 ms (FM default) / 500 = 50 ms (AM default)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Time(uint8_t fm,uint16_t slow_attack,uint16_t slow_decay,uint16_t fast_attack,uint16_t fast_decay)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Highcut_Time,
			11,
			slow_attack,slow_decay,fast_attack,fast_decay);
}

/*
module 32 / 33 FM / AM
cmd 51 Set_Highcut_Mod mode, start, slope, shift

index
1 mode modulation dependent weak signal handling
	0 = off (default)
	1 = on (independent modulation timer)
2 start
	[ 15:0 ]
	weak signal handling modulation start
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control when modulation falls below 10% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	250 = 25% (default)
	( note : for FM band 100% modulation equals 75 kHz deviation )
3 slope
	[ 15:0 ]
	weak signal handling modulation range
	30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = control over modulation range of 3% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	130 = 13% (default)
4 shift
	[ 15:0 ]
	weak signal handling control shift
	50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = maximum weak signal control shift of 5% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	500 = 50% (default)
	(percentage of the linear control range from _Min limit to _Max limit)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Mod(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope,uint16_t shift)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Highcut_Mod,
			11,
			mode,start,slope,shift);
}

/*
module 32 / 33 FM / AM
cmd 52 Set_Highcut_Level mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off (only for evaluation)
	1 = fast timer control
	2 = slow timer control (AM default)
	3 = dual timer control; combined fast and slow timer control (FM default)
2 start
	[ 15:0 ]
	weak signal handling level start
	200 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 600 [*0.1 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V] = control when level is below 20 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 60 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V
	360 = 36 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (FM default) / 400 = 40 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (AM default)
3 slope
	[ 15:0 ]
	weak signal handling level range
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 300 [*0.1 dB] = control over level range of 6 dB ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 30 dB
	300 = 30 dB (FM default) / 200 = 20 dB (AM default)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Level(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Highcut_Level,
			9,
			mode,start,slope);
}


/*
module 32 FM
cmd 53 Set_Highcut_Noise mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off
	1 = fast timer control
	2 = slow timer control (default)
	3 = dual timer control; combined fast and slow timer control
2 start
	[ 15:0 ]
	FM weak signal handling noise start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when noise above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of USN detector
	360 = 36% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling noise range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of USN detector
	300 = 30% (default)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Noise(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Highcut_Noise,
			9,
			mode,start,slope);
}


/*
module 32 FM
cmd 54 Set_Highcut_Mph mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off (only for evaluation)
	1 = fast timer control
	2 = slow timer control (default)
	3 = dual timer control; combined fast and slow timer control
2 start
	[ 15:0 ]
	FM weak signal handling multipath start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when mph above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of WAM detector
	360 = 36% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling multipath range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of WAM detector
	300 = 30% (default)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Mph(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Highcut_Mph,
			9,
			mode,start,slope);
}

/*
module 32 / 33 FM / AM
cmd 55 Set_Highcut_Max mode, limit

index
1 mode
	[ 15:0 ]
	weak signal handling (dynamic control)
	0 = off; for evaluation only
	1 = on; maximum dynamic control set by limit parameter (default)
2 limit
	[ 15:0 ]
	Highcut attenuation limit
	FM 1500 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 7000 [*1 Hz] = 1.5 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 7 kHz Highcut maximum -3 dB att.
	4000 = 4 kHz (default)
	AM 1350 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 7000 [*1 Hz] = 1.35 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 7 kHz Highcut maximum -3 dB att.
	1800 = 1.8 kHz (default)

*/
uint16_t devTEF668x_Radio_Set_Highcut_Max(uint8_t fm,uint16_t mode,uint16_t limit)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Highcut_Max,
			7,
			mode,limit);
}


/*
module 32 / 33 FM / AM
cmd 56 Set_Highcut_Min mode, limit

index
1 mode
	[ 15:0 ]
	strong signal handling
	0 = off; high audio frequency bandwidth is not limited (FM default)
	1 = on; minimum control limit set by limit parameter (AM default)
2 limit
	[ 15:0 ]
	Highcut fixed attenuation limit
	2700 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 15000 [*1 Hz] = 2.7 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 15 kHz -3 dB attenuation fo

*/
uint16_t devTEF668x_Radio_Set_Highcut_Min(uint8_t fm,uint16_t mode,uint16_t limit)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Highcut_Min,
			7,
			mode,limit);
}

/*
module 32 / 33 FM / AM
cmd 58 Set_Lowcut_Min mode, limit

index
1 mode
	[ 15:0 ]
	strong signal handling
	0 = off; low audio frequency bandwidth is not limited (FM default)
	1 = on; minimum control limit set by limit parameter (AM default)
2 limit
	[ 15:0 ]
	Lowcut fixed attenuation limit
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 200 [Hz] = 10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 200 Hz Lowcut minimum -3 dB attenuation
	20 = 20 Hz (default)

*/
uint16_t devTEF668x_Radio_Set_Lowcut_Min(uint8_t fm,uint16_t mode,uint16_t limit)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Lowcut_Min,
			7,
			mode,limit);
}

/*
module 32 FM
cmd 60 Set_Stereo_Time slow_attack, slow_decay, fast_attack, fast_decay

index
1 slow_attack
	[ 15:0 ]
	slow attack time of weak signal handling
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 (ms) = 60 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2 s slow attack time
	1000 = 1 s (default)
2 slow_decay
	[ 15:0 ]
	slow decay time of weak signal handling
	120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12500 (ms) = 120 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 12.5 s slow attack time
	4000 = 4 s (default)
3 fast_attack
	[ 15:0 ]
	fast attack time of weak signal handling
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1200 (*0.1 ms) = 1 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 ms fast attack time
	80 = 8 ms
4 fast_decay
	[ 15:0 ]
	fast decay time of weak signal handling
	20 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 ( *0.1 ms) = 2 ms ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms fast attack time
	80 = 8 ms

*/
uint16_t devTEF668x_Radio_Set_Stereo_Time(uint8_t fm,uint16_t slow_attack,uint16_t slow_decay,uint16_t fast_attack,uint16_t fast_decay)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Time,
			11,
			slow_attack,slow_decay,fast_attack,fast_decay);
}

/*
module 32 FM
cmd 61 Set_Stereo_Mod mode, start, slope, shift

index
1 mode
	[ 15:0 ]
	modulation dependent weak signal handling
	0 = off (default)
	1 = on (independent modulation timer)
2 start
	[ 15:0 ]
	weak signal handling modulation start
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control when modulation falls below 10% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	210 = 21% (default)
	( note : for FM band 100% modulation equals 75 kHz deviation )
3 slope
	[ 15:0 ]
	30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = control over modulation range of 3% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	90 = 9% (default)
4 shift
	[ 15:0 ]
	weak signal handling control shift
	50 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = maximum weak signal control shift of 5% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100%
	500 = 50% (default)
	(percentage of the linear control range from _Min limit to ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½monoÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Mod(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope,uint16_t shift)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Mod,
			11,
			mode,start,slope,shift);
}

/*
module 32 FM
cmd 62 Set_Stereo_Level mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off (only for evaluation)
	1 = fast timer control
	2 = slow timer control
	3 = dual timer control; combined fast and slow timer control (default)
2 start
	[ 15:0 ]
	weak signal handling level start
	300 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 600 [*0.1 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V] = control when level below 30 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 60 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V
	460 = 46 dBÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V (default)
3 slope
	[ 15:0 ]
	weak signal handling level range
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 300 [*0.1 dB] = control over level range of 6 dB ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K 30 dB
	240 = 24 dB (default)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Level(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Level,
			9,
			mode,start,slope);
}

/*
module 32 FM
cmd 63 Set_Stereo_Noise mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off
	1 = fast timer control
	2 = slow timer control
	3 = dual timer control; combined fast and slow timer control (default)
2 start
	[ 15:0 ]
	FM weak signal handling noise start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when noise above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of USN detector
	240 = 24% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling noise range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of USN detector
	200 = 20% (default)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Noise(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Noise,
			9,
			mode,start,slope);
}

/*
module 32 FM
cmd 64 Set_Stereo_Mph mode, start, slope

index
1 mode
	[ 15:0 ]
	timer selection
	0 = off
	1 = fast timer control
	2 = slow timer control
	3 = dual timer control; combined fast and slow timer control (default)
2 start
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 800 [*0.1 %] = control when mph above 0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80% of WAM detector
	240 = 24% (default)
3 slope
	[ 15:0 ]
	FM weak signal handling multipath range
	100 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = control over range of 10ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% of WAM detector
	200 = 20% (default)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Mph(uint8_t fm,uint16_t mode,uint16_t start,uint16_t slope)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Mph,
			9,
			mode,start,slope);
}

/*
module 32 FM
cmd 65 Set_Stereo_Max mode, limit

index
1 mode
	[ 15:0 ]
	weak signal handling (dynamic control)
	0 = off (for evaluation only)
	1 = on; maximum dynamic control is 0 dB channel sep, i.e. mono (default)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Max(uint8_t fm,uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Max,
			5,
			mode);
}

/*
module 32 FM
cmd 66 Set_Stereo_Min mode, limit

index
1 mode
	[ 15:0 ]
	strong signal handling
	0 = off; channel separation is not limited (default)
	1 = on; minimum control limit set by limit parameter
	2 = forced mono
2 limit
	[ 15:0 ]
	Stereo fixed attenuation limit
	60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 400 [0.1* dB] = 6 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 40 dB Stereo minimum channel separation
	400 = 40 dB (default)

*/
uint16_t devTEF668x_Radio_Set_Stereo_Min(uint8_t fm,uint16_t mode,uint16_t limit)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_Stereo_Min,
			7,
			mode,limit);
}

/*
module 32 / 33 FM / AM
cmd 80 Set_Scaler gain

index
1 gain
	[ 15:0 ] (signed)
	channel gain
	-120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +60 [*0.1 dB] = -12 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +6 dB analog radio signal gain
	0 = 0 dB (default)
*/
uint16_t devTEF668x_Radio_Set_Scaler(uint8_t fm,uint16_t gain)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Scaler,
			5,
			gain);
}

/*
module 32 FM
cmd 81 Set_RDS mode, restart, interface

index
1 mode
	[ 15:0 ]
	RDS operation control
	0 = off (RDS function disabled)
	1 = decoder mode (default); output of RDS group data (block A, B, C, D)
	from Get_RDS_Status/Get_RDS_Data (FM cmd = 130/131)
	2 = demodulator mode; output of raw demodulator data from
	Get_RDS_Status/Get_RDS_Data; FM cmd = 130/131)
2 restart
	[ 15:0 ]
	RDS decoder restart
	0 = no control
	1 = manual restart; start looking for new RDS signal immediately
	2 = automatic restart after tuning (default); start looking for new RDS signal
	after Preset, Search, Jump or Check tuning action (see FM cmd = 1)
3 interface
	[ 15:0 ]
	RDS pin signal functionality
	0 = no pin interface (default)
	2 = data-available status output; active low (GPIO feature ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½DAVNÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	4 = legacy 2-wire demodulator data and clock output (ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½RDDAÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ and ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½RDCLÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
*/
uint16_t devTEF668x_Radio_Set_RDS(uint8_t fm,uint16_t mode,uint16_t restart,uint16_t interfac)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Set_RDS,
			9,
			mode,restart,interfac);
}

/*
module 32 / 33 FM / AM
cmd 82 Set_QualityStatus mode, interface

index
1 mode
[ 15:0 ]
	quality status flag after tuning ready
	0 = no flag set after tuning (default)
	[ 8:0 ] : 1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 320 (* 0.1 ms) = set flag at 0.1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 32 ms after tuning ready
	[15] : 1 = set flag when FM AF_Update quality result is available
2 interface
	[ 15:0 ]
	quality status pin signal functionality
	0 = no pin interface (default)
	2 = quality status output; active low (ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½QSIÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
*/
uint16_t devTEF668x_Radio_Set_QualityStatus(uint8_t fm,uint16_t mode,uint16_t interfac)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_QualityStatus,
			7,
			mode,interfac);
}

/*
module 32 / 33 FM / AM
cmd 83 Set_DR_Blend mode, in_time, out_time, gain

index
1 mode
	[ 15:0 ]
	blend pin use (DR_BL input)
	0 = Standard pin use : DR Blend pin High = digital radio (default)
	1 = Inverted pin use : DR Blend pin Low = digital radio
	2 = No pin use; Force blend to digital radio
	3 = No pin use; Force blend to analog radio
2 in_time
	[ 15:0 ]
	blend time from analog radio to digital radio
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 [*0.1 ms] = 1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms
	50 = 5 ms (default)
3 out_time
	[ 15:0 ]
	blend time from digital radio to analog radio
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 5000 [*0.1 ms] = 1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 500 ms
	50 = 5 ms (default)
4 gain
	[ 15:0 ] (signed)
	digital radio channel gain
	-180 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +60 [*0.1 dB] = -18 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +6 dB digital radio signal gain
	0 = 0 dB (default)
*/
uint16_t devTEF668x_Radio_Set_DR_Blend(uint8_t fm,uint16_t mode,uint16_t in_time,uint16_t out_time,uint16_t gain)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_DR_Blend,
			11,
			mode,in_time,out_time,gain);
}


/*
module 32 / 33 FM / AM
cmd 84 Set_DR_Options samplerate

index
1 samplerate
[ 15:0 ]
	baseband digital radio sample rate (DR_I2S output)
	0 = automatic frequency selection based on tuning frequency (default)
	6500 = 650 kHz (not for normal application use)
	6750 = 675 kHz (not for normal application use)
*/
uint16_t devTEF668x_Radio_Set_DR_Options(uint8_t fm,uint16_t samplerate)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_DR_Options,
			5,
			samplerate);
}

/*
module 32 / 33 FM / AM
cmd 85 Set_Specials ana_out, dig_out

index
1 ana_out
	[ 15:0 ]
	analog output use
	0 = normal operation (default)
	1 = DAC_L : FM MPX wideband (DARC) signal / DAC_R : mono audio
	2 = DAC_L : digital radio left channel / DAC_R : analog radio left channel
*/
uint16_t devTEF668x_Radio_Set_Specials(uint8_t fm,uint16_t ana_out)
{
	return devTEF668x_Set_Cmd(fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Set_Specials,
			5,
			ana_out);
}

/*
module 48 AUDIO
cmd 10 Set_Volume volume

index
1 volume
	[ 15:0 ] (signed)
	audio volume
	-599 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +240 = -60 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +24 dB volume
	0 = 0 dB (default)
*/
uint16_t devTEF668x_Audio_Set_Volume(int16_t volume)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Volume,
			5,
			volume*10);
}

/*
module 48 AUDIO
cmd 11 Set_Mute mode

index
1 mode
	[ 15:0 ]
	audio mute
	0 = mute disabled
	1 = mute active (default)
*/
uint16_t devTEF668x_Audio_Set_Mute(uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Mute,
			5,
			mode);
}


/*
module 48 AUDIO
cmd 12 Set_Input source

index
1 source
	[ 15:0 ]
	audio source select
	0 = radio (default)
	(analog radio or digital radio when enabled and available)
	32 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio input (IIS_SD_0)
	240 = sine wave generator
*/
uint16_t devTEF668x_Audio_Set_Input(uint16_t source)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Input,
			5,
			source);
}

/*
module 48 AUDIO
cmd 13 Set_Output_Source signal, source

index
1 signal
	[ 15:0 ]
	audio output
	33 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio IIS_SD_1 (output)
	128 = DAC L/R output
2 source
	[ 15:0 ]
	source
	4 = analog radio
	32 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio IIS_SD0 (input)
	224 = audio processor (default)
	240 = sine wave generator
*/
uint16_t devTEF668x_Audio_Set_Output_Source(uint16_t signal,uint16_t source)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Output_Source,
			7,
			signal,source);
}

/*
module 48 AUDIO
cmd 21 Set_Ana_Out signal, mode

index
1 signal
	[ 15:0 ]
	analog audio output
	128 = DAC L/R output
2 mode
	[ 15:0 ]
	output mode
	0 = off (power down)
	1 = output enabled (default)
*/
uint16_t devTEF668x_Audio_Set_Ana_Out(uint16_t signal,uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Ana_Out,
			7,
			signal,mode);
}

/*
module 48 AUDIO
cmd 22 Set_Dig_IO signal, mode, format, operation, samplerate

index
1 signal
[ 15:0 ]
	digital audio input / output
	32 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio IIS_SD_0 (input)
	33 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio IIS_SD_1 (output)
(2) mode
	0 = off (default)
	1 = input (only available for signal = 32)
	2 = output (only available for signal = 33)
(3) format
	[ 15:0 ]
	digital audio format select
	16 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S 16 bits (fIIS_BCK = 32 * samplerate)
	32 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S 32 bits (fIIS_BCK = 64 * samplerate) (default)
	272 = lsb aligned 16 bit (fIIS_BCK = 64 * samplerate)
	274 = lsb aligned 18 bit (fIIS_BCK = 64 * samplerate)
	276 = lsb aligned 20 bit (fIIS_BCK = 64 * samplerate)
	280 = lsb aligned 24 bit (fIIS_BCK = 64 * samplerate)
(4) operation
	[ 15:0 ]
	operation mode
	0 = slave mode; IIS_BCK and IIS_WS input defined by source (default)
	256 = master mode; IIS_BCK and IIS_WS output defined by device
(5) samplerate
	[ 15:0 ] 3200 = 32.0 kHz
	4410 = 44.1 kHz (default)
	4800 = 48.0 kHz
*/
uint16_t devTEF668x_Audio_Set_Dig_IO(uint16_t signal,uint16_t mode,uint16_t format,uint16_t operation,uint16_t samplerate)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Dig_IO,
			13,
			signal,mode,format,operation,samplerate);
}

/*
module 48 AUDIO
cmd 23 Set_Input_Scaler source, gain

index
1 source
	[ 15:0 ]
	audio source
	32 = IÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½S digital audio input : IIS_SD_0
2 gain
	[ 15:0 ] (signed)
	external source channel gain
	-120 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +60 [*0.1 dB] = -12 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ +6 dB external source signal gain
	0 = 0 dB (default)
*/
uint16_t devTEF668x_Audio_Set_Input_Scaler(uint16_t source,uint16_t gain)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Input_Scaler,
			7,
			source,gain);
}

/*
module 48 AUDIO
cmd 24 Set_WaveGen mode, offset, amplitude1, frequency1, amplitude2, frequency2

index
1 mode
	[ 15:0 ]
	mode
	0 = wave signal off (default)
	1 = wave 1 signal on Left channel
	2 = wave 2 signal on Right channel
	3 = wave 1 signal on Left channel and wave 2 signal on Right channel
	5 = wave 1 signal on Left and Right channel
	6 = wave 2 signal on Left and Right channel
	7 = wave 1 + wave 2 signal on Left and Right channel
2 offset
	[ 15:0 ]
	DC offset
	-32768 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ + 32767 (* 1 LSB of 16 bit) = max negative ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ max positive.
	0 = no offset (default)
3 amplitude1
	[ 15:0 ] signed
	wave 1 amplitude
	-300 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-200 = -20 dB (default)
4 frequency1
	[ 15:0 ]
	wave 1 frequency
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20000 (*1 Hz) = 10 Hz ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20 kHz
	400 = 400 Hz (default)
5 amplitude2
	[ 15:0 ] signed
	wave 2 amplitude
	-300 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 (*0.1 dB) = -30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 0 dB
	-200 = -20 dB (default)
6 frequency2
	[ 15:0 ]
	wave 2 frequency
	10 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20000 (*1 Hz) = 10 Hz ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 20 kHz
	1000 = 1 kHz (default)
*/
uint16_t devTEF668x_Audio_Set_WaveGen(uint16_t mode,uint16_t offset,uint16_t amplitude1,uint16_t frequency1,uint16_t amplitude2,uint16_t frequency2)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_WaveGen,
			15,
			mode,offset,amplitude1,frequency1,amplitude2,frequency2);
}

/*
module 64 APPL
cmd 1 Set_OperationMode mode

index
1 mode
	[ 15:0 ]
	device operation mode
	0 = normal operation
	1 = radio standby mode (low-power mode without radio functionality)
	(default)
*/
uint16_t devTEF668x_APPL_Set_OperationMode(uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Set_OperationMode,
			5,
			mode);
}

/*
module 64 APPL
cmd 3 Set_GPIO pin, module, feature

index
1 pin
	[ 15:0 ]
	GPIO number
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2 = GPIO number
2 module
	[ 15:0 ]
	module
	32 = FM
	33 = AM
3 feature
	[ 15:0 ]
	feature
	0 = no use (default) (FM / AM)
	1 = general purpose input (FM / AM)
	2 = general purpose output ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½0ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ (FM / AM)
	3 = general purpose output ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½1ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ (FM / AM)
	257 = output RDS (FM : see cmd 81 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½DAVNÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	258 = output QSI (FM / AM : see cmd 82 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½timer and AF_Update flagÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	259 = output QSI + RDS (active ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½lowÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ if ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½DAVNÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ is active or ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½QSIÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ is active)
	260 = output RDDA (FM : see cmd 81 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½RDDA, RDCL legacy optionÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	261 = output RDCL (FM : see cmd 81 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½RDDA, RDCL legacy optionÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	262 = output AGC (FM : see cmd 11 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½AGC step extensionÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
*/
uint16_t devTEF668x_APPL_Set_GPIO(uint16_t pin,uint16_t module,uint16_t feature)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Set_GPIO,
			9,
			pin,module,feature);
}

/*
module 64 APPL
cmd 4 Set_ReferenceClock frequency

index
1 frequency_high
	MSB part of the reference clock frequency
	[ 31:16 ]
2 frequency_low
	LSB part of the reference clock frequency
	[ 15:0 ]
	frequency [*1 Hz] (default = 9216000)
3 type
	clock type
	0 = crystal oscillator operation (default)
	1 = external clock input operation
*/
uint16_t devTEF668x_APPL_Set_ReferenceClock(uint16_t frequency_high,uint16_t frequency_low,uint16_t type)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Set_ReferenceClock,
			9,
			frequency_high,frequency_low,type);
}

/*
module 64 APPL
cmd 5 Activate mode

index
1 mode
	[ 15:0 ]
	1 = goto ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½activeÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ state with operation mode of ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½radio standbyÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
*/
uint16_t devTEF668x_APPL_Activate(uint16_t mode)
{
	return devTEF668x_Set_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Activate,
			5,
			mode);
}

/*
module 32 / 33 FM / AM
cmd 128 / 129 Get_Quality_Status/ Get_Quality_Data
FM : | status, level, usn, wam, offset, bandwidth, modulation
AM : | status, level, -, -, offset, bandwidth, modulation

index
1 status
	[ 15:0 ]
	quality detector status
	[15] =
	AF_update flag
	0 = continuous quality data with time stamp
	1 = AF_Update sampled data
	[14:10] = reserved
	[9:0] = quality time stamp
	0 = tuning is in progress, no quality data available
	1 ... 320 (* 0.1 ms) = 0.1 ... 32 ms after tuning,
	quality data available, reliability depending on time stamp
	1000 = > 32 ms after tuning, quality data continuously updated
2 level
	[ 15:0 ] (signed)
	level detector result
	-200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
	actual range and accuracy is limited by noise and agc
3 usn
	[ 15:0 ]
	FM ultrasonic noise detector
	0 ... 1000 (*0.1 %) = 0 ... 100% relative usn detector result
4 wam
	[ 15:0 ]
	FM wideband-AMÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ multipath detector
	0 ... 1000 (*0.1 %) = 0 ... 100% relative wam detector result
5 offset
	[ 15:0 ] (signed)
	radio frequency offset
	-1200 ... 1200 (*0.1 kHz) = -120 kHz ... 120 kHz radio frequency error
	actual range and accuracy is limited by noise and bandwidth
6 bandwidth
	[ 15:0 ]
	IF bandwidth
	FM 560 ...3110 [*0.1 kHz] = IF bandwidth 56 ... 311 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K wide
	AM 30 ... 80 [*0.1 kHz] = IF bandwidth 3 ... 8 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½K wide
7 modulation
	[ 15:0 ]
	modulation detector
	FM 0 ... 1000 [*0.1 %] = 0 ... 100% modulation = 0 ... 75 kHz FM dev.
	1000 ... 2000 [*0.1 %] = 100% ... 200% over-modulation range
	(modulation results are an approximate indication of actual FM dev.)
	AM 0 ... 1000 [*0.1 %] = 0 ... 100% AM modulation index
	1000 ... 2000 [*0.1 %] = 100% ... 200% peak modulation range
*/
uint16_t devTEF668x_Radio_Get_Quality_Status (uint8_t fm,uint8_t *status)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(r == 1){
		*status = ((0x3fff&Convert8bto16b(buf))/10);
	}

	return r;
}
uint8_t devTEF668x_Radio_Is_AF_Update_Available (void)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_FM,TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(r == 1){
		return ((buf[0]&0xC0)== 0x80);
	}

	return 0;
}
uint8_t devTEF668x_Radio_Is_RDAV_Available (void)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_FM,TEF665X_Cmd_Get_RDS_Status,
			buf,sizeof(buf));

	if(r == 1){
		return ((buf[0]&0x02)== 0x02);
	}

	return 0;
}

uint16_t devTEF668x_Radio_Get_Quality_Level (uint8_t fm,uint8_t *status,int16_t *level)
{
	uint8_t buf[4];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Quality_Data,
			buf,sizeof(buf));

	if(r == 1){
		*status = (int16_t)((0x3fff&Convert8bto16b(buf))/10);
		*level = (int16_t)(((int16_t)Convert8bto16b(buf+2))/10);
	}

	return r;
}

/*
status	= 1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 320 (* 0.1 ms) = 0.1 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 32 ms after tuning
level		= -200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
usn		=  0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% relative usn detector result
wam		=  0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1 %) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% relative wam detector result
offset	 = -1200 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1200 (*0.1 kHz) = -120 kHz ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 120 kHz radio frequency error
bandwith	 = FM 560 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 3110 [*0.1 kHz] = IF bandwidth 56 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 311 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ wide
			AM 30 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 80 [*0.1 kHz] = IF bandwidth 3 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 8 kHz; narrow ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ wide
modulation	 = FM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% modulation = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 75 kHz FM dev.
				1000 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 [*0.1 %] = 100% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 200% over-modulation range
			AM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 [*0.1 %] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100% AM modulation index
			1000 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 2000 [*0.1 %] = 100% ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 200% peak modulation range

*/
uint16_t devTEF668x_Radio_Get_Quality_Data (uint8_t fm,uint8_t *usn,uint8_t *wam,uint16_t *offset)
{
	uint8_t buf[14];
	uint16_t r;
	int16_t temp;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Quality_Data,
			buf,sizeof(buf));

	if(r == 1)
	{
		//*status = ((0x3fff&Convert8bto16b(buf))/10);
		//*level = (((int16_t)Convert8bto16b(buf+2))/10);
		*usn = (Convert8bto16b(buf+4)/10);
		*wam = (Convert8bto16b(buf+6)/10);
		temp = (((int16_t)Convert8bto16b(buf+8))/1);
		*offset = temp<0? (0-temp) : temp;
		//*bandwidth = (Convert8bto16b(buf+10)/1);
		//*modulation = (Convert8bto16b(buf+12)/10);
	}

	return r;
}


/*
module 32 FM
cmd 130 / 131 Get_RDS_Status
/ Get_RDS_Data
| status, A_block, B_block, C_block, D_block, dec_error

index
1 status
	[ 15:0 ]
	FM RDS reception status
	[15] = data available flag
	0 = no data available (incomplete group or no first PI)
	1 = RDS group data or first PI data available
	[14] =
	data loss flag
	0 = no data loss
	1 = previous data was not read, replaced by newer data
	[13] =
	data available type
	0 = group data; continuous operation
	1 = first PI data; data with PI code following decoder sync.
	[12] = group type
	0 = type A; A-B-C-D group (with PI code in block A)
	1 = type B; A-B-CÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½-D group (with PI code in block A and CÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½)
	[11:10] = reserved
	[9] = synchronization status
	0 = RDS decoder not synchronized; no RDS data found
	1 = RDS decoder synchronized; RDS data reception active
	[8: 0] = reserved
2 A_block
	[ 15:0 ]
	A block data
3 B_block
	[ 15:0 ]
	B block data
4 C_block
	[ 15:0 ]
	C block data
5 D_block
	[ 15:0 ]
	D block data
6 dec_error error code (determined by decoder)
	[ 15:0 ] [15:14] = A block error code
	[13:12] = B block error code
	[11:10] = C block error code
	[9:8] = D block error code
	0 : no error; block data was received with matching data and syndrome
	1 : small error; possible 1 bit reception error detected; data is corrected
	2 : large error; theoretical correctable error detected; data is corrected
	3 : uncorrectable error; no data correction possible
	[7:0] = reserved
*/
uint16_t devTEF668x_Radio_Get_RDS_Status(uint8_t fm,uint16_t *status)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Get_RDS_Status,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
	}

	return r;
}
uint8_t devTEF668x_Radio_Get_RDS_Data (uint8_t fm,uint16_t *status,uint16_t *A_block,uint16_t *B_block,uint16_t *C_block,uint16_t *D_block,
	uint16_t *dec_error)
{
	uint8_t buf[12];
	uint8_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Get_RDS_Data,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
		*A_block = Convert8bto16b(buf+2);
		*B_block = Convert8bto16b(buf+4);
		*C_block = Convert8bto16b(buf+6);
		*D_block = Convert8bto16b(buf+8);
		*dec_error = Convert8bto16b(buf+10);
	}

	return r;
}
uint16_t devTEF668x_Radio_Get_RDS_DataRaw (uint8_t fm,uint16_t *status,uint32_t *raw_data)
{
	uint8_t buf[6];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_FM,
			TEF665X_Cmd_Get_RDS_Data,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
		*raw_data = ((unsigned long)buf[2]<<24) | ((unsigned long)buf[3]<<16) | ((unsigned long)buf[4]<<8) | ((unsigned long)buf[5]);
	}

	return r;
}

/*
module 32 / 33 FM / AM
cmd 132 Get_AGC | input_att, feedback_att

index
1 input_att
	[ 15:0 ]
	RF AGC input attenuation
	FM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 420 (0.1* dB) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 42 dB attenuation
	AM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 420 (0.1* dB) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 42 dB attenuation
2 feedback_att
	[ 15:0 ]
	RF AGC feedback attenuation
	FM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 60 (0.1* dB) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 6 dB attenuation
	AM 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 180 (0.1* dB) = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 18 dB attenuation
*/
uint16_t devTEF668x_Radio_Get_AGC(uint8_t fm,uint16_t *input_att,uint16_t *feedback_att)
{
	uint8_t buf[4];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_AGC,
			buf,sizeof(buf));

	if(r == 1){
		*input_att = Convert8bto16b(buf);
		*feedback_att = Convert8bto16b(buf+2);
	}

	return r;
}

/*
module 32 / 33 FM / AM
cmd 133 Get_Signal_Status | status

index
1 status
	Radio signal information
	[15] = 0 : mono signal
	[15] = 1 : FM stereo signal (stereo pilot detected)
	[14] = 0 : analog signal
	[14] = 1 : digital signal (blend input activated by digital processor or control)
*/
uint16_t devTEF668x_Radio_Get_Signal_Status(uint8_t fm,uint16_t *status)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Signal_Status,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
	}

	return r;
}

/*
module 32/33 FM / AM
cmd 134 Get_Processing_Status | softmute, highcut, stereo
index
1 softmute Softmute control state
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1%) = 0 % minimum ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100 % max. softmute attenuation
2 highcut Highcut control state
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1%) = 0 % minimum ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100 % max. audio freq. limitation
3 stereo FM Stereo blend control state
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 1000 (*0.1%) = 0 % minimum ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 100 % max. stereo att. (= mono)
*/
uint16_t devTEF668x_Radio_Get_Processing_Status(uint8_t fm,uint16_t *softmute,uint16_t *highcut,uint16_t *stereo)
{
	uint8_t buf[6];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Processing_Status,
			buf,sizeof(buf));

	if(r == 1){
		*softmute = Convert8bto16b(buf);
		*highcut = Convert8bto16b(buf+2);
		*stereo = Convert8bto16b(buf+4);
	}

	return r;
}

/*
module 32 / 33 FM / AM
cmd 135 Get_Interface_Status | samplerate
index
1 samplerate
	[ 15:0 ]
	Baseband digital radio sample rate (DR_I2S output)
	0 = interface disabled (digital radio disabled)
	6500 = 650 kHz
	6750 = 675 kHz
*/
uint16_t devTEF668x_Radio_Get_Interface_Status(uint8_t fm,uint16_t *samplerate)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Interface_Status,
			buf,sizeof(buf));

	if(r == 1){
		*samplerate = Convert8bto16b(buf);
	}

	return r;
}

/*
module 64 APPL
cmd 128 Get_Operation_Status | status
index
1 status
	Device operation status
	0 = boot state; no command support
	1 = idle state
	2 = active state; radio standby
	3 = active state; FM
	4 = active state; AM
*/
uint16_t devTEF668x_APPL_Get_Operation_Status(uint8_t *status)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
	}

	return r;
}


/*
module 64 APPL
cmd 129 Get_GPIO_Status | status
index
1 status
	input state (when assigned for input use)
	[2] = input state of GPIO_2 (no input use suggested for TEF668x)
	[1] = input state of GPIO_1 (no input use suggested for TEF668x)
	[0] = input state of GPIO_0 (0 = low, 1 = high)
*/
uint16_t devTEF668x_APPL_Get_GPIO_Status(uint16_t *status)
{
	uint8_t buf[2];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_GPIO_Status,
			buf,sizeof(buf));

	if(r == 1){
		*status = Convert8bto16b(buf);
	}

	return r;
}



/*
module 64 APPL
cmd 130 Get_Identification | device, hw_version, sw_version
index
1 device
	device type and variant
	[ 15:8 ] type identifier
	8 = TEF668x ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½Atomic-2ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ series
	[ 7:0 ] variant identifier
	15 = TEF6653 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½Atomic-2 StandardÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
	13 = TEF6657 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½Atomic-2 PremiumÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
	12 = TEF6659 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½Atomic-2 Premium DRÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
2 hw_version
	hardware version
	[ 15:8 ] major number
	1 = ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½V1ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
	[ 7:0 ] minor number
	0 = ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½BÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
3 sw_version
	firmware version
	[ 15:8 ] major number
	1 = ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½1ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
	[ 7:0 ] minor number
	1 = ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½.01ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½
*/
uint16_t devTEF668x_APPL_Get_Identification(uint16_t *device,uint16_t *hw_version,uint16_t *sw_version)
{
	uint8_t buf[6];
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(r == 1){
		*device = Convert8bto16b(buf);
		*hw_version = Convert8bto16b(buf+2);
		*sw_version = Convert8bto16b(buf+4);
	}

	return r;
}


/*

module 64 APPL
cmd 131 Get_LastWrite | size/module, cmd/ index, parameter1, parameter2, parameter3, ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½

index
1 size/module transmission size (number of parameters) and module number
	[ 15:8 ] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 6 : number of parameters of the last write transmission
	[ 7:0 ] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 255 : module value of the last write transmission
2 cmd/index
	command byte number and index byte value
	[ 15:8 ] = 0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 255 : cmd value of the last write transmission
	[ 7:0 ] = 0 ... 255 : index value of the last write transmission
3 parameter1 first parameter
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 65535 = value of the first parameter (when available)
4 parameter2 second parameter
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 65535 = value of the second parameter (when available)
5 parameter3 third parameter
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 65535 = value of the third parameter (when available)
6 parameter4 fourth parameter
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 65535 = value of the fourth parameter (when available)
7 parameter5 fifth parameter
	0 ÃƒÆ’Ã‚Â¯Ãƒâ€šÃ‚Â¿Ãƒâ€šÃ‚Â½ 65535 = value of the fifth parameter (when available)
*/
uint16_t devTEF668x_APPL_Get_LastWrite(uint8_t *buf,uint16_t len)
{
	uint16_t r;

	r = devTEF668x_Get_Cmd(TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_LastWrite,
			buf,len);

	return r;
}





//return 1 --> IIC sucess
//unsigned char Tuner_WriteBuffer(unsigned char *buf, uint16_t len)
unsigned char Tuner_WriteBuffer(uint8_t *buf, uint16_t len)
{
  //uint16_t i;
  //uint8_t r;
  HAL_StatusTypeDef r;
  //Serial.println(I2C_ADDR, HEX);
  //Serial.println("Send to I2C: ");
  //Wire.beginTransmission(I2C_ADDR);
  //Serial.println("beginTransmission");
  //for (i = 0; i < len; i++) {
   // Wire.write(buf[i]);
    //Serial.print(buf[i], HEX);
    //Serial.print(" ");
 // }
  //Serial.println("Wire.write");
  //r = Wire.endTransmission();
   r = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DSP_I2C, (uint8_t*)buf, len, 1500);
  //Serial.println("Wire.endTransmission:");
  //Serial.print(r);
  //Serial.println();
  //delay(1);
  HAL_Delay(1);
  //return (r == 0) ? 1 : 0;
  return (r == HAL_OK) ? 1 : 0;
}

//unsigned char Tuner_ReadBuffer(unsigned char *buf, uint16_t len)
unsigned char Tuner_ReadBuffer(uint8_t *buf, uint16_t len)
{
//  uint16_t i;
//  //Serial.println("\nRead from I2C: ");
//  Wire.requestFrom(I2C_ADDR, len);
//  if (Wire.available() == len) {
//    for (i = 0; i < len; i++) {
//      buf[i] = Wire.read();
//      //Serial.print(buf[i], HEX);
//      //Serial.print(" ");
//    }
//    //Serial.println();
//    return 1;
//  }
//  return 0;  //

	HAL_StatusTypeDef r;
	r = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DSP_I2C, (uint8_t *)buf, len, 1500);
	if (r == HAL_OK)
		return 1;
	else
		return 0;
}


void setFrequency(uint16_t frequency) {
  Radio_SetFreq(Radio_PRESETMODE, FM2_BAND, frequency);
}

uint16_t getFrequency() {
  return Radio_GetCurrentFreq();
}

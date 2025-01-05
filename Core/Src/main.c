/* USER CODE BEGIN Header */

// Functioneaza cu dsp_init declarat in RAM

// TODO
/* de afisat mod stereo cu un punct pe display
 * de rezolvat cautarea automata intr-o singura banda
 * animatii display
 * memorii posturi                                       ok
 * de ce dureaza bucla while principala atat de mult ??
 * eroare afisare punct 90.0
 *
 *
 *
 *
 * */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "string.h"
//#include "stdio.h"
//#include <stdint.h>
#include "DISP_14_SEG.h"
#include "tef6686.h"
#include "I2C_reg.h"
#include "display_functions.h"
#include "W25Qxx.h"
#include "RevEng_PAJ7620.h"
//#include "paj7620.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TEF6686_I2C_ADDRESS        0x64
//#define DISPLAY_ADDRESS 0x70<<1
#define DISPLAY_ADDRESS 0x70<<1  //

/*
    Notice: When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s).
            You also can adjust the reaction time according to the actual circumstance.
*/
#define GES_REACTION_TIME	500	 // You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME		800	 // When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s).
#define GES_QUIT_TIME		1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern struct RdsInfo rdsInfo;
extern uint8_t isRDSReady;
uint32_t contor;
extern uint32_t Radio_CurrentFreq;
volatile bool seek_up;
volatile bool seek_down;
//char *start_radio = "Start radio...\n\r";
char str1[30];
char message_frequency[36];
char message_frequency_display[36];
char message_volume[26];

extern int16_t volume;
extern uint32_t freq;

//uint32_t current_freq;  // todo local

extern uint32_t REG_FREQ;
extern uint32_t MODA_FREQ;
extern uint32_t MODF_FREQ;
extern int8_t Filter_AM;
extern int8_t Filter_FM;
extern int8_t current_filter;  // Current FIR filter (-1 is adaptive)

char str[40];

uint16_t encoder_reading_f, old_encoder_reading_f;
uint16_t encoder_reading_v, old_encoder_reading_v;

extern char rdsRadioText[65];
extern char rdsProgramType[17];
extern char rdsProgramService[50];
//extern char display_buffer[]; // 8 caractere

//char message_freq_static[13];

uint16_t seek_frequency;
bool isFmSeekMode;
bool isFmSeekUp;
uint8_t current_band;
uint8_t stereo_status; //0 mono, 1 stereo
bool rds_info_ready_to_display;
extern char programServicePrevious[50];
extern char radioTextPrevious[65];

char memory1[] = "MEMORY 1 ";
char memory2[] = "MEMORY 2 ";
char memory3[] = "MEMORY 3 ";
char memory4[] = "MEMORY 4 ";
char memory5[] = "MEMORY 5 ";
char fm_radio[] = "FM RADIO ";
char search_up[] = "SEARCH->";
char search_down[] = "<-SEARCH";
char hello[]= " HELLO ";

extern char mess_frequency[30];
extern char mess_freq_static[30];
extern char mess_volume[30];
char mess_rds_ps[9];
char mess_rds_rt[65];
char mess_rds_rt1[9];
char mess_rds_rt2[9];
char mess_rds_rt3[9];
char mess_rds_rt4[9];
char mess_rds_rt5[9];
char mess_rds_rt6[9];
//char mess_rds_ps_rt[74];


uint8_t volatile change_rds_display;
bool volatile freq_vol_changed_manual;
uint8_t rds_string_lenght;
uint32_t ID = 0;
bool m1_button_pressed = false;
bool m2_button_pressed = false;
bool m3_button_pressed = false;
bool m4_button_pressed = false;
bool m5_button_pressed = false;

bool save_to_flash_m1 = false;
bool save_to_flash_m2 = false;
bool save_to_flash_m3 = false;
bool save_to_flash_m4 = false;
bool save_to_flash_m5 = false;

bool read_from_flash_m1 = false;
bool read_from_flash_m2 = false;
bool read_from_flash_m3 = false;
bool read_from_flash_m4 = false;
bool read_from_flash_m5 = false;

bool volum_changed = false;
bool save_to_flash_volume = false;
uint8_t volatile timer4 = 0;
// flash variable
uint8_t flash_RxData[20];
uint8_t flash_TxData[20];

bool PAJ_event = false;
uint8_t volum;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void display_rds_info();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //NVIC_SystemReset();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  W25Q_Reset();       // trebuie apelat aici, nu inainte de while
  ID = W25Q_ReadID();

//  uint8_t paj_error = 0;
//  paj_error = paj7620Init();			// initialize Paj7620 registers
//  if (paj_error) {
//      print_serial2_message_number("INIT ERROR,CODE: ", paj_error);
//  } else {
//      print_serial2_message("PAJ init OK");
//  }
  beginPAJ7620();
  enablePAJsensor();
  HAL_IWDG_Refresh(&hiwdg); //
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);  // volum
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);  // frecventa
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin, GPIO_PIN_RESET);
  print_serial2_message("Start radio...");
  HAL_IWDG_Refresh(&hiwdg);
  uint8_t start_rd_ok = init_radio();
  HAL_IWDG_Refresh(&hiwdg); //
  powerOn();
  HAL_IWDG_Refresh(&hiwdg);
  Radio_SetBand(FM1_BAND); // ??
  HAL_IWDG_Refresh(&hiwdg);
  current_band = Radio_GetCurrentBand(); // 1 => 65-108
  HAL_IWDG_Refresh(&hiwdg);
  load_default_station_and_volume();
  sprintf(message_frequency, "START RADIO CODE = %i \r", start_rd_ok); // 1 = OK, 2 = Doesn't exist, 0 = busy
  print_serial2_message(message_frequency);
  //HAL_IWDG_Refresh(&hiwdg); //
  uint16_t current_freq = getFrequency();
  sprintf(message_frequency, "curentFrecv = %i MHz \r", current_freq);
  print_serial2_message(message_frequency);
  change_rds_display = 0;
  HAL_Delay(100);
  HAL_GPIO_WritePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin, GPIO_PIN_SET);
  HAL_IWDG_Refresh(&hiwdg);
  begin_disp(DISPLAY_ADDRESS);  // pass in the address
  display_static_message(fm_radio);
  HAL_Delay(1000);
  clear_display();

  freq_vol_changed_manual = true;
  HAL_TIM_Base_Start_IT(&htim4); //
  HAL_TIM_Base_Start_IT(&htim5); // Enable the timer
  freq = current_freq;
  populate_freq_array(freq);
  change_rds_display = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_IWDG_Refresh(&hiwdg);
      contor++;
      showFmSeek();
                   //stereo_status = getStereoStatus(); // very very slow function!!!
      read_seek_up_down_gpio();
      read_volume_encoder();
      read_frequency_encoder();
      show_rds_informations();
      read_write_memory_stations_vol();
      read_PAJ_gesture();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;
    for(DataIdx = 0; DataIdx < len; DataIdx++)
    {
    	ITM_SendChar(*ptr++);
    }

     return len;
}

void display_rds_info()
{
  isRDSReady = readRDS();
  getRDS(&rdsInfo);

  showPTY();
  showPS();
  showRadioText();
}

void showFmSeek() {
	uint16_t current_freq;
  if (isFmSeekMode) {
    if (seekSync(isFmSeekUp)) {
      isFmSeekMode = false;
      print_serial2_message("Seek stopped");
      current_freq = getFrequency();
      TIM3->CNT = ((current_freq - 8750)/10)<<1;
	  clear_display();
	  disp_freq(current_freq);
	  freq = current_freq;
      sprintf(message_frequency, "curentFrecv = %li MHz \r", freq);
      print_serial2_message(message_frequency);
      populate_freq_array(freq);
      //sprintf(message_frequency_display, "FREQ = %lu MHZ  ", freq);
    }

  }
}

uint8_t prepare_rds_for_display(char *rds_message)
{
	rds_info_ready_to_display = false;
	//char message[9];
	//uint8_t string_length = strlen(rds_message);

	//strncpy(mess_rds_ps, rds_message, NR_OF_DIGITS); // copies first 8 characters from the string 'rds_message' to
	uint8_t last_index =  find_3_space_in_string(rds_message); // sfarsitul stringului
	strncpy(mess_rds_rt, rds_message, last_index);
	mess_rds_rt[last_index + 1] = '\0';
	if (last_index <= 2)
	{
		mess_rds_ps[0] = programServicePrevious[0];
		mess_rds_ps[1] = programServicePrevious[1];
		mess_rds_ps[2] = programServicePrevious[2];
		mess_rds_ps[3] = programServicePrevious[3];
		mess_rds_ps[4] = programServicePrevious[4];
		mess_rds_ps[5] = programServicePrevious[5];
		mess_rds_ps[6] = programServicePrevious[6];
		mess_rds_ps[7] = programServicePrevious[7];
		mess_rds_ps[8] = '\0';
	}
	else if ((last_index > 2) && (last_index <= 8))
	{
		//change_rds_display = 1;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';
	}
	else if (last_index <= 16)
	{
		//change_rds_display = 2;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';

		mess_rds_rt2[0] = mess_rds_rt[8];
		mess_rds_rt2[1] = mess_rds_rt[9];
		mess_rds_rt2[2] = mess_rds_rt[10];
		mess_rds_rt2[3] = mess_rds_rt[11];
		mess_rds_rt2[4] = mess_rds_rt[12];
		mess_rds_rt2[5] = mess_rds_rt[13];
		mess_rds_rt2[6] = mess_rds_rt[14];
		mess_rds_rt2[7] = mess_rds_rt[15];
		mess_rds_rt2[8] = '\0';
	}
	else if (last_index <= 24)
	{
		//change_rds_display = 3;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';

		mess_rds_rt2[0] = mess_rds_rt[8];
		mess_rds_rt2[1] = mess_rds_rt[9];
		mess_rds_rt2[2] = mess_rds_rt[10];
		mess_rds_rt2[3] = mess_rds_rt[11];
		mess_rds_rt2[4] = mess_rds_rt[12];
		mess_rds_rt2[5] = mess_rds_rt[13];
		mess_rds_rt2[6] = mess_rds_rt[14];
		mess_rds_rt2[7] = mess_rds_rt[15];
		mess_rds_rt2[8] = '\0';

		mess_rds_rt3[0] = mess_rds_rt[16];
		mess_rds_rt3[1] = mess_rds_rt[17];
		mess_rds_rt3[2] = mess_rds_rt[18];
		mess_rds_rt3[3] = mess_rds_rt[19];
		mess_rds_rt3[4] = mess_rds_rt[20];
		mess_rds_rt3[5] = mess_rds_rt[21];
		mess_rds_rt3[6] = mess_rds_rt[22];
		mess_rds_rt3[7] = mess_rds_rt[23];
		mess_rds_rt3[8] = '\0';
	}
	else if (last_index <= 32)
	{
		//change_rds_display = 4;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';

		mess_rds_rt2[0] = mess_rds_rt[8];
		mess_rds_rt2[1] = mess_rds_rt[9];
		mess_rds_rt2[2] = mess_rds_rt[10];
		mess_rds_rt2[3] = mess_rds_rt[11];
		mess_rds_rt2[4] = mess_rds_rt[12];
		mess_rds_rt2[5] = mess_rds_rt[13];
		mess_rds_rt2[6] = mess_rds_rt[14];
		mess_rds_rt2[7] = mess_rds_rt[15];
		mess_rds_rt2[8] = '\0';

		mess_rds_rt3[0] = mess_rds_rt[16];
		mess_rds_rt3[1] = mess_rds_rt[17];
		mess_rds_rt3[2] = mess_rds_rt[18];
		mess_rds_rt3[3] = mess_rds_rt[19];
		mess_rds_rt3[4] = mess_rds_rt[20];
		mess_rds_rt3[5] = mess_rds_rt[21];
		mess_rds_rt3[6] = mess_rds_rt[22];
		mess_rds_rt3[7] = mess_rds_rt[23];
		mess_rds_rt3[8] = '\0';

		mess_rds_rt4[0] = mess_rds_rt[24];
		mess_rds_rt4[1] = mess_rds_rt[25];
		mess_rds_rt4[2] = mess_rds_rt[26];
		mess_rds_rt4[3] = mess_rds_rt[27];
		mess_rds_rt4[4] = mess_rds_rt[28];
		mess_rds_rt4[5] = mess_rds_rt[29];
		mess_rds_rt4[6] = mess_rds_rt[30];
		mess_rds_rt4[7] = mess_rds_rt[31];
		mess_rds_rt4[8] = '\0';
	}
	else if (last_index <= 40)
	{
		//change_rds_display = 5;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';

		mess_rds_rt2[0] = mess_rds_rt[8];
		mess_rds_rt2[1] = mess_rds_rt[9];
		mess_rds_rt2[2] = mess_rds_rt[10];
		mess_rds_rt2[3] = mess_rds_rt[11];
		mess_rds_rt2[4] = mess_rds_rt[12];
		mess_rds_rt2[5] = mess_rds_rt[13];
		mess_rds_rt2[6] = mess_rds_rt[14];
		mess_rds_rt2[7] = mess_rds_rt[15];
		mess_rds_rt2[8] = '\0';

		mess_rds_rt3[0] = mess_rds_rt[16];
		mess_rds_rt3[1] = mess_rds_rt[17];
		mess_rds_rt3[2] = mess_rds_rt[18];
		mess_rds_rt3[3] = mess_rds_rt[19];
		mess_rds_rt3[4] = mess_rds_rt[20];
		mess_rds_rt3[5] = mess_rds_rt[21];
		mess_rds_rt3[6] = mess_rds_rt[22];
		mess_rds_rt3[7] = mess_rds_rt[23];
		mess_rds_rt3[8] = '\0';

		mess_rds_rt4[0] = mess_rds_rt[24];
		mess_rds_rt4[1] = mess_rds_rt[25];
		mess_rds_rt4[2] = mess_rds_rt[26];
		mess_rds_rt4[3] = mess_rds_rt[27];
		mess_rds_rt4[4] = mess_rds_rt[28];
		mess_rds_rt4[5] = mess_rds_rt[29];
		mess_rds_rt4[6] = mess_rds_rt[30];
		mess_rds_rt4[7] = mess_rds_rt[31];
		mess_rds_rt4[8] = '\0';

		mess_rds_rt5[0] = mess_rds_rt[32];
		mess_rds_rt5[1] = mess_rds_rt[33];
		mess_rds_rt5[2] = mess_rds_rt[34];
		mess_rds_rt5[3] = mess_rds_rt[35];
		mess_rds_rt5[4] = mess_rds_rt[36];
		mess_rds_rt5[5] = mess_rds_rt[37];
		mess_rds_rt5[6] = mess_rds_rt[38];
		mess_rds_rt5[7] = mess_rds_rt[39];
		mess_rds_rt5[8] = '\0';
	}
	else
	{
		//change_rds_display = 6;
		mess_rds_rt1[0] = mess_rds_rt[0];
		mess_rds_rt1[1] = mess_rds_rt[1];
		mess_rds_rt1[2] = mess_rds_rt[2];
		mess_rds_rt1[3] = mess_rds_rt[3];
		mess_rds_rt1[4] = mess_rds_rt[4];
		mess_rds_rt1[5] = mess_rds_rt[5];
		mess_rds_rt1[6] = mess_rds_rt[6];
		mess_rds_rt1[7] = mess_rds_rt[7];
		mess_rds_rt1[8] = '\0';

		mess_rds_rt2[1] = mess_rds_rt[9];
		mess_rds_rt2[0] = mess_rds_rt[8];
		mess_rds_rt2[2] = mess_rds_rt[10];
		mess_rds_rt2[3] = mess_rds_rt[11];
		mess_rds_rt2[4] = mess_rds_rt[12];
		mess_rds_rt2[5] = mess_rds_rt[13];
		mess_rds_rt2[6] = mess_rds_rt[14];
		mess_rds_rt2[7] = mess_rds_rt[15];
		mess_rds_rt2[8] = '\0';

		mess_rds_rt3[0] = mess_rds_rt[16];
		mess_rds_rt3[1] = mess_rds_rt[17];
		mess_rds_rt3[2] = mess_rds_rt[18];
		mess_rds_rt3[3] = mess_rds_rt[19];
		mess_rds_rt3[4] = mess_rds_rt[20];
		mess_rds_rt3[5] = mess_rds_rt[21];
		mess_rds_rt3[6] = mess_rds_rt[22];
		mess_rds_rt3[7] = mess_rds_rt[23];
		mess_rds_rt3[8] = '\0';

		mess_rds_rt4[0] = mess_rds_rt[24];
		mess_rds_rt4[1] = mess_rds_rt[25];
		mess_rds_rt4[2] = mess_rds_rt[26];
		mess_rds_rt4[3] = mess_rds_rt[27];
		mess_rds_rt4[4] = mess_rds_rt[28];
		mess_rds_rt4[5] = mess_rds_rt[29];
		mess_rds_rt4[6] = mess_rds_rt[30];
		mess_rds_rt4[7] = mess_rds_rt[31];
		mess_rds_rt4[8] = '\0';

		mess_rds_rt5[0] = mess_rds_rt[32];
		mess_rds_rt5[1] = mess_rds_rt[33];
		mess_rds_rt5[2] = mess_rds_rt[34];
		mess_rds_rt5[3] = mess_rds_rt[35];
		mess_rds_rt5[4] = mess_rds_rt[36];
		mess_rds_rt5[5] = mess_rds_rt[37];
		mess_rds_rt5[6] = mess_rds_rt[38];
		mess_rds_rt5[7] = mess_rds_rt[39];
		mess_rds_rt5[8] = '\0';

		mess_rds_rt6[0] = mess_rds_rt[40];
		mess_rds_rt6[1] = mess_rds_rt[41];
		mess_rds_rt6[2] = mess_rds_rt[42];
		mess_rds_rt6[3] = mess_rds_rt[43];
		mess_rds_rt6[4] = mess_rds_rt[44];
		mess_rds_rt6[5] = mess_rds_rt[45];
		mess_rds_rt6[6] = mess_rds_rt[46];
		mess_rds_rt6[7] = mess_rds_rt[47];
		mess_rds_rt6[8] = '\0';
	}
	rds_info_ready_to_display = true;
	return last_index;
}

void clear_buffers()
{
	change_rds_display = 1;
	clear_rds_buffers(radioTextPrevious, 65);
    clear_rds_buffers(rdsRadioText, 65);
    clear_rds_buffers(mess_rds_rt, 64);
	clear_rds_buffers(rdsProgramType, 17);
    clear_rds_buffers(rdsProgramService, 50);

    clear_rds_buffers(mess_rds_rt1, 8);
    clear_rds_buffers(mess_rds_rt2, 8);
    clear_rds_buffers(mess_rds_rt3, 8);
    clear_rds_buffers(mess_rds_rt4, 8);
    clear_rds_buffers(mess_rds_rt5, 8);
    clear_rds_buffers(mess_rds_rt6, 8);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_9)                  // PA9 M1
	 {
	    print_serial2_message("M1 pressed");
	    m1_button_pressed = true;
	    freq_vol_changed_manual = true;
	    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
	    clear_buffers();
	    clear_display();
	 }
	 if(GPIO_Pin == GPIO_PIN_10)                 // PA10 M2
	 {
	    print_serial2_message("M2 pressed");
	    m2_button_pressed = true;
	    freq_vol_changed_manual = true;
	    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
	    clear_buffers();
	    clear_display();
	 }
	 if(GPIO_Pin == GPIO_PIN_11)                 // PA11 M3
	 {
	    print_serial2_message("M3 pressed");
	    m3_button_pressed = true;
	    freq_vol_changed_manual = true;
	    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
	    clear_buffers();
	    clear_display();
	 }
	 if(GPIO_Pin == GPIO_PIN_12)                 // PA12 M4
	 {
	    print_serial2_message("M4 pressed");
	    m4_button_pressed = true;
	    freq_vol_changed_manual = true;
	    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
	    clear_buffers();
	    clear_display();
	 }
	 if(GPIO_Pin == GPIO_PIN_15)                 // PA15 M5
	 {
	    print_serial2_message("M5 pressed");
	    m5_button_pressed = true;
	    freq_vol_changed_manual = true;
	    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
	    clear_buffers();
	    clear_display();
	 }
	 if(GPIO_Pin == PAJ_interrupt_Pin)
	 {
		 PAJ_event = true;
	 }
}
void read_write_memory_stations_vol()
{
	/////////////////////////// vol start /////////////////////////////////////////////////////
	save_volume(encoder_reading_v);


	/////////////////////////// vol end //////////////////////////////////////////////////////


	////////////////////////  M1 start ////////////////////////////////////////////////////////
			 if(m1_button_pressed == true)  //
			 {
				 static uint32_t current_count = 0;
				 if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9) == 0)  // is PA9 still pressed?
				 {

	     	    	 current_count++;
	                 if (current_count > 600000)  //
	                 {
	                     current_count = 0;
	                     save_to_flash_m1 = true;
	        			 m1_button_pressed = false;
	        			 read_from_flash_m1 = false;
	                 }
				 }
				 else
				 {
					 read_from_flash_m1 = true;
					 current_count = 0;
				 }
			 }
			 if(read_from_flash_m1 == true)
			 {
				 m1_button_pressed = false;
				 uint8_t curr_freq_low = W25Q_Read_Byte(1);
				 uint8_t curr_freq_high = W25Q_Read_Byte(2);
				 uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
				 setFrequency(curr_freq);
				 clear_display();
				 curr_freq = getFrequency();
				 TIM3->CNT = ((curr_freq - 8750)/10)<<1;
				 disp_freq(curr_freq);
				 freq = curr_freq;
				 populate_freq_array(freq);
				 read_from_flash_m1 = false;
			 }
			 if(save_to_flash_m1 == true)
			 {
				 uint16_t current_freq = getFrequency();
				 uint8_t current_freq_low = current_freq & 0xff;
				 uint8_t current_freq_high = (current_freq >> 8);
				 __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer
				 W25Q_Erase_Sector(0);
				 W25Q_Write_Byte(1, current_freq_low);
				 W25Q_Write_Byte(2, current_freq_high);
				 save_to_flash_m1 = false;
				 display_static_message(memory1);
				 uint32_t a;
				 for (uint32_t i = 0; i < 14900000; i++)
				 {
					 HAL_IWDG_Refresh(&hiwdg);
				    a++;
				 }
				 read_from_flash_m1 = true;
			 }
	/////////////////////////  M1 end ////////////////////////////////////////////////////////

	/////////////////////////  M2 start ////////////////////////////////////////////////////////
			 if(m2_button_pressed == true)  //
			 {
				static uint32_t current_count = 0;
			 	if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10) == 0)  // is PA10 still pressed?
			 	{

			      	 current_count++;
			         if (current_count > 600000)  //
			         {
			            current_count = 0;
			            save_to_flash_m2 = true;
			            m2_button_pressed = false;
			         	read_from_flash_m2 = false;
			         }
			 	}
			 	else
			 	{
			 		read_from_flash_m2 = true;
			 		current_count = 0;
			 	}
			 }
			 if(read_from_flash_m2 == true)
			 {
			 	m2_button_pressed = false;
			 	uint8_t curr_freq_low = W25Q_Read_Byte(4097);
			 	uint8_t curr_freq_high = W25Q_Read_Byte(4098);
			 	uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
			 	setFrequency(curr_freq);
			 	clear_display();
			 	curr_freq = getFrequency();
			 	TIM3->CNT = ((curr_freq - 8750)/10)<<1;
			 	disp_freq(curr_freq);
			 	freq = curr_freq;
			 	populate_freq_array(freq);
			 	read_from_flash_m2 = false;
			 }
			 if(save_to_flash_m2 == true)
			 {
			 	uint16_t current_freq = getFrequency();
			 	uint8_t current_freq_low = current_freq & 0xff;
			 	uint8_t current_freq_high = (current_freq >> 8);
			    __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer
			 	W25Q_Erase_Sector(1);
			 	W25Q_Write_Byte(4097, current_freq_low);
			 	W25Q_Write_Byte(4098, current_freq_high);
			 	save_to_flash_m2 = false;
			 	display_static_message(memory2);
			 	uint32_t a;
			 	for (uint32_t i = 0; i < 14900000; i++)
			 	{
			 		HAL_IWDG_Refresh(&hiwdg);
			 		a++;
			    }
			 	read_from_flash_m2 = true;
			 }
	/////////////////////////  M2 end ////////////////////////////////////////////////////////

	/////////////////////////  M3 start ////////////////////////////////////////////////////////
			 if(m3_button_pressed == true)  //
			 {
				static uint32_t current_count = 0;
			 	if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_11) == 0)  // is PA11 still pressed?
			 	{
			 		current_count++;
			 		 if (current_count > 600000)
			 		 {
			 		    current_count = 0;
			 		    save_to_flash_m3 = true;
			 		    m3_button_pressed = false;
			 		 	read_from_flash_m3 = false;
			 		 }
			 	}
			 	else
			 	{
			 		 read_from_flash_m3 = true;
			 		 current_count = 0;
			 	}
			 }
			 if(read_from_flash_m3 == true)
			 {
			 	m3_button_pressed = false;
			 	uint8_t curr_freq_low = W25Q_Read_Byte(8193);
			 	uint8_t curr_freq_high = W25Q_Read_Byte(8194);
			 	uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
			 	setFrequency(curr_freq);
			 	clear_display();
			 	curr_freq = getFrequency();
			 	TIM3->CNT = ((curr_freq - 8750)/10)<<1;
			 	disp_freq(curr_freq);
			 	freq = curr_freq;
			 	populate_freq_array(freq);
			 	read_from_flash_m3 = false;
			 }
			 if(save_to_flash_m3 == true)
			{
			 	uint16_t current_freq = getFrequency();
			 	uint8_t current_freq_low = current_freq & 0xff;
			 	uint8_t current_freq_high = (current_freq >> 8);
			 	__HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer
			 	W25Q_Erase_Sector(2);
			 	W25Q_Write_Byte(8193, current_freq_low);
			 	W25Q_Write_Byte(8194, current_freq_high);
			 	save_to_flash_m3 = false;
			 	display_static_message(memory3);
			 	uint32_t a;
			 	for (uint32_t i = 0; i < 14900000; i++)
			 	{
			 		HAL_IWDG_Refresh(&hiwdg);
			 		a++;
			 	}
			 	read_from_flash_m3 = true;
			}
	/////////////////////////  M3 end ////////////////////////////////////////////////////////

	/////////////////////////  M4 start ////////////////////////////////////////////////////////
			 if(m4_button_pressed == true)  //
			{
				static uint32_t current_count = 0;
				if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_12) == 0)  // is PA12 still pressed?
				{
					 current_count++;
					 if (current_count > 600000)  //
					 {
						 current_count = 0;
						 save_to_flash_m4 = true;
						 m4_button_pressed = false;
						 read_from_flash_m4 = false;
					 }
				}
				else
					 {
					 	read_from_flash_m4 = true;
					    current_count = 0;
					 }
			}
			 if(read_from_flash_m4 == true)
			{
				m4_button_pressed = false;
				uint8_t curr_freq_low = W25Q_Read_Byte(12289);
				uint8_t curr_freq_high = W25Q_Read_Byte(12290);
				uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
				setFrequency(curr_freq);
				clear_display();
				curr_freq = getFrequency();
				TIM3->CNT = ((curr_freq - 8750)/10)<<1;
				disp_freq(curr_freq);
				freq = curr_freq;
				populate_freq_array(freq);
				read_from_flash_m4 = false;
			}
			 if(save_to_flash_m4 == true)
			{
				uint16_t current_freq = getFrequency();
				uint8_t current_freq_low = current_freq & 0xff;
				uint8_t current_freq_high = (current_freq >> 8);
				__HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer
				W25Q_Erase_Sector(3);
				W25Q_Write_Byte(12289, current_freq_low);
				W25Q_Write_Byte(12290, current_freq_high);
				save_to_flash_m4 = false;
				display_static_message(memory4);
				uint32_t a;
				for (uint32_t i = 0; i < 14900000; i++)
				{
					 HAL_IWDG_Refresh(&hiwdg);
					 a++;
				}
				read_from_flash_m4 = true;
			}
	/////////////////////////  M4 end ////////////////////////////////////////////////////////

	/////////////////////////  M5 start ////////////////////////////////////////////////////////
			 if(m5_button_pressed == true)  //
			{
				static uint32_t current_count = 0;
				if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) == 0)  // is PA15 still pressed?
				{
					 current_count++;
					 if (current_count > 600000)
					 {
					 	current_count = 0;
			 		    save_to_flash_m5 = true;
					 	m5_button_pressed = false;
					 	read_from_flash_m5 = false;
					 }
				}
				else
				{
					read_from_flash_m5 = true;
				 	current_count = 0;
				}
			}
			 if(read_from_flash_m5 == true)
			{
				m5_button_pressed = false;
				uint8_t curr_freq_low = W25Q_Read_Byte(16385);
				uint8_t curr_freq_high = W25Q_Read_Byte(16386);
				uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
				setFrequency(curr_freq);
				clear_display();
				curr_freq = getFrequency();
				TIM3->CNT = ((curr_freq - 8750)/10)<<1;
				disp_freq(curr_freq);
				freq = curr_freq;
				populate_freq_array(freq);
				read_from_flash_m5 = false;
			}
			 if(save_to_flash_m5 == true)
			{
				uint16_t current_freq = getFrequency();
				uint8_t current_freq_low = current_freq & 0xff;
				uint8_t current_freq_high = (current_freq >> 8);
				__HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer
				W25Q_Erase_Sector(4);
				W25Q_Write_Byte(16385, current_freq_low);
				W25Q_Write_Byte(16386, current_freq_high);
				save_to_flash_m5 = false;
				display_static_message(memory5);
				uint32_t a;
				for (uint32_t i = 0; i < 14900000; i++)
				{
					HAL_IWDG_Refresh(&hiwdg);
					 a++;
				}
				read_from_flash_m5 = true;
			}
	/////////////////////////  M5 end ////////////////////////////////////////////////////////

}

void save_volume(uint8_t vol)
{
	if(volum_changed == true)
	{
		if(timer4 >= 1)
		{
			save_to_flash_volume = true;
			timer4 = 0;
		}
	}
	if(save_to_flash_volume == true)
	{
		save_to_flash_volume = false;
		volum_changed = false;
		W25Q_Erase_Sector(5);
	    W25Q_Write_Byte(20481, vol);
	}
}
void load_default_station_and_volume()
{
	  uint8_t curr_freq_low = W25Q_Read_Byte(1);
	  uint8_t curr_freq_high = W25Q_Read_Byte(2);
	  uint16_t curr_freq = curr_freq_low | (curr_freq_high << 8);
	  setFrequency(curr_freq);
	  uint16_t current_freq = getFrequency();
	  TIM3->CNT = ((current_freq - 8750)/10)<<1;
	  uint8_t curr_vol = W25Q_Read_Byte(20481);
	  if(curr_vol == 255)
		  curr_vol = 10;
	  int vol = map(curr_vol, 0, 30, -40, 15);   // map(value, fromLow, fromHigh, toLow, toHigh)
	  setVolume(vol);
	  TIM2->CNT = curr_vol<<1;
	  volum = curr_vol;
}
void read_volume_encoder()
{
	encoder_reading_v = (TIM2->CNT>>1);
	if (encoder_reading_v > 900)
	{
		TIM2->CNT = 1;                         //limit vol 0
		encoder_reading_v = (TIM2->CNT>>1);
	}
	if (encoder_reading_v > 30)
	{
		TIM2->CNT = 60;                         //limit vol = 30
		encoder_reading_v = (TIM2->CNT>>1);
	}
	if(encoder_reading_v != old_encoder_reading_v)
	{
	   freq_vol_changed_manual = true;
	   __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)

	   volum_changed = true;
	   __HAL_TIM_SET_COUNTER(&htim4, 64000); // reset timer (countdown timer)
	   timer4 = 0;
	   // map(value, fromLow, fromHigh, toLow, toHigh)
	   int vol = map(encoder_reading_v, 0, 30, -40, 15);
	   setVolume(vol);
	   sprintf(message_volume, "Vol = %i  \r", vol);
	   print_serial2_message(message_volume);
	   clear_display();
	   disp_vol(encoder_reading_v);
	   volum = encoder_reading_v;
	}
	old_encoder_reading_v = encoder_reading_v;
}

void read_frequency_encoder()
{
	encoder_reading_f = 10*(TIM3->CNT>>1);
	if(encoder_reading_f > 5000)
	{
		TIM3->CNT = 1;                         //limit f=87.5 Mhz
		encoder_reading_f = 10*(TIM3->CNT>>1);
	}
	if(encoder_reading_f > 2050)
	{
		TIM3->CNT = 410;                       //limit f=108 Mhz
		encoder_reading_f = 10*(TIM3->CNT>>1);
	}
	if(encoder_reading_f != old_encoder_reading_f)
	{
		//disablePAJsensor();
		freq_vol_changed_manual = true;
		__HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
		clear_buffers();
	    freq = 8750 + encoder_reading_f;
		setFrequency(freq);
	    sprintf(message_frequency, "Frecv = %li MHz \r", freq);
	    print_serial2_message(message_frequency);
	    clear_display();
	    disp_freq(freq);
	    uint16_t current_freq = getFrequency();
	    disp_freq(current_freq);
	    freq = current_freq;
	    populate_freq_array(freq);
	}
	old_encoder_reading_f = encoder_reading_f;
}
void show_rds_informations()
{
	if(contor >= 10000)
	{
		contor = 0;
		display_rds_info();
		print_serial2_message("==============================");
		if(isRDSReady)
		{
		    if(freq_vol_changed_manual == false)
		    {
		    	rds_string_lenght = prepare_rds_for_display(radioTextPrevious);
		    	if(rds_info_ready_to_display)
		    	{
		    		 if(rds_string_lenght <= 2)
		    		 {
					     change_rds_display = 1;
		    		 }
		    	     if(change_rds_display == 1)
		    	     {
		    	    	if(rds_string_lenght <= 2)  // if no rds data show ps
		    	    	{
						     prepare_rds_for_display(programServicePrevious);
						     display_static_message(mess_rds_ps);
						     change_rds_display = 1;
		    	    	}
		        	    if ((rds_string_lenght > 2) && (rds_string_lenght <= 8)) // 3-8
		        	    {
		        	    	display_static_message(mess_rds_rt1);
		        	    	change_rds_display = 1;
		        	    }
		        	    if (rds_string_lenght > 8)
		        	    {
		        	    	display_static_message(mess_rds_rt1);
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 2;
                            }
		        	    }
		    	    }
		    	    else if (change_rds_display == 2)
		    	    {
//			    	    	if ((mess_rds_rt2[0] ==
//			    	    		 mess_rds_rt2[1] ==
//			    	    		 mess_rds_rt2[2] ==
//			    	    		 mess_rds_rt2[3] ==
//					    	     mess_rds_rt2[4] ==
//					    	     mess_rds_rt2[5] ==
//					    	     mess_rds_rt2[6] ==
//					    	     mess_rds_rt2[7] == ' ') ||
//					    	    (mess_rds_rt2[0] ==
//					    	     mess_rds_rt2[1] ==
//					    	     mess_rds_rt2[2] ==
//					    	     mess_rds_rt2[3] ==
//					    	     mess_rds_rt2[4] ==
//					    	     mess_rds_rt2[5] ==
//					    	     mess_rds_rt2[6] ==
//					    	     mess_rds_rt2[7] == '\0'))
//			    	    	{
//			    	    		change_rds_display = 3;
//			    	    	}
//			    	    	else
//			    	    	{
//			    	    		 display_static_message(mess_rds_rt2);
//			    	    	}
		    	        display_static_message(mess_rds_rt2);
		        	    if ((rds_string_lenght > 8) && (rds_string_lenght <= 16)) // 9-16
		        	    {

		        	    }
		        	    else if (rds_string_lenght > 16)
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 3;
                            }
		        	    }
		    	    }
		    	    else if (change_rds_display == 3)
		    	    {
//			    	    	if ((mess_rds_rt3[0] ==
//			    	    		 mess_rds_rt3[1] ==
//			    	    		 mess_rds_rt3[2] ==
//			    	    		 mess_rds_rt3[3] ==
//					    	     mess_rds_rt3[4] ==
//					    	     mess_rds_rt3[5] ==
//					    	     mess_rds_rt3[6] ==
//					    	     mess_rds_rt3[7] == ' ') ||
//					    	    (mess_rds_rt3[0] ==
//					    	     mess_rds_rt3[1] ==
//					    	     mess_rds_rt3[2] ==
//					    	     mess_rds_rt3[3] ==
//					    	     mess_rds_rt3[4] ==
//					    	     mess_rds_rt3[5] ==
//					    	     mess_rds_rt3[6] ==
//					    	     mess_rds_rt3[7] == '\0'))
//			    	    	{
//			    	    		change_rds_display = 4;
//			    	    	}
//			    	    	else
//			    	    	{
//			    	    		 display_static_message(mess_rds_rt3);
//			    	    	}
		        	    display_static_message(mess_rds_rt3);
		        	    if ((rds_string_lenght > 16) && (rds_string_lenght <= 24)) // 17-24
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 1;
                            }
		        	    }
		        	    else if (rds_string_lenght > 24)
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 4;
                            }
		        	    }
		    	    }
		    	    else if (change_rds_display == 4)
		    	    {
//			    	    	if ((mess_rds_rt4[0] ==
//			    	    		 mess_rds_rt4[1] ==
//			    	    		 mess_rds_rt4[2] ==
//			    	    		 mess_rds_rt4[3] ==
//					    	     mess_rds_rt4[4] ==
//					    	     mess_rds_rt4[5] ==
//					    	     mess_rds_rt4[6] ==
//					    	     mess_rds_rt4[7] == ' ') ||
//					    	    (mess_rds_rt4[0] ==
//					    	     mess_rds_rt4[1] ==
//					    	     mess_rds_rt4[2] ==
//					    	     mess_rds_rt4[3] ==
//					    	     mess_rds_rt4[4] ==
//					    	     mess_rds_rt4[5] ==
//					    	     mess_rds_rt4[6] ==
//					    	     mess_rds_rt4[7] == '\0'))
//			    	    	{
//			    	    		change_rds_display = 5;
//			    	    	}
//			    	    	else
//			    	    	{
//			    	    		 display_static_message(mess_rds_rt4);
//			    	    	}
		        	    display_static_message(mess_rds_rt4);
		        	    if ((rds_string_lenght > 24) && (rds_string_lenght <= 32))  // 25-32
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 1;
                            }
		        	    }
		        	    else if (rds_string_lenght > 32)
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 5;
                            }
		        	    }
		    	    }
		    	    else if (change_rds_display == 5)
		    	    {
//			    	    	if ((mess_rds_rt5[0] ==
//			    	    		 mess_rds_rt5[1] ==
//			    	    		 mess_rds_rt5[2] ==
//			    	    		 mess_rds_rt5[3] ==
//					    	     mess_rds_rt5[4] ==
//					    	     mess_rds_rt5[5] ==
//					    	     mess_rds_rt5[6] ==
//					    	     mess_rds_rt5[7] == ' ') ||
//					    	    (mess_rds_rt5[0] ==
//					    	     mess_rds_rt5[1] ==
//					    	     mess_rds_rt5[2] ==
//					    	     mess_rds_rt5[3] ==
//					    	     mess_rds_rt5[4] ==
//					    	     mess_rds_rt5[5] ==
//					    	     mess_rds_rt5[6] ==
//					    	     mess_rds_rt5[7] == '\0'))
//			    	    	{
//			    	    		change_rds_display = 6;
//			    	    	}
//			    	    	else
//			    	    	{
//			    	    		 display_static_message(mess_rds_rt5);
//			    	    	}
		        	    display_static_message(mess_rds_rt5);
		        	    if ((rds_string_lenght > 32) && (rds_string_lenght <= 40))  // 33-40
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 1;
                            }
		        	    }
		        	    else if (rds_string_lenght > 40)
		        	    {
		        	    	static uint8_t current_count = 0;
		        	    	current_count++;
                            if (current_count > 10)  // time to display last screen
                            {
                                current_count = 0;
                                change_rds_display = 6;
                            }
		        	    }
		    	    }
		        	else if (change_rds_display == 6)
		    	    {
//			    	    	if ((mess_rds_rt6[0] ==
//			    	    		 mess_rds_rt6[1] ==
//			    	    		 mess_rds_rt6[2] ==
//			    	    		 mess_rds_rt6[3] ==
//					    	     mess_rds_rt6[4] ==
//					    	     mess_rds_rt6[5] ==
//					    	     mess_rds_rt6[6] ==
//					    	     mess_rds_rt6[7] == ' ') ||
//					    	    (mess_rds_rt6[0] ==
//					    	     mess_rds_rt6[1] ==
//					    	     mess_rds_rt6[2] ==
//					    	     mess_rds_rt6[3] ==
//					    	     mess_rds_rt6[4] ==
//					    	     mess_rds_rt6[5] ==
//					    	     mess_rds_rt6[6] ==
//					    	     mess_rds_rt6[7] == '\0'))
//			    	    	{
//			    	    		change_rds_display = 1;
//			    	    	}
//			    	    	else
//			    	    	{
//			    	    		 display_static_message(mess_rds_rt6);
//			    	    	}
		    	    	display_static_message(mess_rds_rt6);
	        	    	static uint8_t current_count = 0;
	        	    	current_count++;
                        if (current_count > 10)  // time to display last screen
                        {
                            current_count = 0;
                            change_rds_display = 1;
                        }
		    	    }
		    	}

		    }
		}
		else  // if(!isRDSReady)
		{
            if((programServicePrevious[0] < 32) && (radioTextPrevious[0] < 32))  // if no information from rds
            {
        	    if(freq_vol_changed_manual == false)
        	    {
        	    	display_scrolling_message(mess_frequency);
        	    }

            }
		}

		HAL_GPIO_TogglePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin);
//			  ID = W25Q_ReadID();
//			  print_serial2_message("==============================-------||||---------|----|-");
//			  print_serial2_message_number("W25Q_ReadID = ", ID);
//			  print_serial2_message("==============================-------||||-------------");
	}
}
void read_seek_up_down_gpio()
{
    if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14) == 0)  // seek up  butonul de freq
    {
  	    clear_display();
  	    display_static_message(search_up);
  	    isFmSeekMode = true;
  	    isFmSeekUp = true;
  	    clear_buffers();
    }
    if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_15) == 0)  // seek down   butonul de volum
    {
			clear_display();
			display_static_message(search_down);
	        isFmSeekMode = true;
	        isFmSeekUp = false;
	        clear_buffers();
    }
}

void read_PAJ_gesture()
{
	 if (PAJ_event)
	  {
		  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
		  gesture = readGesture();          // Read back current gesture (if any) of type Gesture
		  switch (gesture)
		  {
		    case GES_FORWARD:
		      {
		    	  char str1[] = "GES_FORWARD";
		    	  print_serial2_message(str1);

		        break;
		      }
		    case GES_BACKWARD:
		      {
		    	  char str1[] = "GES_BACKWARD";
		    	  print_serial2_message(str1);

		        break;
		      }
		    case GES_LEFT:
		      {
		    	  //char str1[] = "GES_LEFT";
		    	  char str1[] = "GES_RIGHT";
		    	  print_serial2_message(str1);
		    	  clear_display();
		    	  display_static_message(search_up);   // seekup
		    	  isFmSeekMode = true;
		    	  isFmSeekUp = true;
		    	  clear_buffers();
		          break;
		      }
		    case GES_RIGHT:
		      {
		    	  //char str1[] = "GES_RIGHT";
		    	  char str1[] = "GES_LEFT";
		    	  print_serial2_message(str1);
				  clear_display();
				  display_static_message(search_down);   //seekdown
			      isFmSeekMode = true;
			      isFmSeekUp = false;
			      clear_buffers();
		        break;
		      }
		    case GES_UP:                        // vol-
		      {
		    	  //char str1[] = "GES_UP";
		    	  char str1[] = "GES_DOWN";
		    	  print_serial2_message(str1);
		    	  freq_vol_changed_manual = true;
		    	  __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
                  volum = volum - 1;
                  if(volum <= 0)
                    volum = 0;
		    	  int vol = map(volum, 0, 30, -40, 15);
		    	  setVolume(vol);
		    	  TIM2->CNT = volum<<1;
		    	  clear_display();
		    	  disp_vol(volum);
		         break;
		      }
		    case GES_DOWN:                      // vol+
		      {
		    	  //char str1[] = "GES_DOWN";
		    	  char str1[] = "GES_UP";
		    	  print_serial2_message(str1);
		    	  freq_vol_changed_manual = true;
		    	  __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
                  volum = volum + 1;
                  if(volum >= 30)
                    	volum = 30;
		    	  int vol = map(volum, 0, 30, -40, 15);
		    	  setVolume(vol);
		    	  TIM2->CNT = volum<<1;
		    	  clear_display();
		    	  disp_vol(volum);
		          break;
		      }
		    case GES_CLOCKWISE:
		      {
//		    	 char str1[] = "GES_CLOCKWISE";
//		    	 print_serial2_message(str1);
		 		 freq_vol_changed_manual = true;
		 		 __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
		 		 clear_buffers();
		    	 uint16_t current_freq = getFrequency();
		    	 current_freq  = current_freq + 10;
		    	 setFrequency(current_freq);
		    	 TIM3->CNT = ((current_freq - 8750)/10)<<1;
		 	     clear_display();
		 	     disp_freq(freq);
		 	     freq = current_freq;
		 	     populate_freq_array(freq);
		    	 //HAL_Delay(1);
		         break;
		      }
		    case GES_ANTICLOCKWISE:
		      {
//		    	 char str1[] = "GES_ANTICLOCKWISE";
//		    	 print_serial2_message(str1);
		 		 freq_vol_changed_manual = true;
		 		 __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
		 		 clear_buffers();
		    	 uint16_t current_freq = getFrequency();
		    	 current_freq  = current_freq - 10;
		    	 setFrequency(current_freq);
		    	 TIM3->CNT = ((current_freq - 8750)/10)<<1;
		 	     clear_display();
		 	     disp_freq(freq);
		 	     freq = current_freq;
		 	     populate_freq_array(freq);
		    	 //HAL_Delay(1);
		         break;
		      }
		    case GES_WAVE:
		      {
		    	  char str1[] = "GES_WAVE";
		    	  print_serial2_message(str1);
		    	  freq_vol_changed_manual = true;
		    	  clear_display();
		    	  display_static_message(hello);   // radio display hello when you waving :)
		        break;
		      }
		    case GES_NONE:
		      {
		        break;
		      }
		  }

//		  if( gesture != GES_NONE )
//		  {
//			  char str1[16];
//			  sprintf(str1, ", Code: %i \n\r", gesture);
//			  print_serial2_message(str1);
//		  }
		  PAJ_event = false;
		  HAL_Delay(5);
	  }
}

//void read_PAJ_gesture_2()
//{
//    uint8_t data = 0, data1 = 0, error;
//
//    error = paj7620ReadReg(0x43, 1, &data);				// Read Bank_0_Reg_0x43/0x44 for gesture result.
//    if (!error) {
//        switch (data) {								// When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
//            case GES_RIGHT_FLAG:
//            	HAL_Delay(GES_ENTRY_TIME);
//                paj7620ReadReg(0x43, 1, &data);
//                if (data == GES_FORWARD_FLAG)
//                {
//                    print_serial2_message("Forward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else if (data == GES_BACKWARD_FLAG)
//                {
//                    print_serial2_message("Backward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else
//                {
//                    print_serial2_message("Right");
//                }
//                break;
//            case GES_LEFT_FLAG:
//            	HAL_Delay(GES_ENTRY_TIME);
//                paj7620ReadReg(0x43, 1, &data);
//                if (data == GES_FORWARD_FLAG)
//                {
//                    print_serial2_message("Forward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else if (data == GES_BACKWARD_FLAG)
//                {
//                    print_serial2_message("Backward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else
//                {
//                	print_serial2_message("Left");
//                }
//                break;
//            case GES_UP_FLAG:
//            	HAL_Delay(GES_ENTRY_TIME);
//                paj7620ReadReg(0x43, 1, &data);
//                if (data == GES_FORWARD_FLAG)
//                {
//                    print_serial2_message("Forward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else if (data == GES_BACKWARD_FLAG)
//                {
//                    print_serial2_message("Backward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else
//                {
//                    print_serial2_message("Up");
//                }
//                break;
//            case GES_DOWN_FLAG:
//            	HAL_Delay(GES_ENTRY_TIME);
//                paj7620ReadReg(0x43, 1, &data);
//                if (data == GES_FORWARD_FLAG)
//                {
//                    print_serial2_message("Forward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else if (data == GES_BACKWARD_FLAG)
//                {
//                    print_serial2_message("Backward");
//                    HAL_Delay(GES_QUIT_TIME);
//                }
//                else
//                {
//                    print_serial2_message("Down");
//                }
//                break;
//            case GES_FORWARD_FLAG:
//                print_serial2_message("Forward");
//                HAL_Delay(GES_QUIT_TIME);
//                break;
//            case GES_BACKWARD_FLAG:
//                print_serial2_message("Backward");
//                HAL_Delay(GES_QUIT_TIME);
//                break;
//            case GES_CLOCKWISE_FLAG:
//                print_serial2_message("Clockwise");
//                break;
//            case GES_COUNT_CLOCKWISE_FLAG:
//                print_serial2_message("anti-clockwise");
//                break;
//            default:
//                paj7620ReadReg(0x44, 1, &data1);
//                if (data1 == GES_WAVE_FLAG)
//                {
//                    print_serial2_message("wave");
//                }
//                break;
//        }
//    }
//    HAL_Delay(100);
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4) {        // ~2.7 sec
	  timer4++;

  }

  if (htim->Instance == TIM5) {        // 3 sec
	  freq_vol_changed_manual = false;
//	  change_rds_display++;
//	  if (change_rds_display > 6)
//		  change_rds_display = 1;
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	print_serial2_message("!!! Error_Handler !!!!!!");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

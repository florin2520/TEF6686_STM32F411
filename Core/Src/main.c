/* USER CODE BEGIN Header */

// Functioneaza cu dsp_init declarat in RAM

// TODO
/* de afisat mod stereo cu un punct pe display
 * de rezolvat cautarea automata intr-o singura banda
 * animatii display
 * memorii posturi
 * de ce dureaza bucla while principala atat de mult ??
 *
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TEF6686_I2C_ADDRESS        0x64
//#define DISPLAY_ADDRESS 0x70<<1
#define DISPLAY_ADDRESS 0x70<<1  //
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
extern char rdsProgramService[9];
extern char rdsProgramType[17];

//char display_buffer[] = "        "; // 8 caractere

//char message_freq_static[13];

uint16_t seek_frequency;
bool isFmSeekMode;
bool isFmSeekUp;
uint8_t current_band;
uint8_t stereo_status; //0 mono, 1 stereo

extern char radioTextPrevious[65];
extern char rdsProgramService[9];

char message_to_scroll_1[] = "ABCD";                                  // 4+1
char message_to_scroll_2[] = "MESAJUL2";                              // 8+1
char message_to_scroll_3[] = "ACEST MESAJ3";                          // 12+1
char message_to_scroll_4[] = "MESSAGE TO SCROLL ON DISPLAY  ";        // 30+1
char message_to_scroll_5[] = "MESAJUL DE AVERTIZARE CU NUMARUL 5  ";  // 36+1


char message_1char[] = "1";                                // 1+1
char message_2char[] = "12";                               // 2+1
char message_3char[] = "123 ";                              // 3+1

char message_5char[] = "UNU12";                                // 5+1
char message_6char[] = "UNU123";                               // 6+1
char message_7char[] = "UNU1234";                              // 7+1

char message_9char[] = "UNU123456";                                //
char message_10char[] = "UNU1234567";                               //
char message_11char[] = "UNU12345678";                              //

extern char mess_frequency[30];
extern char mess_freq_static[30];
extern char mess_volume[30];

uint8_t test_timer4;
bool freq_vol_changed_manual;
/* USER CODE END PV */

/* Private function prototypes --------------s---------------------------------*/
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);  // volum
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);  // frecventa
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin, GPIO_PIN_RESET);

  print_serial2_message("Start radio...");
  //start radio
  uint8_t start_rd_ok = init_radio();
  powerOn();

  Radio_SetBand(FM1_BAND); // ??
  current_band = Radio_GetCurrentBand(); // 1 => 65-108
  setFrequency(8870);
  setVolume(-10);// -60 --> 24
  sprintf(message_frequency, "START RADIO CODE = %i \r", start_rd_ok); // 1 = OK, 2 = Doesn't exist, 0 = busy
  print_serial2_message(message_frequency);

  uint16_t current_freq = getFrequency();
  sprintf(message_frequency, "curentFrecv = %i MHz \r", current_freq);
  print_serial2_message(message_frequency);

  HAL_Delay(500);
  HAL_GPIO_WritePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin, GPIO_PIN_SET);

  begin_disp(DISPLAY_ADDRESS);  // pass in the address
    //clear_entire_display();
  writeDigitAscii(0, 'F', false);
  writeDigitAscii(1, 'M', false);
  writeDigitAscii(2, ' ', false);
  writeDigitAscii(3, 'R', false);
  writeDigitAscii(4, 'A', false);
  writeDigitAscii(5, 'D', false);
  writeDigitAscii(6, 'I', false);
  writeDigitAscii(7, 'O', false);
  writeDisplay(DISPLAY_ADDRESS);
  HAL_Delay(1500);
  clear_display();
  freq_vol_changed_manual = true;
  HAL_TIM_Base_Start_IT(&htim4); // Enable the timer
  HAL_TIM_Base_Start_IT(&htim5); // Enable the timer
  freq = current_freq;
  populate_freq_array(freq);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      contor++;
      //print_serial2_message_number("contor = ", contor);

      showFmSeek();
     //stereo_status = getStereoStatus(); // very very slow function!!!
      if(freq_vol_changed_manual == false)
      {
    	  //HAL_TIM_Base_Start_IT(&htim5);
    	  display_scrolling_message(mess_frequency);
      }

	  //display_scrolling_message(rdsProgramService);
      //display_static_message(rdsProgramService);


      // IF Button seek up Is Pressed
      if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14) == 0)  // seek up  butonul de freq
      {
    	    clear_display();
    	   	writeDigitAscii(0, 'S', false);
    	    writeDigitAscii(1, 'E', false);
    	    writeDigitAscii(2, 'A', false);
    	    writeDigitAscii(3, 'R', false);
    	    writeDigitAscii(4, 'C', false);
    	    writeDigitAscii(5, 'H', false);
    	    writeDigitAscii(6, '>', false);
    	    writeDigitAscii(7, '>', false);
    	    writeDisplay(DISPLAY_ADDRESS);
    	    isFmSeekMode = true;
    	    isFmSeekUp = true;
      }

      // IF Button seek down Is Pressed
      if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_15) == 0)  // seek down   butonul de volum
      {
			clear_display();
			writeDigitAscii(0, '<', false);
			writeDigitAscii(1, '<', false);
			writeDigitAscii(2, 'S', false);
			writeDigitAscii(3, 'E', false);
			writeDigitAscii(4, 'A', false);
			writeDigitAscii(5, 'R', false);
			writeDigitAscii(6, 'C', false);
			writeDigitAscii(7, 'H', false);
			writeDisplay(DISPLAY_ADDRESS);
  	        isFmSeekMode = true;
  	        isFmSeekUp = false;
      }

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
		 // map(value, fromLow, fromHigh, toLow, toHigh)
		   freq_vol_changed_manual = true;
		  // HAL_TIM_Base_Stop_IT(&htim5);
		   __HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)

		   int vol = map(encoder_reading_v, 0, 30, -40, 15);
		   setVolume(vol);
		   sprintf(message_volume, "Vol = %i  \r", vol);
		   print_serial2_message(message_volume);
		   clear_display();
		   disp_vol(encoder_reading_v);
		}
		old_encoder_reading_v = encoder_reading_v;




		encoder_reading_f = 10*(TIM3->CNT>>1);
		if (encoder_reading_f > 5000)
		{
			TIM3->CNT = 1;                         //limit f=87.5 Mhz
			encoder_reading_f = 10*(TIM3->CNT>>1);
		}
		if (encoder_reading_f > 2050)
		{
			TIM3->CNT = 410;                       //limit f=108 Mhz
			encoder_reading_f = 10*(TIM3->CNT>>1);
		}
		if(encoder_reading_f != old_encoder_reading_f)
		{
			freq_vol_changed_manual = true;
			__HAL_TIM_SET_COUNTER(&htim5, 9999); // reset timer (countdown timer)
			clear_rds_buffers(rdsProgramType,17);
		    clear_rds_buffers(rdsProgramService, 9);
		    clear_rds_buffers(rdsRadioText, 65);

		    freq = 8750 + encoder_reading_f;
			setFrequency(freq);
		    sprintf(message_frequency, "Frecv = %li MHz \r", freq);
		    print_serial2_message(message_frequency);
		    clear_display();
		    disp_freq(freq);
		    current_freq = getFrequency();
		    disp_freq(current_freq);
		    freq = current_freq;
		    populate_freq_array(freq);
		}
		old_encoder_reading_f = encoder_reading_f;

		if(contor >= 50000)
		{
			contor = 0;
			//display_rds_info();
			print_serial2_message("==================");
			HAL_GPIO_TogglePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin);
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  if (htim->Instance == TIM4) {        // 0.5 sec
	  test_timer4++;
  }

  if (htim->Instance == TIM5) {        // 3 sec
	  freq_vol_changed_manual = false;
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

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


char message_to_scroll_1[] = "ABCD";                                  // 4+1
char message_to_scroll_2[] = "MESAJUL2";                              // 8+1
char message_to_scroll_3[] = "ACEST MESAJ3";                          // 12+1
char message_to_scroll_4[] = "MESSAGE TO SCROLL ON DISPLAY  ";        // 30+1
char message_to_scroll_5[] = "MESAJUL DE AVERTIZARE CU NUMARUL 5  ";  // 36+1

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

uint8_t volatile test_timer4;
uint8_t volatile change_rds_display;
bool volatile freq_vol_changed_manual;
uint8_t rds_string_lenght;
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
  change_rds_display = 0;
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
 // rds_info_ready_to_display = false;
  change_rds_display = 1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      contor++;
      //print_serial2_message_number("contor = ", contor);

      showFmSeek();
     //stereo_status = getStereoStatus(); // very very slow function!!!

//      if(!isRDSReady)
//      {
//          if(freq_vol_changed_manual == false)
//          {
//        	  //HAL_TIM_Base_Start_IT(&htim5);
//        	  display_scrolling_message(mess_frequency);
//          }
//      }

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
    	    clear_buffers();
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
  	        clear_buffers();
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
			clear_buffers();
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
			        	    	static uint8_t current_count = 0;
			        	    	current_count++;
                                if (current_count > 10)  // time to display last screen
                                {
                                    current_count = 0;
                                    change_rds_display = 1;
                                }
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
  if (htim->Instance == TIM4) {        // 2 sec

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

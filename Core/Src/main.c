/* USER CODE BEGIN Header */

// Functioneaza cu dsp_init declarat in RAM


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

volatile bool seek_up;
volatile bool seek_down;
//char *start_radio = "Start radio...\n\r";
char str1[30];
char message_frequency[36];
char message_volume[26];

extern int16_t volume;
extern uint32_t freq;

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

char display_buffer[] = "        "; // 8 caractere

char message_freq_static[13];

uint16_t seek_frequency;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void display_rds_info();
void disp_vol(uint32_t vol);
void disp_freq(uint32_t freq);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);  // volum
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);  // frecventa
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //sprintf(str1, "Freq = %i.%i Mhz\n\r", freq_1, freq_2);

  HAL_GPIO_WritePin(HCMS_CE_LED_GPIO_Port, HCMS_CE_LED_Pin, GPIO_PIN_RESET);

  //start radio
  setup();
  //printf("Start radio\n");
  //get_RDS();
  //char *str1 = "Start radio...\n\r";
  //HAL_UART_Transmit(&huart2, (uint8_t *)str1, strlen (str1), HAL_MAX_DELAY);
  print_serial2_message("Start radio...");
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
	setFrequency(10290);
  // set volume
//    volume = 80;
//    Set_Cmd(48, 11, 1, 0);  //unmute
//    int Setvolume = map(volume, 0, 100, -599, 50);
//    Set_Cmd(48, 10, 1, Setvolume);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      contor++;
      //print_serial2_message_number("contor = ", contor);


      // IF Button seek up Is Pressed
      if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14) == 0)  // seek up
      {
    	    clear_display();
    	   	writeDigitAscii(0, 'S', false);
    	    writeDigitAscii(1, 'E', false);
    	    writeDigitAscii(2, 'E', false);
    	    writeDigitAscii(3, 'K', false);
    	    writeDigitAscii(4, ' ', false);
    	    writeDigitAscii(5, 'U', false);
    	    writeDigitAscii(6, 'P', false);
    	    writeDisplay(DISPLAY_ADDRESS);

    	    //tuneUp();
    	    //seek_frequency = seekUp();
		    //sprintf(message_frequency, "seek_frequency %i\r", seek_frequency);
		    //print_serial2_message(message_frequency);
    	    setFrequency(10290);
    	    //uint16_t current_freq =  getFrequency();
    	    //sprintf(message_frequency, "current_freq %i\r", current_freq);
    	    //print_serial2_message(message_frequency);
      }
      // IF Button seek down Is Pressed
      if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_15) == 0)  // seek down
      {
			clear_display();
			writeDigitAscii(0, 'S', false);
			writeDigitAscii(1, 'E', false);
			writeDigitAscii(2, 'E', false);
			writeDigitAscii(3, 'K', false);
			writeDigitAscii(4, ' ', false);
			writeDigitAscii(5, 'D', false);
			writeDigitAscii(6, 'O', false);
			writeDigitAscii(7, 'W', false);
			writeDisplay(DISPLAY_ADDRESS);

			//tuneDown();
			seek_frequency = seekDown();
		    sprintf(message_frequency, "seek_frequency %i\r", seek_frequency);
		    print_serial2_message(message_frequency);
      }

		encoder_reading_v = (TIM2->CNT>>1) + 30;
		if (encoder_reading_v < 30)
		{
			TIM2->CNT = 1;                         //limit vol 0
			encoder_reading_v = (TIM2->CNT>>1) + 30;
		}
		if (encoder_reading_v > 90)
		{
			TIM2->CNT = 120;                         //limit vol 60
			encoder_reading_v = (TIM2->CNT>>1) + 30;
		}
		if(encoder_reading_v != old_encoder_reading_v)
		{
		   volume = map(encoder_reading_v, 0, 100, -599, 50);
		   Set_Cmd(48, 10, 1, volume);
		   sprintf(message_volume, "Vol = %i  \r", encoder_reading_v - 30);
		   print_serial2_message(message_volume);
		   clear_display();
		   disp_vol(encoder_reading_v - 30);
		}
		old_encoder_reading_v = encoder_reading_v;



//		encoder_reading_f = 100*(TIM3->CNT>>1);
//		if (encoder_reading_f > 50000)
//		{
//			TIM3->CNT = 1;                         //limit f=87.5 Mhz
//			encoder_reading_f = 100*(TIM3->CNT>>1);
//		}
//
//		if (encoder_reading_f > 20500)
//		{
//			TIM3->CNT = 410;                       //limit f=108 Mhz
//			encoder_reading_f = 100*(TIM3->CNT>>1);
//		}
		encoder_reading_f = (TIM3->CNT>>1);
		if(encoder_reading_f != old_encoder_reading_f)
		{
			clear_rds_buffers(rdsProgramType,17);
		    clear_rds_buffers(rdsProgramService, 9);
		    clear_rds_buffers(rdsRadioText, 65);
		    uint16_t current_freq =  getFrequency();
		    freq = current_freq + 10*encoder_reading_f;
		    setFrequency(freq);
		   //freq = 87500 + encoder_reading_f;
		    sprintf(message_frequency, "Frecv = %lu MHz \r", freq);
		    print_serial2_message(message_frequency);
//	  	   REG_FREQ = freq;
//		   if ((REG_FREQ >= 65000) && (REG_FREQ <= 108000))
//		   {
//			 Set_Cmd(32, 1, 2, 1, REG_FREQ / 10);
//			 MODF_FREQ = REG_FREQ;
//		   }
            //uint16_t freq_before_point = freq / 1000;
            //uint16_t freq_after_point = freq % 1000;
           // freq_after_point = freq_after_point / 100;
		    //sprintf(message_frequency, "Frecv = %i.%i MHz \r", freq_before_point, freq_after_point);
		    //print_serial2_message(message_frequency);
		    clear_display();
		    disp_freq(freq);
		}
		old_encoder_reading_f = encoder_reading_f;

		if(contor >= 70000)
		{
			contor = 0;
			//display_rds_info();
			//print_serial2_message("==================");
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

void disp_vol(uint32_t vol)  // 0-60
{
    uint8_t vol_0 = vol / 10;  // prima cifra
    uint8_t vol_1 = vol % 10;  // a doua cifra
    if(vol_0 > 0)
    {
    	writeDigitAscii(0, 'V', false);
    	writeDigitAscii(1, 'O', false);
    	writeDigitAscii(2, 'L', false);
    	writeDigitAscii(3, ' ', false);
    	writeDigitAscii(4, vol_0 + 48, false);
    	writeDigitAscii(5, vol_1 + 48, false);
    	writeDisplay(DISPLAY_ADDRESS);
    }
    else
    {
    	writeDigitAscii(0, 'V', false);
    	writeDigitAscii(1, 'O', false);
    	writeDigitAscii(2, 'L', false);
    	writeDigitAscii(3, ' ', false);
    	writeDigitAscii(4, ' ', false);
    	writeDigitAscii(5, vol_1 + 48, false);
    	writeDisplay(DISPLAY_ADDRESS);
    }


}
void disp_freq(uint32_t freq)
{
	if (freq >= 100000)
	{
		uint16_t freq_0 = freq / 100000; // prima cifra
		uint16_t f_0 = freq % 100000;
		uint16_t freq_1 = f_0 / 10000;   // a doua cifra
		uint16_t f_1 =  f_0 % 10000;
		uint16_t freq_2 = f_1 / 1000;   // a treia cifra  (*cu punct jos)
		uint16_t freq_3 = f_1 %  1000;
		uint16_t freq_4 = freq_3 / 100;   // a patra cifra

		writeDigitAscii(0, 'F', false);
		writeDigitAscii(1, ' ', false);
		writeDigitAscii(2, freq_0 + 48, false);
		writeDigitAscii(3, freq_1 + 48, false);
		writeDigitAscii(4, freq_2 + 48, true);
		writeDigitAscii(5, freq_4 + 48, false);
		writeDisplay(DISPLAY_ADDRESS);
	}
	else
	{
		uint16_t freq_0 = freq / 10000; // prima cifra
		uint16_t f_0 = freq % 10000;
		uint16_t freq_1 = f_0 / 1000;   // a doua cifra
		uint16_t f_1 =  f_0 % 1000;
		uint16_t freq_2 = f_1 / 100;   // a treia cifra  (*cu punct jos)
		//uint16_t freq_3 = f_1 % 100;   // a patra cifra

		writeDigitAscii(0, 'F', false);
		writeDigitAscii(1, ' ', false);
		writeDigitAscii(2, ' ', false);
		writeDigitAscii(3, freq_0 + 48, false);
		writeDigitAscii(4, freq_1 + 48, true);
		writeDigitAscii(5, freq_2+ 48, false);
		writeDisplay(DISPLAY_ADDRESS);
	}


}

void populate_vol_array(uint16_t vol)
{
  // char message_volume[] = "VOLUME 10";
	uint8_t v1 = vol / 10;
	uint8_t v2 = vol % 10;

	message_volume[0] = 'V';
	message_volume[1] = 'O';
	message_volume[2] = 'L';
	message_volume[3] = 'U';
	message_volume[4] = 'M';
	message_volume[5] = 'E';
	message_volume[6] = ' ';
	//message_volume[7] = v1 + 48;
	if(v1 + 48 == '0')
	{
		message_volume[7] = ' ';
	}
	else
	{
		message_volume[7] = v1 + 48;
	}
	message_volume[8] = v2 + 48;
}

void populate_freq_array(uint16_t freq)
{
	//char message_frequency[] = "FREQ: 1045 MHZ  ";

	uint16_t freq_0 = freq / 1000; // prima cifra
	uint16_t f_0 = freq % 1000;
	uint16_t freq_1 = f_0 / 100;   // a doua cifra
	uint16_t f_1 =  f_0 % 100;
	uint16_t freq_2 = f_1 / 10;   // a treia cifra  (*cu punct jos)
	uint16_t freq_3 = f_1 % 10;   // a patra cifra


	message_frequency[0] = 'F';
	message_frequency[1] = 'R';
	message_frequency[2] = 'E';
	message_frequency[3] = 'Q';
	message_frequency[4] = ' ';
	//message_frequency[5] = freq_0 + 48;
	if(freq_0 + 48 == '0')
	{
		message_frequency[5] = ' ';
	}
	else
	{
		message_frequency[5] = freq_0 + 48;
	}
    message_frequency[6] = freq_1 + 48;
    message_frequency[7] = freq_2 + 48; // punct
	message_frequency[8] = freq_3 + 48;
	message_frequency[9] = ' ';
	message_frequency[10] = 'M';
	message_frequency[11] = 'H';
	message_frequency[12] = 'Z';
	message_frequency[13] = ' ';
	message_frequency[14] = ' ';

	message_freq_static[0] = 'F';
	message_freq_static[1] = ' ';
	//message_freq_static[2] = freq_0 + 48;
	if(freq_0 + 48 == '0')
	{
		message_freq_static[2] = ' ';
	}
	else
	{
		message_freq_static[2] = freq_0 + 48;
	}
	message_freq_static[3] = freq_1 + 48;
	message_freq_static[4] = freq_2 + 48; // punct;
	message_freq_static[5] = freq_3 + 48;
	message_freq_static[6] = ' ';
	message_freq_static[7] = 'M';
	message_freq_static[8] = 'H';
	message_freq_static[9] = 'Z';
	message_freq_static[10] = ' ';
	message_freq_static[11] = ' ';

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

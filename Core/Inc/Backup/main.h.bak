/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
void showFmSeek();
uint8_t prepare_rds_for_display(char *rds_message);
void clear_buffers();
void read_write_memory_stations_vol();
void save_volume(uint8_t vol);
void load_default_station_and_volume();
void read_volume_encoder();
void read_frequency_encoder();
void show_rds_informations();
void read_seek_up_down_gpio();
void read_PAJ_gesture();
void read_PAJ_gesture_2();
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HCMS_CE_LED_Pin GPIO_PIN_13
#define HCMS_CE_LED_GPIO_Port GPIOC
#define FREQ_SEEK_UP_Pin GPIO_PIN_14
#define FREQ_SEEK_UP_GPIO_Port GPIOC
#define FREQ_SEEK_DOWN_Pin GPIO_PIN_15
#define FREQ_SEEK_DOWN_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define LIGHT_SENSOR_Pin GPIO_PIN_0
#define LIGHT_SENSOR_GPIO_Port GPIOB
#define IR_SENSOR_Pin GPIO_PIN_1
#define IR_SENSOR_GPIO_Port GPIOB
#define PAJ_interrupt_Pin GPIO_PIN_2
#define PAJ_interrupt_GPIO_Port GPIOB
#define PAJ_interrupt_EXTI_IRQn EXTI2_IRQn
#define VFD_CS_Pin GPIO_PIN_12
#define VFD_CS_GPIO_Port GPIOB
#define HCMS_RS_Pin GPIO_PIN_13
#define HCMS_RS_GPIO_Port GPIOB
#define HCMS_BLANK_Pin GPIO_PIN_14
#define HCMS_BLANK_GPIO_Port GPIOB
#define HCMS_RESET_Pin GPIO_PIN_15
#define HCMS_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

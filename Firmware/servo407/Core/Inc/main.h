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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIN3_Pin GPIO_PIN_2
#define DIN3_GPIO_Port GPIOE
#define DIN4_Pin GPIO_PIN_3
#define DIN4_GPIO_Port GPIOE
#define DIN5_Pin GPIO_PIN_4
#define DIN5_GPIO_Port GPIOE
#define DOUT1_Pin GPIO_PIN_5
#define DOUT1_GPIO_Port GPIOE
#define DOUT2_Pin GPIO_PIN_6
#define DOUT2_GPIO_Port GPIOE
#define ADC_MISO_Pin GPIO_PIN_2
#define ADC_MISO_GPIO_Port GPIOC
#define ADC_MOSI_Pin GPIO_PIN_3
#define ADC_MOSI_GPIO_Port GPIOC
#define BRAKE_Pin GPIO_PIN_0
#define BRAKE_GPIO_Port GPIOA
#define IGBT_FAULT_Pin GPIO_PIN_1
#define IGBT_FAULT_GPIO_Port GPIOA
#define DAC1_Pin GPIO_PIN_4
#define DAC1_GPIO_Port GPIOA
#define DAC2_Pin GPIO_PIN_5
#define DAC2_GPIO_Port GPIOA
#define I_U_Pin GPIO_PIN_6
#define I_U_GPIO_Port GPIOA
#define I_V_Pin GPIO_PIN_7
#define I_V_GPIO_Port GPIOA
#define SOFTSTART_Pin GPIO_PIN_4
#define SOFTSTART_GPIO_Port GPIOC
#define ENC_SHORT_Pin GPIO_PIN_5
#define ENC_SHORT_GPIO_Port GPIOC
#define ADC_CLK_Pin GPIO_PIN_10
#define ADC_CLK_GPIO_Port GPIOB
#define ADC_CS_Pin GPIO_PIN_11
#define ADC_CS_GPIO_Port GPIOB
#define BTN_ENT_Pin GPIO_PIN_8
#define BTN_ENT_GPIO_Port GPIOD
#define BTN_UP_Pin GPIO_PIN_9
#define BTN_UP_GPIO_Port GPIOD
#define BTN_DOWN_Pin GPIO_PIN_10
#define BTN_DOWN_GPIO_Port GPIOD
#define BTN_BACK_Pin GPIO_PIN_11
#define BTN_BACK_GPIO_Port GPIOD
#define STEP_F_Pin GPIO_PIN_12
#define STEP_F_GPIO_Port GPIOD
#define STEP_R_Pin GPIO_PIN_13
#define STEP_R_GPIO_Port GPIOD
#define LED_STATUS_Pin GPIO_PIN_14
#define LED_STATUS_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_15
#define LED_ERROR_GPIO_Port GPIOD
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define ENC_Z_Pin GPIO_PIN_8
#define ENC_Z_GPIO_Port GPIOC
#define ENC_ENABLE_Pin GPIO_PIN_9
#define ENC_ENABLE_GPIO_Port GPIOC
#define ENCODER_DE_Pin GPIO_PIN_8
#define ENCODER_DE_GPIO_Port GPIOA
#define ENCODER_TX_Pin GPIO_PIN_9
#define ENCODER_TX_GPIO_Port GPIOA
#define ENCODER_RX_Pin GPIO_PIN_10
#define ENCODER_RX_GPIO_Port GPIOA
#define ETH_INT_Pin GPIO_PIN_2
#define ETH_INT_GPIO_Port GPIOD
#define ETH_RESET_Pin GPIO_PIN_3
#define ETH_RESET_GPIO_Port GPIOD
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOD
#define MODBUS_TX_Pin GPIO_PIN_5
#define MODBUS_TX_GPIO_Port GPIOD
#define MODBUS_RX_Pin GPIO_PIN_6
#define MODBUS_RX_GPIO_Port GPIOD
#define OLED_CLK_Pin GPIO_PIN_3
#define OLED_CLK_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_4
#define OLED_CS_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_5
#define OLED_MOSI_GPIO_Port GPIOB
#define OLED_RESET_Pin GPIO_PIN_6
#define OLED_RESET_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_8
#define EEPROM_SCL_GPIO_Port GPIOB
#define MODBUS_DE_Pin GPIO_PIN_9
#define MODBUS_DE_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_0
#define DIN1_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_1
#define DIN2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

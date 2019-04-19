/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key1_Pin GPIO_PIN_3
#define Key1_GPIO_Port GPIOE
#define Key0_Pin GPIO_PIN_4
#define Key0_GPIO_Port GPIOE
#define ADC3_IN10_PC0_Slider1_Pin GPIO_PIN_0
#define ADC3_IN10_PC0_Slider1_GPIO_Port GPIOC
#define ADC3_IN11_PC1_Slider2_Pin GPIO_PIN_1
#define ADC3_IN11_PC1_Slider2_GPIO_Port GPIOC
#define SPI2_MISO_Pin GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOC
#define ADC1_IN1_PA1_X_Small_Pin GPIO_PIN_1
#define ADC1_IN1_PA1_X_Small_GPIO_Port GPIOA
#define ADC2_IN2_PA2_Y_Small_Pin GPIO_PIN_2
#define ADC2_IN2_PA2_Y_Small_GPIO_Port GPIOA
#define DAC1_PA4_Pin GPIO_PIN_4
#define DAC1_PA4_GPIO_Port GPIOA
#define DAC2_PA5_Pin GPIO_PIN_5
#define DAC2_PA5_GPIO_Port GPIOA
#define LED_D2_Pin GPIO_PIN_6
#define LED_D2_GPIO_Port GPIOA
#define LED_D3_Pin GPIO_PIN_7
#define LED_D3_GPIO_Port GPIOA
#define ADS1_CS_Pin GPIO_PIN_4
#define ADS1_CS_GPIO_Port GPIOC
#define ADS2_CS_Pin GPIO_PIN_5
#define ADS2_CS_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOB
#define MAX1_CS_Pin GPIO_PIN_7
#define MAX1_CS_GPIO_Port GPIOE
#define TM1637_CS1_Pin GPIO_PIN_14
#define TM1637_CS1_GPIO_Port GPIOE
#define TM1637_CS2_Pin GPIO_PIN_15
#define TM1637_CS2_GPIO_Port GPIOE
#define MCP4_EXTI8_Pin GPIO_PIN_8
#define MCP4_EXTI8_GPIO_Port GPIOD
#define MCP4_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define MCP1_EXTI5_Pin GPIO_PIN_5
#define MCP1_EXTI5_GPIO_Port GPIOD
#define MCP1_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define MCP2_EXTI6_Pin GPIO_PIN_6
#define MCP2_EXTI6_GPIO_Port GPIOD
#define MCP2_EXTI6_EXTI_IRQn EXTI9_5_IRQn
#define MCP3_EXTI7_Pin GPIO_PIN_7
#define MCP3_EXTI7_GPIO_Port GPIOD
#define MCP3_EXTI7_EXTI_IRQn EXTI9_5_IRQn
#define FUEL_METER_Pin GPIO_PIN_9
#define FUEL_METER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PERIPHERAL_TIMEOUT 20

extern volatile uint32_t Timer13;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

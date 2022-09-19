/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdint.h>
#include "usart.h"
#include "spi.h"
#include "w25q256.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define VECT_TAB_FLASH_BASE_ADDRESS  FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_SRAM_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET              0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define NVIC_SET_VECTOR(vector, offset)  (SCB->VTOR = ((uint32_t)(vector)) | ((uint32_t)(offset) & (uint32_t)0x1FFFFF80))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define LED_R_Pin GPIO_PIN_10
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_12
#define LED_B_GPIO_Port GPIOH
#define LAN8720A_RESET_Pin GPIO_PIN_1
#define LAN8720A_RESET_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

#define BytesSwap16(value)        \
    ( ((value) & 0xFF00U) >> 8U | \
      ((value) & 0x00FFU) << 8U )
#define BytesSwap32(value)             \
    ( ((value) & 0xFF000000U) >> 24U | \
      ((value) & 0x00FF0000U) >> 8U  | \
      ((value) & 0x0000FF00U) << 8U  | \
      ((value) & 0x000000FFU) << 24U )


inline uint16_t Swap16(uint16_t *p_value)
{
    *p_value = ((*p_value) & 0xFF00U) >> 8U |
               ((*p_value) & 0x00FFU) << 8U ;
    
    return *p_value;
}


inline uint32_t Swap32(uint32_t *p_value)
{
    *p_value = ((*p_value) & 0xFF000000U) >> 24U |
               ((*p_value) & 0x00FF0000U) >> 8U  |
               ((*p_value) & 0x0000FF00U) << 8U  |
               ((*p_value) & 0x000000FFU) << 24U ;
               
    return *p_value;
}


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

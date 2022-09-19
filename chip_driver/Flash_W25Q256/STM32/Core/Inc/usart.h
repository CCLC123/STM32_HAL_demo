/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

#define USART1_Printf(p_format, ...)         USART1_Printf_Ex(garr_usart1_send_buff, p_format, ## __VA_ARGS__)



#define USART1_BUFF_SIZE                    (256U)   /* 串口缓冲区大小，单位：字节 */
extern uint8_t garr_usart1_send_buff[USART1_BUFF_SIZE];
extern uint8_t garr_usart1_recevie_buff0[USART1_BUFF_SIZE];
extern uint8_t garr_usart1_recevie_buff1[USART1_BUFF_SIZE];


/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef USART1_DMA_Send_Ex(const uint8_t *p_send_buff, uint16_t length);
HAL_StatusTypeDef USART1_Printf_Ex(const uint8_t *p_send_buff, const char *p_format, ...);
void UARTx_RxCallback(UART_HandleTypeDef *p_huart, uint8_t *p_recevie_buff, uint16_t length);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


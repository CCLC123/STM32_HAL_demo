/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdarg.h>


#define USART1_TIMEOUT       (0x0000FFFFU)   /* 串口最大超时等待时间, 单位: CPU 时间 */

uint8_t garr_usart1_send_buff[USART1_BUFF_SIZE] = {0};      /* 串口1 发送缓冲区 */
uint8_t garr_usart1_recevie_buff0[USART1_BUFF_SIZE] = {0};  /* 串口1 接收缓冲区0 */
uint8_t garr_usart1_recevie_buff1[USART1_BUFF_SIZE] = {0};  /* 串口1 接收缓冲区1 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

    
    /* 解锁并设置状态标志 */
    huart1.gState = HAL_UART_STATE_READY;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.hdmatx->State = HAL_DMA_STATE_READY;
    huart1.hdmarx->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(huart1.hdmatx);
    __HAL_UNLOCK(huart1.hdmarx);
    __HAL_UNLOCK(&huart1);
  
    /* 使能外设 */
    __HAL_UART_ENABLE(&huart1);
  
  
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

    /* 设置 DMA 的外设地址为串口 DR 寄存器地址 */
    WRITE_REG(huart1.hdmatx->Instance->PAR, (uint32_t)&huart1.Instance->DR);
    WRITE_REG(huart1.hdmarx->Instance->PAR, (uint32_t)&huart1.Instance->DR);
    
    /* 设置 RX DMA 的存储器 0 和存储器 1 地址, 并使能 DMA 的双缓冲模式, 并强制 DMA 为循坏模式 */
    HAL_DMAEx_MultiBufferStart(huart1.hdmarx, (uint32_t)&huart1.Instance->DR, (uint32_t)garr_usart1_recevie_buff0, (uint32_t)garr_usart1_recevie_buff1, USART1_BUFF_SIZE);
    
    /* 使能串口 DMA 传输功能 */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    
    /* 使能串口接收空闲中断, 使能前先清除标志位 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
    
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/**
  * @brief  串口通过 DMA 发送缓冲区 p_send_buff 中数据 
  * @param  p_send_buff: 数据地址
  * @param  size: 数据长度, 单位: 字节
  * @return HAL status
  */
HAL_StatusTypeDef USART1_DMA_Send_Ex(const uint8_t *p_send_buff, uint16_t size)
{
    uint32_t timeout = 0;
    
    /* 检查函数参数 */
    if (p_send_buff == NULL || size == 0)
    {
        return HAL_ERROR;
    }
    
    /* 检查上一次发送是否完成, 置位则代表发送完成 */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        timeout++;
        if (timeout >= USART1_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    
    /* 解锁并设置状态标志 */
    huart1.gState = HAL_UART_STATE_READY;
    huart1.hdmatx->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(huart1.hdmatx);
    __HAL_UNLOCK(&huart1);
    
    /* 设置 DMA 的传输次数 */
    __HAL_DMA_SET_COUNTER(huart1.hdmatx, size);
    /* 设置 DMA 的存储区 0 地址 */
    WRITE_REG(huart1.hdmatx->Instance->M0AR, (uint32_t)p_send_buff);
    
    /* 使能 DMA, 注意: DMA 传输前需要清除 DMA 相应的标志位 */
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_ENABLE(huart1.hdmatx);
    
    return HAL_OK;
}


/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    WRITE_REG(huart1.Instance->DR, (uint8_t)ch);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);

    return ch;
}



/**
  * @brief  仿 printf 写入数据到 p_send_buff, 串口通过 DMA 发送 p_send_buff
  * @note   需要确保 p_buff 的大小大于待写入字符串的长度, 至少要大于 1 个字节
  * @param  p_send_buff: 数据地址
  * @param  p_format: 格式化字符串
  * @return HAL status
  */
HAL_StatusTypeDef USART1_Printf_Ex(const uint8_t *p_send_buff, const char *p_format, ...)
{
    uint32_t timeout = 0;
    int size = 0;
    va_list list;
    
    /* 检查函数参数 */
    if (p_send_buff == NULL || p_format == NULL)
    {
        return HAL_ERROR;
    }
    
    /* 检查上一次发送是否完成, 置位则代表发送完成 */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        timeout++;
        if (timeout >= USART1_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    
    /* 解锁并设置状态标志 */
    huart1.gState = HAL_UART_STATE_READY;
    huart1.hdmatx->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(huart1.hdmatx);
    __HAL_UNLOCK(&huart1);
    
    /* 将 format 字符串写入 p_buff_addr 中 */
    va_start(list, p_format);
    size = vsprintf((void *)p_send_buff, p_format, list);
    va_end(list);
    
    if(size == -1)
    {
        return HAL_ERROR;
    }
    
    /* 发送数据 */
    /* 设置 DMA 的传输次数 */
    __HAL_DMA_SET_COUNTER(huart1.hdmatx, size);
    /* 设置 DMA 的存储区 0 地址 */
    WRITE_REG(huart1.hdmatx->Instance->M0AR, (uint32_t)p_send_buff);
    
    /* 使能 DMA, 注意: DMA 传输前需要清除 DMA 相应的标志位 */
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_ENABLE(huart1.hdmatx);
    
    return HAL_OK;
}


/**
  * @brief  串口接收回调函数
  * @param  huart: 串口结构体地址
  * @param  p_receive_buff: 接收到的数据起始地址
  * @param  size: 数据长度，单位：字节
  * @return None
  */
__WEAK void UARTx_RxCallback(UART_HandleTypeDef *p_huart, uint8_t *p_receive_buff, uint16_t size)
{
    /* 串口1接收回调 */
    if(p_huart == &huart1)
    {
        USART1_Printf("本次接收到字节数为: %u\r\n", size);
        USART1_DMA_Send_Ex(p_receive_buff, size);
    }
}








/* USER CODE END 1 */

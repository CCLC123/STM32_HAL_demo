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

#define USART1_TIMEOUT       (0x0000FFFFU)   /* �������ʱ�ȴ�ʱ��, ��λ: CPU ʱ�� */


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
  
  /* ʹ������ */
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
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 6);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

    /* ���� DMA �������ַΪ���� DR �Ĵ�����ַ */
    WRITE_REG(huart1.hdmatx->Instance->PAR, (uint32_t)&huart1.Instance->DR);
    WRITE_REG(huart1.hdmarx->Instance->PAR, (uint32_t)&huart1.Instance->DR);
    
    /* ���� RX DMA �Ĵ洢�� 0 �ʹ洢�� 1 ��ַ, ��ʹ�� DMA ��˫����ģʽ, ��ǿ�� DMA Ϊѭ��ģʽ */
    HAL_DMAEx_MultiBufferStart(huart1.hdmarx, (uint32_t)&huart1.Instance->DR, (uint32_t)garr_usart1_recevie_buff0, (uint32_t)garr_usart1_recevie_buff1, USART1_BUFF_SIZE);
    huart1.hdmarx->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(huart1.hdmarx);
    __HAL_UNLOCK(&huart1);
    
    /* ʹ�ܴ��� DMA ���书�� */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    
    /* ʹ�ܴ��ڽ��տ����ж�, ʹ��ǰ�������־λ */
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
  * @brief  ����ͨ�� DMA ���ͻ����� p_send_buff ������ 
  * @param  p_send_buff: ���ݵ�ַ
  * @param  length: ���ݳ���, ��λ: �ֽ�
  * @return HAL status
  */
HAL_StatusTypeDef USART1_DMA_Send_Ex(const uint8_t *p_send_buff, uint16_t length)
{
    uint32_t timeout = 0;
    
    /* ��麯������ */
    if (p_send_buff == NULL || length == 0)
    {
        return HAL_ERROR;
    }
    
    /* �����һ�η����Ƿ����, ��λ���������� */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        timeout++;
        if (timeout >= USART1_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    
    /* ���� DMA �Ĵ������ */
    __HAL_DMA_SET_COUNTER(huart1.hdmatx, length);
    /* ���� DMA �Ĵ洢�� 0 ��ַ */
    WRITE_REG(huart1.hdmatx->Instance->M0AR, (uint32_t)p_send_buff);
    
    /* ʹ�� DMA, ע��: DMA ����ǰ��Ҫ��� DMA ��Ӧ�ı�־λ */
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_ENABLE(huart1.hdmatx);
    
    return HAL_OK;
}


/**
  * @brief  �� printf д�����ݵ� p_send_buff, ����ͨ�� DMA ���� p_send_buff
  * @note   ��Ҫȷ�� p_buff �Ĵ�С���ڴ�д���ַ����ĳ���, ����Ҫ���� 1 ���ֽ�
  * @param  p_send_buff: ���ݵ�ַ
  * @param  p_format: ��ʽ���ַ���
  * @return HAL status
  */
HAL_StatusTypeDef USART1_Printf_Ex(const uint8_t *p_send_buff, const char *p_format, ...)
{
    uint32_t timeout = 0;
    int bytes = 0;
    va_list list;
    
    /* ��麯������ */
    if (p_send_buff == NULL || p_format == NULL)
    {
        return HAL_ERROR;
    }
    
    /* �����һ�η����Ƿ����, ��λ���������� */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        timeout++;
        if (timeout >= USART1_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }
    }
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    
    /* �� format �ַ���д�� p_buff_addr �� */
    va_start(list, p_format);
    bytes = vsprintf((void *)p_send_buff, p_format, list);
    va_end(list);
    
    if(bytes == -1)
    {
        return HAL_ERROR;
    }
    
    /* �������� */
    /* ���� DMA �Ĵ������ */
    __HAL_DMA_SET_COUNTER(huart1.hdmatx, bytes);
    /* ���� DMA �Ĵ洢�� 0 ��ַ */
    WRITE_REG(huart1.hdmatx->Instance->M0AR, (uint32_t)p_send_buff);
    
    /* ʹ�� DMA, ע��: DMA ����ǰ��Ҫ��� DMA ��Ӧ�ı�־λ */
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_ENABLE(huart1.hdmatx);
    
    return HAL_OK;
}


/**
  * @brief  ���ڽ��ջص�����
  * @param  huart UART handle
  * @param  p_recevie_buff 
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
/**
  * @brief  ���ڽ��ջص�����
  * @param  huart: ���ڽṹ���ַ
  * @param  p_receive_buff: ���յ���������ʼ��ַ
  * @param  length: ���ݳ���
  * @return None
  */
void UARTx_RxCallback(UART_HandleTypeDef *p_huart, uint8_t *p_receive_buff, uint16_t length)
{
    /* ����1���ջص� */
    if(p_huart == &huart1)
    {
        USART1_Printf("���ν��յ��ֽ���Ϊ: %u", length);
        USART1_DMA_Send_Ex(p_receive_buff, length);
    }
}





/* USER CODE END 1 */

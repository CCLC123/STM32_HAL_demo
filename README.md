# STM32_HAL_demo
基于HAL库和STM32CUBEMX的STM32外设驱动demo和常用芯片的驱动



# 文件夹分类及介绍

|     文件夹名     |       内容       |
| :--------------: | :--------------: |
| peripheral_drive |   外设驱动demo   |
|   chip_driver    | 常用芯片驱动demo |



# peripheral_drive外设驱动demo

## usart_printf_dma_idle

#### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	串口使用 DMA 进行收发数据，仿照printf的形式输出数据，利用DMD + IDLE（串口接收空闲中断） + 双缓冲进行接收数据。

### 如何使用

- ​	仿printf发送函数`USART1_Printf_Ex`

```c
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
    int bytes = 0;
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
    
    /* 将 format 字符串写入 p_buff_addr 中 */
    va_start(list, p_format);
    bytes = vsprintf((void *)p_send_buff, p_format, list);
    va_end(list);
    
    if(bytes == -1)
    {
        return HAL_ERROR;
    }
    
    /* 发送数据 */
    /* 设置 DMA 的传输次数 */
    __HAL_DMA_SET_COUNTER(huart1.hdmatx, bytes);
    /* 设置 DMA 的存储区 0 地址 */
    WRITE_REG(huart1.hdmatx->Instance->M0AR, (uint32_t)p_send_buff);
    
    /* 使能 DMA, 注意: DMA 传输前需要清除 DMA 相应的标志位 */
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_ENABLE(huart1.hdmatx);
    
    return HAL_OK;
}
```

- 发送和接收缓冲区

```c
#define USART1_BUFF_SIZE                    (256U)   /* 串口缓冲区大小，单位：字节 */
extern uint8_t garr_usart1_send_buff[USART1_BUFF_SIZE];
extern uint8_t garr_usart1_recevie_buff0[USART1_BUFF_SIZE];
extern uint8_t garr_usart1_recevie_buff1[USART1_BUFF_SIZE];
```

- 串口接收回调函数

    本例中，串口将接收到的数据原封不动发送回去

```c
/**
  * @brief  串口接收回调函数
  * @param  huart: 串口结构体地址
  * @param  p_receive_buff: 接收到的数据起始地址
  * @param  length: 数据长度
  * @return None
  */
__WEAK void UARTx_RxCallback(UART_HandleTypeDef *p_huart, uint8_t *p_receive_buff, uint16_t length)
{
    /* 串口1接收回调 */
    if(p_huart == &huart1)
    {
        USART1_Printf("本次接收到字节数为: %u", length);
        USART1_DMA_Send_Ex(p_receive_buff, length);
    }
}
```

### 注意点

- ​	串口初始化的时候，需要开启串口的DMA传输功能，并设置DMA的外设地址和存储器地址，并开启串口接收空闲中断

```c
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
    huart1.hdmarx->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(huart1.hdmarx);
    __HAL_UNLOCK(&huart1);
    
    /* 使能串口 DMA 传输功能 */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    
    /* 使能串口接收空闲中断, 使能前先清除标志位 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
  /* USER CODE END USART1_MspInit 1 */
  }
}
```

- 串口初始化完毕后，记得使能外设

```c
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
  
  /* 使能外设 */
  __HAL_UART_ENABLE(&huart1);
  
  /* USER CODE END USART1_Init 2 */

}
```



# chip_driver常用芯片驱动demo

## Flash_W25Q256

### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	W25Q256是一款NOR-Flash芯片，可通过SPI与MCU通信，容量有32MB，每个扇区有4096字节。

​	将W25Q256抽象成一个个实例（对象），需要用户提供实例接口函数，通过接口函数与W25Q256进行交互；也就是分成2层，Device层和Driver层，用户提供Driver层，实现Device层的接口函数，从而与W25Q256交互。

​	用户需要实现的Device层中接口如下：

```c
typedef en_w25q256_status_t (*w25q256_Init_Func)(void);       /* w25q256 初始化函数 */
typedef en_w25q256_status_t (*w25q256_Send_Receive_Func)(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, uint8_t is_continue_com);  /* 向 w25q256 发送 p_send_buff 和接收 W25Q256 的数据到 p_receive_buff 函数 */
```

​	运用结构体`w25q256_obj_t`将W25Q256进行抽象，结构体`w25q256_obj_t`的定义如下：

```c
typedef struct
{
    w25q256_Init_Func           m_p_Init;           /* 初始化函数指针 */
    w25q256_Send_Receive_Func   m_p_Send_Receive;   /* 发送与接收函数指针 */
} w25q256_interface_func_t;


typedef struct
{
    uint64_t m_unique_ID;                           /* 设备唯一ID */
    w25q256_interface_func_t m_interface_func;      /* 接口函数 */
} w25q256_obj_t;
```

​	每个W25Q256实例都拥有自己的`m_unique_ID`和`m_interface_func`（接口函数），用户需要实现这些函数接口。

### 如何使用

​	本例中使用SPI+DMA的方式与W25Q256通信。需要实现`w25q256_Init_Func`和`w25q256_Send_Receive_Func`

- ​	实现`w25q256_Init_Func`函数接口

​	可以在该函数中调用STM32CUBEMX生成SPI初始化函数`MX_SPI5_Init()`

```c
/**
  * @brief   W25Q256 初始化
  * @note    该函数应初始化与 W25Q256 通信的接口，比如 SPI
  * @param   None
  * @return  en_w25q256_status_t
  */
static en_w25q256_status_t I_W25Q256_Init(void)
{
    /* 该函数应该初始化与 W25Q256 通信的接口，比如 SPI */
    /* HAL已经帮我们初始化与 W25Q256 通信的接口了 */
    return EN_W25Q256_OK;
}
```

- ​	实现`w25q256_Send_Receive_Func`函数接口

```c
/**
  * @brief   向 W25Q256 发送 p_send_buff 和接收 W25Q256 的数据到 p_receive_buff
  * @note    使用与 W25Q256 通信的接口，比如 SPI , 与 W25Q256 进行数据的收发
  * @param   p_send_buff: 发送缓冲区起始地址
  * @param   p_receive_buff: 接收缓冲区起始地址
  * @param   length: 发送/接收的数据长度，单位：字节
  * @param   is_continue_com: 本次发送完毕之后是否要继续通信，继续通信则 CS 保持低电平
  *             1: 继续通信, 0: 结束通信
  * @return  en_w25q256_status_t
  */
static en_w25q256_status_t I_W25Q256_Send_Receive(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, uint8_t is_continue_com)
{
    uint32_t timeout = 0;
    HAL_StatusTypeDef state = HAL_OK;
    
    HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
    state = SPI5_DMA_Send_Receive(p_send_buff, p_receive_buff, length);
    
    /* 等待发送完成, 复位则代表发送完成 */
    while (__HAL_SPI_GET_FLAG(&hspi5, SPI_FLAG_BSY) == SET)
    {
        timeout++;
        if (timeout >= W25Q256_TIMEOUT)
        {
            W25Q256_DEBUG_PRINTF(DEBUG_ERROR(EN_W25Q256_TIMEOUT));
            return EN_W25Q256_TIMEOUT;
        }
        W25Q256_Wait_Callback();
    }
    /* 根据参数来设置 CS 的电平 */
    if (!is_continue_com)
    {
        HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
    }
    
    if (state == HAL_OK)
    {
        return EN_W25Q256_OK;
    }
    else
    {
        return EN_W25Q256_ERROR;
    }
}
```

- 设置接口函数数组`garr_w25q256_interface_func`和W25Q256实例数组`garr_w25q256`的数量

```c
#define W25Q256_NUM                 (1U)            /* W25Q256 的数量 */
const w25q256_interface_func_t garr_w25q256_interface_func[W25Q256_NUM] = /* 函数接口数组 */
{{I_W25Q256_Init, I_W25Q256_Send_Receive}};

w25q256_obj_t garr_w25q256[W25Q256_NUM] = {0};     /* W25Q256 实例对象数组 */
```

- 调用`W25Q256_Init()`初始化W25Q256对象

```c
/* 初始化第 0 个 W25Q256 实例（对象） */
W25Q256_Init(&garr_w25q256[0], &garr_w25q256_interface_func[0]);
```

### 注意点

- 在STM32CUBEMX中没有开启SPI与DMA的中断，需要每次使用使用SPI和DMA的时候，需要进行设置状态标志位和解锁操作。

```c
/* 设置状态标志为 READY, 解锁 SPI 和 DMA  */
hspi5.State = HAL_SPI_STATE_READY;
hspi5.hdmatx->State = HAL_DMA_STATE_READY;
hspi5.hdmarx->State = HAL_DMA_STATE_READY;
__HAL_UNLOCK(&hspi5);
__HAL_UNLOCK(hspi5.hdmatx);
__HAL_UNLOCK(hspi5.hdmarx);

/* DMA 传输前需要清除相应的标志位 */
__HAL_DMA_CLEAR_FLAG(hspi5.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi5.hdmatx));
__HAL_DMA_CLEAR_FLAG(hspi5.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi5.hdmarx));
```

- 初始化完SPI记得使能外设

```c
/* SPI5 init function */
void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* 使能 SPI */
  __HAL_SPI_ENABLE(&hspi5);
  
  /* USER CODE END SPI5_Init 2 */

}
```


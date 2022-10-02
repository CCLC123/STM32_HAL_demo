# STM32_HAL_demo
基于HAL库和STM32CUBEMX的STM32外设驱动demo和常用芯片的驱动



# 文件夹分类及介绍

|     文件夹名     |       内容       |
| :--------------: | :--------------: |
| peripheral_drive |   外设驱动demo   |
|   chip_driver    | 常用芯片驱动demo |



# peripheral_drive外设驱动demo

## usart_printf_dma_idle

### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	串口使用 DMA 进行收发数据，仿照printf的形式输出数据，利用DMD + IDLE（串口接收空闲中断） + 双缓冲进行接收数据。

### 如何使用

- 注册printf函数接口

```c
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    WRITE_REG(huart1.Instance->DR, (uint8_t)ch);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);

    return ch;
}
```

- ​	仿printf发送函数`USART1_Printf_Ex`

```c
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
  * @param  size: 数据长度，单位：字节
  * @return None
  */
__WEAK void UARTx_RxCallback(UART_HandleTypeDef *p_huart, uint8_t *p_receive_buff, uint16_t size)
{
    /* 串口1接收回调 */
    if(p_huart == &huart1)
    {
        USART1_Printf("本次接收到字节数为: %u", size);
        USART1_DMA_Send_Ex(p_receive_buff, size);
    }
}
```

### 注意点

- 在STM32CUBEMX中没有开启DMA的中断，需要每次使用DMA和USART（调用HAL库的函数）的时候，需要进行设置状态标志位和解锁操作。

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
```

- 串口初始化的时候，需要开启串口的DMA传输功能，并设置DMA的外设地址和存储器地址，并开启串口接收空闲中断

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

- 屏蔽HAL库的串口中断

```c
/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
/* USER CODE BEGIN USART1_IRQn 0 */

/* 发送串口接收中断 */
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
{
__HAL_UART_CLEAR_IDLEFLAG(&huart1);

/* 获取本次接收到的字节数与数据缓冲区地址, 并重新设置 DMA 的传输数量和接收缓冲区
1. 失能 DMA
2. 获取本次接收到的字节数与数据缓冲区地址，并重新设置数据缓冲区地址
2. 重新设置 DMA 的传输数量
3. 清除 DMA 传输标志位
4. 使能 DMA */
__HAL_DMA_DISABLE(huart1.hdmarx);
uint16_t receive_size = USART1_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
uint8_t *p_receive_buff = NULL;
if(READ_BIT(huart1.hdmarx->Instance->CR, DMA_SxCR_CT))
{
p_receive_buff = garr_usart1_recevie_buff1;
CLEAR_BIT(huart1.hdmarx->Instance->CR, DMA_SxCR_CT);
}
else
{
p_receive_buff = garr_usart1_recevie_buff0;
SET_BIT(huart1.hdmarx->Instance->CR, DMA_SxCR_CT);
}

/* 解锁并设置状态标志 */
huart1.RxState = HAL_UART_STATE_READY;
huart1.hdmarx->State = HAL_DMA_STATE_READY;
__HAL_UNLOCK(huart1.hdmarx);

__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_BUFF_SIZE);
__HAL_DMA_CLEAR_FLAG(huart1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmatx));
__HAL_DMA_ENABLE(huart1.hdmarx);

/* 进入接收回调函数 */
UARTx_RxCallback(&huart1, p_receive_buff, receive_size);
}

#if 0

/* USER CODE END USART1_IRQn 0 */
HAL_UART_IRQHandler(&huart1);
/* USER CODE BEGIN USART1_IRQn 1 */

#endif

/* USER CODE END USART1_IRQn 1 */
}
```

- 生成二进制文件`fromelf --bin -o "$L@L.bin" "#L"`
- 勾选微库

![image-20220919232334496](C:\Users\10854\AppData\Roaming\Typora\typora-user-images\image-20220919232334496.png)

- 设置中断向量表偏移

```c
/* 设置中断向量表 */
NVIC_SET_VECTOR(VECT_TAB_FLASH_BASE_ADDRESS, VECT_TAB_OFFSET);
```



- 常用函数、宏函数

```c
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


#define BytesSwap16(value)             \
    ( ((value) & 0xFF00U) >> 8U |      \
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
```

## gpio_i2c

### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	使用GPIO模拟I2C总线的形式

### 如何使用

​	用户需要实现的函数如下：

```c
/**
  * @brief   初始化 I2C 引脚
  * @note    该函数应开启 GPIO 的时钟
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Init_Pin_Func)(en_i2c_pin_t e_pin);


/**
  * @brief   设置 I2C 引脚模式
  * @note    该函数应提供配置 GPIO 工作模式的接口
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @param   e_pin_mode: I2C 的引脚模式
  *   @arg     EN_I2C_PIN_MODE_OUTPUT: 上拉输出模式
  *   @arg     EN_I2C_PIN_MODE_INPUT: 浮空输入模式
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Config_Pin_Mode_Func)(en_i2c_pin_t e_pin, en_i2c_pin_mode_t e_pin_mode);


/**
  * @brief   读取 I2C 引脚的状态
  * @note    该函数应提供获取 GPIO 引脚电平状态的接口
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @return  en_i2c_pin_status_t
  */
typedef en_i2c_pin_status_t (*I2C_Read_Pin_Func)(en_i2c_pin_t e_pin);


/**
  * @brief   写入 I2C 引脚的状态
  * @note    该函数应提供设置 GPIO 引脚电平状态的接口
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @param   e_pin_status: I2C 的引脚状态选择
  *   @arg     EN_I2C_RESET: 低电平
  *   @arg     EN_I2C_SET: 高电平
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Write_Pin_Func)(en_i2c_pin_t e_pin, en_i2c_pin_status_t e_pin_status);



/**
  * @brief   微秒级延迟
  * @note    该函数应提供微秒级延迟的接口
  * @param   us: 微秒值
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Delay_us_Func)(uint32_t us);
```

​	定义I2C对象与上述函数接口对象

```c
typedef struct
{
    I2C_Init_Pin_Func           m_p_Init_Pin;           /* 初始化 I2C 引脚 */
    I2C_Config_Pin_Mode_Func    m_p_Config_Pin_Mode;    /* 设置 I2C 引脚模式 */
    I2C_Read_Pin_Func           m_p_Read_Pin;           /* 读取 I2C 引脚的状态 */
    I2C_Write_Pin_Func          m_p_Write_Pin;          /* 写入 I2C 引脚的状态 */
    I2C_Delay_us_Func           m_p_Delay_us;           /* 微秒级延迟 */
} i2c_interface_func_t;


typedef struct
{
    en_i2c_speed_t m_speed;                              /* 速度 */
    i2c_interface_func_t m_interface_func;               /* 接口函数 */
} i2c_obj_t;
```

​	调用初始化函数进行初始化

```c
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, en_i2c_speed_t speed);
```

​	例如

```c
i2c_interface_func_t i2c_interface = {I_I2C_Init_Pin_Func, I_I2C_Config_Pin_Mode_Func, I_I2C_Read_Pin_Func, I_I2C_Write_Pin_Func, I_I2C_Delay_us_Func};
I2C_Init(&i2c_at24cxx, &i2c_interface, EN_I2C_SPEED_250KHZ);
```



# chip_driver常用芯片驱动demo

## Flash_W25QXX

### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	W25QXX是一款NOR-Flash芯片，可通过SPI与MCU通信，每个扇区有4096字节。

​	将W25QXX抽象成一个个实例（对象），需要用户提供实例接口函数，通过接口函数与W25QXX进行交互；也就是分成2层，Device层和Driver层，用户提供Driver层，实现Device层的接口函数，从而与W25QXX交互。

​	用户需要实现的Device层中接口如下：

```c
/**
  * @brief   W25QXX 初始化
  * @note    该函数应初始化与 W25QXX 通信的接口, 比如 SPI
  * @param   None
  * @return  en_w25qxx_status_t
  */
typedef en_w25qxx_status_t (*w25qxx_Init_Func)(void);


/**
  * @brief   向 W25QXX 发送 p_send_buff 和接收 W25QXX 的数据到 p_receive_buff
  * @note    该函数实现与 W25QXX 进行数据的收发, 如使用与 W25QXX 通信的接口, 比如 SPI
  * @param   p_send_buff: 发送缓冲区起始地址
  * @param   p_receive_buff: 接收缓冲区起始地址
  * @param   length: 发送/接收的数据长度, 单位: 字节
  * @param   is_continue_com: 本次发送完毕之后是否要继续通信, 继续通信则 CS 保持低电平
  *             1: 继续通信, 0: 结束通信
  * @return  en_w25qxx_status_t
  */
typedef en_w25qxx_status_t (*w25qxx_Send_Receive_Func)(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t is_continue_com);
```

​	运用结构体`w25qxx_obj_t`将W25QXX进行抽象，结构体`w25qxx_obj_t`的定义如下：

```c
typedef struct
{
    w25qxx_Init_Func           m_p_Init;           /* 初始化函数指针 */
    w25qxx_Send_Receive_Func   m_p_Send_Receive;   /* 发送与接收函数指针 */
} w25qxx_interface_func_t;


typedef struct
{
    uint64_t m_unique_ID;                           /* 设备唯一ID */
    w25qxx_interface_func_t m_interface_func;       /* 接口函数 */
} w25qxx_obj_t;
```

​	每个W25QXX实例都拥有自己的`m_unique_ID`和`m_interface_func`（接口函数），用户需要实现这些函数接口。

### 如何使用

​	本例中使用SPI+DMA的方式与W25QXX通信。需要实现`w25qxx_Init_Func`和`w25qxx_Send_Receive_Func`

- ​	实现`w25qxx_Init_Func`函数接口

​	可以在该函数中调用STM32CUBEMX生成SPI初始化函数`MX_SPI5_Init()`

```c
/**
  * @brief   W25QXX 初始化
  * @note    该函数应初始化与 W25QXX 通信的接口, 比如 SPI
  * @param   None
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t I_W25QXX_Init(void)
{
    /* 该函数应该初始化与 W25QXX 通信的接口, 比如 SPI */
    /* HAL已经帮我们初始化与 W25QXX 通信的接口了 */
    return EN_W25QXX_OK;
}
```

- ​	实现`w25qxx_Send_Receive_Func`函数接口

本例使用SPI+DMA的方式与W25QXX芯片通信，SPI双向通信，作为主设备，W25QXX作为从设备

```c
/**
  * @brief   向 W25QXX 发送 p_send_buff 和接收 W25QXX 的数据到 p_receive_buff
  * @note    该函数实现与 W25QXX 进行数据的收发, 如使用与 W25QXX 通信的接口, 比如 SPI
  * @param   p_send_buff: 发送缓冲区起始地址
  * @param   p_receive_buff: 接收缓冲区起始地址
  * @param   length: 发送/接收的数据长度, 单位: 字节
  * @param   is_continue_com: 本次发送完毕之后是否要继续通信, 继续通信则 CS 保持低电平
  *             EN_W25QXX_CLOSE_COM: 结束通信
  *             EN_W25QXX_CONTINUE_COM: 继续通信
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t I_W25QXX_Send_Receive(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t is_continue_com)
{
    uint32_t timeout = 0;
    HAL_StatusTypeDef state = HAL_OK;
    
    HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
    state = SPI5_DMA_Send_Receive(p_send_buff, p_receive_buff, length);
    
    /* 等待发送完成, 复位则代表发送完成 */
    while (__HAL_SPI_GET_FLAG(&hspi5, SPI_FLAG_BSY) == SET)
    {
        timeout++;
        if (timeout >= W25QXX_TIMEOUT)
        {
            return EN_W25QXX_TIMEOUT;
        }
    }
    /* 根据参数来设置 CS 的电平 */
    if (is_continue_com == EN_W25QXX_CLOSE_COM)
    {
        HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
    }
    
    if (state == HAL_OK)
    {
        return EN_W25QXX_OK;
    }
    else
    {
        return EN_W25QXX_ERROR;
    }
}
```

- 定义接口函数数组`garr_w25qxx_interface_func`和W25QXX实例数组`garr_w25qxx`的数量

```c
#define W25QXX_NUM (1U)
/* 函数接口数组 */
const w25qxx_interface_func_t garr_w25qxx_interface_func[W25QXX_NUM] = {{I_W25QXX_Init, I_W25QXX_Send_Receive}};

/* W25QXX 实例对象数组 */
w25qxx_obj_t garr_w25qxx[W25QXX_NUM] = {0};

/* 初始化第 0 个 W25QXX 实例（对象） */
W25QXX_Init(&garr_w25qxx[0], &garr_w25qxx_interface_func[0]);
```

- 调用`W25QXX_Init()`初始化W25QXX对象

```c
/* 初始化第 0 个 W25QXX 实例（对象） */
W25QXX_Init(&garr_w25qxx[0], &garr_w25qxx_interface_func[0]);
```

- 选择W25QXX型号与芯片ID

```c
/**
  * @brief W25QXX 设备类型
  *        有如下类型: 
  *             W25Q256
  *             W25Q128
  *             W25Q64
  *             W25Q32
  */
#define W25QXX_TYPE             W25Q128


/**
  * @brief W25QXX 相关的 ID 定义, 请查阅芯片数据手册填写     
  */
#define W25QXX_MF_ID           (0xEFU)
#define W25QXX_JEDEC_ID        (0x4019U)
```

- demo见`main.c`文件

### 注意点

- 向W25QXX芯片写数据地址需要按照扇区地址对齐

- 在STM32CUBEMX中没有开启SPI与DMA的中断，需要每次使用SPI和DMA的时候，需要进行设置状态标志位和解锁操作。

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

## AT24CXX

### 开发环境

- KEIL-MDK：5.27

- STM32CUBEMX：6.6.1

- HAL库：1.27.1

- MCU：STM32F429IGT6

### 基本介绍

​	将AT24CXX抽象成一个个实例（对象），需要用户提供实例接口函数，通过接口函数与AT24CXX进行交互；也就是分成2层，Device层和Driver层，用户提供Driver层，实现Device层的接口函数，从而与AT24CXX交互。

### 如何使用

​	用户需要实现的Device层中接口如下：

```c
/**
  * @brief   AT24CXX 初始化
  * @note    该函数应初始化与 AT24CXX 通信的接口, 如 I2C
  * @param   None
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Init_Func)(void);


/**
  * @brief   向 AT24CXX 发送 p_send_buff 中的数据
  * @note    该函数实现与 AT24CXX 进行数据的发送, 如使用与 AT24CXX 通信的接口, 如 I2C
  * @param   p_send_buff: 发送缓冲区起始地址
  * @param   length: 发送的数据长度, 单位: 字节
  * @param   e_start_status: 本次通信前是否发送起始信号
  *            @arg EN_AT24CXX_GENETATE_START: 发送起始信号
  *            @arg EN_AT24CXX_NOT_GENETATE_START: 不发送起始信号
  * @param   e_stop_status: 本次通信后是否发送停止信号
  *            @arg EN_AT24CXX_GENETATE_STOP: 发送停止信号
  *            @arg EN_AT24CXX_NOT_GENETATE_STOP: 不发送停止信号
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Send_Func)(const uint8_t *p_send_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);


/**
  * @brief   从 AT24CXX 接收数据到 p_receive_buff
  * @note    该函数实现与 AT24CXX 进行数据的接收, 如使用与 AT24CXX 通信的接口, 如 I2C
  * @param   p_receive_buff: 接收缓冲区起始地址
  * @param   length: 接收的数据长度, 单位: 字节
  * @param   e_start_status: 本次通信前是否发送起始信号
  *            @arg EN_AT24CXX_GENETATE_START: 发送起始信号
  *            @arg EN_AT24CXX_NOT_GENETATE_START: 不发送起始信号
  * @param   e_stop_status: 本次通信后是否发送停止信号
  *            @arg EN_AT24CXX_GENETATE_STOP: 发送停止信号
  *            @arg EN_AT24CXX_NOT_GENETATE_STOP: 不发送停止信号
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Receive_Func)(uint8_t *p_receive_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);

```

​	 函数接口定义与AT24CXX对象定义如下：

```
typedef struct
{
    AT24CXX_Init_Func           m_p_Init;           /* 初始化函数指针 */
    AT24CXX_Send_Func           m_Send_Func;        /* 发送数据函数指针 */
    AT24CXX_Receive_Func        m_Receive_Func;     /* 接收数据函数指针 */
} at24cxx_interface_func_t;


typedef struct
{
    at24cxx_interface_func_t m_interface_func;      /* 接口函数 */
} at24cxx_obj_t;
```


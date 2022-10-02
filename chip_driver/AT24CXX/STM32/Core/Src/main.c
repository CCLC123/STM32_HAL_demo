/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
i2c_obj_t i2c_at24cxx;
at24cxx_obj_t at24c02;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static en_i2c_status_t I_I2C_Init_Pin_Func(en_i2c_pin_t e_pin);
static en_i2c_status_t I_I2C_Config_Pin_Mode_Func(en_i2c_pin_t e_pin, en_i2c_pin_mode_t e_pin_mode);
static en_i2c_pin_status_t I_I2C_Read_Pin_Func(en_i2c_pin_t e_pin);
static en_i2c_status_t I_I2C_Write_Pin_Func(en_i2c_pin_t e_pin, en_i2c_pin_status_t e_pin_status);
static en_i2c_status_t I_I2C_Delay_us_Func(uint32_t us);


en_at24cxx_status_t I_AT24CXX_Init_Func(void);
en_at24cxx_status_t I_AT24CXX_Send_Func(const uint8_t *p_send_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);
en_at24cxx_status_t I_AT24CXX_Receive_Func(uint8_t *p_receive_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);

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
    
    /* 设置中断向量表 */
    NVIC_SET_VECTOR(VECT_TAB_FLASH_BASE_ADDRESS, VECT_TAB_OFFSET);
    
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
    MX_DMA_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    
    {
        at24cxx_interface_func_t at24c02_interface = {I_AT24CXX_Init_Func, I_AT24CXX_Send_Func, I_AT24CXX_Receive_Func};
        AT24CXX_Init(&at24c02, &at24c02_interface);
    }
    static uint8_t send_buff[256] = {0};
    static uint8_t receive_buff[256] = {0};    
    
    AT24CXX_Test(&at24c02, send_buff, receive_buff);
    
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        HAL_Delay(100);
        printf("Hello ");
        USART1_Printf("World\r\n");
        
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
    RCC_OscInitStruct.PLL.PLLM = 15;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */


/**
  * @brief   初始化 I2C 引脚
  * @note    该函数应开启 GPIO 的时钟
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @return  en_i2c_status_t
  */
static en_i2c_status_t I_I2C_Init_Pin_Func(en_i2c_pin_t e_pin)
{
    /* 该函数应该开启 GPIO 的时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    return EN_I2C_OK;
}


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
static en_i2c_status_t I_I2C_Config_Pin_Mode_Func(en_i2c_pin_t e_pin, en_i2c_pin_mode_t e_pin_mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (e_pin == EN_I2C_PIN_SCL)
    {
        GPIO_InitStruct.Pin = I2C_AT24CXX_SCL_Pin;
    }
    else
    {
        GPIO_InitStruct.Pin = I2C_AT24CXX_SDA_Pin;
    }
    
    if (e_pin_mode == EN_I2C_PIN_MODE_OUTPUT)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    }
    else
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }
    
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    return EN_I2C_OK;
}


/**
  * @brief   读取 I2C 引脚的状态
  * @note    该函数应提供获取 GPIO 引脚电平状态的接口
  * @param   e_pin: I2C 的引脚选择
  *   @arg     EN_I2C_PIN_SCL: SCL引脚
  *   @arg     EN_I2C_PIN_SDA: SDA引脚
  * @return  en_i2c_pin_status_t
  */
static en_i2c_pin_status_t I_I2C_Read_Pin_Func(en_i2c_pin_t e_pin)
{
    if (e_pin == EN_I2C_PIN_SCL)
    {
        return (en_i2c_pin_status_t)HAL_GPIO_ReadPin(I2C_AT24CXX_SCL_GPIO_Port, I2C_AT24CXX_SCL_Pin);
    }
    else
    {
        return (en_i2c_pin_status_t)HAL_GPIO_ReadPin(I2C_AT24CXX_SDA_GPIO_Port, I2C_AT24CXX_SDA_Pin);
    }
}


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
static en_i2c_status_t I_I2C_Write_Pin_Func(en_i2c_pin_t e_pin, en_i2c_pin_status_t e_pin_status)
{
    if (e_pin == EN_I2C_PIN_SCL)
    {
        HAL_GPIO_WritePin(I2C_AT24CXX_SCL_GPIO_Port, I2C_AT24CXX_SCL_Pin, (GPIO_PinState)e_pin_status);
    }
    else
    {
        HAL_GPIO_WritePin(I2C_AT24CXX_SDA_GPIO_Port, I2C_AT24CXX_SDA_Pin, (GPIO_PinState)e_pin_status);
    }
    
    return EN_I2C_OK;
}


/**
  * @brief   微秒级延迟
  * @note    该函数应提供微秒级延迟的接口
  * @param   us: 微秒值
  * @return  en_i2c_status_t
  */
static en_i2c_status_t I_I2C_Delay_us_Func(uint32_t us)
{
    uint32_t delay = us * SystemCoreClock / 1000000;
    do
    {
        __NOP;
    } while (delay--);
    
    return EN_I2C_OK;
}




/**
  * @brief   AT24CXX 初始化
  * @note    该函数应初始化与 AT24CXX 通信的接口, 如 I2C
  * @param   None
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t I_AT24CXX_Init_Func(void)
{
    i2c_interface_func_t i2c_interface = {I_I2C_Init_Pin_Func, I_I2C_Config_Pin_Mode_Func, I_I2C_Read_Pin_Func, I_I2C_Write_Pin_Func, I_I2C_Delay_us_Func};
    I2C_Init(&i2c_at24cxx, &i2c_interface, EN_I2C_SPEED_250KHZ);
    
    return EN_AT24CXX_ACK;
}



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
static en_at24cxx_status_t I_AT24CXX_Send_Func(const uint8_t *p_send_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status)
{
    en_i2c_status_t status = EN_I2C_OK;
    /* 发送起始信号 */
    if (e_start_status == EN_AT24CXX_GENETATE_START)
    {
        status = I2C_Start(&i2c_at24cxx);
        if (status != EN_I2C_OK)
        {
            return EN_AT24CXX_ERROR;
        }
    }
    /* 发送数据 */
    status = I2C_Write_With_Ack(&i2c_at24cxx, p_send_buff, length);
    
    /* 发送停止信号 */
    if (e_stop_status == EN_AT24CXX_GENETATE_STOP)
    {
        status = I2C_Stop(&i2c_at24cxx);
        if ( status != EN_I2C_OK)
        {
            return EN_AT24CXX_ERROR;
        }
    }
    return EN_AT24CXX_ACK;
}


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
static en_at24cxx_status_t I_AT24CXX_Receive_Func(uint8_t *p_receive_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status)
{
    en_i2c_status_t status = EN_I2C_OK;
    /* 发送起始信号 */
    if (e_start_status == EN_AT24CXX_GENETATE_START)
    {
        status = I2C_Start(&i2c_at24cxx);
        if (status != EN_I2C_OK)
        {
            return EN_AT24CXX_ERROR;
        }
    }
    /* 接收数据 */
    status = I2C_Read_With_Ack(&i2c_at24cxx, p_receive_buff, length);
    
    /* 发送停止信号 */
    if (e_stop_status == EN_AT24CXX_GENETATE_STOP)
    {
        status = I2C_Stop(&i2c_at24cxx);
        if ( status != EN_I2C_OK)
        {
            return EN_AT24CXX_ERROR;
        }
    }
    return EN_AT24CXX_ACK;
}




/* USER CODE END 4 */

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

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
#include "spi.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static en_w25qxx_status_t I_W25QXX_Init(void);
static en_w25qxx_status_t I_W25QXX_Send_Receive(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t is_continue_com);

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
    
    /* ���ж������� */
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
    MX_SPI5_Init();
    /* USER CODE BEGIN 2 */
    
    
#define W25QXX_NUM (1U)
    extern uint8_t garr_w25qxx_send_buff[W25QXX_SECTOR_SIZE];
    extern uint8_t garr_w25qxx_receive_buff[W25QXX_SECTOR_SIZE];

    /* �����ӿ����� */
    const w25qxx_interface_func_t garr_w25qxx_interface_func[W25QXX_NUM] = {{I_W25QXX_Init, I_W25QXX_Send_Receive}};
    
     /* W25QXX ʵ���������� */
    w25qxx_obj_t garr_w25qxx[W25QXX_NUM] = {0};
    
    /* ��ʼ���� 0 �� W25QXX ʵ�������� */
    W25QXX_Init(&garr_w25qxx[0], &garr_w25qxx_interface_func[0]);
    
    for (uint32_t i = 0; i < sizeof(garr_w25qxx_send_buff); i++)
    {
        garr_w25qxx_send_buff[i] = 0x2A;
    }
    
    Write_W25QXX(&garr_w25qxx[0], 22, garr_w25qxx_send_buff, garr_w25qxx_receive_buff, sizeof(garr_w25qxx_send_buff));
    Read_W25QXX(&garr_w25qxx[0], 0, garr_w25qxx_send_buff, garr_w25qxx_receive_buff, sizeof(garr_w25qxx_send_buff));
    
    
    Test_W25QXX(&garr_w25qxx[0], garr_w25qxx_send_buff, garr_w25qxx_receive_buff);
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        HAL_Delay(100);
        USART1_Printf("Hello World\r\n");
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
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

/* ---------- �ӿ� BEGIN ---------- */
/**
  * @brief   W25QXX ��ʼ��
  * @note    �ú���Ӧ��ʼ���� W25QXX ͨ�ŵĽӿ�, ���� SPI
  * @param   None
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t I_W25QXX_Init(void)
{
    /* �ú���Ӧ�ó�ʼ���� W25QXX ͨ�ŵĽӿ�, ���� SPI */
    /* HAL�Ѿ������ǳ�ʼ���� W25QXX ͨ�ŵĽӿ��� */
    return EN_W25QXX_OK;
}


/**
  * @brief   �� W25QXX ���� p_send_buff �ͽ��� W25QXX �����ݵ� p_receive_buff
  * @note    �ú���ʵ���� W25QXX �������ݵ��շ�, ��ʹ���� W25QXX ͨ�ŵĽӿ�, ���� SPI
  * @param   p_send_buff: ���ͻ�������ʼ��ַ
  * @param   p_receive_buff: ���ջ�������ʼ��ַ
  * @param   length: ����/���յ����ݳ���, ��λ: �ֽ�
  * @param   is_continue_com: ���η������֮���Ƿ�Ҫ����ͨ��, ����ͨ���� CS ���ֵ͵�ƽ
  *             EN_W25QXX_CLOSE_COM: ����ͨ��
  *             EN_W25QXX_CONTINUE_COM: ����ͨ��
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t I_W25QXX_Send_Receive(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t is_continue_com)
{
    uint32_t timeout = 0;
    HAL_StatusTypeDef state = HAL_OK;
    
    HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
    state = SPI5_DMA_Send_Receive(p_send_buff, p_receive_buff, length);
    
    /* �ȴ��������, ��λ���������� */
    while (__HAL_SPI_GET_FLAG(&hspi5, SPI_FLAG_BSY) == SET)
    {
        timeout++;
        if (timeout >= W25QXX_TIMEOUT)
        {
            return EN_W25QXX_TIMEOUT;
        }
    }
    /* ���ݲ��������� CS �ĵ�ƽ */
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
/* ---------- �ӿ� END ---------- */

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

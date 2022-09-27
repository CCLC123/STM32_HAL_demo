/* Includes ------------------------------------------------------------------*/
#include "gpio_i2c.h"

/* Private typedef -----------------------------------------------------------*/




/* Private define ------------------------------------------------------------*/
#ifndef ENABLE_I2C_DEBUG
    #define I2C_DEBUG_PRINTF(p_format, ...)
#endif /* ENABLE_I2C_DEBUG */

#ifndef NULL
    #define NULL ((void*)0)
#endif /* NULL */


/* Private macro -------------------------------------------------------------*/
#define CONVERT_TO_STRING(value)    #value    /* 字符串化 */
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_BEGIN(str)            "[+++]%s()->%s begin\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_END(str)              "[---]%s()->%s end\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_INFO(str, ...)        "[INFO]%s()->" str "\r\n", __FUNCTION__, ## __VA_ARGS__

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   初始化 I2C, 初始化 I2C 实例
  * @param   p_obj: i2c_obj_t 的地址
  * @param   p_interface_func: i2c_interface_func_t 的地址
  * @param   speed: I2C 的速率
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, uint32_t speed)
{
    en_i2c_status_t e_status = EN_I2C_OK;
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN(init));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    /* 初始化函数接口 */
    p_obj->m_interface_func = *p_interface_func;
    
    /* 执行初始化函数 */
    if (p_obj->m_interface_func.m_p_Init_Pin == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    e_status = p_obj->m_interface_func.m_p_Init_Pin(EN_I2C_PIN_SCL);
    e_status = p_obj->m_interface_func.m_p_Init_Pin(EN_I2C_PIN_SDA);
    
    /* 设置 GPIO 工作模式 */
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SCL, EN_I2C_PIN_MODE_PP);
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_PP);
    
    /* 设置 GPIO 引脚电平状态为高电平 */
    e_status = p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    e_status = p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    
    /* 设置 I2C 的速率 */
    p_obj->m_speed = speed;
    
    if (e_status != EN_I2C_OK)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_ERROR));
        return EN_I2C_ERROR;
    }
    
    I2C_DEBUG_PRINTF(DEBUG_END(init));
    
    return e_status;
}


/**
  * @brief   产生 I2C 起始信号
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Start(i2c_obj_t *p_obj)
{
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_PP);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("start"));
    
    return EN_I2C_OK;
}


/**
  * @brief   产生 I2C 停止信号
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Stop(i2c_obj_t *p_obj)
{
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_PP);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("stop"));
    
    return EN_I2C_OK;
}


/**
  * @brief   等待 I2C 从设备的应答
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Wait_for_ACK(i2c_obj_t *p_obj)
{
    uint32_t timeout = 0;
    
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_INPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF / 4);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_ms(0xFFFF / 4);
    while(p_obj->m_interface_func.m_p_Read_Pin(EN_I2C_PIN_SDA) == EN_I2C_SET)
    {
        timeout++;
        if(timeout >= I2C_TIMEOUT)
        {
            I2C_Stop(p_obj);
            
            I2C_DEBUG_PRINTF(DEBUG_INFO("wait for ack fail"));
            I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_TIMEOUT));
            return EN_I2C_TIMEOUT;
        }
    }
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("wait for ack ok"));
    
    return EN_I2C_OK;
}


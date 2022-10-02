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
#define DEBUG_ERROR(error)          "[ERROR]%s: %s()->[%u]: %s\r\n", __FILE__, __FUNCTION__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_INFO(str, ...)        "[INFO]%s()->" str "\r\n", __FUNCTION__, ## __VA_ARGS__
#define DEBUG_BEGIN()               "[+++]%s()->begin\r\n", __FUNCTION__
#define DEBUG_END()                 "[---]%s()->end\r\n", __FUNCTION__

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static inline uint32_t Get_Delay_Us_value(i2c_obj_t *p_obj)
{
    switch (p_obj->m_speed)
    {
    case EN_I2C_SPEED_100KHZ:
        return 10U;
    case EN_I2C_SPEED_200KHZ:
        return 5U;
    case EN_I2C_SPEED_500KHZ:
        return 2U;
    case EN_I2C_SPEED_1000KHZ:
        return 1U;
    case EN_I2C_SPEED_250KHZ:
    default :
        return 4U;
    }
}

/**
  * @brief   初始化 I2C, 初始化 I2C 实例
  * @param   p_obj: i2c_obj_t 的地址
  * @param   p_interface_func: i2c_interface_func_t 的地址
  * @param   speed: I2C 的速率
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, en_i2c_speed_t speed)
{
    en_i2c_status_t e_status = EN_I2C_OK;
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN());
    
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
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SCL, EN_I2C_PIN_MODE_OUTPUT);
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    
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
    
    I2C_DEBUG_PRINTF(DEBUG_END());
    
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
    
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
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
    
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("stop"));
    
    return EN_I2C_OK;
}


/**
  * @brief   等待 I2C 从设备的应答
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Wait_For_Ack(i2c_obj_t *p_obj)
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
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    while (p_obj->m_interface_func.m_p_Read_Pin(EN_I2C_PIN_SDA) == EN_I2C_SET)
    {
        timeout++;
        if (timeout >= I2C_TIMEOUT)
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


/**
  * @brief   产生 I2C 的应答信号
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Ack(i2c_obj_t *p_obj)
{
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("ack ok"));
    
    return EN_I2C_OK;
}


/**
  * @brief   不产生 I2C 的应答信号
  * @param   p_obj: i2c_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Not_Ack(i2c_obj_t *p_obj)
{
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("not ack ok"));
    
    return EN_I2C_OK;
}



/**
  * @brief   在 I2C 总线上写入一个字节数据, 不检查应答信号
  * @param   p_obj: i2c_obj_t 的地址
  * @param   data: 数据
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Write_One_Byte_Without_Ack(i2c_obj_t *p_obj, uint8_t data)
{
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("data = %02X", data));
    
    /* 发送数据 */
    for (uint8_t i = 0; i < 8; i++)
    {
        #if (I2C_BIT_FIRST == I2C_MSB)
        /* MSB, 先发送最高位 */
        if (data & 0x80)
        {
            p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
        }
        else
        {
            p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
        }
        data <<= 1;
        #else
        /*  LSB, 先发送最低位*/
        if (data & 0x01)
        {
            p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
        }
        else
        {
            p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_RESET);
        }
        data >>= 1;
        #endif /* I2C_BIT_FIRST */
        p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
        p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
        p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
        p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
        p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    }
    
    return EN_I2C_OK;
}


/**
  * @brief   在 I2C 总线上写入指定的长度的数据 p_data
  * @param   p_obj: i2c_obj_t 的地址
  * @param   p_data: 数据缓冲区地址
  * @param   length: 数据长度, 单位: 字节
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Write_With_Ack(i2c_obj_t *p_obj, const uint8_t *p_data, uint32_t length)
{
    /* 检查函数参数 */
    if (p_obj == NULL || p_data == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN());
    I2C_DEBUG_PRINTF(DEBUG_INFO("p_data = %p, length = %u", p_data, length));
    
    for (uint32_t i = 0; i < length; i++)
    {
        /* 发送数据 */
        I2C_Write_One_Byte_Without_Ack(p_obj, p_data[i]);
        /* 等待应答 */
        if (I2C_Wait_For_Ack(p_obj) != EN_I2C_OK)
        {
            I2C_Stop(p_obj);
            I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_ERROR));
            return EN_I2C_ERROR;
        }
    }
    
    I2C_DEBUG_PRINTF(DEBUG_END());
    
    return EN_I2C_OK;
}



/**
  * @brief   在 I2C 总线上读取一个字节数据到 p_data 中
  * @param   p_obj: i2c_obj_t 的地址
  * @param   p_data: 数据地址
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Read_One_Byte_Without_Ack(i2c_obj_t *p_obj, uint8_t *p_data)
{
    *p_data = 0;
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_INPUT);
    
    for (uint8_t i = 0; i < 8; i++)
    {
        p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
        p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
        p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
        #if (I2C_BIT_FIRST == I2C_MSB)
        *p_data <<= 1;
        if (p_obj->m_interface_func.m_p_Read_Pin(EN_I2C_PIN_SDA) == EN_I2C_SET)
        {
            (*p_data)++;
        }
        #else
        *p_data >>= 1;
        if (p_obj->m_interface_func.m_p_Read_Pin(EN_I2C_PIN_SDA) == EN_I2C_SET)
        {
            *p_data |= 0x80;
        }
        #endif /* I2C_BIT_FIRST */
        p_obj->m_interface_func.m_p_Delay_us(Get_Delay_Us_value(p_obj));
    }
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("read data: %02X", *p_data));
    
    return EN_I2C_OK;
}


/**
  * @brief   在 I2C 总线上读取指定长度的数据到 p_data 中, 自动产生应答信号, 并且最后一个字节不产生应答信号
  * @param   p_obj: i2c_obj_t 的地址
  * @param   p_data: 数据缓冲区地址
  * @param   length: 数据长度, 单位: 字节
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Read_With_Ack(i2c_obj_t *p_obj, uint8_t *p_data, uint32_t length)
{
    /* 检查函数参数 */
    if (p_obj == NULL || p_data == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    for (uint32_t i = 0; i < length - 1; i++)
    {
        I2C_Read_One_Byte_Without_Ack(p_obj, p_data);
        I2C_Ack(p_obj);
        p_data++;
    }
    
    /* 读取最后一个字节后不产生应答信号 */
    I2C_Read_One_Byte_Without_Ack(p_obj, p_data);
    I2C_Not_Ack(p_obj);
    
    return EN_I2C_OK;
}

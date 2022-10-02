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
#define CONVERT_TO_STRING(value)    #value    /* �ַ����� */
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
  * @brief   ��ʼ�� I2C, ��ʼ�� I2C ʵ��
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   p_interface_func: i2c_interface_func_t �ĵ�ַ
  * @param   speed: I2C ������
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, en_i2c_speed_t speed)
{
    en_i2c_status_t e_status = EN_I2C_OK;
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN());
    
    /* ��麯������ */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    /* ��ʼ�������ӿ� */
    p_obj->m_interface_func = *p_interface_func;
    
    /* ִ�г�ʼ������ */
    if (p_obj->m_interface_func.m_p_Init_Pin == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    e_status = p_obj->m_interface_func.m_p_Init_Pin(EN_I2C_PIN_SCL);
    e_status = p_obj->m_interface_func.m_p_Init_Pin(EN_I2C_PIN_SDA);
    
    /* ���� GPIO ����ģʽ */
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SCL, EN_I2C_PIN_MODE_OUTPUT);
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    
    /* ���� GPIO ���ŵ�ƽ״̬Ϊ�ߵ�ƽ */
    e_status = p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_SET);
    e_status = p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SDA, EN_I2C_SET);
    
    /* ���� I2C ������ */
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
  * @brief   ���� I2C ��ʼ�ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Start(i2c_obj_t *p_obj)
{
    /* ��麯������ */
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
  * @brief   ���� I2C ֹͣ�ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Stop(i2c_obj_t *p_obj)
{
    /* ��麯������ */
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
  * @brief   �ȴ� I2C ���豸��Ӧ��
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Wait_For_Ack(i2c_obj_t *p_obj)
{
    uint32_t timeout = 0;
    
    /* ��麯������ */
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
  * @brief   ���� I2C ��Ӧ���ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Ack(i2c_obj_t *p_obj)
{
    /* ��麯������ */
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
  * @brief   ������ I2C ��Ӧ���ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Not_Ack(i2c_obj_t *p_obj)
{
    /* ��麯������ */
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
  * @brief   �� I2C ������д��һ���ֽ�����, �����Ӧ���ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   data: ����
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Write_One_Byte_Without_Ack(i2c_obj_t *p_obj, uint8_t data)
{
    p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_OUTPUT);
    p_obj->m_interface_func.m_p_Write_Pin(EN_I2C_PIN_SCL, EN_I2C_RESET);
    
    I2C_DEBUG_PRINTF(DEBUG_INFO("data = %02X", data));
    
    /* �������� */
    for (uint8_t i = 0; i < 8; i++)
    {
        #if (I2C_BIT_FIRST == I2C_MSB)
        /* MSB, �ȷ������λ */
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
        /*  LSB, �ȷ������λ*/
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
  * @brief   �� I2C ������д��ָ���ĳ��ȵ����� p_data
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   p_data: ���ݻ�������ַ
  * @param   length: ���ݳ���, ��λ: �ֽ�
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Write_With_Ack(i2c_obj_t *p_obj, const uint8_t *p_data, uint32_t length)
{
    /* ��麯������ */
    if (p_obj == NULL || p_data == NULL)
    {
        I2C_DEBUG_PRINTF(DEBUG_ERROR(EN_I2C_PARAM_IS_NULL));
        return EN_I2C_PARAM_IS_NULL;
    }
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN());
    I2C_DEBUG_PRINTF(DEBUG_INFO("p_data = %p, length = %u", p_data, length));
    
    for (uint32_t i = 0; i < length; i++)
    {
        /* �������� */
        I2C_Write_One_Byte_Without_Ack(p_obj, p_data[i]);
        /* �ȴ�Ӧ�� */
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
  * @brief   �� I2C �����϶�ȡһ���ֽ����ݵ� p_data ��
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   p_data: ���ݵ�ַ
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
  * @brief   �� I2C �����϶�ȡָ�����ȵ����ݵ� p_data ��, �Զ�����Ӧ���ź�, �������һ���ֽڲ�����Ӧ���ź�
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   p_data: ���ݻ�������ַ
  * @param   length: ���ݳ���, ��λ: �ֽ�
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Read_With_Ack(i2c_obj_t *p_obj, uint8_t *p_data, uint32_t length)
{
    /* ��麯������ */
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
    
    /* ��ȡ���һ���ֽں󲻲���Ӧ���ź� */
    I2C_Read_One_Byte_Without_Ack(p_obj, p_data);
    I2C_Not_Ack(p_obj);
    
    return EN_I2C_OK;
}

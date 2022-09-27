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
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_BEGIN(str)            "[+++]%s()->%s begin\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_END(str)              "[---]%s()->%s end\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_INFO(str, ...)        "[INFO]%s()->" str "\r\n", __FUNCTION__, ## __VA_ARGS__

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   ��ʼ�� I2C, ��ʼ�� I2C ʵ��
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @param   p_interface_func: i2c_interface_func_t �ĵ�ַ
  * @param   speed: I2C ������
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, uint32_t speed)
{
    en_i2c_status_t e_status = EN_I2C_OK;
    
    I2C_DEBUG_PRINTF(DEBUG_BEGIN(init));
    
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
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SCL, EN_I2C_PIN_MODE_PP);
    e_status = p_obj->m_interface_func.m_p_Config_Pin_Mode(EN_I2C_PIN_SDA, EN_I2C_PIN_MODE_PP);
    
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
    
    I2C_DEBUG_PRINTF(DEBUG_END(init));
    
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
  * @brief   �ȴ� I2C ���豸��Ӧ��
  * @param   p_obj: i2c_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_i2c_status_t I2C_Wait_for_ACK(i2c_obj_t *p_obj)
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


/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __GPIO_I2C_H
#define __GPIO_I2C_H

#ifdef __cplusplus
extern "C" {
#endif




/* Includes -------------------------------------------------------------------*/
#include <stdint.h>
#include "gpio_i2c_conf.h"

/* Exported typedef -----------------------------------------------------------*/
typedef enum
{
    EN_I2C_PIN_SCL = 0,
    EN_I2C_PIN_SDA = 1
} en_i2c_pin_t;

typedef enum
{
    EN_I2C_PIN_MODE_PP = 0,
    EN_I2C_PIN_MODE_OD = 1,
    EN_I2C_PIN_MODE_INPUT = 2
} en_i2c_pin_mode_t;


typedef enum
{
    EN_I2C_RESET = 0,
    EN_I2C_SET = 1
} en_i2c_pin_status_t;

typedef enum
{
    EN_I2C_ERROR = 0,            /* ���� */
    EN_I2C_OK = 1,               /* �ɹ� */
    EN_I2C_TIMEOUT = 2,          /* ��ʱ */
    EN_I2C_PARAM_IS_NULL = 3,    /* ��������Ϊ�� */
    
    EN_I2C_FAIL = EN_I2C_ERROR,
    EN_I2C_SUCCESS = EN_I2C_OK
} en_i2c_status_t;



/**
  * @brief   ��ʼ�� I2C ����
  * @note    �ú���Ӧ���� GPIO ��ʱ��
  * @param   e_pin: I2C ������ѡ��
  *   @arg     EN_I2C_PIN_SCL: SCL����
  *   @arg     EN_I2C_PIN_SDA: SDA����
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Init_Pin_Func)(en_i2c_pin_t e_pin);


/**
  * @brief   ���� I2C ����ģʽ
  * @note    �ú���Ӧ�ṩ���� GPIO ����ģʽ�Ľӿ�
  * @param   e_pin: I2C ������ѡ��
  *   @arg     EN_I2C_PIN_SCL: SCL����
  *   @arg     EN_I2C_PIN_SDA: SDA����
  * @param   e_pin_mode: I2C ������ģʽ
  *   @arg     EN_I2C_PIN_MODE_PP: �����������ģʽ
  *   @arg     EN_I2C_PIN_MODE_OD: ��©�������ģʽ
  *   @arg     EN_I2C_PIN_MODE_INPUT: ��������ģʽ
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Config_Pin_Mode_Func)(en_i2c_pin_t e_pin, en_i2c_pin_mode_t e_pin_mode);


/**
  * @brief   ��ȡ I2C ���ŵ�״̬
  * @note    �ú���Ӧ�ṩ��ȡ GPIO ���ŵ�ƽ״̬�Ľӿ�
  * @param   e_pin: I2C ������ѡ��
  *   @arg     EN_I2C_PIN_SCL: SCL����
  *   @arg     EN_I2C_PIN_SDA: SDA����
  * @return  en_i2c_pin_status_t
  */
typedef en_i2c_pin_status_t (*I2C_Read_Pin_Func)(en_i2c_pin_t e_pin);


/**
  * @brief   д�� I2C ���ŵ�״̬
  * @note    �ú���Ӧ�ṩ���� GPIO ���ŵ�ƽ״̬�Ľӿ�
  * @param   e_pin: I2C ������ѡ��
  *   @arg     EN_I2C_PIN_SCL: SCL����
  *   @arg     EN_I2C_PIN_SDA: SDA����
  * @param   e_pin_status: I2C ������״̬ѡ��
  *   @arg     EN_I2C_RESET: �͵�ƽ
  *   @arg     EN_I2C_SET: �ߵ�ƽ
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Write_Pin_Func)(en_i2c_pin_t e_pin, en_i2c_pin_status_t e_pin_status);



/**
  * @brief   ���뼶�ӳ�
  * @note    �ú���Ӧ�ṩ���뼶�ӳٵĽӿ�
  * @param   ms: ����ֵ
  * @return  en_i2c_status_t
  */
typedef en_i2c_status_t (*I2C_Delay_ms_Func)(uint32_t ms);



typedef struct
{
    I2C_Init_Pin_Func           m_p_Init_Pin;           /* ��ʼ�� I2C ���� */
    I2C_Config_Pin_Mode_Func    m_p_Config_Pin_Mode;    /* ���� I2C ����ģʽ */
    I2C_Read_Pin_Func           m_p_Read_Pin;           /* ��ȡ I2C ���ŵ�״̬ */
    I2C_Write_Pin_Func          m_p_Write_Pin;          /* д�� I2C ���ŵ�״̬ */
    I2C_Delay_ms_Func           m_p_Delay_ms;           /* ���뼶�ӳ� */
} i2c_interface_func_t;


typedef struct
{
    uint32_t m_speed;                              /* �ٶ� */
    i2c_interface_func_t m_interface_func;         /* �ӿں��� */
} i2c_obj_t;
/* Exported define ------------------------------------------------------------*/

/* Exported macro -------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /* __GPIO_I2C_H */

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
    EN_I2C_ERROR = 0,            /* 出错 */
    EN_I2C_OK = 1,               /* 成功 */
    EN_I2C_TIMEOUT = 2,          /* 超时 */
    EN_I2C_PARAM_IS_NULL = 3,    /* 函数参数为空 */
    
    EN_I2C_FAIL = EN_I2C_ERROR,
    EN_I2C_SUCCESS = EN_I2C_OK
} en_i2c_status_t;

typedef enum
{
    EN_I2C_PIN_SCL = 0,
    EN_I2C_PIN_SDA = 1
} en_i2c_pin_t;

typedef enum
{
    EN_I2C_PIN_MODE_OUTPUT = 0,
    EN_I2C_PIN_MODE_INPUT = 1
} en_i2c_pin_mode_t;


typedef enum
{
    EN_I2C_RESET = 0,
    EN_I2C_SET = 1
} en_i2c_pin_status_t;


typedef enum
{
    EN_I2C_SPEED_100KHZ = 0,
    EN_I2C_SPEED_200KHZ = 1,
    EN_I2C_SPEED_250KHZ = 2,
    EN_I2C_SPEED_500KHZ = 3,
    EN_I2C_SPEED_1000KHZ = 4
} en_i2c_speed_t;


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
/* Exported define ------------------------------------------------------------*/

/* Exported macro -------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
en_i2c_status_t I2C_Init(i2c_obj_t *p_obj, const i2c_interface_func_t *p_interface_func, en_i2c_speed_t speed);

en_i2c_status_t I2C_Start(i2c_obj_t *p_obj);
en_i2c_status_t I2C_Stop(i2c_obj_t *p_obj);
en_i2c_status_t I2C_Wait_For_Ack(i2c_obj_t *p_obj);
en_i2c_status_t I2C_Ack(i2c_obj_t *p_obj);
en_i2c_status_t I2C_Not_Ack(i2c_obj_t *p_obj);
en_i2c_status_t I2C_Write_One_Byte_Without_Ack(i2c_obj_t *p_obj, uint8_t data);
en_i2c_status_t I2C_Write_With_Ack(i2c_obj_t *p_obj, const uint8_t *p_data, uint32_t length);
en_i2c_status_t I2C_Read_One_Byte_Without_Ack(i2c_obj_t *p_obj, uint8_t *p_data);
en_i2c_status_t I2C_Read_With_Ack(i2c_obj_t *p_obj, uint8_t *p_data, uint32_t length);
/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /* __GPIO_I2C_H */

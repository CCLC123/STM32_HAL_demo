/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT24CXX_H
#define __AT24CXX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Export Includes ----------------------------------------------------------*/
#include <stdint.h>
#include "at24cxx_conf.h"

/* Export typedef -----------------------------------------------------------*/
typedef enum
{
    EN_AT24CXX_ERROR = 0,            /* 出错 */
    EN_AT24CXX_OK = 1,               /* 成功 */
    EN_AT24CXX_TIMEOUT = 2,          /* 超时 */
    EN_AT24CXX_PARAM_IS_NULL = 3,    /* 函数参数为空 */
    
    EN_AT24CXX_NACK = EN_AT24CXX_ERROR, /* 不应答 */
    EN_AT24CXX_ACK = EN_AT24CXX_OK      /* 应答 */
} en_at24cxx_status_t;


typedef enum
{
    EN_AT24CXX_GENETATE_START = 0,
    EN_AT24CXX_NOT_GENETATE_START = 1
}en_at24cxx_start_t;


typedef enum
{
    EN_AT24CXX_GENETATE_STOP = 0,
    EN_AT24CXX_NOT_GENETATE_STOP = 1
}en_at24cxx_stop_t;


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

/* Export define ------------------------------------------------------------*/

/* Export macro -------------------------------------------------------------*/

/* Export variables ---------------------------------------------------------*/

/* Export function prototypes -----------------------------------------------*/
en_at24cxx_status_t AT24CXX_Init(at24cxx_obj_t *p_obj, const at24cxx_interface_func_t *p_interface_func);

en_at24cxx_status_t AT24CXX_Read(at24cxx_obj_t *p_obj, uint32_t addr, uint8_t *p_receive_buff, uint32_t length);
en_at24cxx_status_t AT24CXX_Write(at24cxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint32_t length);

en_at24cxx_status_t AT24CXX_Test(at24cxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff);


#ifdef ENABLEAT24CXX_WAIT_CALLBACK
void AT24CXX_Wait_Callback(void);
#endif /* ENABLEAT24CXX_WAIT_CALLBACK */


/* Export functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __AT24CXX_H */

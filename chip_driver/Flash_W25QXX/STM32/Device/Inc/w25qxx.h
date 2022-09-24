/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Export Includes ----------------------------------------------------------*/
#include <stdint.h>
#include "w25qxx_conf.h"

/* Export typedef -----------------------------------------------------------*/
typedef enum
{
    EN_W25QXX_ERROR = 0,            /* 出错 */
    EN_W25QXX_OK = 1,               /* 成功 */
    EN_W25QXX_TIMEOUT = 2,          /* 超时 */
    EN_W25QXX_PARAM_IS_NULL = 3,    /* 函数参数为空 */
    
    EN_W25QXX_FAIL = EN_W25QXX_ERROR,
    EN_W25QXX_SUCCESS = EN_W25QXX_OK
} en_w25qxx_status_t;

typedef enum
{
    EN_W25QXX_CLOSE_COM = 0,        /* 结束(关闭)通信 */
    EN_W25QXX_CONTINUE_COM = 1,     /* 继续通信 */
} en_w25qxx_com_action_status_t;


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
  *             EN_W25QXX_CLOSE_COM: 结束通信
  *             EN_W25QXX_CONTINUE_COM: 继续通信
  * @return  en_w25qxx_status_t
  */
typedef en_w25qxx_status_t (*w25qxx_Send_Receive_Func)(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t is_continue_com);


typedef struct
{
    w25qxx_Init_Func           m_p_Init;           /* 初始化函数指针 */
    w25qxx_Send_Receive_Func   m_p_Send_Receive;   /* 发送与接收函数指针 */
} w25qxx_interface_func_t;


typedef struct
{
    uint64_t m_unique_ID;                          /* 设备唯一ID */
    w25qxx_interface_func_t m_interface_func;      /* 接口函数 */
} w25qxx_obj_t;

/* Export define ------------------------------------------------------------*/
#define W25QXX_SECTOR_SIZE         (0x00001000U)   /* 4096 */  /* 扇区大小, 单位: 字节 */

/* Export macro -------------------------------------------------------------*/

/* Export variables ---------------------------------------------------------*/

/* Export function prototypes -----------------------------------------------*/
en_w25qxx_status_t W25QXX_Init(w25qxx_obj_t *p_obj, const w25qxx_interface_func_t *p_interface_func);

en_w25qxx_status_t Read_W25QXX_Unique_ID(w25qxx_obj_t *p_obj);
en_w25qxx_status_t W25QXX_Power_Down(w25qxx_obj_t *p_obj);
en_w25qxx_status_t W25QXX_Wakeup(w25qxx_obj_t *p_obj);
en_w25qxx_status_t Get_W25QXX_JEDEC_ID(w25qxx_obj_t *p_obj, uint8_t *p_MF_ID, uint16_t *p_ID);
en_w25qxx_status_t Read_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);
en_w25qxx_status_t W25QXX_Erase(w25qxx_obj_t *p_obj, uint32_t addr, uint32_t length);
en_w25qxx_status_t Write_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);
en_w25qxx_status_t Test_W25QXX(w25qxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff);




#ifdef ENABLEW25QXX_WAIT_CALLBACK
void W25QXX_Wait_Callback(void);
#endif /* ENABLEW25QXX_WAIT_CALLBACK */


/* Export functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */

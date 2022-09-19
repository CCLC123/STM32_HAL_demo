/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __W25Q256_H
#define __W25Q256_H

#ifdef __cplusplus
extern "C" {
#endif

/* Export Includes ----------------------------------------------------------*/
#include "main.h"

/* Export typedef -----------------------------------------------------------*/
typedef enum
{
    EN_W25Q256_ERROR = 0,
    EN_W25Q256_OK = 1,
    EN_W25Q256_TIMEOUT = 2,
    EN_W25Q256_PARAM_IS_NULL = 3,
    
    EN_W25Q256_FAIL = EN_W25Q256_ERROR,
    EN_W25Q256_SUCCESS = EN_W25Q256_OK
} en_w25q256_status_t;

typedef en_w25q256_status_t (*w25q256_Init_Func)(void);       /* w25q256 初始化函数 */
typedef en_w25q256_status_t (*w25q256_Send_Receive_Func)(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, uint8_t is_continue_com);  /* 向 w25q256 发送 p_send_buff 和接收 W25Q256 的数据到 p_receive_buff 函数 */


typedef struct
{
    w25q256_Init_Func           m_p_Init;           /* 初始化函数指针 */
    w25q256_Send_Receive_Func   m_p_Send_Receive;   /* 发送与接收函数指针 */
} w25q256_interface_func_t;


typedef struct
{
    uint64_t m_unique_ID;                           /* 设备唯一ID */
    w25q256_interface_func_t m_interface_func;      /* 接口函数 */
} w25q256_obj_t;

/* Export define ------------------------------------------------------------*/
//#define ENABLEW25Q256_WAIT_CALLBACK    /* 使能等待回调函数 */
#ifndef ENABLEW25Q256_WAIT_CALLBACK
     #define W25Q256_Wait_Callback()         ((void)0U)
#endif /* ENABLEW25Q256_WAIT_CALLBACK */
    
    
//#define ENABLE_W25Q256_DEBUG          /* 使能调试 */
#ifdef ENABLE_W25Q256_DEBUG
#define W25Q256_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)
#else
#define W25Q256_DEBUG_PRINTF(p_format, ...)           
#endif /* ENABLE_W25Q256_DEBUG */

#define W25Q256_NUM                 (1U)            /* W25Q256 的数量 */
#define W25Q256_SECTOR_SIZE         (0x00001000U)   /* 4096 */  /* 扇区大小，单位：字节 */

/* Export macro -------------------------------------------------------------*/

/* Export variables ---------------------------------------------------------*/
extern uint8_t garr_w25q256_send_buff[W25Q256_SECTOR_SIZE];
extern uint8_t garr_w25q256_receive_buff[W25Q256_SECTOR_SIZE];

extern const w25q256_interface_func_t garr_w25q256_interface_func[W25Q256_NUM];
extern w25q256_obj_t garr_w25q256[W25Q256_NUM];
     
/* Export function prototypes -----------------------------------------------*/
en_w25q256_status_t W25Q256_Init(w25q256_obj_t *p_obj, const w25q256_interface_func_t *p_interface_func);

en_w25q256_status_t Read_W25Q256_Unique_ID(w25q256_obj_t *p_obj);
en_w25q256_status_t W25Q256_Power_Down(w25q256_obj_t *p_obj);
en_w25q256_status_t W25Q256_Wakeup(w25q256_obj_t *p_obj);
en_w25q256_status_t Get_W25Q256_JEDEC_ID(w25q256_obj_t *p_obj, uint8_t *p_MF_ID, uint16_t *p_ID);
en_w25q256_status_t Read_W25Q256(w25q256_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);
en_w25q256_status_t Write_W25Q256(w25q256_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);
en_w25q256_status_t Test_W25Q256(w25q256_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff);




#ifdef ENABLEW25Q256_WAIT_CALLBACK
void W25Q256_Wait_Callback(void);
#endif /* ENABLEW25Q256_WAIT_CALLBACK */


/* Export functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __W25Q256_H */

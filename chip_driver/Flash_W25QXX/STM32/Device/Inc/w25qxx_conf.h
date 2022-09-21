/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __W25QXX_CONF_H
#define __W25QXX_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Export Includes ----------------------------------------------------------*/
#include "main.h"

/* Export typedef -----------------------------------------------------------*/


/* Export define ------------------------------------------------------------*/
/**
  * @brief W25QXX 芯片类型
  */
#define W25Q256 (0U)
#define W25Q128 (1U)
#define W25Q64  (2U)
#define W25Q32  (3U)


/**
  * @brief 使能等待回调函数
  *        等待回调函数为: void W25QXX_Wait_Callback(void);
  */
//#define ENABLEW25QXX_WAIT_CALLBACK


/**
  * @brief 使能调试
  *        如果使能调试则需要用户实现 W25QXX_DEBUG_PRINTF(p_format, ...) 接口, 比如使用 printf 函数实现
  */
#define ENABLE_W25QXX_DEBUG
#ifdef ENABLE_W25QXX_DEBUG
    #define W25QXX_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)
#endif /* ENABLE_W25QXX_DEBUG */


/**
  * @brief W25QXX 设备类型
  *        有如下类型: 
  *             W25Q256
  *             W25Q128
  *             W25Q64
  *             W25Q32
  */
#define W25QXX_TYPE             W25Q128 


/**
  * @brief W25QXX 相关的 ID 定义, 请查阅芯片数据手册填写     
  */
#define W25QXX_MF_ID           (0xEFU)
#define W25QXX_JEDEC_ID        (0x4019U)


/**
  * @brief 最大超时时间, 单位: CPU 时间    
  */
#define W25QXX_TIMEOUT          (0x000FFFFFU)

/* Export macro -------------------------------------------------------------*/

/* Export variables ---------------------------------------------------------*/

     
/* Export function prototypes -----------------------------------------------*/



/* Export functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_CONF_H */

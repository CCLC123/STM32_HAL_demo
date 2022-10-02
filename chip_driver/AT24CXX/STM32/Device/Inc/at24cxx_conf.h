/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __AT24CXX_CONF_H
#define __AT24CXX_CONF_H

#ifdef __cplusplus
extern "C" {
#endif




/* Includes -------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/
/**
  * @brief AT24CXX 芯片类型
  */
#define AT24C01  (0U)
#define AT24C02  (1U)
#define AT24C04  (2U)
#define AT24C08  (3U)
#define AT24C16  (4U)

/**
  * @brief 使能调试
  *        如果使能调试则需要用户实现 AT24CXX_DEBUG_PRINTF(p_format, ...) 接口, 比如使用 printf 函数实现
  */
#define ENABLE_AT24CXX_DEBUG
#ifdef ENABLE_AT24CXX_DEBUG
#define AT24CXX_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)
#endif /* ENABLE_AT24CXX_DEBUG */


/**
  * @brief AT24C 设备类型
  *   @arg   AT24C01
  *   @arg   AT24C02
  *   @arg   AT24C04
  *   @arg   AT24C08
  *   @arg   AT24C16
  */
#define AT24CXX_TYPE              AT24C02


/**
  * @brief AT24C 设备地址
  */
#define AT24CXX_DEVICE_ADDR     (0xA0U)


/**
  * @brief 最大超时时间, 单位: CPU 时间    
  */
#define AT24CXX_TIMEOUT          (0x000FFFFFU)


/* Exported macro -------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /* __AT24CXX_CONF_H */

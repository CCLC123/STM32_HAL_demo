/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __GPIO_I2C_CONF_H
#define __GPIO_I2C_CONF_H

#ifdef __cplusplus
extern "C" {
#endif




/* Includes -------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/
/**
  * @brief 使能调试
  *        如果使能调试则需要用户实现 I2C_DEBUG_PRINTF(p_format, ...) 接口, 比如使用 printf 函数实现
  */
#define ENABLE_I2C_DEBUG
#ifdef ENABLE_I2C_DEBUG
#define I2C_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)


/**
  * @brief 最大超时时间, 单位: CPU 时间    
  */
#define I2C_TIMEOUT          (0x000FFFFFU)

#endif /* ENABLE_I2C_DEBUG */

/* Exported macro -------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /* __GPIO_I2C_CONF_H */

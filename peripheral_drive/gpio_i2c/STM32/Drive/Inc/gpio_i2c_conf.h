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
  * @brief ʹ�ܵ���
  *        ���ʹ�ܵ�������Ҫ�û�ʵ�� I2C_DEBUG_PRINTF(p_format, ...) �ӿ�, ����ʹ�� printf ����ʵ��
  */
#define ENABLE_I2C_DEBUG
#ifdef ENABLE_I2C_DEBUG
#define I2C_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)


/**
  * @brief ���ʱʱ��, ��λ: CPU ʱ��    
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

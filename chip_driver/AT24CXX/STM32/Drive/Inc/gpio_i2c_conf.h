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
#define I2C_MSB     0
#define I2C_LSB     1


/**
  * @brief ʹ�ܵ���
  *        ���ʹ�ܵ�������Ҫ�û�ʵ�� I2C_DEBUG_PRINTF(p_format, ...) �ӿ�, ����ʹ�� printf ����ʵ��
  */
//#define ENABLE_I2C_DEBUG
#ifdef ENABLE_I2C_DEBUG
#define I2C_DEBUG_PRINTF(p_format, ...)           USART1_Printf(p_format, ## __VA_ARGS__)
#endif /* ENABLE_I2C_DEBUG */

/**
  * @brief λ����ģʽ
  *   @arg   I2C_MSB
  *   @arg   I2C_LSB
  */
#define I2C_BIT_FIRST           I2C_MSB

/**
  * @brief ����Դ���
  */
#define I2C_TIMEOUT          (0x00000008U)

/* Exported macro -------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /* __GPIO_I2C_CONF_H */

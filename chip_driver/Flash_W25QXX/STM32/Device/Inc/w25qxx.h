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
    EN_W25QXX_ERROR = 0,            /* ���� */
    EN_W25QXX_OK = 1,               /* �ɹ� */
    EN_W25QXX_TIMEOUT = 2,          /* ��ʱ */
    EN_W25QXX_PARAM_IS_NULL = 3,    /* ��������Ϊ�� */
    
    EN_W25QXX_FAIL = EN_W25QXX_ERROR,
    EN_W25QXX_SUCCESS = EN_W25QXX_OK
} en_w25qxx_status_t;

typedef enum
{
    EN_W25QXX_CLOSE_COM = 0,        /* ����(�ر�)ͨ�� */
    EN_W25QXX_CONTINUE_COM = 1,     /* ����ͨ�� */
} en_w25qxx_com_action_status_t;


/**
  * @brief   W25QXX ��ʼ��
  * @note    �ú���Ӧ��ʼ���� W25QXX ͨ�ŵĽӿ�, ���� SPI
  * @param   None
  * @return  en_w25qxx_status_t
  */
typedef en_w25qxx_status_t (*W25QXX_Init_Func)(void);


/**
  * @brief   �� W25QXX ���� p_send_buff �ͽ��� W25QXX �����ݵ� p_receive_buff
  * @note    �ú���ʵ���� W25QXX �������ݵ��շ�, ��ʹ���� W25QXX ͨ�ŵĽӿ�, ���� SPI
  * @param   p_send_buff: ���ͻ�������ʼ��ַ
  * @param   p_receive_buff: ���ջ�������ʼ��ַ
  * @param   length: ����/���յ����ݳ���, ��λ: �ֽ�
  * @param   e_com_action_status: ���η������֮���Ƿ�Ҫ����ͨ��, ����ͨ���� CS Ӧ���ֵ͵�ƽ
  *            @arg EN_W25QXX_CLOSE_COM: ����ͨ��
  *            @arg EN_W25QXX_CONTINUE_COM: ����ͨ��
  * @return  en_w25qxx_status_t
  */
typedef en_w25qxx_status_t (*W25QXX_Send_Receive_Func)(const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length, en_w25qxx_com_action_status_t e_com_action_status);


typedef struct
{
    W25QXX_Init_Func           m_p_Init;           /* ��ʼ������ָ�� */
    W25QXX_Send_Receive_Func   m_p_Send_Receive;   /* ��������պ���ָ�� */
} w25qxx_interface_func_t;


typedef struct
{
    uint64_t m_unique_ID;                          /* �豸ΨһID */
    w25qxx_interface_func_t m_interface_func;      /* �ӿں��� */
} w25qxx_obj_t;

/* Export define ------------------------------------------------------------*/
#define W25QXX_SECTOR_SIZE         (0x00001000U)   /* 4096 */  /* ������С, ��λ: �ֽ� */

/* Export macro -------------------------------------------------------------*/

/* Export variables ---------------------------------------------------------*/

/* Export function prototypes -----------------------------------------------*/
en_w25qxx_status_t W25QXX_Init(w25qxx_obj_t *p_obj, const w25qxx_interface_func_t *p_interface_func);

en_w25qxx_status_t W25QXX_Read_Unique_ID(w25qxx_obj_t *p_obj);
en_w25qxx_status_t W25QXX_Power_Down(w25qxx_obj_t *p_obj);
en_w25qxx_status_t W25QXX_Wakeup(w25qxx_obj_t *p_obj);
en_w25qxx_status_t W25QXX_Get_JEDEC_ID(w25qxx_obj_t *p_obj, uint8_t *p_MF_ID, uint16_t *p_ID);
en_w25qxx_status_t W25QXX_Read(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);
en_w25qxx_status_t W25QXX_Erase(w25qxx_obj_t *p_obj, uint32_t addr, uint32_t length);
en_w25qxx_status_t W25QXX_Write(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length);

en_w25qxx_status_t W25QXX_Test(w25qxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff);




#ifdef ENABLEW25QXX_WAIT_CALLBACK
void W25QXX_Wait_Callback(void);
#endif /* ENABLEW25QXX_WAIT_CALLBACK */


/* Export functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */

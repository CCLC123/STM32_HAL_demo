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
    EN_AT24CXX_ERROR = 0,            /* ���� */
    EN_AT24CXX_OK = 1,               /* �ɹ� */
    EN_AT24CXX_TIMEOUT = 2,          /* ��ʱ */
    EN_AT24CXX_PARAM_IS_NULL = 3,    /* ��������Ϊ�� */
    
    EN_AT24CXX_NACK = EN_AT24CXX_ERROR, /* ��Ӧ�� */
    EN_AT24CXX_ACK = EN_AT24CXX_OK      /* Ӧ�� */
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
  * @brief   AT24CXX ��ʼ��
  * @note    �ú���Ӧ��ʼ���� AT24CXX ͨ�ŵĽӿ�, �� I2C
  * @param   None
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Init_Func)(void);


/**
  * @brief   �� AT24CXX ���� p_send_buff �е�����
  * @note    �ú���ʵ���� AT24CXX �������ݵķ���, ��ʹ���� AT24CXX ͨ�ŵĽӿ�, �� I2C
  * @param   p_send_buff: ���ͻ�������ʼ��ַ
  * @param   length: ���͵����ݳ���, ��λ: �ֽ�
  * @param   e_start_status: ����ͨ��ǰ�Ƿ�����ʼ�ź�
  *            @arg EN_AT24CXX_GENETATE_START: ������ʼ�ź�
  *            @arg EN_AT24CXX_NOT_GENETATE_START: ��������ʼ�ź�
  * @param   e_stop_status: ����ͨ�ź��Ƿ���ֹͣ�ź�
  *            @arg EN_AT24CXX_GENETATE_STOP: ����ֹͣ�ź�
  *            @arg EN_AT24CXX_NOT_GENETATE_STOP: ������ֹͣ�ź�
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Send_Func)(const uint8_t *p_send_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);


/**
  * @brief   �� AT24CXX �������ݵ� p_receive_buff
  * @note    �ú���ʵ���� AT24CXX �������ݵĽ���, ��ʹ���� AT24CXX ͨ�ŵĽӿ�, �� I2C
  * @param   p_receive_buff: ���ջ�������ʼ��ַ
  * @param   length: ���յ����ݳ���, ��λ: �ֽ�
  * @param   e_start_status: ����ͨ��ǰ�Ƿ�����ʼ�ź�
  *            @arg EN_AT24CXX_GENETATE_START: ������ʼ�ź�
  *            @arg EN_AT24CXX_NOT_GENETATE_START: ��������ʼ�ź�
  * @param   e_stop_status: ����ͨ�ź��Ƿ���ֹͣ�ź�
  *            @arg EN_AT24CXX_GENETATE_STOP: ����ֹͣ�ź�
  *            @arg EN_AT24CXX_NOT_GENETATE_STOP: ������ֹͣ�ź�
  * @return  en_at24cxx_status_t
  */
typedef en_at24cxx_status_t (*AT24CXX_Receive_Func)(uint8_t *p_receive_buff, uint32_t length, en_at24cxx_start_t e_start_status, en_at24cxx_stop_t e_stop_status);


typedef struct
{
    AT24CXX_Init_Func           m_p_Init;           /* ��ʼ������ָ�� */
    AT24CXX_Send_Func           m_Send_Func;        /* �������ݺ���ָ�� */
    AT24CXX_Receive_Func        m_Receive_Func;     /* �������ݺ���ָ�� */
} at24cxx_interface_func_t;


typedef struct
{
    at24cxx_interface_func_t m_interface_func;      /* �ӿں��� */
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

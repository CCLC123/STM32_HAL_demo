/* Includes ------------------------------------------------------------------*/
#include "at24cxx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#if (AT24CXX_TYPE == AT24C01 || AT24CXX_TYPE == AT24C02)
    #define PAGE_SIZE               (8U)   /* ҳ��С, ��λ: �ֽ� */
#else
    #define PAGE_SIZE               (16U)   /* ҳ��С, ��λ: �ֽ� */
    #define SECTOR_SIZE             (256U)  /* ������С, ��λ: �ֽ� */
#endif /* AT24CXX_TYPE */


#ifndef ENABLEAT24CXX_WAIT_CALLBACK
    #define AT24CXX_Wait_Callback()
#endif /* ENABLEAT24CXX_WAIT_CALLBACK */

#ifndef ENABLE_AT24CXX_DEBUG
    #define AT24CXX_DEBUG_PRINTF(p_format, ...)
#endif /* ENABLE_AT24CXX_DEBUG */

#ifndef NULL
    #define NULL ((void*)0)
#endif /* NULL */

/* Private macro -------------------------------------------------------------*/
#define CONVERT_TO_STRING(value)    #value    /* �ַ����� */
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_INFO(str, ...)        "[INFO]%s()->" str "\r\n", __FUNCTION__, ## __VA_ARGS__
#define DEBUG_BEGIN()               "[+++]%s()->begin\r\n", __FUNCTION__
#define DEBUG_END()                 "[---]%s()->end\r\n", __FUNCTION__


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   ��ʼ�� AT24CXX, ��ʼ�� AT24CXX ʵ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   p_interface_func: at24cxx_interface_func_t �ĵ�ַ
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Init(at24cxx_obj_t *p_obj, const at24cxx_interface_func_t *p_interface_func)
{
    en_at24cxx_status_t e_status = EN_AT24CXX_ACK;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    
    /* ��麯������ */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    /* ��ʼ�������ӿ� */
    p_obj->m_interface_func = *p_interface_func;
    
    /* ִ�г�ʼ������ */
    if (p_obj->m_interface_func.m_p_Init == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    e_status = p_obj->m_interface_func.m_p_Init();
    if (e_status != EN_AT24CXX_OK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_ERROR));
        return EN_AT24CXX_ERROR;
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return e_status;
}


/**
  * @brief   �ȴ� AT24CXX ׼����
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Wait_For_Ready(at24cxx_obj_t *p_obj)
{
    uint32_t timeout = 0;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    while (p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_GENETATE_STOP) != EN_AT24CXX_ACK)
    {
        timeout++;
        if (timeout >= AT24CXX_TIMEOUT)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_TIMEOUT));
            return EN_AT24CXX_TIMEOUT;
        }
        #ifdef ENABLEAT24CXX_WAIT_CALLBACK
        AT24CXX_Wait_Callback();
        #endif /* ENABLEAT24CXX_WAIT_CALLBACK */
    }
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("ready ok"));
    
    return EN_AT24CXX_ACK;
}


#if (AT24CXX_TYPE == AT24C01 || AT24CXX_TYPE == AT24C02)
/**
  * @brief   �� AT24CXX ��ȡ���ݵ� p_receive_buff ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   addr: ��ȡ�� AT24CXX ��ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: ��ȡ�ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Read(at24cxx_obj_t *p_obj, uint32_t addr, uint8_t *p_receive_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, addr = %02X, length = %u", device_addr, addr, length));
    
    /* ��麯������ */
    if (p_obj == NULL || p_receive_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    /* �ȴ��豸׼���� */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ���Ͷ�д��ַ */
    status = p_obj->m_interface_func.m_Send_Func((uint8_t *)&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    device_addr++;
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ��ȡ���� */
    status = p_obj->m_interface_func.m_Receive_Func(p_receive_buff, length, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}


/**
  * @brief   ��ҳд AT24CXX
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   addr: д��� AT24CXX ��ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Page_Write(at24cxx_obj_t *p_obj, uint8_t addr, const uint8_t *p_send_buff, uint8_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, addr = %02X, length = %u", device_addr, addr, length));
    
    /* �ȴ��豸׼���� */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ���Ͷ�д��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* д������ */
    status = p_obj->m_interface_func.m_Send_Func(p_send_buff, length, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}


/**
  * @brief   д p_send_buff �е����ݵ� AT24CXX ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   addr: д��� AT24CXX ��ַ, ���밴��ҳ��ַ����
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Write(at24cxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    /* ��ַ��������ַ���� */
    addr &= (~(PAGE_SIZE - 1));
    
    /* ����Ҫд����ٸ�ҳ */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("page_num: %u, page_mode: %u", page_num, page_mode));
    
    /* ��ҳд */
    for (uint32_t i = 0; i < page_num; i++)
    {
        status = AT24CXX_Page_Write(p_obj, addr, p_send_buff, PAGE_SIZE);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        addr += PAGE_SIZE;
        p_send_buff += PAGE_SIZE;
    }
    
    /* ����һҳд */
    if (page_mode)
    {
        status = AT24CXX_Page_Write(p_obj, addr, p_send_buff, page_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}

#else

/**
  * @brief   ��������ʽ�ķ�ʽ�� AT24CXX ��ȡ���ݵ� p_receive_buff ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   sector_count: �������
  * @param   addr: ��ȡ�� AT24CXX ��ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: ��ȡ�ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Read_Sector(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, uint8_t *p_receive_buff, uint16_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    device_addr |= (sector_count << 1U);
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, sector_num = %u, addr = %02X, length = %u", device_addr, sector_count, addr, length));
    
    /* �ȴ��豸׼���� */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ���Ͷ�д��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    device_addr++;
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ��ȡ���� */
    status = p_obj->m_interface_func.m_Receive_Func(p_receive_buff, length, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}


/**
  * @brief   �� AT24CXX ��ȡ���ݵ� p_receive_buff ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   addr: ��ȡ�� AT24CXX ��ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: ��ȡ�ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Read(at24cxx_obj_t *p_obj, uint32_t addr, uint8_t *p_receive_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint16_t sector_num = 0;
    uint8_t sector_mode = 0;
    uint8_t sector_count = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* ��麯������ */
    if (p_obj == NULL || p_receive_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    sector_num = length / SECTOR_SIZE;
    sector_mode = length % SECTOR_SIZE;
    sector_count = addr / SECTOR_SIZE;
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("sector_num: %u, sector_mode: %u, sector_count", sector_num, sector_mode, sector_count));
    
    /* ��ȡ�ĵ�ַ����������ַδ���� */
    if ((addr % SECTOR_SIZE) != 0)
    {
        /* �ȶ����ݵ���������ĵ�ַ�� */
        status = AT24CXX_Read_Sector(p_obj, sector_count, sector_mode, p_receive_buff, SECTOR_SIZE - sector_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        addr += (SECTOR_SIZE - sector_mode);
        p_receive_buff += (SECTOR_SIZE - sector_mode);
        sector_count++;
    }
    /* ���������� */
    for (; sector_count < sector_num; sector_count++)
    {
        status = AT24CXX_Read_Sector(p_obj, sector_count, 0, p_receive_buff, SECTOR_SIZE);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        p_receive_buff += SECTOR_SIZE;
    }
    /* ����һ�������� */
    if (sector_mode)
    {
        status = AT24CXX_Read_Sector(p_obj, sector_num, 0, p_receive_buff, sector_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}


/**
  * @brief   ��ҳд AT24CXX
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   sector_count: �������
  * @param   addr: д��� AT24CXX ��ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Page_Write(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, const uint8_t *p_send_buff, uint8_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    device_addr |= (sector_count << 1U);
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, sector_num = %u, addr = %02X, length = %u", device_addr, sector_count, addr, length));
    
    /* �ȴ��豸׼���� */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* �����豸��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* ���Ͷ�д��ַ */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* д������ */
    status = p_obj->m_interface_func.m_Send_Func(p_send_buff, length, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}


/**
  * @brief   �������ķ�ʽд p_send_buff �е����ݵ� AT24CXX ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   sector_count: �������
  * @param   addr: д��� AT24CXX ��ַ, ���밴��ҳ��ַ����
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Write_Sector(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, const uint8_t *p_send_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* ��ַ��ҳ��ַ���� */
    addr &= (~(PAGE_SIZE - 1));
    /* ������Ҫд����ٸ�ҳ */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("page_num: %u, page_mode: %u", page_num, page_mode));
    
    /* ��ҳд */
    for (uint32_t i = 0; i < page_num; i++)
    {
        status = AT24CXX_Page_Write(p_obj, sector_count, addr, p_send_buff, PAGE_SIZE);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        addr += PAGE_SIZE;
        p_send_buff += PAGE_SIZE;
    }
    
    /* ����һҳд */
    if (page_mode)
    {
        status = AT24CXX_Page_Write(p_obj, sector_count, addr, p_send_buff, page_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return EN_AT24CXX_ACK;
}


/**
  * @brief   д p_send_buff �е����ݵ� AT24CXX ��
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   addr: д��� AT24CXX ��ַ, ���밴��ҳ��ַ����
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Write(at24cxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint16_t sector_num = 0;
    uint8_t sector_mode = 0;
    uint8_t sector_count = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    sector_num = length / SECTOR_SIZE;
    sector_mode = length % SECTOR_SIZE;
    sector_count = addr / SECTOR_SIZE;
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("sector_num: %u, sector_mode: %u, sector_count", sector_num, sector_mode, sector_count));
    
    if ((addr % SECTOR_SIZE) != 0)
    {
        /* ��д���ݵ���������ĵ�ַ�� */
        status = AT24CXX_Write_Sector(p_obj, sector_count, addr, p_send_buff, SECTOR_SIZE - sector_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        addr += (SECTOR_SIZE - sector_mode);
        p_send_buff += (SECTOR_SIZE - sector_mode);
        sector_count++;
    }
    /* ��������д */
    for (; sector_count < sector_num; sector_count++)
    {
        status = AT24CXX_Write_Sector(p_obj, sector_count, 0, p_send_buff, SECTOR_SIZE);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
        p_send_buff += SECTOR_SIZE;
    }
    /* ����һ������д */
    if (sector_mode)
    {
        status = AT24CXX_Write_Sector(p_obj, sector_num, 0, p_send_buff, sector_mode);
        if (status != EN_AT24CXX_ACK)
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
            return status;
        }
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    
    return status;
}
#endif /* AT24CXX_TYPE */


/**
  * @brief   ���� AT24CXX
  * @note    �뱣֤ p_send_buff �� p_receive_buff ����Ϊһ������(256B)��С
  * @param   p_obj: at24cxx_obj_t �ĵ�ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Test(at24cxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff)
{
    en_at24cxx_status_t e_status = EN_AT24CXX_ACK;
    uint32_t i = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    #ifndef SECTOR_SIZE
#define SECTOR_SIZE 256U
    #endif
    
    for (i = 0; i < SECTOR_SIZE; i++)
    {
        p_send_buff[i] = 0xAA;
        p_receive_buff[i] = 0;
    }
    
    e_status = AT24CXX_Write(p_obj, 0, p_send_buff, SECTOR_SIZE);
    if (e_status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    e_status = AT24CXX_Read(p_obj, 0, p_receive_buff, SECTOR_SIZE);
    if (e_status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    for (i = 0; i < SECTOR_SIZE; i++)
    {
        if (p_send_buff[i] != p_receive_buff[i])
        {
            AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_ERROR));
            return EN_AT24CXX_ERROR;
        }
    }
    
    AT24CXX_DEBUG_PRINTF(DEBUG_END());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("test ok!"));
    
    return EN_AT24CXX_ACK;
}


#ifdef ENABLEAT24CXX_WAIT_CALLBACK

__WEAK void AT24CXX_Wait_Callback(void)
{

}

#endif /* ENABLEAT24CXX_WAIT_CALLBACK */

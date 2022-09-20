/* Includes ------------------------------------------------------------------*/
#include "w25qxx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PAGE_SIZE               (0x0100U)   /* 256 */        /* ҳ��С, ��λ: �ֽ� */


/* W25QXX �����б� */
#define READ_STATUS1_REG        (0x05U)     /* ��״̬�Ĵ���1 */
#define WRITE_ENABLE            (0x06U)     /* дʹ�� */
//#define ENTER_4BYTE_ADDR_MODE   (0xB7U)   /* ����4�ֽڵ�ַģʽ */
#define READ_STATUS3_REG        (0x15U)     /* ��״̬�Ĵ���3 */
#define ENTER_POWER_DOWN        (0xB9U)     /* �������ģʽ */
#define RELEASE_POWER_DOWN      (0xABU)     /* ���� */
#define GET_DEVICE_ID           (0xABU)     /* ��ȡ�豸ID */
#define GET_JEDEC_ID            (0x9FU)     /* ��ȡ JEDEC ID */
#define READ_UNIQUE_ID          (0x4BU)     /* ��ȡ�豸ΨһID */

#if (W25QXX_TYPE == W25Q256)
    #define READ_COMMAND            (0x13U)     /* 4�ֽڵ�ַģʽ�� */
    #define PAGE_WRITE_COMMAND      (0x12U)     /* 4�ֽڵ�ַģʽ��ҳд���� */
    #define ERASE_SECTOR            (0x21U)     /* 4�ֽڵ�ַģʽ�������� */
    
#else
    #define READ_COMMAND            (0x03U)     /* 4�ֽڵ�ַģʽ�� */
    #define PAGE_WRITE_COMMAND      (0x02U)     /* 4�ֽڵ�ַģʽ��ҳд���� */
    #define ERASE_SECTOR            (0x20U)     /* 4�ֽڵ�ַģʽ�������� */
#endif /* W25QXX_TYPE */

/* �Ĵ������� */
#define STATUS1_BUSY            (0x01U)
#define STATUS1_WEL             (0x02U)


//#define STATUS3_ADS             (0x01U)


#ifndef ENABLEW25QXX_WAIT_CALLBACK
    #define W25QXX_Wait_Callback()
#endif /* ENABLEW25QXX_WAIT_CALLBACK */

#ifndef ENABLE_W25QXX_DEBUG
    #define W25QXX_DEBUG_PRINTF(p_format, ...)
#endif /* ENABLE_W25QXX_DEBUG */


/* Private macro -------------------------------------------------------------*/
#define CONVERT_TO_STRING(value)    #value    /* �ַ����� */
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_BEGIN(str)            "[+++]%s()->%s begin\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_END(str)              "[---]%s()->%s end\r\n", __FUNCTION__, CONVERT_TO_STRING(str)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   ��ʼ�� W25QXX, ��ʼ�� W25QXX ʵ��
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   p_interface_func: w25qxx_interface_func_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Init(w25qxx_obj_t *p_obj, const w25qxx_interface_func_t *p_interface_func)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t MF_ID = 0;
    uint16_t JEDEC_ID = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(init));
    
    /* ��麯������ */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* ��ʼ�������ӿ� */
    p_obj->m_interface_func.m_p_Init = p_interface_func->m_p_Init;
    p_obj->m_interface_func.m_p_Send_Receive = p_interface_func->m_p_Send_Receive;
    
    /* ִ�г�ʼ������ */
    if (p_obj->m_interface_func.m_p_Init == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    status = p_obj->m_interface_func.m_p_Init();
    
    /* �����豸 */
    status = W25QXX_Wakeup(p_obj);
    
    /* ����豸�� MF_ID */
    status = Get_W25QXX_JEDEC_ID(p_obj, &MF_ID, &JEDEC_ID);
    if (MF_ID != W25QXX_MF_ID)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    /* ��ȡ�豸ΨһID */
    status = Read_W25QXX_Unique_ID(p_obj);
    
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(init));
    
    return status;
}


/**
  * @brief   ��ȡ W25QXX ��ΨһID
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Read_W25QXX_Unique_ID(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[13] = {0};
    uint8_t arr_receive_buff[13] = {0};
    
    arr_send_buff[0] = READ_UNIQUE_ID;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    
    /* ��ȡΨһID */
    p_obj->m_unique_ID =
        ((((uint64_t)arr_receive_buff[5]) << 56)  & ((uint64_t)0xFF00000000000000)) |
        ((((uint64_t)arr_receive_buff[6]) << 48)  & ((uint64_t)0x00FF000000000000)) |
        ((((uint64_t)arr_receive_buff[7]) << 40)  & ((uint64_t)0x0000FF0000000000)) |
        ((((uint64_t)arr_receive_buff[8]) << 32)  & ((uint64_t)0x000000FF00000000)) |
        ((((uint64_t)arr_receive_buff[9]) << 24)  & ((uint64_t)0x00000000FF000000)) |
        ((((uint64_t)arr_receive_buff[10]) << 16) & ((uint64_t)0x0000000000FF0000)) |
        ((((uint64_t)arr_receive_buff[11]) << 8)  & ((uint64_t)0x000000000000FF00)) |
        (((uint64_t)arr_receive_buff[12])         & ((uint64_t)0x00000000000000FF));
        
    W25QXX_DEBUG_PRINTF("%s()->Unique ID: 0x%llX\r\n", __FUNCTION__, p_obj->m_unique_ID);
    
    return EN_W25QXX_OK;
}


/**
  * @brief   �ȴ� W25QXX ׼����
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t Wait_for_W25QXX_Ready(w25qxx_obj_t *p_obj)
{
    uint32_t timeout = 0;
    uint8_t arr_send_buff[2] = {0};
    uint8_t arr_receive_buff[2] = {0};
    
    arr_send_buff[0] = READ_STATUS1_REG;
    do
    {
        p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
        timeout++;
        if (timeout >= W25QXX_TIMEOUT)
        {
            W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_TIMEOUT));
            return EN_W25QXX_TIMEOUT;
        }
        W25QXX_Wait_Callback();
    } while ((arr_receive_buff[1] & STATUS1_BUSY) != 0);
    
    W25QXX_DEBUG_PRINTF("%s()->ready ok\r\n", __FUNCTION__);
    
    return EN_W25QXX_OK;
}


/**
  * @brief   W25QXX дʹ��
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t W25QXX_Write_Enable(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[2] = {0};
    uint8_t arr_receive_buff[2] = {0};
    
    arr_send_buff[0] = WRITE_ENABLE;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, 1, EN_W25QXX_CLOSE_COM);
    
    arr_send_buff[0] = READ_STATUS1_REG;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    if ((arr_receive_buff[1] & STATUS1_WEL) != 0)
    {
        W25QXX_DEBUG_PRINTF("%s()->write enable ok\r\n", __FUNCTION__);
        
        return EN_W25QXX_OK;
    }
    else
    {
        W25QXX_DEBUG_PRINTF("%s()->write enable fail\r\n", __FUNCTION__);
        
        return EN_W25QXX_ERROR;
    }
}


/**
  * @brief   W25QXX �������ģʽ
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Power_Down(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[1] = {0};
    uint8_t arr_receive_buff[1] = {0};
    
    /* ��麯������ */
    if (p_obj == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    arr_send_buff[0] = ENTER_POWER_DOWN;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF("%s()->power down ok\r\n", __FUNCTION__);
    
    return EN_W25QXX_OK;
}


/**
  * @brief   ���� W25QXX
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Wakeup(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[1] = {0};
    uint8_t arr_receive_buff[1] = {0};
    
    /* ��麯������ */
    if (p_obj == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    arr_send_buff[0] = RELEASE_POWER_DOWN;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF("%s()->wakeup ok\r\n", __FUNCTION__);
    
    return EN_W25QXX_OK;
}


/**
  * @brief   ��ȡ W25QXX �� JEDEC_ID
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   p_MF_ID: �洢 MF_ID �����ĵ�ַ
  * @param   p_ID: �洢 ID �����ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Get_W25QXX_JEDEC_ID(w25qxx_obj_t *p_obj, uint8_t *p_MF_ID, uint16_t *p_ID)
{
    uint8_t arr_send_buff[4] = {0};
    uint8_t arr_receive_buff[4] = {0};
    
    /* ��麯������ */
    if (p_obj == NULL || p_MF_ID == NULL || p_ID == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    arr_send_buff[0] = GET_JEDEC_ID;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    
    *p_MF_ID = arr_receive_buff[1];
    *p_ID = (((uint16_t)(arr_receive_buff[2])) << 8) | ((uint16_t)arr_receive_buff[3]);
    
    W25QXX_DEBUG_PRINTF("%s()->MF ID: 0x%X, ID: 0x%X\r\n", __FUNCTION__, *p_MF_ID, *p_ID);
    
    return EN_W25QXX_OK;
}


/**
  * @brief   �� W25QXX ��ȡ���ݵ� p_receive_buff ��
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   addr: ��ȡ�� W25QXX ��ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: ��ȡ�ĳ���, ��λ: �ֽ�
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Read_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length)
{
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(read));
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* �ȴ��豸׼���� */
    Wait_for_W25QXX_Ready(p_obj);
    
    /* �����������ַ */
    arr_send_buff[0] = READ_COMMAND;
    #if (W25QXX_TYPE == W25Q256)
    arr_send_buff[1] = (addr & 0xFF000000U) >> 24;
    arr_send_buff[2] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[3] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[4] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CONTINUE_COM);
    #else
    arr_send_buff[1] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[2] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[3] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff) - 1, EN_W25QXX_CONTINUE_COM);
    #endif /* W25QXX_TYPE */
    
    
    /* �������� */
    p_obj->m_interface_func.m_p_Send_Receive(p_send_buff, p_receive_buff, length, EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, length: %u\r\n", __FUNCTION__, addr, length);
    W25QXX_DEBUG_PRINTF(DEBUG_END(read));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   ���� W25QXX ������
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   addr: ������ַ
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t W25QXX_Erase(w25qxx_obj_t *p_obj, uint32_t addr)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(erase));
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X\r\n", __FUNCTION__, addr);
    
    /* �ȴ��豸׼���� */
    status = Wait_for_W25QXX_Ready(p_obj);
    
    /* дʹ�� */
    status = W25QXX_Write_Enable(p_obj);
    
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    
    /* �����������ַ */
    arr_send_buff[0] = ERASE_SECTOR;
    #if (W25QXX_TYPE == W25Q256)
    arr_send_buff[1] = (addr & 0xFF000000U) >> 24;
    arr_send_buff[2] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[3] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[4] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    #else
    arr_send_buff[1] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[2] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[3] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff) - 1, EN_W25QXX_CLOSE_COM);
    #endif /* W25QXX_TYPE */
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(erase));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   ��ҳд W25QXX
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   addr: д��� W25QXX ��ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t Page_Write_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(page write));
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, length: %u\r\n", __FUNCTION__, addr, length);
    
    /* �ȴ��豸׼���� */
    status = Wait_for_W25QXX_Ready(p_obj);
    
    /* дʹ�� */
    status = W25QXX_Write_Enable(p_obj);
    
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    
    /* �����������ַ */
    arr_send_buff[0] = PAGE_WRITE_COMMAND;
    #if (W25QXX_TYPE == W25Q256)
    arr_send_buff[1] = (addr & 0xFF000000U) >> 24;
    arr_send_buff[2] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[3] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[4] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CONTINUE_COM);
    #else
    arr_send_buff[1] = (addr & 0x00FF0000U) >> 16;
    arr_send_buff[2] = (addr & 0x0000FF00U) >> 8;
    arr_send_buff[3] = (addr & 0x000000FFU);
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff) - 1, EN_W25QXX_CONTINUE_COM);
    #endif /* W25QXX_TYPE */
    
    /* �������� */
    p_obj->m_interface_func.m_p_Send_Receive(p_send_buff, p_receive_buff, length, EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(page write));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   д p_send_buff �е����ݵ� W25QXX ��
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   addr: д��� W25QXX ��ַ, ��ַ��Ҫ����������ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @param   length: д��ĳ���, ��λ: �ֽ�
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Write_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length)
{
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(write));
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* ��ַ��������ַ���� */
    addr &= (~(W25QXX_SECTOR_SIZE - 1));
    
    /* ����Ҫд����ٸ�ҳ */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, page_num: %u, page_mode: %u\r\n", __FUNCTION__, addr, page_num, page_mode);
    
    /* ��ҳд */
    for (uint32_t i = 0; i < page_num; i++)
    {
        /* �жϵ�ǰ��ַ�ǲ���������ַ, ��������ַ���Ȳ�����д������ */
        if ((addr & (W25QXX_SECTOR_SIZE - 1)) == 0)
        {
            W25QXX_Erase(p_obj, addr);
        }
        Page_Write_W25QXX(p_obj, addr, p_send_buff, p_receive_buff, PAGE_SIZE);
        addr += PAGE_SIZE;
        p_send_buff += PAGE_SIZE;
        p_receive_buff += PAGE_SIZE;
    }
    
    /* ����һҳд */
    if (page_mode)
    {
        /* �жϵ�ǰ��ַ�ǲ���������ַ, ��������ַ���Ȳ�����д������ */
        if ((addr & (W25QXX_SECTOR_SIZE - 1)) == 0)
        {
            W25QXX_Erase(p_obj, addr);
        }
        Page_Write_W25QXX(p_obj, addr, p_send_buff, p_receive_buff, page_mode);
    }
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(write));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   ���� W25QXX
  * @note    �뱣֤ p_send_buff �� p_receive_buff ����Ϊһ��������С
  * @param   p_obj: w25qxx_obj_t �ĵ�ַ
  * @param   p_send_buff: ���ͻ������ĵ�ַ
  * @param   p_receive_buff: ���ջ������ĵ�ַ
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Test_W25QXX(w25qxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint32_t i = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(test));
    
    /* ��麯������ */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    for (i = 0; i < W25QXX_SECTOR_SIZE; i++)
    {
        p_send_buff[i] = 0xAA;
        p_receive_buff[i] = 0;
    }
    
    status = Write_W25QXX(p_obj, 0, p_send_buff, p_receive_buff, W25QXX_SECTOR_SIZE);
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    status = Read_W25QXX(p_obj, 0, p_send_buff, p_receive_buff, W25QXX_SECTOR_SIZE);
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    for (i = 0; i < W25QXX_SECTOR_SIZE; i++)
    {
        if (p_send_buff[i] != p_receive_buff[i])
        {
            W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
            return EN_W25QXX_ERROR;
        }
    }
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(test));
    
    return EN_W25QXX_OK;
}


#ifdef ENABLEW25QXX_WAIT_CALLBACK

__WEAK void W25QXX_Wait_Callback(void)
{

}

#endif /* ENABLEW25QXX_WAIT_CALLBACK */

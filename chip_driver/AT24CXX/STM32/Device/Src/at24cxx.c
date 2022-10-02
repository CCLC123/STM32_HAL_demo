/* Includes ------------------------------------------------------------------*/
#include "at24cxx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#if (AT24CXX_TYPE == AT24C01 || AT24CXX_TYPE == AT24C02)
    #define PAGE_SIZE               (8U)   /* 页大小, 单位: 字节 */
#else
    #define PAGE_SIZE               (16U)   /* 页大小, 单位: 字节 */
    #define SECTOR_SIZE             (256U)  /* 扇区大小, 单位: 字节 */
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
#define CONVERT_TO_STRING(value)    #value    /* 字符串化 */
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_INFO(str, ...)        "[INFO]%s()->" str "\r\n", __FUNCTION__, ## __VA_ARGS__
#define DEBUG_BEGIN()               "[+++]%s()->begin\r\n", __FUNCTION__
#define DEBUG_END()                 "[---]%s()->end\r\n", __FUNCTION__


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   初始化 AT24CXX, 初始化 AT24CXX 实例
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   p_interface_func: at24cxx_interface_func_t 的地址
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Init(at24cxx_obj_t *p_obj, const at24cxx_interface_func_t *p_interface_func)
{
    en_at24cxx_status_t e_status = EN_AT24CXX_ACK;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    /* 初始化函数接口 */
    p_obj->m_interface_func = *p_interface_func;
    
    /* 执行初始化函数 */
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
  * @brief   等待 AT24CXX 准备好
  * @param   p_obj: at24cxx_obj_t 的地址
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
  * @brief   从 AT24CXX 读取数据到 p_receive_buff 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   addr: 读取的 AT24CXX 地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 读取的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Read(at24cxx_obj_t *p_obj, uint32_t addr, uint8_t *p_receive_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, addr = %02X, length = %u", device_addr, addr, length));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_receive_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    /* 等待设备准备好 */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送读写地址 */
    status = p_obj->m_interface_func.m_Send_Func((uint8_t *)&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    device_addr++;
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 读取数据 */
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
  * @brief   按页写 AT24CXX
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   addr: 写入的 AT24CXX 地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Page_Write(at24cxx_obj_t *p_obj, uint8_t addr, const uint8_t *p_send_buff, uint8_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, addr = %02X, length = %u", device_addr, addr, length));
    
    /* 等待设备准备好 */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送读写地址 */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 写入数据 */
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
  * @brief   写 p_send_buff 中的数据到 AT24CXX 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   addr: 写入的 AT24CXX 地址, 必须按照页地址对齐
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Write(at24cxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_send_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    /* 地址按扇区地址对齐 */
    addr &= (~(PAGE_SIZE - 1));
    
    /* 计算要写入多少个页 */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("page_num: %u, page_mode: %u", page_num, page_mode));
    
    /* 整页写 */
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
    
    /* 不足一页写 */
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
  * @brief   以扇区方式的方式从 AT24CXX 读取数据到 p_receive_buff 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   sector_count: 扇区编号
  * @param   addr: 读取的 AT24CXX 地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 读取的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Read_Sector(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, uint8_t *p_receive_buff, uint16_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    device_addr |= (sector_count << 1U);
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, sector_num = %u, addr = %02X, length = %u", device_addr, sector_count, addr, length));
    
    /* 等待设备准备好 */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送读写地址 */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    device_addr++;
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 读取数据 */
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
  * @brief   从 AT24CXX 读取数据到 p_receive_buff 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   addr: 读取的 AT24CXX 地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 读取的长度, 单位: 字节
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
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_receive_buff == NULL)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_PARAM_IS_NULL));
        return EN_AT24CXX_PARAM_IS_NULL;
    }
    
    sector_num = length / SECTOR_SIZE;
    sector_mode = length % SECTOR_SIZE;
    sector_count = addr / SECTOR_SIZE;
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("sector_num: %u, sector_mode: %u, sector_count", sector_num, sector_mode, sector_count));
    
    /* 读取的地址按照扇区地址未对齐 */
    if ((addr % SECTOR_SIZE) != 0)
    {
        /* 先读数据到扇区对齐的地址上 */
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
    /* 整个扇区读 */
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
    /* 不足一个扇区读 */
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
  * @brief   按页写 AT24CXX
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   sector_count: 扇区编号
  * @param   addr: 写入的 AT24CXX 地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Page_Write(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, const uint8_t *p_send_buff, uint8_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint8_t device_addr = AT24CXX_DEVICE_ADDR;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    device_addr |= (sector_count << 1U);
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("device_addr = %02X, sector_num = %u, addr = %02X, length = %u", device_addr, sector_count, addr, length));
    
    /* 等待设备准备好 */
    status = AT24CXX_Wait_For_Ready(p_obj);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送设备地址 */
    status = p_obj->m_interface_func.m_Send_Func(&device_addr, 1, EN_AT24CXX_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 发送读写地址 */
    status = p_obj->m_interface_func.m_Send_Func(&addr, 1, EN_AT24CXX_NOT_GENETATE_START, EN_AT24CXX_NOT_GENETATE_STOP);
    if (status != EN_AT24CXX_ACK)
    {
        AT24CXX_DEBUG_PRINTF(DEBUG_ERROR(EN_AT24CXX_NACK));
        return status;
    }
    /* 写入数据 */
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
  * @brief   以扇区的方式写 p_send_buff 中的数据到 AT24CXX 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   sector_count: 扇区编号
  * @param   addr: 写入的 AT24CXX 地址, 必须按照页地址对齐
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_at24cxx_status_t
  */
static en_at24cxx_status_t AT24CXX_Write_Sector(at24cxx_obj_t *p_obj, uint8_t sector_count, uint8_t addr, const uint8_t *p_send_buff, uint32_t length)
{
    en_at24cxx_status_t status = EN_AT24CXX_ACK;
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("addr = %02X, length = %u", addr, length));
    
    /* 地址按页地址对齐 */
    addr &= (~(PAGE_SIZE - 1));
    /* 计算需要写入多少个页 */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    AT24CXX_DEBUG_PRINTF(DEBUG_INFO("page_num: %u, page_mode: %u", page_num, page_mode));
    
    /* 整页写 */
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
    
    /* 不足一页写 */
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
  * @brief   写 p_send_buff 中的数据到 AT24CXX 中
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   addr: 写入的 AT24CXX 地址, 必须按照页地址对齐
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
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
    
    /* 检查函数参数 */
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
        /* 先写数据到扇区对齐的地址上 */
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
    /* 整个扇区写 */
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
    /* 不足一个扇区写 */
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
  * @brief   测试 AT24CXX
  * @note    请保证 p_send_buff 和 p_receive_buff 至少为一个扇区(256B)大小
  * @param   p_obj: at24cxx_obj_t 的地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @return  en_at24cxx_status_t
  */
en_at24cxx_status_t AT24CXX_Test(at24cxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff)
{
    en_at24cxx_status_t e_status = EN_AT24CXX_ACK;
    uint32_t i = 0;
    
    AT24CXX_DEBUG_PRINTF(DEBUG_BEGIN());
    
    /* 检查函数参数 */
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

/* Includes ------------------------------------------------------------------*/
#include "w25qxx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PAGE_SIZE               (0x0100U)   /* 256 */        /* 页大小, 单位: 字节 */


/* W25QXX 命令列表 */
#define READ_STATUS1_REG        (0x05U)     /* 读状态寄存器1 */
#define WRITE_ENABLE            (0x06U)     /* 写使能 */
//#define ENTER_4BYTE_ADDR_MODE   (0xB7U)   /* 进入4字节地址模式 */
#define READ_STATUS3_REG        (0x15U)     /* 读状态寄存器3 */
#define ENTER_POWER_DOWN        (0xB9U)     /* 进入掉电模式 */
#define RELEASE_POWER_DOWN      (0xABU)     /* 唤醒 */
#define GET_DEVICE_ID           (0xABU)     /* 获取设备ID */
#define GET_JEDEC_ID            (0x9FU)     /* 获取 JEDEC ID */
#define READ_UNIQUE_ID          (0x4BU)     /* 获取设备唯一ID */

#if (W25QXX_TYPE == W25Q256)
    #define READ_COMMAND            (0x13U)     /* 4字节地址模式读 */
    #define PAGE_WRITE_COMMAND      (0x12U)     /* 4字节地址模式按页写命令 */
    #define ERASE_SECTOR            (0x21U)     /* 4字节地址模式擦除扇区 */
    
#else
    #define READ_COMMAND            (0x03U)     /* 4字节地址模式读 */
    #define PAGE_WRITE_COMMAND      (0x02U)     /* 4字节地址模式按页写命令 */
    #define ERASE_SECTOR            (0x20U)     /* 4字节地址模式擦除扇区 */
#endif /* W25QXX_TYPE */

/* 寄存器掩码 */
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
#define CONVERT_TO_STRING(value)    #value    /* 字符串化 */
#define DEBUG_ERROR(error)          "[ERROR]%s()->[%u]: %s\r\n", __FILE__, __LINE__, CONVERT_TO_STRING(error)
#define DEBUG_BEGIN(str)            "[+++]%s()->%s begin\r\n", __FUNCTION__, CONVERT_TO_STRING(str)
#define DEBUG_END(str)              "[---]%s()->%s end\r\n", __FUNCTION__, CONVERT_TO_STRING(str)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   初始化 W25QXX, 初始化 W25QXX 实例
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   p_interface_func: w25qxx_interface_func_t 的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Init(w25qxx_obj_t *p_obj, const w25qxx_interface_func_t *p_interface_func)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t MF_ID = 0;
    uint16_t JEDEC_ID = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(init));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_interface_func == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* 初始化函数接口 */
    p_obj->m_interface_func.m_p_Init = p_interface_func->m_p_Init;
    p_obj->m_interface_func.m_p_Send_Receive = p_interface_func->m_p_Send_Receive;
    
    /* 执行初始化函数 */
    if (p_obj->m_interface_func.m_p_Init == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    status = p_obj->m_interface_func.m_p_Init();
    
    /* 唤醒设备 */
    status = W25QXX_Wakeup(p_obj);
    
    /* 检查设备的 MF_ID */
    status = Get_W25QXX_JEDEC_ID(p_obj, &MF_ID, &JEDEC_ID);
    if (MF_ID != W25QXX_MF_ID)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    /* 获取设备唯一ID */
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
  * @brief   获取 W25QXX 的唯一ID
  * @param   p_obj: w25qxx_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Read_W25QXX_Unique_ID(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[13] = {0};
    uint8_t arr_receive_buff[13] = {0};
    
    arr_send_buff[0] = READ_UNIQUE_ID;
    p_obj->m_interface_func.m_p_Send_Receive(arr_send_buff, arr_receive_buff, sizeof(arr_send_buff), EN_W25QXX_CLOSE_COM);
    
    /* 获取唯一ID */
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
  * @brief   等待 W25QXX 准备好
  * @param   p_obj: w25qxx_obj_t 的地址
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
  * @brief   W25QXX 写使能
  * @param   p_obj: w25qxx_obj_t 的地址
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
  * @brief   W25QXX 进入掉电模式
  * @param   p_obj: w25qxx_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Power_Down(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[1] = {0};
    uint8_t arr_receive_buff[1] = {0};
    
    /* 检查函数参数 */
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
  * @brief   唤醒 W25QXX
  * @param   p_obj: w25qxx_obj_t 的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Wakeup(w25qxx_obj_t *p_obj)
{
    uint8_t arr_send_buff[1] = {0};
    uint8_t arr_receive_buff[1] = {0};
    
    /* 检查函数参数 */
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
  * @brief   获取 W25QXX 的 JEDEC_ID
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   p_MF_ID: 存储 MF_ID 变量的地址
  * @param   p_ID: 存储 ID 变量的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Get_W25QXX_JEDEC_ID(w25qxx_obj_t *p_obj, uint8_t *p_MF_ID, uint16_t *p_ID)
{
    uint8_t arr_send_buff[4] = {0};
    uint8_t arr_receive_buff[4] = {0};
    
    /* 检查函数参数 */
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
  * @brief   从 W25QXX 读取数据到 p_receive_buff 中
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   addr: 读取的 W25QXX 地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 读取的长度, 单位: 字节
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Read_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length)
{
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(read));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* 等待设备准备好 */
    Wait_for_W25QXX_Ready(p_obj);
    
    /* 发送命令与地址 */
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
    
    
    /* 接收数据 */
    p_obj->m_interface_func.m_p_Send_Receive(p_send_buff, p_receive_buff, length, EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, length: %u\r\n", __FUNCTION__, addr, length);
    W25QXX_DEBUG_PRINTF(DEBUG_END(read));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   擦除 W25QXX 的一个扇区
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   addr: 扇区地址, 必须按照扇区地址对齐
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t W25QXX_Erase_One_Sector(w25qxx_obj_t *p_obj, uint32_t addr)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(erase));
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X\r\n", __FUNCTION__, addr);
    
    /* 等待设备准备好 */
    status = Wait_for_W25QXX_Ready(p_obj);
    
    /* 写使能 */
    status = W25QXX_Write_Enable(p_obj);
    
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    
    /* 发送命令与地址 */
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
  * @brief   擦除 W25QXX 的扇区
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   addr: 扇区地址, 必须按照扇区地址对齐
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t W25QXX_Erase(w25qxx_obj_t *p_obj, uint32_t addr, uint32_t length)
{
    uint32_t sector_num = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(erase));
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, length: %u\r\n", __FUNCTION__, addr, length);
    
    /* 检查函数参数 */
    if (p_obj == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* 地址按扇区地址对齐 */
    addr &= (~(W25QXX_SECTOR_SIZE - 1));
    
    /* 计算需要擦除的扇区数量 */
    sector_num = length / W25QXX_SECTOR_SIZE;
    if(length % W25QXX_SECTOR_SIZE)
    {
        sector_num++;
    }
    
    /* 擦除扇区 */
    for(uint32_t i = 0; i < sector_num; i++)
    {
        W25QXX_Erase_One_Sector(p_obj, addr);
        addr += W25QXX_SECTOR_SIZE;
    }
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(erase));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   按页写 W25QXX
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   addr: 写入的 W25QXX 地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_w25qxx_status_t
  */
static en_w25qxx_status_t Page_Write_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint16_t length)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint8_t arr_send_buff[5] = {0};
    uint8_t arr_receive_buff[5] = {0};
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(page write));
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, length: %u\r\n", __FUNCTION__, addr, length);
    
    /* 等待设备准备好 */
    status = Wait_for_W25QXX_Ready(p_obj);
    
    /* 写使能 */
    status = W25QXX_Write_Enable(p_obj);
    
    if (status != EN_W25QXX_OK)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_ERROR));
        return EN_W25QXX_ERROR;
    }
    
    /* 发送命令与地址 */
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
    
    /* 发送数据 */
    p_obj->m_interface_func.m_p_Send_Receive(p_send_buff, p_receive_buff, length, EN_W25QXX_CLOSE_COM);
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(page write));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   写 p_send_buff 中的数据到 W25QXX 中
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   addr: 写入的 W25QXX 地址, 必须按照扇区地址对齐
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @param   length: 写入的长度, 单位: 字节
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Write_W25QXX(w25qxx_obj_t *p_obj, uint32_t addr, const uint8_t *p_send_buff, uint8_t *p_receive_buff, uint32_t length)
{
    uint32_t page_num = 0;
    uint16_t page_mode = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(write));
    
    /* 检查函数参数 */
    if (p_obj == NULL || p_send_buff == NULL || p_receive_buff == NULL)
    {
        W25QXX_DEBUG_PRINTF(DEBUG_ERROR(EN_W25QXX_PARAM_IS_NULL));
        return EN_W25QXX_PARAM_IS_NULL;
    }
    
    /* 地址按扇区地址对齐 */
    addr &= (~(W25QXX_SECTOR_SIZE - 1));
    
    /* 计算要写入多少个页 */
    page_num = length / PAGE_SIZE;
    page_mode = length % PAGE_SIZE;
    
    W25QXX_DEBUG_PRINTF("%s()->addr: 0x%X, page_num: %u, page_mode: %u\r\n", __FUNCTION__, addr, page_num, page_mode);
    
    /* 整页写 */
    for (uint32_t i = 0; i < page_num; i++)
    {
        /* 判断当前地址是不是扇区地址, 是扇区地址则先擦除再写入数据 */
        if ((addr & (W25QXX_SECTOR_SIZE - 1)) == 0)
        {
            W25QXX_Erase_One_Sector(p_obj, addr);
        }
        Page_Write_W25QXX(p_obj, addr, p_send_buff, p_receive_buff, PAGE_SIZE);
        addr += PAGE_SIZE;
        p_send_buff += PAGE_SIZE;
        p_receive_buff += PAGE_SIZE;
    }
    
    /* 不足一页写 */
    if (page_mode)
    {
        /* 判断当前地址是不是扇区地址, 是扇区地址则先擦除再写入数据 */
        if ((addr & (W25QXX_SECTOR_SIZE - 1)) == 0)
        {
            W25QXX_Erase_One_Sector(p_obj, addr);
        }
        Page_Write_W25QXX(p_obj, addr, p_send_buff, p_receive_buff, page_mode);
    }
    
    W25QXX_DEBUG_PRINTF(DEBUG_END(write));
    
    return EN_W25QXX_OK;
}


/**
  * @brief   测试 W25QXX
  * @note    请保证 p_send_buff 和 p_receive_buff 至少为一个扇区大小
  * @param   p_obj: w25qxx_obj_t 的地址
  * @param   p_send_buff: 发送缓冲区的地址
  * @param   p_receive_buff: 接收缓冲区的地址
  * @return  en_w25qxx_status_t
  */
en_w25qxx_status_t Test_W25QXX(w25qxx_obj_t *p_obj, uint8_t *p_send_buff, uint8_t *p_receive_buff)
{
    en_w25qxx_status_t status = EN_W25QXX_OK;
    uint32_t i = 0;
    
    W25QXX_DEBUG_PRINTF(DEBUG_BEGIN(test));
    
    /* 检查函数参数 */
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

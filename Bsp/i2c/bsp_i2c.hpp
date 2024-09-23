#ifndef BSP_I2C_HPP
#define BSP_I2C_HPP

#define I2C_DEVICE_CNT 2   // C板引出了I2C2和I2C3
#define MX_I2C_SLAVE_CNT 8 // 最大从机数目,根据需要修改

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include <stdint.h>
#include "i2c.h"
#include "stdlib.h"

#ifdef __cplusplus
}

#endif


/********************************************硬件I2C*****************************************************************/
/* I2C 工作模式枚举 */
typedef enum
{
    // 基本工作模式
    I2C_BLOCK_MODE = 0, // 阻塞模式
    I2C_IT_MODE,        // 中断模式
    I2C_DMA_MODE,       // DMA模式

} I2C_Work_Mode_e;

/* I2C MEM工作模式枚举,这两种方法都是阻塞 */
typedef enum
{
    I2C_READ_MEM = 0, // 读取从机内部的寄存器or内存
    I2C_WRITE_MEM,    // 写入从机内部的寄存器or内存
} I2C_Mem_Mode_e;

/* I2C回调函数类型枚举 */
typedef enum
{
    I2C_Master = 0, //Master
    I2C_Mem,        //Mem
}I2C_Callback_e;

/* I2C状态枚举 */
typedef enum _I2C_Result_t {
    I2C_Result_Ok = 0x00,
    I2C_Result_Error,
} I2C_Result_t;

/* I2C 初始化结构体配置 */
typedef struct 
{
    I2C_HandleTypeDef *i2c_handle;       // i2c handle
    uint8_t device_address;             // 设置写入数据的地址
    I2C_Work_Mode_e work_mode;       // 工作模式
} I2C_Init_Config_s;


class Bsp_HW_I2C_c
{
public:
    Bsp_HW_I2C_c(I2C_Init_Config_s *I2C_Init_Config ,void (*hw_iic_callback)(Bsp_HW_I2C_c* I2C_Instance));
    void HW_I2C_Transmit(uint8_t *data, uint16_t size);     //I2C发送数据
    void HW_I2C_Receive(uint8_t *data, uint16_t size);      //I2C接收数据
    void HW_I2CAccessMem(uint16_t mem_addr, uint8_t *data, uint16_t size, I2C_Mem_Mode_e mem_mode, uint8_t mem8bit_flag);  //I2C读取从机寄存器(内存),只支持阻塞模式,超时默认为1ms
    I2C_Result_t writeByte(uint8_t device_address, uint8_t register_address, uint8_t data);                                 
    I2C_Result_t writeByte(uint8_t device_address, uint8_t data);                                                           
    I2C_Result_t write2Bytes(uint8_t device_address, uint8_t register_address, uint16_t data);                              
    I2C_Result_t readMultiBytes(uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
    uint8_t readOneByte(uint8_t device_address, uint8_t register_address);
    static void Bsp_HW_I2C_TxCallback(I2C_HandleTypeDef *hi2c, I2C_Callback_e Callback_type);
    I2C_Callback_e Callback_type_;          //回调类型
private:
    static Bsp_HW_I2C_c *hw_i2c_instance_[I2C_DEVICE_CNT];      //I2C实例指针数组
    static uint8_t idx_;                                        // 全局硬件I2C实例索引,每次有新的模块注册会自增
    I2C_HandleTypeDef *i2c_handle_ = nullptr;                             // i2c句柄
    uint8_t device_address_;
    I2C_Work_Mode_e work_mode_;             //工作模式
    uint8_t *rx_buffer;                     // 接收缓冲区指针
    uint8_t rx_len;                         // 接收长度 
    void (*hw_iic_callback)(Bsp_HW_I2C_c* I2C_Instance);        // 接收完成后的回调函数
};


/*******************************************模拟I2C*********************************************************************/
class Bsp_SW_I2C_c
{
public:

private:

};

#endif // !BSP_I2C_HPP

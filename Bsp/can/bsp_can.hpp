#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP



// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 18     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)

/* can instance typedef, every module registered to CAN should have this variable */
#ifdef __cplusplus
extern "C" {

#include <stdint.h>
#include "can.h"


#endif


#ifdef __cplusplus
}


namespace Bsp_CAN_n
{
    typedef struct
    {
        CAN_HandleTypeDef *can_handle; // can句柄
        CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
        uint32_t tx_id;                // 发送id
        uint32_t tx_mailbox;           // CAN消息填入的邮箱号
        uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
        uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
        uint32_t rx_id;                // 接收id
        uint8_t rx_len;                // 接收长度,可能为0-8
        float tx_wait_time;            // 使用 CAN_Transmit中等待发送的时间
        // 接收的回调函数,用于解析接收到的数据
        void (*can_module_callback)(uint8_t *data, uint32_t StdId); // callback needs an instance to tell among registered ones
        void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
    } CANInstance_t;

    /* CAN实例初始化结构体,将此结构体指针传入注册函数 */
    typedef struct
    {
        CAN_HandleTypeDef *can_handle;              // can句柄
        uint32_t tx_id;                             // 发送id
        uint32_t rx_id;                             // 接收id
        void (*can_module_callback)(uint8_t *data, uint32_t StdId); // 处理接收数据的回调函数
        void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
    } CAN_Init_Config_t;

    class Bsp_CAN_c
    {
    private:
    public:
        CANInstance_t *can_instance[CAN_MX_REGISTER_CNT];
        uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增
        Bsp_CAN_c(/* args */);
        ~Bsp_CAN_c();
    };
    
    CANInstance_t *CANRegister(Bsp_CAN_n::CAN_Init_Config_t *config);
    void CANSetDLC(CANInstance_t *_instance, uint8_t length);
    uint8_t CANTransmit(CANInstance_t *_instance,float timeout);
}

#endif


#endif

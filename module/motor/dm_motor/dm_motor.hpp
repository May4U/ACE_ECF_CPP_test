/**
 * @file bsp_DM_Motor.h
 * @author wuage2335
*/
#pragma once
#ifndef __DM_MOTOR_HPP
#define __DM_MOTOR_HPP

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can.h"

#ifdef __cplusplus
}
#endif

#define DM_MOTOR_CNT 8


typedef enum{
    DM_MOTOR_STOP,
    DM_MOTOR_ENALBED,
    DM_MOTOR_START
}Motor_Working_Type_e;

typedef enum{
    MOTOR_DIRECTION_NOMAL, // 正转
    MOTOR_DIRECTION_REVERSE
}Motor_Working_Way;

// 电机返回的错误信息
typedef enum{
    DM_NONE_ERROR,
    OVERVOLTAGE = 0X08,     // 输入电机的电压过高
    UNDERVOLTAGE,           // 输入电机的电压过低   
    UNDERCURRENT,           // 输入电机的电流过高
    MOS_OVERTEMPERTURE,     // 驱动板上的 MOS 温度过高
    MOTOR_OVERTEMPERTURE,   // 电机线圈过温
    COMMUNICATE_LOSE,       // 通讯丢失
    OVERLOAD                // 电机过载    
}DM_Motor_Error_Type_e;

typedef enum{
    DM_UNABLE = 0,                 // 失能
    DM_ENABLE = 1,                 // 使能
    DM_PORTECT_ZERO_POSITION = 2,  // 保存位置零点
    DM_CLEAR_FAULT = 3             // 清除电机反馈错误内容
}DM_Motor_Status_e;

// 电机驱动方式, 只能在上位机设置
typedef enum{
    DM_MIT_MODE = 0,
    DM_POSI_SPEED_MODE,
    DM_SPEED_MODE,
    DM_EMIT_MODE
}DM_Motor_Mode_e;

// DM电机内置参数, 通过上位机设置, 这里赋值用于int↔float计算
typedef struct{
    float posi_max;    // 内置参数中的最大的位置 rad
    float posi_min;    // 内置参数中的最小的位置 rad
    
    float v_max;    // 内置参数中的最大的速度 rad/s
    float v_min;    // 内置参数中的最小的速度 rad/s
    
    float t_max;    // 内置参数中的最大的扭矩 NM
    float t_min;    // 内置参数中的最小的扭矩 NM

    float kp_max;    // MIT控制模式中的最大 kp
    float kp_min;    // MIT控制模式中的最小 kp
    float kd_max;    // MIT控制模式中的最大 kd
    float kd_min;    // MIT控制模式中的最小 kd

}DM_Motor_Builtin_Para_s;

// 电机返回数据
typedef struct
{
    DM_Motor_Error_Type_e error_type;   // 电机反馈的故障类型
    uint8_t feed_back_ID;   // 电机驱动反馈的控制器 ID, 取 CAN_ID 的低八位
    float position_rad;     // 电机反馈的位置信息, 单位为 rad
    float speed_rad;        // 电机反馈的速度信息, 单位为 rad/s
    float torque;           // 电机反馈力矩, 单位为 NM
    float mos_temp;         // 电机内 mos  温度
    float motor_temp;       // 电机内 线圈 温度
    float position_angle;   // 角度, 由 position_rad 处理得到, 单位为 ° 
    float speed_angle;      // 角速度, 由speed_rad处理得到, 单位为 °/s
}DM_Motor_Measure_s;

// 达妙电机发送控制值
typedef struct{
    float position_rad;         
    float speed_rad;
    float position_angle;         
    float speed_angle;
    float kp;
    float kd;
    float torque;
}DM_Motor_Send_Data_s;


typedef struct{
    DM_Motor_Measure_s measure;                 // 电机测量值
    DM_Motor_Send_Data_s send_datas;            // 发送数据

    DM_Motor_Builtin_Para_s builtin_para;

    // uint16_t send_id;                        // 电机发送id
    // uint16_t rece_id;

    Motor_Working_Type_e stop_flag;             // 启停标志
    Motor_Working_Way  motor_reverse_flag;

    DM_Motor_Mode_e motor_mode;                 // 电机驱动方式
    CAN_HandleTypeDef *hcan_handle;
    uint8_t hcan_send[8];
    
}DM_MOTORInstance;

typedef struct{

    DM_Motor_Builtin_Para_s builtin_para;
    Motor_Working_Way  motor_reverse_flag;
    DM_Motor_Mode_e motor_mode;                 // 电机驱动方式
    Bsp_CAN_n::CAN_Init_Config_t *hcan_init;

}DM_MOTORConfig;



class DM_motor_c
{
public:
    uint32_t feed_cnt;                         // 该实例时间戳, 用于数据计算返回的时间间隔 dt
    float dt;                               // 数据接收间隔时间
    DM_MOTORInstance *instance;             // 电机运行实例
    Bsp_CAN_n::CANInstance_t *can_instance; // 


    void MotorSetStatue(DM_Motor_Status_e status);
    void MotorSetValue(float posi, float speed, float kp, float kd, float torque);
    void MotorInit(DM_MOTORConfig *config);
};






/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DMMotorControl(void);



#endif // !__DM_MOTOR_H

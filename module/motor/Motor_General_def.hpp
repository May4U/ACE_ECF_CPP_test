/************************** Dongguan-University of Technology -ACE**************************
* @file Motor_General_def.h
* @brief 电机的通用配置定义文件
* @author wuage2335俞永祺 
* @version 1.0
* @date 2022-11-24
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#ifndef __MOTOR_GENERAL_DEF_H
#define __MOTOR_GENERAL_DEF_H


#include "pid_alg.hpp"
#include "bsp_can.hpp"
#include "General_def.hpp"

namespace Motor_General_Def_n
{
    
    /* 电机正反转标志 */
    typedef enum
    {
        MOTOR_DIRECTION_NORMAL = 0,
        MOTOR_DIRECTION_REVERSE = 1
    } Motor_Reverse_Flag_e;

    
    /* 反馈量正反标志 */
    typedef enum
    {
        FEEDBACK_DIRECTION_NORMAL = 0,
        FEEDBACK_DIRECTION_REVERSE = 1
    } Feedback_Reverse_Flag_e;
    typedef enum
    {
        MOTOR_STOP = 0,
        MOTOR_ENALBED = 1,
        MOTOR_Lock = 2,
        MOTOR_INIT = 3 // 表示此时正在初始化状态 
    } Motor_Working_Type_e;
    
    /* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
    typedef enum
    {
        MOTOR_FEED = 0,
        OTHER_FEED,
    } Feedback_Source_e;
    // 闭环部分
    typedef enum
    {
        OPEN_LOOP = 0b0000,
        CURRENT_LOOP = 0b0001,
        SPEED_LOOP = 0b0010,
        ANGLE_LOOP = 0b0100,

        // only for checking
        SPEED_AND_CURRENT_LOOP = 0b0011,
        ANGLE_AND_SPEED_LOOP = 0b0110,
        ALL_THREE_LOOP = 0b0111,
    } Closeloop_Type_e;

    typedef enum
    {
        FEEDFORWARD_NONE = 0b00,
        CURRENT_FEEDFORWARD = 0b01,
        SPEED_FEEDFORWARD = 0b10,
        CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
    } Feedfoward_Type_e;
    /* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
    typedef struct
    {
        Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
        Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
        Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
        Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
        Feedback_Source_e angle_feedback_source;       // 角度反馈类型
        Feedback_Source_e speed_feedback_source;       // 速度反馈类型
        Feedfoward_Type_e feedforward_flag;            // 前馈标志

    } Motor_Control_Setting_s;
    
    /* 电机类型枚举 */
    typedef enum
    {
        MOTOR_TYPE_NONE = 0,
        GM6020,
        M3508,
        M2006,
        LK9025,
        HT04,
    } Motor_Type_e;

    /**
     * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
     *        如果不需要某个控制环,可以不设置对应的pid config
     *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
     */
    typedef struct
    {
        pid_alg_n::PID_Init_Config_t current_PID;
        pid_alg_n::PID_Init_Config_t speed_PID;
        pid_alg_n::PID_Init_Config_t angle_PID;
    } Motor_Controller_Init_s;
    
    /* 用于初始化CAN电机的结构体,各类电机通用 */
    typedef struct
    {
        Motor_Controller_Init_s controller_param_init_config;
        Motor_Control_Setting_s controller_setting_init_config;
        Motor_Type_e motor_type;
        Bsp_CAN_n::CAN_Init_Config_t can_init_config;
        float radius;       // 输出轴半径
        float ecd2length;
    } Motor_Init_Config_s;

    /* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
    // 后续增加前馈数据指针
    class Motor_Controller_c
    {
        private :
            float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
        public:
            pid_alg_n::pid_alg_c current_PID;
            pid_alg_n::pid_alg_c speed_PID;
            pid_alg_n::pid_alg_c angle_PID;
            void RefValChange(float ref_val);
            float GetRefVal(void);
    };


}



#endif // !__MOTOR_GENERAL_DEF_H

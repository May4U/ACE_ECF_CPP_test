#ifndef __PID_ALG_HPP
#define __PID_ALG_HPP

#include "filter_alg.hpp"
#ifdef __cplusplus
extern "C" {

#include <stdint.h>
#include "main.h"

#endif


#ifdef __cplusplus
}

// PID 枚举结构体
typedef enum
{
    NONE = 0X00,					  // 0000 0000 无
    Deadzone = 0x01,				  // 0000 0001 死区
    Integral_Limit = 0x02,			  // 0000 0010 积分限幅
    Output_Limit = 0x04,			  // 0000 0100 输出限幅
    Derivative_On_Measurement = 0x08, // 0000 1000 微分先行 TODO:
    Separated_Integral = 0x10,		  // 0001 0000 积分分离
    ChangingIntegrationRate = 0x20,	  // 0010 0000 变积分
    OutputFilter = 0x40,			  // 0100 0000 输出滤波
    DerivativeFilter = 0x80,		  // 1000 0000 微分滤波
    StepIn = 0x0100,				  // 0000 0001 0000 0000 步进式
} PID_mode_e;

namespace pid_alg_n
{

    typedef struct 
    {
        float Kp;
        float Ki;
        float Kd;
        
        float *ActualValueSource;

        uint32_t mode; // pid模式
        /* 输出限幅 */
        float max_out; 
        /* 积分限幅 */
        float max_Ierror; //最大积分输出
        /* 微分先行 */
        float gama; // 微分先行滤波系数
        /* 误差死区 */
        float deadband;
        /* 积分分离 */
        float threshold_max; //积分分离最大值
        float threshold_min; //积分分离最小值
        /* 变积分 */
        float errorabsmax; //偏差绝对值最大值
        float errorabsmin; //偏差绝对值最小值
        /* 微分滤波 */
        float d_filter_num;
        /* 输出滤波 */
        float out_filter_num;
        /* 步进数 */
        float stepIn;
    }PID_Init_Config_t;

    
    
    typedef  struct Pid_parameter_t // pid结构体变量
    {
        float Kp;
        float Ki;
        float Kd;

        float SetValue;
        float LastSetValue;
        float LastActualValue;
        float ActualValue;

        float Ierror;
        float Pout;
        float Iout;
        float Dout;
        float out;

        float Derror; //微分项
        float LastDerror;
        float LastLastDerror;
        float error; //误差项
        float LastError;

        float max_out; //最大输出

        uint32_t mode; // pid模式

        /* 积分限幅 */
        float max_Ierror; //最大积分输出
        /* 误差死区 */
        float deadband;
        /* 积分分离 */
        float threshold_max; //积分分离最大值
        float threshold_min; //积分分离最小值
        /* 抗积分饱和 */
        // float maximum; //最大值
        // float minimum; //最小值
        /* 变积分 */
        float errorabsmax; //偏差绝对值最大值
        float errorabsmin; //偏差绝对值最小值
        /* 微分先行 */
        float gama; // 微分先行滤波系数

        /* 微分滤波 */
        filter_alg_n::first_order_filter_c d_filter; //微分滤波结构体
        /* 输出滤波 */
        filter_alg_n::first_order_filter_c out_filter; //输出滤波结构体
        /* 步进数 */
        float stepIn;

    } pid_parameter_t;

    // pid算法类
    class pid_alg_c
    {
        private :
        
        public:
        float *ActualValueSource;   // pid 运算实际值来源
        pid_parameter_t pid_measure;
        void PidInit(pid_alg_n::PID_Init_Config_t pid_config);
        void pid_clear(void);
        float PidCalculate(float SetValue);
        void PidChangeActValSource(float *ActValSource);
        void f_Separated_Integral(void);
        void f_Integral_Limit(void);
        void f_Derivative_On_Measurement(void);
        void Changing_Integration_Rate(void);
        void f_Output_Limit(void);
        void f_StepIn(void);
    };


}



#endif

#endif // !__PID_ALG_HPP
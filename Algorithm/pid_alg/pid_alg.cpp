#include "pid_alg.hpp"



/**
 * @brief          PID初始化
 * @param[in]      pid_config pid 初始化参数结构体
 * @note 如想使用多个模式可以使用 ' | ', 
 * @note 比如使用 积分分离 和 输出滤波 则可以在 mode 入参 (OutputFilter | Separated_Integral)
 */
void pid_alg_n::pid_alg_c::PidInit(pid_alg_n::PID_Init_Config_t pid_config)
{
    memset(&this->pid_measure, 0, sizeof(pid_alg_n::pid_parameter_t));
    this->ActualValueSource = pid_config.ActualValueSource;
    this->pid_measure.Kp = pid_config.Kp;
    this->pid_measure.Ki = pid_config.Ki;
    this->pid_measure.Kd = pid_config.Kd;
    this->pid_measure.mode = pid_config.mode;

    this->pid_measure.max_Ierror = pid_config.max_Ierror;

    this->pid_measure.gama = pid_config.gama;

    this->pid_measure.threshold_max = pid_config.threshold_max;
    this->pid_measure.threshold_min = pid_config.errorabsmin;

    this->pid_measure.max_out = pid_config.max_out;

    this->pid_measure.out_filter.first_order_filter_init(pid_config.out_filter_num);
        
    this->pid_measure.errorabsmax = pid_config.errorabsmax;
    this->pid_measure.errorabsmin = pid_config.errorabsmin;

    this->pid_measure.d_filter.first_order_filter_init(pid_config.d_filter_num);

    this->pid_measure.deadband = pid_config.deadband;

    this->pid_measure.stepIn = pid_config.stepIn;

}

/**
 * @brief 更改闭环的实际值来源
 * @param ActValSource 
*/
void pid_alg_n::pid_alg_c::PidChangeActValSource(float *ActValSource)
{
    this->ActualValueSource = ActValSource;
}

// /**
//  * @brief          PID模式初始化
//  * @param[in]      初始化的pid结构体
//  * @param[in]      要初始化的 mode 值 详见pid_mode_e枚举
//  * @note           以下为不同的mode需要填入的值
//  * @note           NONE                         num1、num2都不填
//  * @note           Deadzone                     num1 死区范围
//  * @note           Integral_Limit               num1 积分限幅最大值
//  * @note           Output_Limit                 num1 输出限幅值 这个是绝对值
//  * @note           Derivative_On_Measurement    num1 惯性系数
//  * @note           Separated_Integral           num1 积分分离上限 num2积分分离下限
//  * @note           ChangingIntegrationRate      num1 为变积分上限 num2为变积分下限
//  * @note           OutputFilter                 num1 为滤波系数
//  * @note           DerivativeFilter             num1 为滤波系数
//  * @note           StepIN                       num1 步进数
//  * @retval         none
//  * @note 如果在 PidInit 函数设定了模式, 则每一个模式都需要调用该函数进行初始化, 否则会影响正常使用
//  */
// void pid_alg_n::pid_alg_c::PidInitMode(uint32_t mode, float num1, float num2)
// {
//     switch (mode)
//     {
//         case NONE:
//             return;
//         case Integral_Limit:
//             this->pid_measure.max_Ierror = num1;
//             return;
//         case Derivative_On_Measurement:
//             this->pid_measure.gama = num1;
//             return;
//         case Separated_Integral:
//             this->pid_measure.threshold_max = num1;
//             this->pid_measure.threshold_min = num2;
//             return;
//         case Output_Limit:
//             this->pid_measure.max_out = num1;
//             return;
//         case OutputFilter:
//             this->pid_measure.out_filter.first_order_filter_init(num1);
//             return;
//         case ChangingIntegrationRate:
//             this->pid_measure.errorabsmax = num1;
//             this->pid_measure.errorabsmin = num2;
//             return;
//         case DerivativeFilter:
//             this->pid_measure.d_filter.first_order_filter_init(num1);
//             return;
//         case Deadzone:
//             this->pid_measure.deadband = num1;
//             return;
//         case StepIn:
//             this->pid_measure.stepIn = num1;
//             return;
//     }
//     return;
// }

/**
 * @brief          PID清除
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除pid或者模式的数据
 */
void pid_alg_n::pid_alg_c::pid_clear(void)
{
    this->pid_measure.error = this->pid_measure.LastError = 0.0f;
    this->pid_measure.Derror = this->pid_measure.LastDerror = this->pid_measure.LastLastDerror = 0.0f;
    this->pid_measure.out = this->pid_measure.Pout = this->pid_measure.Iout = this->pid_measure.Dout = this->pid_measure.Ierror = 0.0f;
    this->pid_measure.ActualValue = this->pid_measure.SetValue = this->pid_measure.LastActualValue = this->pid_measure.LastSetValue = 0.0f;
    this->pid_measure.d_filter.first_order_filter_clear();
    this->pid_measure.out_filter.first_order_filter_clear();

}

/**
 * @brief          PID通用计算
 * @param[in]      设定参考值
 * @param[in]      实际值
 * @retval         none
 */
float pid_alg_n::pid_alg_c::PidCalculate(float SetValue)
{
    this->pid_measure.SetValue = SetValue;
    // 步进式pid
    if (this->pid_measure.mode & StepIn)
        this->f_StepIn();
    this->pid_measure.ActualValue = *this->ActualValueSource;
    this->pid_measure.error = this->pid_measure.SetValue - this->pid_measure.ActualValue;
    this->pid_measure.Derror = this->pid_measure.error - this->pid_measure.LastError;
    if (abs(this->pid_measure.error) >= this->pid_measure.deadband) //死区
    {
        this->pid_measure.Pout = this->pid_measure.error * this->pid_measure.Kp;
        this->pid_measure.Ierror += this->pid_measure.error;

        // 微分先行
        if (this->pid_measure.mode & Derivative_On_Measurement)
            this->f_Derivative_On_Measurement();
        else
            this->pid_measure.Dout = this->pid_measure.Kd * this->pid_measure.Derror;

        // 变积分
        if (this->pid_measure.mode & ChangingIntegrationRate)
            this->Changing_Integration_Rate();

        // 积分限幅
        if (this->pid_measure.mode & Integral_Limit)
            this->f_Integral_Limit();
        this->pid_measure.Iout = this->pid_measure.Ki * this->pid_measure.Ierror;

        // 积分分离 注意需要放在iout计算后
        if (this->pid_measure.mode & Separated_Integral)
            this->f_Separated_Integral();

        // 微分滤波
        if (this->pid_measure.mode & DerivativeFilter)
            this->pid_measure.Dout = this->pid_measure.d_filter.first_order_filter(this->pid_measure.Dout);

        this->pid_measure.out = this->pid_measure.Pout + this->pid_measure.Iout + this->pid_measure.Dout;

        // 输出滤波
        if (this->pid_measure.mode & OutputFilter)
            this->pid_measure.out = this->pid_measure.out_filter.first_order_filter(this->pid_measure.out);

        // 输出限幅
        if (this->pid_measure.mode & Output_Limit)
            this->f_Output_Limit();
    }
    else
    {
        this->pid_clear(); 
    }

    this->pid_measure.LastActualValue = this->pid_measure.ActualValue;
    this->pid_measure.LastSetValue = this->pid_measure.SetValue;
    this->pid_measure.LastDerror = this->pid_measure.Derror;
    this->pid_measure.LastError = this->pid_measure.error;

    return this->pid_measure.out;

}



/**
 * @brief          积分分离
 * @retval         none
 * @note      error值超过阈值的时候把iout清零就行
 * @note 当设定值和参考值差别过大时, 只使用 PD 控制
 */
void pid_alg_n::pid_alg_c::f_Separated_Integral(void)
{
    if (this->pid_measure.threshold_min > this->pid_measure.error && this->pid_measure.error < this->pid_measure.threshold_max)
        this->pid_measure.Iout = 0;
}

/**
 * @brief          积分限幅
 * @retval         none
 * @note      防止积分误差值(Ierror)超过限幅
 */
void pid_alg_n::pid_alg_c::f_Integral_Limit(void)
{
    if (this->pid_measure.Ierror > this->pid_measure.max_Ierror)
    {
        this->pid_measure.Ierror = this->pid_measure.max_Ierror;
    }
    if (this->pid_measure.Ierror < -(this->pid_measure.max_Ierror))
    {
        this->pid_measure.Ierror = -(this->pid_measure.max_Ierror);
    }
}

/**
 * @brief          微分先行
 * @retval         none
 * @attention      似乎存在问题 详见 https://blog.csdn.net/foxclever/article/details/80633275
 */
void pid_alg_n::pid_alg_c::f_Derivative_On_Measurement(void)
{
    float c1, c2, c3, temp;

    temp = this->pid_measure.gama * this->pid_measure.Kd + this->pid_measure.Kp;
    c3 = this->pid_measure.Kd / temp;
    c2 = (this->pid_measure.Kd + this->pid_measure.Kp) / temp;
    c1 = this->pid_measure.gama * c3;
    this->pid_measure.Dout = c1 * this->pid_measure.Dout + c2 * this->pid_measure.ActualValue + c3 * this->pid_measure.LastActualValue;
}


/**
 * @brief           变积分系数处理函数，实现一个输出0和1之间的分段线性函数
 * @param[in]       pid结构体
 * @retval          none
 * @note            当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
 * @note            当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间线性变化
 */
void pid_alg_n::pid_alg_c::Changing_Integration_Rate(void)
{
    if (std::abs(this->pid_measure.error) <= this->pid_measure.errorabsmin) //最小值
    {
        return;
    }
    else if (std::abs(this->pid_measure.error) > this->pid_measure.errorabsmax) //最大值
    {
        this->pid_measure.Ierror = 0.0f;
    }
    else
    {
        this->pid_measure.Ierror *= ((this->pid_measure.errorabsmax - abs(this->pid_measure.error)) / (this->pid_measure.errorabsmax - this->pid_measure.errorabsmin));
    }
}


/**
 * @brief          输出限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      none
 */
void pid_alg_n::pid_alg_c::f_Output_Limit(void)
{
    if (this->pid_measure.out > this->pid_measure.max_out)
    {
        this->pid_measure.out = this->pid_measure.max_out;
    }
    if (this->pid_measure.out < -(this->pid_measure.max_out))
    {
        this->pid_measure.out = -(this->pid_measure.max_out);
    }
}


/**
 * @brief           步进式pid
 * @retval          none
 * @attention       详见 https://blog.csdn.net/foxclever/article/details/81151898
 * @note            固定每周期的设定值变化值
 */
void pid_alg_n::pid_alg_c::f_StepIn(void)
{
    float kFactor = 0.0f;
    //    if ((this->pid_measure.LastSetValue - this->pid_measure.SetValue <= this->pid_measure.stepIn) && (this->pid_measure.LastSetValue - this->pid_measure.SetValue >= this->pid_measure.stepIn))
    //    {
    //        return;
    //    }
    if (abs(this->pid_measure.LastSetValue - this->pid_measure.SetValue) <= this->pid_measure.stepIn)
    {
        return;
    }
    else
    {
        if ((this->pid_measure.LastSetValue - this->pid_measure.SetValue) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((this->pid_measure.LastSetValue - this->pid_measure.SetValue) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        this->pid_measure.SetValue = this->pid_measure.LastSetValue + kFactor * this->pid_measure.stepIn;
    }

}



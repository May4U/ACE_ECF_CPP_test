/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_dwt.cpp
* @brief dwt 内部隐藏计时, 参考跃鹿
* @author wuage2335俞永祺 
* @version 1.0
* @date 2022-11-24
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_dwt.hpp"

#ifdef INC_FREERTOS_H
#include "cmsis_os.h"

#endif // DEBUG

using namespace Bsp_DWT_n;
Bsp_DWT_n::Bsp_DWT_c dwt_time(168);
// std::shared_ptr<Bsp_DWT_n::Bsp_DWT_c> dwt_time = std::make_shared<Bsp_DWT_n::Bsp_DWT_c>(168);


// /**
//  * @brief 返回 DWT 实例
//  * @note    DWT 实例只能有一个, 
//  *          需要调用该 Bsp_DWT_n::Bsp_DWT_c类, 请创建一个指针, 并通过这个函数赋值
//  */
// Bsp_DWT_n::Bsp_DWT_c *Bsp_DWT_n::Get_DWT_Pointer(void)
// {
    
//     return &dwt_time;
// }

/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 *
 * @param CPU_Freq_mHz c板为168MHz,A板为180MHz
 */
Bsp_DWT_n::Bsp_DWT_c::Bsp_DWT_c(uint32_t CPU_Freq_mHz)
{
    // this->SysTime.s = 0;
    // this->SysTime.us = 0;
    // this->SysTime.ms = 0;
    if(this->bsp_dwt_if_init == false)
    {
        /* 使能DWT外设 */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

        /* DWT CYCCNT寄存器计数清0 */
        DWT->CYCCNT = (uint32_t)0u;

        /* 使能Cortex-M DWT CYCCNT寄存器 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        this->CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
        this->CPU_FREQ_Hz_ms = this->CPU_FREQ_Hz / 1000;
        this->CPU_FREQ_Hz_us = this->CPU_FREQ_Hz / 1000000;
        this->CYCCNT_RountCount = 0;
        bsp_dwt_if_init = true;
    }
}


Bsp_DWT_n::Bsp_DWT_c::~Bsp_DWT_c()
{
}
    
/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
void Bsp_DWT_n::DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < dwt_time.CYCCNT_LAST)
            dwt_time.CYCCNT_RountCount++;

        dwt_time.CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}


/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为: 秒/s
 */
float Bsp_DWT_n::DWT_GetDeltaT(uint32_t *cnt_last)
{
    
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(dwt_time.CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    Bsp_DWT_n::DWT_CNT_Update();

    return dt;
}
/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double Bsp_DWT_n::DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(dwt_time.CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    Bsp_DWT_n::DWT_CNT_Update();

    return dt;
}
/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间轴
 */
float Bsp_DWT_n::DWT_GetTimeline_s(void)
{
    Bsp_DWT_n::DWT_SysTimeUpdate();

    float DWT_Timelinef32 = dwt_time.SysTime.s + dwt_time.SysTime.ms * 0.001f + dwt_time.SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}
/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 *
 * @return float
 */
float Bsp_DWT_n::DWT_GetTimeline_ms(void)
{
    
    Bsp_DWT_n::DWT_SysTimeUpdate();

    float DWT_Timelinef32 = dwt_time.SysTime.s * 1000 + dwt_time.SysTime.ms + dwt_time.SysTime.us * 0.001f;

    return DWT_Timelinef32;
}
/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t Bsp_DWT_n::DWT_GetTimeline_us(void)
{
    
    Bsp_DWT_n::DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = dwt_time.SysTime.s * 1000000 + dwt_time.SysTime.ms * 1000 + dwt_time.SysTime.us;

    return DWT_Timelinef32;
}
/**
 * @brief DWT更新时间轴函数,会被三个timeline函数调用
 * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void Bsp_DWT_n::DWT_SysTimeUpdate(void)
{
    
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    Bsp_DWT_n::DWT_CNT_Update();

    dwt_time.CYCCNT64 = (uint64_t)dwt_time.CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = dwt_time.CYCCNT64 / dwt_time.CPU_FREQ_Hz;
    CNT_TEMP2 = dwt_time.CYCCNT64 - CNT_TEMP1 * dwt_time.CPU_FREQ_Hz;
    dwt_time.SysTime.s = CNT_TEMP1;
    dwt_time.SysTime.ms = CNT_TEMP2 / dwt_time.CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - dwt_time.SysTime.ms * dwt_time.CPU_FREQ_Hz_ms;
    dwt_time.SysTime.us = CNT_TEMP3 / dwt_time.CPU_FREQ_Hz_us;
}
/**
 * @brief DWT延时函数,单位为毫秒(ms)
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为毫秒(ms)
 */
void Bsp_DWT_n::DWT_Delay(float Delay)
{
    
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)dwt_time.CPU_FREQ_Hz_ms)
        ;
}

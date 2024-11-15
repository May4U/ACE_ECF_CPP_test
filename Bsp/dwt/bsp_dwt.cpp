/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_dwt.cpp
 * @author  study-sheep
 * @version V1.2
 * @date    2024/9/14
 * @brief   DWT的BSP层文件，用于计时
 ******************************************************************************
 * @verbatim
 *  DWT计时函数,支持防止dwt被多次初始化
 *  使用方法：
 *      在目标文件里面定义一个BSP_DWT_c实例指针，并调用ECF_Get_DwtInstance()函数来获取唯一实例的地址
 *      然后就可以用指针愉快地调用成员函数啦
 *  demo：
 *      // 定义一个BSP_DWT_c实例指针，并调用ECF_Get_DwtInstance()函数让外部实例指针指向唯一实例的地址
        static BSP_DWT_n::BSP_DWT_c* dwt_time = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
 *      
 *      // 延时函数
 *      dwt_time->ECF_DWT_Delay_s(1); // 延时1s
 * 
 *      // 获取当前时间,单位为秒/s,即初始化后的时间
 *      float time = dwt_time->ECF_DWT_GetTimeline_s(); // 获取当前时间,单位为秒/s
 * 
 *      // 获取两次调用之间的时间间隔,单位为秒/s
 *      // @param cnt_last 定义一个uint32_t的计时变量，将其地址作为参数传入
 *      // @return float dt 时间间隔,单位为: 秒/s
 *      dwt_time->ECF_DWT_GetDeltaT(uint32_t *cnt_last) 
 *      float dt = dwt_time->ECF_DWT_GetDeltaT(uint32_t *cnt_last) 
 * 
 *      // 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *      // @param cnt_last 定义一个uint32_t的计时变量，将其地址作为参数传入
 *      // @return double dt 时间间隔,单位为: 秒/s
 *      dwt_time->ECF_DWT_GetDeltaT64(uint32_t *cnt_last)  
 *      double dt = dwt_time->ECF_DWT_GetDeltaT64(uint32_t *cnt_last)
 * @attention
 *      无
 * @version           time
 * v1.0   基础版本     2024-9-11   
 * v1.1   C++优化版本  2024-9-14   已测试
 * v1.2   采用命名空间，而且定时更准确，支持ms、us的延时  2024-9-29   已测试
 ************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_dwt.hpp"      // 本文件头文件

#include "stm32f4xx_hal.h"  // HAL库文件

namespace BSP_DWT_n
{
    /**
     * @brief 初始化DWT,传入参数为CPU频率,单位MHz
     *
     * @param CPU_Freq_mHz c板为168MHz,A板为180MHz
     */
    BSP_DWT_c::BSP_DWT_c(uint32_t CPU_Freq_mHz) 
    {
        /* 使能DWT外设 */        
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

        /* DWT CYCCNT寄存器计数清0 */        
        DWT->CYCCNT = 0u;
        /* 使能Cortex-M DWT CYCCNT寄存器 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        this->CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
        this->CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
        this->CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
        this->CYCCNT_RountCount = 0u;
        DWT_CntUpdate();
    }

    /**
     * @brief 获取两次调用之间的时间间隔,单位为秒/s
     *
     * @param cnt_last 上一次调用的时间戳
     * @return float 时间间隔,单位为: 秒/s
     */
    float BSP_DWT_c::ECF_DWT_GetDeltaT(uint32_t *cnt_last) 
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(this->CPU_FREQ_Hz));
        *cnt_last = cnt_now;
        DWT_CntUpdate();
        return dt;
    }

    /**
     * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
     *
     * @param cnt_last 上一次调用的时间戳
     * @return double 时间间隔,单位为秒/s
     */
    double BSP_DWT_c::ECF_DWT_GetDeltaT64(uint32_t *cnt_last) 
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(this->CPU_FREQ_Hz));
        *cnt_last = cnt_now;
        DWT_CntUpdate();
        return dt;
    }

    /**
     * @brief DWT更新时间轴函数,会被三个timeline函数调用
     * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
     */
    void BSP_DWT_c::DWT_SystimeUpdate() 
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;
        DWT_CntUpdate();
        this->CYCCNT64 = (uint64_t)this->CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
        CNT_TEMP1 = this->CYCCNT64 / this->CPU_FREQ_Hz;
        CNT_TEMP2 = this->CYCCNT64 - CNT_TEMP1 * this->CPU_FREQ_Hz;
        this->SysTime.s = CNT_TEMP1;
        this->SysTime.ms = CNT_TEMP2 / this->CPU_FREQ_Hz_ms;
        CNT_TEMP3 = CNT_TEMP2 - this->SysTime.ms * this->CPU_FREQ_Hz_ms;
        this->SysTime.us = CNT_TEMP3 / this->CPU_FREQ_Hz_us;
    }

    /**
     * @brief 获取当前时间,单位为秒/s,即初始化后的时间
     *
     * @return float 时间轴
     */
    float BSP_DWT_c::ECF_DWT_GetTimeline_s() 
    {
        DWT_SystimeUpdate();
        float DWT_Timelinef32 = this->SysTime.s + this->SysTime.ms * 0.001f + this->SysTime.us * 0.000001f;
        return DWT_Timelinef32;
    }

    /**
     * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
     *
     * @return float
     */
    float BSP_DWT_c::ECF_DWT_GetTimeline_ms() 
    {
        DWT_SystimeUpdate();
        float DWT_Timelinef32 = this->SysTime.s * 1000 + this->SysTime.ms + this->SysTime.us * 0.001f;
        return DWT_Timelinef32;
    }

    /**
     * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
     *
     * @return uint64_t
     */
    uint64_t BSP_DWT_c::ECF_DWT_GetTimeline_us() 
    {
        DWT_SystimeUpdate();
        uint64_t DWT_Timelinef32 = this->SysTime.s * 1000000 + this->SysTime.ms * 1000 + this->SysTime.us;
        return DWT_Timelinef32;
    }

    /**
     * @brief DWT延时函数,单位为秒(s)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(s)
     */
    void BSP_DWT_c::ECF_DWT_Delay_s(float Delay) 
    {
        uint32_t tickstart = DWT->CYCCNT;
        float wait = Delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->CPU_FREQ_Hz);
    }

    /**
     * @brief DWT延时函数,单位为秒(ms)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(ms)
     */
    void BSP_DWT_c::ECF_DWT_Delay_ms(float Delay) 
    {
        Delay *= 0.001;
        uint32_t tickstart = DWT->CYCCNT;
        float wait = Delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->CPU_FREQ_Hz);
    }

    /**
     * @brief DWT延时函数,单位为秒(us)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(us)
     */
    void BSP_DWT_c::ECF_DWT_Delay_us(float Delay) 
    {
        Delay *= 0.000001;
        uint32_t tickstart = DWT->CYCCNT;
        float wait = Delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->CPU_FREQ_Hz);
    }

    /**
     * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
     * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
     *            它会检测DWT_CYCCNT寄存器的值是否小于上一次读取时的值，如果是，则认为计数器已经溢出，将CYCCNT_RountCount加1，
     *            然后更新CYCCNT_LAST的值为当前的计数器值，以备下次比较使用。
     */
    void BSP_DWT_c::DWT_CntUpdate() 
    {
        static volatile uint8_t bit_locker = 0;
        if (!bit_locker) {
            bit_locker = 1;
            volatile uint32_t cnt_now = DWT->CYCCNT;
            if (cnt_now < this->CYCCNT_LAST) {
                this->CYCCNT_RountCount++;
            }
            this->CYCCNT_LAST = DWT->CYCCNT;
            bit_locker = 0;
        }
    }


    /**
     * @brief 用于获取DWT单例
     * @return BSP_DWT_c* DWT实例
     */
    BSP_DWT_c* BSP_DWT_c::ECF_Get_DwtInstance()
    {
        static BSP_DWT_c dwt_time(ARM_CHIP_Freq_MHz);
        return &dwt_time;
    }
}
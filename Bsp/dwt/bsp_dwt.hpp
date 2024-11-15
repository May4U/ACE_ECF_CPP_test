#ifndef __BSP_DWT_HPP
#define __BSP_DWT_HPP

#ifdef  __cplusplus
extern "C"
{
#endif
#include <stdint.h>

#ifdef  __cplusplus
}
#endif
namespace BSP_DWT_n
{
    // STM32控制芯片主频，单位：MHz
    #define ARM_CHIP_Freq_MHz 168

    typedef struct              
    {
        uint32_t s;
        uint16_t ms;
        uint16_t us;
    } DWT_Time_t;

    class BSP_DWT_c
    {
        public:
            static BSP_DWT_c* ECF_Get_DwtInstance();
            float ECF_DWT_GetDeltaT(uint32_t *cnt_last);
            double ECF_DWT_GetDeltaT64(uint32_t *cnt_last);
            float ECF_DWT_GetTimeline_s();
            float ECF_DWT_GetTimeline_ms();
            uint64_t ECF_DWT_GetTimeline_us();
            void ECF_DWT_Delay_s(float Delay);
            void ECF_DWT_Delay_ms(float Delay);
            void ECF_DWT_Delay_us(float Delay);
        private:
            BSP_DWT_c(uint32_t CPU_Freq_mHz);  // 私有构造函数
            static BSP_DWT_c* DWT_Instance;  // 单例模式
            uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;   // 分别表示CPU的频率，以及以毫秒和微秒为单位的CPU频率
            uint32_t CYCCNT_RountCount;                             // 用于记录CYCCNT计数器溢出的次数
            uint32_t CYCCNT_LAST;                                   // 用于记录上一次读取CYCCNT计数器的值
            uint64_t CYCCNT64;                                      // 用于存储CYCCNT计数器的值，作为64位整数
            DWT_Time_t SysTime;
            void DWT_CntUpdate();
            void DWT_SystimeUpdate();
    };            
}
#endif /* BSP_DWT_HPP */
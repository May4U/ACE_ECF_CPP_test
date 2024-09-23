#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "real_main.hpp"

#ifdef __cplusplus
extern "C" {

#include <stdint.h>

#endif


#ifdef __cplusplus
}

namespace Bsp_DWT_n
{
    
    typedef struct
    {
        uint32_t s;
        uint16_t ms;
        uint16_t us;
    } DWT_Time_t;

    class Bsp_DWT_c
    {
        private:
        public:
            bool bsp_dwt_if_init = false;   // 防止多次dwt初始化
            DWT_Time_t SysTime;
            uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
            uint32_t CYCCNT_RountCount;
            uint32_t CYCCNT_LAST;
            uint64_t CYCCNT64;
            Bsp_DWT_c(uint32_t CPU_Freq_mHz);
            ~Bsp_DWT_c();
            
    };
    
        void DWT_CNT_Update(void);
        float DWT_GetDeltaT(uint32_t *cnt_last);
        double DWT_GetDeltaT64(uint32_t *cnt_last);
        void DWT_SysTimeUpdate(void);
        float DWT_GetTimeline_s(void);
        float DWT_GetTimeline_ms(void);
        uint64_t DWT_GetTimeline_us(void);
        void DWT_Delay(float Delay);

}


#endif


#endif // !__BSP_DWT_H

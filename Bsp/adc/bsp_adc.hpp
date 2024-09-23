#ifndef BSP_ADC_HPP
#define BSP_ADC_HPP


#define ADC_MAX_NUM 20

#ifdef __cplusplus
extern "C"{

#include <stdint.h>
#include "adc.h"

#endif

#ifdef __cplusplus
}

namespace Bsp_ADC_n
{
    
    class Bsp_ADC_c
    {
    private:
        ADC_HandleTypeDef *_hadc;
    public:
        uint16_t ADC_Value[ADC_MAX_NUM];
        uint32_t ADC_Length;
        uint32_t GetAdcValues(void);
        Bsp_ADC_c( ADC_HandleTypeDef *hadc ,uint16_t ADC_Length);
        void ADC_Start(void);
        void ADC_Stop(void);
    };
}

#endif

#endif // !BSP_ADC_HPP

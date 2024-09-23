/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_adc.cpp
 * @author Lann 梁健蘅 (rendezook@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-07
 *******************************************************************************************
 * @verbatim 仅支持ADC多通道DMA采集
 * @verbatim
 * 
 * 
 * @copyright Copyright (c) 2024
 * 
************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_adc.hpp"
#include "stdlib.h"

using namespace Bsp_ADC_n;

/**
 * @brief ADC初始化构造函数，会在每次创建ADC类的新对象时执行。
 * 
 * @param hadc 
 */
Bsp_ADC_c::Bsp_ADC_c(ADC_HandleTypeDef *hadc, uint16_t _ADC_Length)
://初始化列表
_hadc(hadc),
ADC_Length(_ADC_Length)
{
    // this->_hadc = hadc;
    // this->ADC_Length = _ADC_Length;
    //HAL_ADC_Start_DMA(hadc, (uint32_t *)ADC_Value, this->ADC_Length);
}

/**
 * @brief 获取ADC值
 * 
 * @return uint32_t 
 * @todo 在C++里好像没必要用到，先丢着
 */
uint32_t Bsp_ADC_c::GetAdcValues(void)
{   
    return (uint32_t)&ADC_Value;
}

/**
 * @brief 开启ADC_DMA
 * 
 */
void Bsp_ADC_c::ADC_Start(void)
{
    HAL_ADC_Start_DMA(this->_hadc ,(uint32_t *)this->ADC_Value ,this->ADC_Length);
}

/**
 * @brief 关闭ADC_DMA
 * 
 */
void Bsp_ADC_c::ADC_Stop(void)
{
    HAL_ADC_Stop_DMA(this->_hadc);
}

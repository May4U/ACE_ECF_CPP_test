#include "filter_alg.hpp"


/**
 * @brief 一阶低通滤波计算
 * @param input 采样值
 * @return 输出值
*/
float filter_alg_n::first_order_filter_c::first_order_filter(float input)        
{
  this->measure.input = input;
  this->measure.out = this->measure.input * this->measure.num + (1 - this->measure.num) * this->measure.last_input;
  this->measure.last_input = this->measure.out;

  return this->measure.out;
}

/**
 * @brief 一阶低通滤波初始化
 * @param num 一阶滤波系数
 * @note 滤波系数在 0~1.0 区间, 超出这个范围则默认为 1
 * @note 系数越小, 得到的波形越平滑, 但是也更加迟钝
*/
void filter_alg_n::first_order_filter_c::first_order_filter_init(float num)
{
    if(num > 0.0 && num < 1.0)
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = num;
        this->measure.out = 0;
    }
    else
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = 1;
        this->measure.out = 0;
    }
}


/**
 * @brief          一阶低通滤波清除
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除系数
 */
void filter_alg_n::first_order_filter_c::first_order_filter_clear(void)
{
    this->measure.last_input = 0;
    this->measure.input = 0;
    this->measure.out = 0;
}





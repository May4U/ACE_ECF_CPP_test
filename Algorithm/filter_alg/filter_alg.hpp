#ifndef __FILTER_ALG_HPP
#define __FILTER_ALG_HPP


#ifdef __cplusplus
extern "C" {

#include <stdint.h>
#include "arm_math.h"
#include "main.h"

#endif


#ifdef __cplusplus
}

/**
 * @brief 滤波算法命名空间
*/
namespace filter_alg_n
{
    //一阶低通滤波参数
    typedef  struct
    {
        float input;		 //输入数据
        float last_input; //上次数据
        float out;		 //滤波输出的数据
        float num;		 //滤波参数
    } first_order_filter_t;
    
    // 一阶低通滤波类
    class first_order_filter_c
    {
        private:
        first_order_filter_t measure;
        public:
        float first_order_filter(float input);
        void first_order_filter_init(float num);
        void first_order_filter_clear(void);
    };


    typedef struct
    {
        float raw_value;
        float filtered_value[4];
        arm_matrix_instance_f32 xhat, xhatminus, z, A, H, AT, HT, Q, R, p, Pminus, k;
    } kalman_filter_t;

    typedef struct
    {
        float raw_value;
        float filtered_value[4];
        float xhat_data[4], xhatminus_data[4], z_data[2], Pminus_data[16], K_data[8];
        float P_data[16];
        float AT_data[16], HT_data[8];
        float A_data[16];
        float H_data[8];
        float Q_data[16];
        float R_data[4];
    } kalman_filter_init_t;

    typedef struct
    {
        float X_last; //上一时刻的最优结果  X(k-|k-1)
        float X_mid;  //当前时刻的预测结果  X(k|k-1)
        float X_now;  //当前时刻的最优结果  X(k|k)
        float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
        float P_now;  //当前时刻最优结果的协方差  P(k|k)
        float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
        float kg;     // kalman增益
        float A;      //系统参数
        float B;
        float Q;
        float R;
        float H;
    } extKalman_t;

}



#endif

#endif // !__FILTER_ALG_HPP


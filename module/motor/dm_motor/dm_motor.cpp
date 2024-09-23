#include "dm_motor.hpp"
#include <string.h>
static uint8_t DM_motor_idx = 0; // register DM_motor_idx,是该文件的全局电机索引,在注册时使用


static DM_MOTORInstance DM_motor_instance[DM_MOTOR_CNT]; // 会在control任务中遍历该数组
static DM_motor_c *DM_Motor_Class[DM_MOTOR_CNT];
uint8_t rx_test;


#define RAD_2_DEGREE 57.2957795f    // 180/pi
using namespace Bsp_CAN_n;
using namespace Bsp_DWT_n;
/********函数声明********/

float int_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_int(float x_float, float x_min, float x_max, int bits);

/**
 * @brief 在can的回调函数中进行
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 * @note 返回数据处理由达妙控制协议可知
*/
static void DecodeDMMotor(uint8_t  *rxbuff, uint32_t rx_id);
DM_MOTORInstance *DMMotorInit(DM_MOTORConfig DM_config);
void DMMotorSetValue(DM_MOTORInstance *instance, float posi, float speed, float kp, float kd, float torque);
void DMMotorStatus(DM_MOTORInstance *instance, DM_Motor_Status_e status, CANInstance_t *CAN_instance);

/***********************/


void DM_motor_c::MotorInit(DM_MOTORConfig *config)
{
    DM_Motor_Class[DM_motor_idx] = this;
    config->hcan_init->can_module_callback = DecodeDMMotor;
    config->hcan_init->id = this;
    this->instance = DMMotorInit(*config);
    this->can_instance = Bsp_CAN_n::CANRegister(config->hcan_init);
}

// 设置达妙电机发送值
/***
 * @brief 设置电机发送值
 * @param instance 达妙电机实例
 * @param posi 位置设定, 单位为 °
 * @param speed 速度设定, 单位为 °/s
 * @param kp kp 参数设定
 * @param kd kd 参数设定
 * @param torque 力矩设定
 * @note 直接给对应模式下需要的数据就好, 其他给零, 具体入参控制方式请参考 调试助手使用说明书(达妙驱动)
*/
void DM_motor_c::MotorSetValue(float posi, float speed, float kp, float kd, float torque)
{
    DMMotorSetValue(this->instance, posi, speed, kp, kd, torque);
}

void DM_motor_c::MotorSetStatue(DM_Motor_Status_e status)
{
    DMMotorStatus(this->instance, status, this->can_instance);
}


/**
 * @brief 在can的回调函数中进行
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 * @note 返回数据处理由达妙控制协议可知
*/
static void DecodeDMMotor(uint8_t  *rxbuff, uint32_t rx_id)
{ 
    
    // 根据 int↔float, 需要先将接收值转为int型, 然后再转float
    DM_MOTORInstance *motor = NULL;
    uint8_t veri = 0;

    for(int i = 0; i < DM_motor_idx; i++)
    {
        if(rx_id == DM_Motor_Class[i]->can_instance->rx_id)
        {    
            motor =  DM_Motor_Class[i]->instance;
            DM_Motor_Class[i]->dt =  DWT_GetDeltaT(&DM_Motor_Class[i]->feed_cnt);
            veri = 1;
            break;
        }
    }
    if(veri == 0)
    {
        return;
    }
    int posi_int, v_int, t_int;
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    rx_test = rx_id;
    DM_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    measure->feed_back_ID = (rxbuff[0] & 0x0f);
    measure->error_type = (DM_Motor_Error_Type_e)(rxbuff[0] & (0xf0 >> 4)); 

    posi_int = (rxbuff[1] << 8) | (rxbuff[2]);
    v_int = (rxbuff[3]<<4)| (rxbuff[4]>>4);
    t_int = ((rxbuff[4]&0xF)<<8)| (rxbuff[5]);
    
    // if(motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    // {
    //     measure->position_rad = -int_to_float(posi_int, motor->builtin_para.posi_min, motor->builtin_para.posi_max, 16);
    //     measure->speed_rad = -int_to_float(v_int, motor->builtin_para.v_min, motor->builtin_para.v_max, 12);
    //     measure->torque = -int_to_float(t_int, motor->builtin_para.t_min, motor->builtin_para.t_max, 12);
    // }
    // else
    // {
        measure->position_rad = int_to_float(posi_int, motor->builtin_para.posi_min, motor->builtin_para.posi_max, 16);
        measure->speed_rad = int_to_float(v_int, motor->builtin_para.v_min, motor->builtin_para.v_max, 12);
        measure->torque = int_to_float(t_int, motor->builtin_para.t_min, motor->builtin_para.t_max, 12);
    // }

    measure->mos_temp = rxbuff[6];
    measure->motor_temp = rxbuff[7];

    measure->position_angle = measure->position_rad * RAD_2_DEGREE;
    measure->speed_angle = measure->speed_rad * RAD_2_DEGREE;
}


// 用于调整电机工作状态
/***
 * @brief 切换达妙电机状态, 具体参数读手册
 * @param instance  需要控制的达妙电机的实例
 * @param status    设置的状态
*/
void DMMotorStatus(DM_MOTORInstance *instance, DM_Motor_Status_e status, CANInstance_t *CAN_instance)
{
    // uint32_t Tx_MailBox;
    // CAN_TxHeaderTypeDef Txmessage;
    // Txmessage.StdId = instance->send_id;
    // Txmessage.IDE = CAN_ID_STD;
    // Txmessage.RTR = CAN_RTR_DATA;
    // Txmessage.DLC = 8;
    CAN_instance->tx_buff[0] = 0XFF;
    CAN_instance->tx_buff[1] = 0XFF;
    CAN_instance->tx_buff[2] = 0XFF;
    CAN_instance->tx_buff[3] = 0XFF;
    CAN_instance->tx_buff[4] = 0XFF;
    CAN_instance->tx_buff[5] = 0XFF;
    CAN_instance->tx_buff[6] = 0XFF;
    switch (status)
    {
        case DM_ENABLE:
            instance->stop_flag = DM_MOTOR_ENALBED;
            CAN_instance->tx_buff[7] = 0XFC;
            break;
        
        case DM_UNABLE:
            instance->stop_flag = DM_MOTOR_STOP;
            CAN_instance->tx_buff[7] = 0XFD;
            break;
            
        case DM_PORTECT_ZERO_POSITION:
            CAN_instance->tx_buff[7] = 0XFE;
            break;
            
        case DM_CLEAR_FAULT:
            CAN_instance->tx_buff[7] = 0XFB;
            break;
        default:
            break;
    }

    Bsp_CAN_n::CANTransmit(CAN_instance, 3);

    memset(CAN_instance->tx_buff, 0, 8);
}

// 电机初始化,返回一个电机实例
/**
 * @brief 达妙电机初始化
 * @param DM_config 初始设置参数
*/
DM_MOTORInstance *DMMotorInit(DM_MOTORConfig DM_config)
{
    if(DM_motor_idx >= DM_MOTOR_CNT)
        return NULL;

    DM_motor_instance[DM_motor_idx].builtin_para = DM_config.builtin_para;
    DM_motor_instance[DM_motor_idx].motor_mode = DM_config.motor_mode;
    DM_motor_instance[DM_motor_idx].motor_reverse_flag = DM_config.motor_reverse_flag;

    DM_motor_idx++;
    return &DM_motor_instance[DM_motor_idx - 1];
}


// 设置达妙电机发送值
/***
 * @brief 设置电机发送值
 * @param instance 达妙电机实例
 * @param posi 位置设定, 单位为 °
 * @param speed 速度设定, 单位为 °/s
 * @param kp kp 参数设定
 * @param kd kd 参数设定
 * @param torque 力矩设定
 * @note 直接给对应模式下需要的数据就好, 其他给零, 具体入参控制方式请参考 调试助手使用说明书(达妙驱动)
*/
void DMMotorSetValue(DM_MOTORInstance *instance, float posi, float speed, float kp, float kd, float torque)
{
    instance->send_datas.position_angle = posi;
    instance->send_datas.speed_angle = speed;
    instance->send_datas.kp = kp;
    instance->send_datas.kd = kd;
    instance->send_datas.torque = torque;
}

// 达妙电机控制实例
void DMMotorControl(void)
{
    // 根据 int↔float, 需要先将接收值转为 float 型, 然后再转 int
    int posi_int, v_int, kp_int, kd_int, t_int;
    uint8_t *tx_buffer;
    DM_MOTORInstance *motor_instance;
    // static DM_motor_c *motor_class;
    DM_Motor_Send_Data_s *send_data;
    uint8_t *pbuf, *vbuf;


    for(uint8_t i = 0; i < DM_motor_idx; i++)
    {
        motor_instance = &DM_motor_instance[i];
        // motor_class = DM_Motor_Class[i];
        send_data = &motor_instance->send_datas;
        tx_buffer = DM_Motor_Class[i]->can_instance->tx_buff;
        
        memset(tx_buffer, 0, sizeof(uint8_t) * 8);

        if(motor_instance->measure.error_type > 0X07 && motor_instance->measure.error_type < 0X0F)
        {
            DM_Motor_Class[i]->MotorSetStatue(DM_UNABLE);
            continue;
        }

        if(motor_instance->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            send_data->position_angle = -send_data->position_angle; 
            send_data->speed_angle = -send_data->speed_angle; 
            send_data->torque = -send_data->torque; 
        }

        send_data->position_rad = send_data->position_angle / RAD_2_DEGREE;
        send_data->speed_rad = send_data->speed_angle / RAD_2_DEGREE;

        switch(motor_instance->motor_mode)
        {
            case DM_MIT_MODE : 
            {
                
                posi_int = float_to_int(send_data->position_rad, motor_instance->builtin_para.posi_min, motor_instance->builtin_para.posi_max, 16);
                v_int = float_to_int(send_data->speed_rad, motor_instance->builtin_para.v_min, motor_instance->builtin_para.v_max, 12);
                t_int = float_to_int(send_data->torque, motor_instance->builtin_para.t_min, motor_instance->builtin_para.t_max, 12);
                kp_int = float_to_int(send_data->kp, motor_instance->builtin_para.kp_min, motor_instance->builtin_para.kp_max, 12);
                kd_int = float_to_int(send_data->kd, motor_instance->builtin_para.kd_min, motor_instance->builtin_para.kd_max, 12);
               tx_buffer[0] = (posi_int >> 8);
               tx_buffer[1] = posi_int;
               tx_buffer[2] = (v_int >> 4);
               tx_buffer[3] = ((v_int&0xF)<<4)|(kp_int>>8);
               tx_buffer[4] = kp_int;
               tx_buffer[5] = (kd_int >> 4);
               tx_buffer[6] = ((kd_int&0xF)<<4)|(t_int>>8);
               tx_buffer[7] = t_int;
                
                Bsp_CAN_n::CANTransmit(DM_Motor_Class[i]->can_instance, 2);
                break;
            }
            case DM_POSI_SPEED_MODE :
            {
                pbuf = (uint8_t *)&send_data->position_rad;
                vbuf = (uint8_t *)&send_data->speed_rad;
                tx_buffer[0] = *(pbuf);
                tx_buffer[1] = *(pbuf + 1);
                tx_buffer[2] = *(pbuf + 2);
                tx_buffer[3] = *(pbuf + 3);
                tx_buffer[4] = *(vbuf);
                tx_buffer[5] = *(vbuf + 1);
                tx_buffer[6] = *(vbuf + 2);
                tx_buffer[7] = *(vbuf + 3);
                
                Bsp_CAN_n::CANTransmit(DM_Motor_Class[i]->can_instance, 2);
                break;
            }
            case DM_SPEED_MODE:
            {
                vbuf = (uint8_t *)&send_data->speed_rad;
                tx_buffer[0] = *(vbuf);
                tx_buffer[1] = *(vbuf + 1);
                tx_buffer[2] = *(vbuf + 2);
                tx_buffer[3] = *(vbuf + 3);
             
                Bsp_CAN_n::CANTransmit(DM_Motor_Class[i]->can_instance, 2);
                break;
            }
            case DM_EMIT_MODE:
            {
                break;
            }

            default :
                break;
        }

    }

}


/**
 * @brief  uint转float
 * @param  x_int   需要转换的值
 * @param  x_min   最小值
 * @param  x_max   最大值
 * @param  bits    位数
 */
float int_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  uint转float
 * @param  x_float   需要转换的值
 * @param  x_min   最小值
 * @param  x_max   最大值
 * @param  bits    位数
 */
int float_to_int(float x_float, float x_min, float x_max, int bits)
{ 
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
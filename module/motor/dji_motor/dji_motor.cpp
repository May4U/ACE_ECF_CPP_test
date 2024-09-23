#include "dji_motor.hpp"
#include "bsp_dwt.hpp"
#include "user_maths.hpp"

//debug
uint32_t dji_rxid_1; 

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9

using namespace DJI_Motor_n;

/***函数声明***/
static void DecodeDJIMotor(uint8_t *data, uint32_t StdId);

int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);
/*************/

static DJI_Motor_n::DJI_Motor_Instance *dji_motor_instance_p[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算
static uint8_t dji_motor_idx = 0; // register idx,是该文件的全局电机索引,在注册时使用

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
CAN_TxHeaderTypeDef txconf_init =   {.StdId = 0x1ff, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08};
CAN_TxHeaderTypeDef txconf_init_1 = {.StdId = 0x200, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08};
CAN_TxHeaderTypeDef txconf_init_2 = {.StdId = 0x2ff, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08};
static Bsp_CAN_n::CANInstance_t sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf = txconf_init,    .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf = txconf_init_1,  .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf = txconf_init_2,  .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf = txconf_init,    .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf = txconf_init_1,  .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf = txconf_init_2,  .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief Dji 电机实例初始化
 * @param config 电机初始化结构体
 * @return none
*/
void DJI_Motor_n::DJI_Motor_Instance::DJIMotorInit(Motor_General_Def_n::Motor_Init_Config_s config)
{
    // motor basic setting 电机基本设置
    this->motor_type = config.motor_type;                         // 6020 or 2006 or 3508
    this->motor_settings = config.controller_setting_init_config; // 正反转,闭环类型等
    this->motor_controller.angle_PID.PidInit(config.controller_param_init_config.angle_PID);
    this->motor_controller.speed_PID.PidInit(config.controller_param_init_config.speed_PID);
    this->motor_controller.current_PID.PidInit(config.controller_param_init_config.current_PID);
    this->MotorMeasure.radius = config.radius;
    this->MotorMeasure.ecd2length = config.ecd2length;
    this->motor_can_instance->tx_id = config.can_init_config.tx_id;

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    config.can_init_config.rx_id = 0x200 + config.can_init_config.tx_id;
    // 注册电机到CAN总线
    config.can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config.can_init_config.id = this;                        // set id,eq to address(it is identity)
    dji_rxid_1 = config.can_init_config.rx_id;
    this->motor_can_instance = CANRegister(&config.can_init_config);
    this->MotorSenderGrouping();
    dji_motor_instance_p[dji_motor_idx] = this;
    dji_motor_idx++;
    switch(this->motor_type)
    {
        case Motor_General_Def_n::M3508:
        {
            this->MotorMeasure.lap_encoder =  8192;
            this->MotorMeasure.gear_Ratio = 19;
            break;
        }
        case Motor_General_Def_n::M2006:
        {
            this->MotorMeasure.lap_encoder =  8192;
            this->MotorMeasure.gear_Ratio = 36;
            break;
        }
        case Motor_General_Def_n::GM6020:
        {
            this->MotorMeasure.lap_encoder =  16384;
            this->MotorMeasure.gear_Ratio = 1;
            break;
        }
    
        default:
            break;
    }
    this->DJIMotorStop();
}

/***
 * @brief 设定闭环的参考值
 * @param ref 用作闭环计算大的参考值
 * @note ref 的值具体设置为什么看你在初始化设置的闭环环路, 以及最外环的数据来源 ActualValueSource
*/
void DJI_Motor_n::DJI_Motor_Instance::DJIMotorSetRef(float ref)
{
    if(this->stop_flag == Motor_General_Def_n::MOTOR_Lock || this->stop_flag == Motor_General_Def_n::MOTOR_STOP)
    {
        return;
    }
    if ((this->motor_settings.close_loop_type & Motor_General_Def_n::ANGLE_LOOP) && this->motor_settings.outer_loop_type == Motor_General_Def_n::ANGLE_LOOP)
    {   
       this->set_length = ref;
    }

        
    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((this->motor_settings.close_loop_type & Motor_General_Def_n::SPEED_LOOP) && (this->motor_settings.outer_loop_type &  Motor_General_Def_n::SPEED_LOOP))
    {
        this->set_speed = ref;
    }
        
    this->motor_controller.RefValChange(ref);
}




void DJI_Motor_n::DJI_Motor_Instance::MotorSenderGrouping()
{
    uint8_t motor_id = this->motor_can_instance->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (this->motor_type)
    {
        case Motor_General_Def_n::M3508:
        case Motor_General_Def_n::M2006:
        {
            if (motor_id < 4) // 根据ID分组
            {
                motor_send_num = motor_id;
                motor_grouping =  this->motor_can_instance->can_handle == &hcan1 ? 1 : 4;
            }
            else
            {
                motor_send_num = motor_id - 4;
                motor_grouping = this->motor_can_instance->can_handle == &hcan1 ? 0 : 3;
            }
        // 计算接收id并设置分组发送id
            this->motor_can_instance->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
            dji_rxid_1 = this->motor_can_instance->rx_id;
            sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
            this->message_num = motor_send_num;
            this->sender_group = motor_grouping;
        // 检查是否发生id冲突
            for (size_t i = 0; i < dji_motor_idx; ++i)
            {
                if (dji_motor_instance_p[i]->motor_can_instance->can_handle == this->motor_can_instance->can_handle && dji_motor_instance_p[i]->motor_can_instance->rx_id == this->motor_can_instance->rx_id)
                {
                    while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                    {}
                }
            }

            break;
        }
        case Motor_General_Def_n::GM6020:
        {
            if (motor_id < 4)
            {
                motor_send_num = motor_id;
                motor_grouping = this->motor_can_instance->can_handle == &hcan1 ? 0 : 3;
            }
            else
            {
                motor_send_num = motor_id - 4;
                motor_grouping = this->motor_can_instance->can_handle == &hcan1 ? 2 : 5;
            }
            this->motor_can_instance->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
            sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
            this->message_num = motor_send_num;
            this->sender_group = motor_grouping;
            for (size_t i = 0; i < dji_motor_idx; ++i)
            {
                if (dji_motor_instance_p[i]->motor_can_instance->can_handle == this->motor_can_instance->can_handle && dji_motor_instance_p[i]->motor_can_instance->rx_id == this->motor_can_instance->rx_id)
                {
                    while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                    {}
                }
            }
            break;
        }
    
        default:
                    while (1) // 其他类型电机不适用于该初始化
                    {}
            break;
    }
}
/**
 * @brief DJI 电机使能
 * @note 默认为使能
*/
void DJI_Motor_n::DJI_Motor_Instance::DJIMotorEnable(void)
{
    this->stop_flag = Motor_General_Def_n::MOTOR_ENALBED;
}
/**
 * @brief 电机堵转初始化
 * @param init_current 堵转初始化设置电流, 注意方向
 * @return 如果初始化完成, 则返回 0;
 * @note 为了方便可能的重复初始化, 因此在初始化判断完成, 返回 1 之后, 再次调用会重新开始初始化进程
 * @note 因此, 最好在外部额外用一个变量, 判断是否完成初始化, 初始化完成, 后不再继续调用该函数 
*/
uint8_t DJI_Motor_Instance::DJIMotorBlockInit(int16_t init_current)
{
        switch (this->motor_type)
        {
            case Motor_General_Def_n::GM6020:
            case Motor_General_Def_n::M3508:
            {
                if(init_current > 16384 )
                    init_current = (int16_t)16384;
                else if(init_current < -16384)
                    init_current = (int16_t)-16384;
                
                break;
            }
            case Motor_General_Def_n::M2006:
            {

                if(init_current > 10000 )
                    init_current = (int16_t)10000;
                else if(init_current < -10000)
                    init_current = (int16_t)-10000;
                
                break;
            }
            
            default:
                break;
        }

    this->block_val.init_current = init_current;
    if(this->block_val.block_init_if_finish)
    {  
        this->block_val.block_init_if_finish = 0;
        this->block_val.last_position = 0;
        this->block_val.last_position = 0;
        return 1;
    }
    this->stop_flag = Motor_General_Def_n::MOTOR_INIT;
    return 0;
}

/**
 * @brief DJI 电机停止
 * @note 具体实现为在发送时对应数据发0
*/
void DJI_Motor_n::DJI_Motor_Instance::DJIMotorStop(void)
{
    this->stop_flag = Motor_General_Def_n::MOTOR_STOP;
}

/**
 * @brief Dji 电机锁死
 * @note 具体实现为不允许更改闭环运算参考值
*/
void DJI_Motor_n::DJI_Motor_Instance::DJIMotorLock(void)
{
    this->stop_flag = Motor_General_Def_n::MOTOR_Lock;
}

/**
 * @brief 更新 dt 值
 * @param new_dt 新的dt值
 * @return none
*/
void DJI_Motor_n::DJI_Motor_Instance::Update_DJIMotor_dt(float new_dt)
{
    this->dt = new_dt;
}

/**
 * @brief 获取当前 dt 值
 * @return 当前 dt 值
*/
float DJI_Motor_n::DJI_Motor_Instance::Get_DJIMotor_dt(void)
{
    return this->dt;
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(uint8_t *data, uint32_t StdId)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t current_idx;
    uint8_t *rxbuff;
    if(StdId > 0x200 && StdId < 0x20C)
    {

    }
    else
    {
        return;
    }
    for(uint8_t i = 0; i < dji_motor_idx; i++)
    {
        if(dji_motor_instance_p[i]->motor_can_instance->rx_id == StdId)
        {
            rxbuff = data;
            current_idx = i;
            DJI_Motor_n::DJI_Motor_Instance *motor = dji_motor_instance_p[current_idx];
            DJI_Motor_n::DJIMotorMeasure_c *measure = &motor->MotorMeasure; // measure要多次使用,保存指针减小访存开销
            motor->Update_DJIMotor_dt(Bsp_DWT_n::DWT_GetDeltaT(&motor->feed_cnt));
            // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
            measure->measure.last_ecd = measure->measure.feedback_ecd;
            measure->measure.feedback_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];

            if(measure->measure.init_flag == false)
            {
                measure->measure.init_flag = true;
                measure->measure.last_ecd = measure->measure.init_ecd = measure->measure.feedback_speed;
                measure->measure.record_ecd = 0;
                measure->measure.last_record_ecd = measure->measure.record_ecd;
                motor->DJIMotorEnable();
            }
            measure->measure.angle_single_round = (float)measure->measure.feedback_ecd / measure->lap_encoder * 360.0;
            measure->measure.feedback_speed = (float)(int16_t)(rxbuff[2] << 8 | rxbuff[3]);

            measure->measure.speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->measure.speed_aps +
                                RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF *  measure->measure.feedback_speed;
            measure->measure.feedback_real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->measure.feedback_real_current +
                                    CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
            measure->measure.feedback_temperature = rxbuff[6];

            // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
            volatile int erro = measure->measure.feedback_ecd - measure->measure.last_ecd;
            if (erro > (measure->lap_encoder / 2.0f))
            {
                measure->measure.total_round--;
            }
            else if (erro< -(measure->lap_encoder / 2.0f) )
            {
                measure->measure.total_round++;
            }
            measure->measure.last_record_ecd = measure->measure.record_ecd;
            erro = angle_limiting_int16(erro, measure->lap_encoder);
            
            measure->measure.record_ecd += erro;

            measure->measure.total_round  = measure->measure.record_ecd / measure->lap_encoder + measure->measure.angle_single_round / 360.0f;
            measure->measure.total_angle = measure->measure.total_round * 360.0f;
            measure->measure.record_length = measure->measure.record_ecd / measure->ecd2length;
            measure->measure.angular_speed = measure->measure.speed_aps / measure->gear_Ratio;
            measure->measure.linear_speed = measure->measure.angular_speed * measure->radius;
            break;
        }
    }


}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJI_Motor_n::DJI_Motor_Instance *motor;
    Motor_General_Def_n::Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_General_Def_n::Motor_Controller_c *motor_controller;   // 电机控制器
    // DJI_Motor_n::DJIMotorMeasure_c *measure;           // 电机测量值
    // float pid_measure;
    float pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < dji_motor_idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance_p[i];
        motor_setting = &dji_motor_instance_p[i]->motor_settings;
        motor_controller = &dji_motor_instance_p[i]->motor_controller;
        pid_ref = motor_controller->GetRefVal(); // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        
        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == Motor_General_Def_n::MOTOR_STOP)
        {
            memset(sender_assignment[group].tx_buff + 2 * num, 0, 16u);
            continue;
        }

        if (motor->stop_flag == Motor_General_Def_n::MOTOR_INIT)
        {
            motor->DJIMotorBlockInitAchieve();
            set = motor->block_val.init_current;
            sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位
            motor->give_current = set;
            continue;
        }

        if (motor_setting->motor_reverse_flag == Motor_General_Def_n::MOTOR_DIRECTION_REVERSE)
            pid_ref = -pid_ref; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & Motor_General_Def_n::ANGLE_LOOP) && motor_setting->outer_loop_type == Motor_General_Def_n::ANGLE_LOOP)
        {   
            // 更新pid_ref进入下一个环
            pid_ref = motor_controller->angle_PID.PidCalculate(pid_ref);
        }

        
        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & Motor_General_Def_n::SPEED_LOOP) && (motor_setting->outer_loop_type & (Motor_General_Def_n::ANGLE_LOOP | Motor_General_Def_n::SPEED_LOOP)))
        {
            // if (motor_setting->feedforward_flag & Motor_General_Def_n::SPEED_FEEDFORWARD)
            //     pid_ref += *motor_controller->speed_feedforward_ptr;

            // 更新pid_ref进入下一个环
            pid_ref = motor_controller->speed_PID.PidCalculate( pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        // if (motor_setting->feedforward_flag & Motor_General_Def_n::CURRENT_FEEDFORWARD)
        //     pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & Motor_General_Def_n::CURRENT_LOOP)
        {
            pid_ref = motor_controller->speed_PID.PidCalculate(pid_ref);
        }
        
        // 获取最终输出

        set = (int16_t)pid_ref;

        switch (motor->motor_type)
        {
            case Motor_General_Def_n::GM6020:
            case Motor_General_Def_n::M3508:
            {
                if(set > 16384 )
                    set = (int16_t)16384;
                else if(set < -16384)
                    set = (int16_t)-16384;
                
                break;
            }
            case Motor_General_Def_n::M2006:
            {

                if(set > 10000 )
                    set = (int16_t)10000;
                else if(set < -10000)
                    set = (int16_t)-10000;
                
                break;
            }
            
            default:
                break;
        }
        motor->give_current = set;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

    }
    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        if (sender_enable_flag[i])
        {
            Bsp_CAN_n::CANTransmit(&sender_assignment[i], 1);
        }
    }
}

void DJI_Motor_Instance::DJIMotorBlockInitAchieve()
{
    if(this->stop_flag == Motor_General_Def_n::MOTOR_INIT)
    {
        if(this->block_val.block_init_if_finish)
        {
            this->MotorMeasure.MeasureClear();
            this->DJIMotorEnable();
            return;
        }
        if(this->MotorMeasure.measure.init_flag)
        {
            if(user_abs(this->block_val.last_position - this->MotorMeasure.measure.record_ecd) < 20)
            {
                this->block_val.block_times++;
            }
            else
            {
                this->block_val.block_times = 0;
            }

            if(this->block_val.block_times > 400)
            {
                this->block_val.block_init_if_finish = 1;
            }
        }
        this->block_val.last_position =  this->MotorMeasure.measure.record_ecd;
    }
}

/***
 * @brief 清空measure 结构体中的内容 
*/
void DJIMotorMeasure_c::MeasureClear()
{
    memset(&this->measure, 0, sizeof(DjiMotorMeasure_t));
    this->measure.init_flag = false;
}









//临角处理16位（对应角度正值）
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder) {
   //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
   if (Angl_Err < -(lap_encoder / 2))
   {
       Angl_Err += (lap_encoder - 1);
   }
   if (Angl_Err > (lap_encoder / 2)) {
       Angl_Err -= (lap_encoder - 1);
   }
   return Angl_Err;
}





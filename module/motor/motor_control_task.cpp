#include "dm_motor.hpp"
#include "dji_motor.hpp"
#include "motor_control_task.h"


void Motor_TASK(void const * argument)
{
    osDelay(200);
    while (1)
    {
        DJIMotorControl();
        DMMotorControl();
        osDelay(1);
    }
}
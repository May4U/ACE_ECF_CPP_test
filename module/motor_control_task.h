#ifndef __MOTOR_CONTROL_TASK_H
#define __MOTOR_CONTROL_TASK_H


#ifdef __cplusplus   //这里是两个下滑线
extern "C"{
#endif

void Motor_TASK(void const * argument);
void EngineChassisBoardCtrl_Task(void const *argument);
void Safe_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif // !__MOTOR_CONTROL_TASK_H

#include "Motor_General_def.hpp"

using namespace Motor_General_Def_n;

void Motor_Controller_c::RefValChange(float ref_val)
{
    this->pid_ref = ref_val;
}

float Motor_Controller_c::GetRefVal(void)
{
    return this->pid_ref;
}
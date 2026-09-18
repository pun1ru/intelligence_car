#include "pid.h"
#include "math.h"

void PID_Init(PIDStruct * const pid,float kp,float ki,float kd,float lowpass,float sum_error_max,float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->lowpass = lowpass;
    pid->sum_error_max = sum_error_max;
    pid->output_max = output_max;
    PID_Refresh(pid);
}
void PID_Refresh(PIDStruct * const pid)
{
    pid->cur_error = 0;
    pid->last_error = 0;
    pid->delta_error = 0;
    pid->last_dif_error = 0;
    pid->dif_error = 0;
    pid->sum_error = 0;
    pid->output = 0;
}
float PID_Update(PIDStruct *const pid,float const error){
    pid->cur_error= error;
    if(fabs(pid->sum_error)-fabs(pid->sum_error_max)<0.0001f)
        pid->sum_error+= pid->cur_error;
    pid->sum_error=limiter(pid->sum_error,pid->sum_error_max);
    pid->delta_error = pid->cur_error - pid->last_error;
    pid->dif_error = pid->delta_error*(1-pid->lowpass)+pid->last_dif_error*pid->lowpass;
    pid->output = pid->kp*pid->cur_error+\
                  pid->ki*pid->sum_error+\
                  pid->kd*pid->dif_error ;
    pid->output = limiter(pid->output,pid->output_max);
    pid->last_error = pid->cur_error;
    pid->last_dif_error = pid->dif_error;
    return pid->output;

}   
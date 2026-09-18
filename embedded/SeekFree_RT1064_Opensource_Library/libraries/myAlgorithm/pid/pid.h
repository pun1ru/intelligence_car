#ifndef PID_H
#define PID_H

#include "stdint.h"

#define limiter(val,max) (((val)<-(max))?-(max):((val)>(max)?(max):(val)))

typedef struct{
    float kp,ki,kd;
    float cur_error,last_error,sum_error,delta_error,last_dif_error,dif_error;
    float lowpass;
    float output;
    float sum_error_max,output_max;
}PIDStruct;

void PID_Init(PIDStruct * const pid,float kp,float ki,float kd,float lowpass,float sum_error_max,float output_max);
void PID_Refresh(PIDStruct * const pid);
float PID_Update(PIDStruct * const pid,float const error);

#endif
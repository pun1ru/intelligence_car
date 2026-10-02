#include "control_pid.h"

#include <math.h>
#include <stddef.h>

static float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void control_pid_init(control_pid_t *pid, float kp, float ki, float kd,
                      float output_limit)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral = 0.0f;
}

void control_pid_reset(control_pid_t *pid)
{
    if (pid != NULL)
    {
        pid->integral = 0.0f;
    }
}

float control_pid_step(control_pid_t *pid, float setpoint, float measurement,
                       float measurement_rate, float dt_s)
{
    float error;
    float candidate_integral;
    float candidate_output;

    if ((pid == NULL) || (dt_s <= 0.0f) || (pid->output_limit <= 0.0f) ||
        !isfinite(setpoint) || !isfinite(measurement) ||
        !isfinite(measurement_rate) || !isfinite(dt_s))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    candidate_integral = (pid->ki == 0.0f) ? pid->integral :
                         pid->integral + error * dt_s;
    if (pid->ki != 0.0f)
    {
        candidate_integral = clamp(candidate_integral,
                                   pid->output_limit / fabsf(pid->ki));
    }
    candidate_output = pid->kp * error + pid->ki * candidate_integral -
                       pid->kd * measurement_rate;
    if ((candidate_output <= pid->output_limit || error < 0.0f) &&
        (candidate_output >= -pid->output_limit || error > 0.0f))
    {
        pid->integral = candidate_integral;
    }
    return clamp(pid->kp * error + pid->ki * pid->integral -
                 pid->kd * measurement_rate, pid->output_limit);
}

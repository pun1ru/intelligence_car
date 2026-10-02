#ifndef CONTROL_PID_H
#define CONTROL_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float output_limit;
} control_pid_t;

void control_pid_init(control_pid_t *pid, float kp, float ki, float kd,
                      float output_limit);
void control_pid_reset(control_pid_t *pid);
float control_pid_step(control_pid_t *pid, float setpoint, float measurement,
                       float measurement_rate, float dt_s);

#endif

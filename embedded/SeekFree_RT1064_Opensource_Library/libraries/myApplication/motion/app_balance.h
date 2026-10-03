#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include <stdint.h>
#include "control_pid.h"

typedef struct
{
    control_pid_t wheel_left;
    control_pid_t wheel_right;
    control_pid_t angle;
    control_pid_t speed;
    float base_pitch_deg;
    float target_tilt_deg;
    uint32_t speed_elapsed_ms;
} app_balance_t;

typedef struct
{
    int32_t left_pwm;
    int32_t right_pwm;
    float target_tilt_deg;
} app_balance_output_t;

uint8_t app_balance_update_pid(app_balance_t *control, uint8_t controller,
                               uint8_t parameter, float value);
void app_balance_set_pitch_target(app_balance_t *control, float target_pitch_deg);

void app_balance_init(app_balance_t *control);
void app_balance_reset(app_balance_t *control);
void app_balance_wheel_speed_step(app_balance_t *control,
                                  float left_target_m_s, float right_target_m_s,
                                  float left_speed_m_s, float right_speed_m_s,
                                  app_balance_output_t *output);
void app_balance_angle_step(app_balance_t *control, uint8_t enabled,
                            float target_pitch_deg, float pitch_deg,
                            float pitch_rate_dps,
                            app_balance_output_t *output);
void app_balance_step(app_balance_t *control, uint8_t enabled,
                      float target_speed_m_s, float pitch_deg,
                      float pitch_rate_dps, float left_speed_m_s,
                      float right_speed_m_s, app_balance_output_t *output);
void app_balance_apply(const app_balance_output_t *output);

#endif

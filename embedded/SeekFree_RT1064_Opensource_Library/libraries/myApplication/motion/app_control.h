#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "app_attitude.h"
#include "app_state.h"

typedef struct
{
    float left_speed_m_s;
    float right_speed_m_s;
    int16_t left_delta_count;
    int16_t right_delta_count;
    int16_t left_pwm;
    int16_t right_pwm;
    float target_pitch_deg;
    uint8_t wheel_valid;
} app_control_telemetry_t;

uint8_t app_control_step(const app_state_t *state,
                         const app_attitude_t *attitude);
uint8_t app_control_apply_pid_update(uint8_t controller, uint8_t parameter,
                                     float value);
void app_control_get_telemetry(app_control_telemetry_t *telemetry);

#endif

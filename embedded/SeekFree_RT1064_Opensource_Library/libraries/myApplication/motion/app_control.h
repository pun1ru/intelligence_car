#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "app_attitude.h"
#include "app_nav.h"
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
    /* Set when encoder speed is abnormal while a meaningful motor command is
     * active. This is diagnostic telemetry only and does not force protection. */
    uint8_t lift_detected;
    /* 本周期控制环实际跟随的目标值：没有跟踪指令时就是状态机里的值，
     * 有跟踪指令时是路径跟踪算出来的。1 = 这两个值来自路径跟踪。 */
    uint8_t nav_active;
    float effective_speed_m_s;
    float effective_yaw_deg;
} app_control_telemetry_t;

/* 每个控制周期调用一次。nav 为 NULL 或 nav->active 为 0 时按状态机的目标值控制；
 * 只有在导航状态且 nav->active 为 1 时才改用路径跟踪的目标（同时强制打开转向环）。
 * nav 指向的快照由 decision_task 发布，调用方负责保证其新鲜。 */
uint8_t app_control_step(const app_state_t *state,
                         const app_attitude_t *attitude,
                         const app_nav_command_t *nav);
uint8_t app_control_apply_pid_update(uint8_t controller, uint8_t parameter,
                                     float value);
void app_control_apply_feedforward(float value);
void app_control_apply_steering_feedforward(float value);
void app_control_set_steering_enabled(uint8_t enabled);
void app_control_get_telemetry(app_control_telemetry_t *telemetry);

#endif

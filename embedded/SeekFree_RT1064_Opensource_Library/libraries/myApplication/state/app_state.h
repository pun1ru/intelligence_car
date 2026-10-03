#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include "app_serial.h"

typedef enum
{
    APP_STATE_BUTTON_DEBUG,
    APP_STATE_SERIAL_DEBUG,
    APP_STATE_CALIBRATION
} app_control_mode_t;

typedef enum
{
    APP_VEHICLE_PROTECT,
    APP_VEHICLE_BALANCE,
    APP_VEHICLE_NAVIGATION,
    APP_VEHICLE_SUPPORT
} app_vehicle_mode_t;

typedef struct
{
    uint8_t c12_pressed;
    uint8_t c13_pressed;
    uint8_t c14_pressed;
    uint8_t c15_pressed;
    uint8_t switches_on;
    uint8_t switches_off;
} app_state_buttons_t;

typedef struct
{
    app_control_mode_t control_mode;
    app_vehicle_mode_t vehicle_mode;
    float target_speed_m_s;
    float target_yaw_deg;
    uint32_t serial_idle_ms;
    uint32_t button_age_ms[4];
    uint8_t button_raw[4];
    uint8_t button_stable[4];
    uint8_t calibrate_requested;
    uint8_t reset_ekf_requested;
    uint8_t pid_update_requested;
    uint8_t pid_controller;
    uint8_t pid_parameter;
    float pid_value;
    float wheel_feedforward_gain;
    float steering_feedforward_gain;
    uint8_t steering_enabled;
    uint8_t gyro_bias_calibrate_requested;
    uint32_t support_pitch_age_ms;
    uint32_t support_exit_age_ms;
    uint32_t balance_entry_kick_age_ms;
    uint8_t balance_entry_kick_active;
} app_state_t;

void app_state_init(app_state_t *state);
void app_state_read_buttons(app_state_buttons_t *buttons);
void app_state_step(app_state_t *state, const app_state_buttons_t *buttons,
                    uint8_t attitude_valid, float pitch_deg, uint32_t dt_ms,
                    const app_serial_command_t *command);
uint8_t app_state_balance_enabled(const app_state_t *state);
void app_state_force_protection(app_state_t *state);

#endif

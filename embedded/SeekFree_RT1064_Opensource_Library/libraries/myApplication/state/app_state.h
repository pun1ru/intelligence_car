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
    APP_VEHICLE_NAVIGATION
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
} app_state_t;

void app_state_init(app_state_t *state);
void app_state_read_buttons(app_state_buttons_t *buttons);
void app_state_step(app_state_t *state, const app_state_buttons_t *buttons,
                    uint8_t attitude_valid, float pitch_deg, uint32_t dt_ms,
                    const app_serial_command_t *command);
uint8_t app_state_balance_enabled(const app_state_t *state);
void app_state_force_protection(app_state_t *state);

#endif

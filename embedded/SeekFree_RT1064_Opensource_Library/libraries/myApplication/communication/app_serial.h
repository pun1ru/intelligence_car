#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>
#include "general_define.h"

typedef struct
{
    uint8_t bytes[SERIAL_FRAME_LENGTH];
    uint8_t length;
} app_serial_parser_t;

typedef struct
{
    uint8_t type;
    int16_t value;
    int16_t value2;
    uint8_t pid_controller;
    uint8_t pid_parameter;
    float pid_value;
} app_serial_command_t;

typedef struct
{
    uint8_t valid;
    uint8_t control_mode;
    uint8_t vehicle_mode;
    uint32_t uptime_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float pitch_rate_dps;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float left_speed_m_s;
    float right_speed_m_s;
    int16_t left_delta_count;
    int16_t right_delta_count;
    int16_t left_pwm;
    int16_t right_pwm;
    float target_speed_m_s;
    float target_yaw_deg;
    float target_pitch_deg;
} app_serial_telemetry_t;

void app_serial_init(void);
void app_serial_send(const app_serial_telemetry_t *telemetry);
uint8_t app_serial_encode_telemetry(const app_serial_telemetry_t *telemetry,
                                    uint8_t sequence,
                                    uint8_t frame[SERIAL_TELEMETRY_LENGTH]);
void app_serial_parser_reset(app_serial_parser_t *parser);
uint8_t app_serial_feed_byte(app_serial_parser_t *parser, uint8_t byte,
                             app_serial_command_t *command);
uint8_t app_serial_poll_command(app_serial_command_t *command);

#endif

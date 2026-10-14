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
    float feedforward_gain;
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

typedef struct
{
    uint8_t active;
    uint8_t complete;
    uint32_t sample_count;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
} app_serial_calibration_t;

/* 导航位姿遥测的内容。真正上线时按 SERIAL_NAV_POSE_LENGTH 编码，帧格式见
 * docs/SERIAL_PROTOCOL.md 的 0x82 帧。单位都是浮点原值，编码函数内部换算。 */
typedef struct
{
    uint8_t valid;          /* SERIAL_POSE_VALID_* 位：bit0 位姿可信、bit1 在导航中 */
    float x_m;              /* 相对原点前后位移，+X 向前，m */
    float y_m;              /* 相对原点左右位移，+Y 向左，m */
    float heading_deg;      /* 相对原点航向偏角，deg */
    float path_length_m;    /* 自原点起的累计行程，m */
    float yaw_rate_rad_s;   /* 编码器推算的车体角速度，rad/s */
} app_serial_pose_t;

void app_serial_init(void);
void app_serial_send(const app_serial_telemetry_t *telemetry);
void app_serial_send_calibration(const app_serial_calibration_t *calibration);
/* 发送一帧导航位姿（0x82）。序号与遥测/校准帧共用，逐帧加一。 */
void app_serial_send_pose(const app_serial_pose_t *pose);
uint8_t app_serial_encode_calibration(
    const app_serial_calibration_t *calibration, uint8_t sequence,
    uint8_t frame[SERIAL_CALIBRATION_LENGTH]);
/* 只填帧不发送，便于用主机侧测试核对线上字节（见 tests/host/pose_frame_dump.c）。
 * 成功返回 1，参数为空返回 0。 */
uint8_t app_serial_encode_pose(const app_serial_pose_t *pose, uint8_t sequence,
                               uint8_t frame[SERIAL_NAV_POSE_LENGTH]);
uint8_t app_serial_encode_telemetry(const app_serial_telemetry_t *telemetry,
                                    uint8_t sequence,
                                    uint8_t frame[SERIAL_TELEMETRY_LENGTH]);
void app_serial_parser_reset(app_serial_parser_t *parser);
uint8_t app_serial_feed_byte(app_serial_parser_t *parser, uint8_t byte,
                             app_serial_command_t *command);
uint8_t app_serial_poll_command(app_serial_command_t *command);

#endif

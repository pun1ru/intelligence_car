#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdint.h>

typedef struct
{
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    uint32_t sample_count;
    uint32_t capture_tick;
} app_imu_sample_t;

typedef struct
{
    int16_t left_count;
    int16_t right_count;
} app_encoder_sample_t;

void app_sensor_init(void);
uint8_t app_sensor_imu_ready(void);
uint8_t app_sensor_read_imu(app_imu_sample_t *sample);
void app_sensor_read_encoder(app_encoder_sample_t *sample);

#endif

#ifndef RECEIVE_TASK_H_
#define RECEIVE_TASK_H_

#include <stdint.h>

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    uint32_t sample_count;
    uint8_t imu_ready;
} imu_receive_data_t;

extern volatile imu_receive_data_t g_imu_receive_data;

#endif

#ifndef RECEIVE_TASK_H_
#define RECEIVE_TASK_H_

#include <stdint.h>
#include "semphr.h"

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
    float quaternion[4];
    float roll;
    float pitch;
    float yaw;
} imu_receive_data_t;

typedef struct
{
    uint8_t active;
    uint8_t complete;
    uint32_t sample_count;
    uint32_t bias_sample_count;
    uint32_t noise_sample_count;
    float gyro_bias_dps[3];
    float gyro_noise_variance_dps2[3];
    float gyro_noise_stddev_dps[3];
} imu_gyro_calibration_t;

typedef struct
{
    uint8_t active;
    uint8_t complete;
    uint32_t sample_count;
    float mean_norm[3];
    float noise_variance_norm2[3];
    float noise_stddev_norm[3];
} imu_accel_calibration_t;

extern volatile imu_receive_data_t g_imu_receive_data;
extern SemaphoreHandle_t g_imu_data_ready_sem;
extern volatile imu_gyro_calibration_t g_imu_gyro_calibration;
extern volatile imu_accel_calibration_t g_imu_accel_calibration;

void imu_gyro_noise_calibrate(void);
void imu_accel_noise_calibrate(void);

#endif

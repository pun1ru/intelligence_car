#ifndef APP_CALIBRATION_H
#define APP_CALIBRATION_H

#include <stdint.h>
#include "app_sensor.h"

typedef struct
{
    uint8_t active;
    uint8_t complete;
    uint32_t sample_count;
    float mean_norm[3];
    float noise_variance_norm2[3];
    float noise_stddev_norm[3];
} app_accel_calibration_result_t;

typedef struct
{
    uint8_t active;
    uint8_t complete;
    uint32_t bias_sample_count;
    uint32_t noise_sample_count;
    float gyro_bias_dps[3];
    float gyro_noise_variance_dps2[3];
    float gyro_noise_stddev_dps[3];
} app_gyro_calibration_result_t;

void app_calibration_accel_start(void);
void app_calibration_accel_sample(const app_imu_sample_t *sample);
void app_calibration_accel_complete(void);
/* Synchronize in the calling task if calibration may update concurrently. */
void app_calibration_accel_result(app_accel_calibration_result_t *result);
void app_calibration_gyro_start(void);
void app_calibration_gyro_bias_sample(const app_imu_sample_t *sample);
void app_calibration_gyro_noise_sample(const app_imu_sample_t *sample);
void app_calibration_gyro_complete(void);
void app_calibration_gyro_result(app_gyro_calibration_result_t *result);

#endif

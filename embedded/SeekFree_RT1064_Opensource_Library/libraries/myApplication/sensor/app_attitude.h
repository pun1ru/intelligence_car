#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stddef.h>
#include "app_sensor.h"
#include "app_calibration.h"

typedef struct
{
    float quaternion[4];
    float roll;
    float pitch;
    float yaw;
    float pitch_rate_dps;
    float yaw_rate_dps;
} app_attitude_t;

void app_attitude_init(void *(*allocate)(size_t));
uint8_t app_attitude_step(const app_imu_sample_t *sample, float dt_s,
                          app_attitude_t *attitude);
void app_attitude_reset(void);
void app_attitude_apply_calibration(const app_gyro_calibration_result_t *gyro,
                                    const app_accel_calibration_result_t *accel);
void app_attitude_apply_gyro_calibration(
    const app_gyro_calibration_result_t *gyro);

#endif

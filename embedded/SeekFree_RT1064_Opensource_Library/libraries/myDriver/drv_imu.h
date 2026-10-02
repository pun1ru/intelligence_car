#ifndef DRV_IMU_H
#define DRV_IMU_H

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
} drv_imu_sample_t;

uint8_t drv_imu_init(void);
void drv_imu_read(drv_imu_sample_t *sample);

#endif

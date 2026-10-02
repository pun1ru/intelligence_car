#include <math.h>
#include <stdio.h>

#include "app_calibration.h"
#include "running_stats.h"

static int near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

int main(void)
{
    running_stats3_t stats;
    float sample_a[3] = {1.0f, 2.0f, 3.0f};
    float sample_b[3] = {3.0f, 4.0f, 5.0f};
    float variance[3];
    app_imu_sample_t imu = {0};
    app_accel_calibration_result_t accel;
    app_gyro_calibration_result_t gyro;

    running_stats3_reset(&stats);
    running_stats3_add(&stats, sample_a);
    running_stats3_add(&stats, sample_b);
    running_stats3_variance(&stats, variance);
    if ((stats.count != 2U) || !near(stats.mean[0], 2.0f) ||
        !near(stats.mean[1], 3.0f) || !near(stats.mean[2], 4.0f) ||
        !near(variance[0], 2.0f) || !near(variance[2], 2.0f))
    {
        puts("running statistics failed");
        return 1;
    }

    app_calibration_accel_start();
    app_calibration_accel_sample(&imu);
    imu.acc_z_g = 1.0f;
    app_calibration_accel_sample(&imu);
    app_calibration_accel_sample(&imu);
    app_calibration_accel_complete();
    app_calibration_accel_result(&accel);
    if ((accel.complete == 0U) || (accel.sample_count != 2U) ||
        !near(accel.mean_norm[2], 1.0f) ||
        !near(accel.noise_variance_norm2[2], 0.0f))
    {
        puts("accelerometer calibration failed");
        return 1;
    }

    app_calibration_gyro_start();
    imu.gyro_x_dps = 1.0f;
    imu.gyro_y_dps = 2.0f;
    imu.gyro_z_dps = 3.0f;
    app_calibration_gyro_bias_sample(&imu);
    imu.gyro_x_dps = 3.0f;
    imu.gyro_y_dps = 4.0f;
    imu.gyro_z_dps = 5.0f;
    app_calibration_gyro_bias_sample(&imu);
    app_calibration_gyro_noise_sample(&imu);
    imu.gyro_x_dps = 5.0f;
    imu.gyro_y_dps = 6.0f;
    imu.gyro_z_dps = 7.0f;
    app_calibration_gyro_noise_sample(&imu);
    app_calibration_gyro_complete();
    app_calibration_gyro_result(&gyro);
    if ((gyro.complete == 0U) || (gyro.bias_sample_count != 2U) ||
        (gyro.noise_sample_count != 2U) ||
        !near(gyro.gyro_bias_dps[0], 2.0f) ||
        !near(gyro.gyro_noise_variance_dps2[0], 2.0f))
    {
        puts("gyroscope calibration failed");
        return 1;
    }

    puts("calibration tests passed");
    return 0;
}

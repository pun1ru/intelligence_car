#include <math.h>
#include <stdio.h>

#include "app_attitude.h"

static int settle_at(app_imu_sample_t *sample, float expected_pitch,
                     app_attitude_t *attitude)
{
    unsigned int i;

    for (i = 0U; i < 10000U; i++)
    {
        if (app_attitude_step(sample, 0.001f, attitude) == 0U)
        {
            return 0;
        }
    }
    return fabsf(attitude->pitch - expected_pitch) < 1.0f &&
           fabsf(attitude->roll) < 1.0f &&
           fabsf(attitude->yaw) < 1.0f;
}

int main(void)
{
    app_imu_sample_t sample = {0};
    app_attitude_t attitude = {0};
    app_gyro_calibration_result_t gyro = {0};
    app_accel_calibration_result_t accel = {0};
    float upright_pitch;
    unsigned int i;

    gyro.complete = 1U;
    accel.complete = 1U;
    app_attitude_init(NULL);
    app_attitude_apply_calibration(&gyro, &accel);

    sample.acc_y_g = -1.0f;
    if (!settle_at(&sample, -90.0f, &attitude))
    {
        puts("negative X rotation does not map to vehicle pitch -90");
        return 1;
    }

    upright_pitch = attitude.pitch;
    sample.gyro_x_dps = 60.0f;
    for (i = 0U; i < 25U; i++)
    {
        if (app_attitude_step(&sample, 0.001f, &attitude) == 0U)
        {
            return 1;
        }
    }
    if (attitude.pitch <= upright_pitch ||
        fabsf(attitude.pitch_rate_dps - 60.0f) > 0.01f)
    {
        puts("positive gyro X does not increase vehicle pitch");
        return 1;
    }

    app_attitude_reset();
    sample.gyro_x_dps = 0.0f;
    sample.acc_y_g = 1.0f;
    if (!settle_at(&sample, 90.0f, &attitude))
    {
        puts("positive X rotation does not map to vehicle pitch +90");
        return 1;
    }

    puts("Madgwick attitude mapping passed");
    return 0;
}

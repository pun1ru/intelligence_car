#include "app_calibration.h"

#include "running_stats.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    running_stats3_t stats;
    app_accel_calibration_result_t result;
} app_accel_calibration_t;

typedef struct
{
    running_stats3_t bias_stats;
    running_stats3_t noise_stats;
    app_gyro_calibration_result_t result;
} app_gyro_calibration_t;

static app_accel_calibration_t accel_calibration;
static app_gyro_calibration_t gyro_calibration;

static void app_accel_calibration_begin(app_accel_calibration_t *calibration)
{
    if (calibration != NULL)
    {
        memset(calibration, 0, sizeof(*calibration));
        calibration->result.active = 1U;
    }
}

static void app_accel_calibration_add(app_accel_calibration_t *calibration,
                               const app_imu_sample_t *sample)
{
    float norm;
    float normalized[3];

    if ((calibration == NULL) || (sample == NULL))
    {
        return;
    }
    norm = sqrtf(sample->acc_x_g * sample->acc_x_g +
                 sample->acc_y_g * sample->acc_y_g +
                 sample->acc_z_g * sample->acc_z_g);
    if (norm <= 1.0e-6f)
    {
        return;
    }
    normalized[0] = sample->acc_x_g / norm;
    normalized[1] = sample->acc_y_g / norm;
    normalized[2] = sample->acc_z_g / norm;
    running_stats3_add(&calibration->stats, normalized);
}

static void app_accel_calibration_finish(app_accel_calibration_t *calibration)
{
    uint8_t axis;

    if (calibration == NULL)
    {
        return;
    }
    calibration->result.sample_count = calibration->stats.count;
    running_stats3_variance(&calibration->stats,
                            calibration->result.noise_variance_norm2);
    for (axis = 0U; axis < 3U; axis++)
    {
        calibration->result.mean_norm[axis] = calibration->stats.mean[axis];
        calibration->result.noise_stddev_norm[axis] =
            sqrtf(calibration->result.noise_variance_norm2[axis]);
    }
    calibration->result.active = 0U;
    calibration->result.complete = (calibration->stats.count > 1U) ? 1U : 0U;
}

static void app_gyro_calibration_begin(app_gyro_calibration_t *calibration)
{
    if (calibration != NULL)
    {
        memset(calibration, 0, sizeof(*calibration));
        calibration->result.active = 1U;
    }
}

static void app_gyro_calibration_add_bias(app_gyro_calibration_t *calibration,
                                   const app_imu_sample_t *sample)
{
    float gyro[3];

    if ((calibration == NULL) || (sample == NULL))
    {
        return;
    }
    gyro[0] = sample->gyro_x_dps;
    gyro[1] = sample->gyro_y_dps;
    gyro[2] = sample->gyro_z_dps;
    running_stats3_add(&calibration->bias_stats, gyro);
}

static void app_gyro_calibration_add_noise(app_gyro_calibration_t *calibration,
                                    const app_imu_sample_t *sample)
{
    float corrected[3];

    if ((calibration == NULL) || (sample == NULL))
    {
        return;
    }
    corrected[0] = sample->gyro_x_dps - calibration->bias_stats.mean[0];
    corrected[1] = sample->gyro_y_dps - calibration->bias_stats.mean[1];
    corrected[2] = sample->gyro_z_dps - calibration->bias_stats.mean[2];
    running_stats3_add(&calibration->noise_stats, corrected);
}

static void app_gyro_calibration_finish(app_gyro_calibration_t *calibration)
{
    uint8_t axis;

    if (calibration == NULL)
    {
        return;
    }
    calibration->result.bias_sample_count = calibration->bias_stats.count;
    calibration->result.noise_sample_count = calibration->noise_stats.count;
    running_stats3_variance(&calibration->noise_stats,
                            calibration->result.gyro_noise_variance_dps2);
    for (axis = 0U; axis < 3U; axis++)
    {
        calibration->result.gyro_bias_dps[axis] = calibration->bias_stats.mean[axis];
        calibration->result.gyro_noise_stddev_dps[axis] =
            sqrtf(calibration->result.gyro_noise_variance_dps2[axis]);
    }
    calibration->result.active = 0U;
    calibration->result.complete =
        (calibration->bias_stats.count > 1U &&
         calibration->noise_stats.count > 1U) ? 1U : 0U;
}

void app_calibration_accel_start(void)
{
    app_accel_calibration_begin(&accel_calibration);
}

void app_calibration_accel_sample(const app_imu_sample_t *sample)
{
    app_accel_calibration_add(&accel_calibration, sample);
}

void app_calibration_accel_complete(void)
{
    app_accel_calibration_finish(&accel_calibration);
}

void app_calibration_accel_result(app_accel_calibration_result_t *result)
{
    if (result != NULL)
    {
        *result = accel_calibration.result;
    }
}

void app_calibration_gyro_start(void)
{
    app_gyro_calibration_begin(&gyro_calibration);
}

void app_calibration_gyro_bias_sample(const app_imu_sample_t *sample)
{
    app_gyro_calibration_add_bias(&gyro_calibration, sample);
}

void app_calibration_gyro_noise_sample(const app_imu_sample_t *sample)
{
    app_gyro_calibration_add_noise(&gyro_calibration, sample);
}

void app_calibration_gyro_complete(void)
{
    app_gyro_calibration_finish(&gyro_calibration);
}

void app_calibration_gyro_bias_complete(void)
{
    uint8_t axis;

    gyro_calibration.result.bias_sample_count =
        gyro_calibration.bias_stats.count;
    for (axis = 0U; axis < 3U; axis++)
    {
        gyro_calibration.result.gyro_bias_dps[axis] =
            gyro_calibration.bias_stats.mean[axis];
        gyro_calibration.result.gyro_noise_variance_dps2[axis] = 0.0f;
        gyro_calibration.result.gyro_noise_stddev_dps[axis] = 0.0f;
    }
    gyro_calibration.result.noise_sample_count = 0U;
    gyro_calibration.result.active = 0U;
    gyro_calibration.result.complete =
        (gyro_calibration.bias_stats.count > 1U) ? 1U : 0U;
}

void app_calibration_gyro_result(app_gyro_calibration_result_t *result)
{
    if (result != NULL)
    {
        *result = gyro_calibration.result;
    }
}

#include "app_attitude.h"

#include "madgwick.h"
#include "general_define.h"
#include <math.h>

#define APP_IMU_DPS_TO_RAD 0.017453292519943f
#define APP_IMU_RAD_TO_DEG 57.295779513082f

static madgwick_t attitude_filter;

static float gyro_bias_dps[3] =
{
    -0.05030889651f, -0.20363670400f, -0.50371301400f
};

void app_attitude_init(void *(*allocate)(size_t))
{
    (void)allocate;
    madgwick_init(&attitude_filter, ATTITUDE_MADGWICK_BETA);
}

void app_attitude_reset(void)
{
    madgwick_reset(&attitude_filter);
}

void app_attitude_apply_calibration(const app_gyro_calibration_result_t *gyro,
                                    const app_accel_calibration_result_t *accel)
{
    if ((gyro == NULL) || (accel == NULL) ||
        (gyro->complete == 0U) || (accel->complete == 0U))
    {
        return;
    }
    app_attitude_apply_gyro_calibration(gyro);
}

void app_attitude_apply_gyro_calibration(
    const app_gyro_calibration_result_t *gyro)
{
    uint8_t axis;

    if ((gyro == NULL) || (gyro->complete == 0U))
    {
        return;
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        gyro_bias_dps[axis] = gyro->gyro_bias_dps[axis];
    }
    madgwick_reset(&attitude_filter);
}

uint8_t app_attitude_step(const app_imu_sample_t *sample, float dt_s,
                          app_attitude_t *attitude)
{
    float acc_norm2;
    uint8_t axis;

    if ((sample == NULL) || (attitude == NULL) ||
        !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return 0U;
    }
    acc_norm2 = sample->acc_x_g * sample->acc_x_g +
                sample->acc_y_g * sample->acc_y_g +
                sample->acc_z_g * sample->acc_z_g;
    if (!isfinite(acc_norm2) ||
        (acc_norm2 < ATTITUDE_MIN_ACCEL_NORM_G * ATTITUDE_MIN_ACCEL_NORM_G) ||
        !isfinite(sample->gyro_x_dps) ||
        !isfinite(sample->gyro_y_dps) ||
        !isfinite(sample->gyro_z_dps))
    {
        return 0U;
    }

    madgwick_update(&attitude_filter,
        (sample->gyro_x_dps - gyro_bias_dps[0]) * APP_IMU_DPS_TO_RAD,
        (sample->gyro_y_dps - gyro_bias_dps[1]) * APP_IMU_DPS_TO_RAD,
        (sample->gyro_z_dps - gyro_bias_dps[2]) * APP_IMU_DPS_TO_RAD,
        sample->acc_x_g,
        sample->acc_y_g,
        sample->acc_z_g,
        dt_s);

    for (axis = 0U; axis < 4U; axis++)
    {
        attitude->quaternion[axis] = attitude_filter.q[axis];
    }
    /* The existing vehicle pitch is rotation around the IMU X axis. */
    attitude->pitch = attitude_filter.euler[MADGWICK_ROLL] * APP_IMU_RAD_TO_DEG;
    attitude->roll = attitude_filter.euler[MADGWICK_PITCH] *
                     APP_IMU_RAD_TO_DEG * ATTITUDE_ROLL_SIGN;
    attitude->yaw = attitude_filter.euler[MADGWICK_YAW] * APP_IMU_RAD_TO_DEG;
    attitude->pitch_rate_dps = sample->gyro_x_dps - gyro_bias_dps[0];
    attitude->yaw_rate_dps = sample->gyro_z_dps - gyro_bias_dps[2];
    return (isfinite(attitude->roll) && isfinite(attitude->pitch) &&
            isfinite(attitude->yaw) &&
            isfinite(attitude->pitch_rate_dps) &&
            isfinite(attitude->yaw_rate_dps)) ? 1U : 0U;
}

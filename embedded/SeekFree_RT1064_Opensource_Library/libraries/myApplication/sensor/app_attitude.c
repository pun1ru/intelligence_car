#include "app_attitude.h"

#include "ekf_quaternion.h"
#include "general_define.h"
#include <math.h>

#define APP_IMU_G_TO_MPS2  9.80665f
#define APP_IMU_DPS_TO_RAD 0.017453292519943f

static float gyro_bias_dps[3] =
{
    -0.05030889651f, -0.20363670400f, -0.50371301400f
};

void app_attitude_init(void *(*allocate)(size_t))
{
    Kalman_Filter_SetAllocator(allocate);
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f,
                           0.002098333868f, 0.002603255680f, 0.001160622460f,
                           1.57132149e-03f, 1.4076345e-03f, 5.64783109e-03f);
    IMU_QuaternionEKF_SetAccelFilter(ATTITUDE_ACCEL_LPF_ALPHA,
                                     ATTITUDE_EKF_CHI_SQUARE_LIMIT,
                                     ATTITUDE_EKF_CORRECTION_LIMIT);
}

void app_attitude_reset(void)
{
    IMU_QuaternionEKF_Reset();
}

void app_attitude_apply_calibration(const app_gyro_calibration_result_t *gyro,
                                    const app_accel_calibration_result_t *accel)
{
    float gyro_noise[3];
    float accel_noise[3];
    uint8_t axis;

    if ((gyro == NULL) || (accel == NULL) ||
        (gyro->complete == 0U) || (accel->complete == 0U))
    {
        return;
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        gyro_bias_dps[axis] = gyro->gyro_bias_dps[axis];
        gyro_noise[axis] = fmaxf(gyro->gyro_noise_variance_dps2[axis],
                                 IMU_CALIBRATION_GYRO_NOISE_FLOOR_DPS2);
        accel_noise[axis] = fmaxf(accel->noise_variance_norm2[axis],
                                  IMU_CALIBRATION_ACC_NOISE_FLOOR_NORM2);
    }
    IMU_QuaternionEKF_SetNoise(gyro_noise, accel_noise);
    IMU_QuaternionEKF_Reset();
}

uint8_t app_attitude_step(const app_imu_sample_t *sample, float dt_s,
                          app_attitude_t *attitude)
{
    float acc_norm2;

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

    IMU_QuaternionEKF_Update(
        (sample->gyro_x_dps - gyro_bias_dps[0]) * APP_IMU_DPS_TO_RAD,
        (sample->gyro_y_dps - gyro_bias_dps[1]) * APP_IMU_DPS_TO_RAD,
        (sample->gyro_z_dps - gyro_bias_dps[2]) * APP_IMU_DPS_TO_RAD,
        sample->acc_x_g * APP_IMU_G_TO_MPS2,
        sample->acc_y_g * APP_IMU_G_TO_MPS2,
        sample->acc_z_g * APP_IMU_G_TO_MPS2,
        dt_s);

    IMU_QuaternionEKF_GetAttitude(attitude->quaternion,
                                   &attitude->roll,
                                   &attitude->pitch,
                                   &attitude->yaw);
    return (isfinite(attitude->roll) && isfinite(attitude->pitch) &&
            isfinite(attitude->yaw)) ? 1U : 0U;
}

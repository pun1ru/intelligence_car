#include "madgwick.h"

#include <math.h>

static float madgwick_inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(value);
}

static void madgwick_update_euler(madgwick_t *filter)
{
    const float q0 = filter->q[0];
    const float q1 = filter->q[1];
    const float q2 = filter->q[2];
    const float q3 = filter->q[3];
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    filter->euler[MADGWICK_ROLL] = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                          1.0f - 2.0f * (q1 * q1 + q2 * q2));
    filter->euler[MADGWICK_PITCH] = asinf(sin_pitch);
    filter->euler[MADGWICK_YAW] = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                         1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void madgwick_init(madgwick_t *filter, float beta)
{
    if (filter == 0)
    {
        return;
    }

    filter->beta = beta;
    filter->dt = 0.0f;
    madgwick_reset(filter);
}

void madgwick_reset(madgwick_t *filter)
{
    if (filter == 0)
    {
        return;
    }

    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    filter->euler[0] = 0.0f;
    filter->euler[1] = 0.0f;
    filter->euler[2] = 0.0f;
    filter->dt = 0.0f;
}

void madgwick_set_beta(madgwick_t *filter, float beta)
{
    if (filter != 0)
    {
        filter->beta = beta;
    }
}

void madgwick_update(madgwick_t *filter,
                     float gyro_x, float gyro_y, float gyro_z,
                     float accel_x, float accel_y, float accel_z,
                     float dt)
{
    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    float recip_norm;
    float s0;
    float s1;
    float s2;
    float s3;
    float q0;
    float q1;
    float q2;
    float q3;
    const float accel_norm = accel_x * accel_x + accel_y * accel_y + accel_z * accel_z;

    if (filter == 0)
    {
        return;
    }

    filter->dt = (dt > 0.0f) ? dt : 0.0f;
    q0 = filter->q[0];
    q1 = filter->q[1];
    q2 = filter->q[2];
    q3 = filter->q[3];

    /* Rate of change of quaternion from gyroscope. *///四元数微分方程
    q_dot0 = 0.5f * (-q1 * gyro_x - q2 * gyro_y - q3 * gyro_z);
    q_dot1 = 0.5f * (q0 * gyro_x + q2 * gyro_z - q3 * gyro_y);
    q_dot2 = 0.5f * (q0 * gyro_y - q1 * gyro_z + q3 * gyro_x);
    q_dot3 = 0.5f * (q0 * gyro_z + q1 * gyro_y - q2 * gyro_x);

    /* Accelerometer correction is skipped when the vector is zero. */
    if (accel_norm > 0.0f)
    {
        const float ax = accel_x * madgwick_inv_sqrt(accel_norm);
        const float ay = accel_y * madgwick_inv_sqrt(accel_norm);
        const float az = accel_z * madgwick_inv_sqrt(accel_norm);
        const float q0q0 = q0 * q0;
        const float q1q1 = q1 * q1;
        const float q2q2 = q2 * q2;
        const float q3q3 = q3 * q3;
        const float q2q0 = 2.0f * q0;
        const float q2q1 = 2.0f * q1;
        const float two_q2 = 2.0f * q2;
        const float q2q3 = 2.0f * q3;
        const float q4q0 = 4.0f * q0;
        const float q4q1 = 4.0f * q1;
        const float q4q2 = 4.0f * q2;
        const float q8q1 = 8.0f * q1;
        const float q8q2 = 8.0f * q2;
        float step_norm;
        //梯度
        s0 = q4q0 * q2q2 + two_q2 * ax + q4q0 * q1q1 - q2q1 * ay;
        s1 = q4q1 * q3q3 - q2q3 * ax + 4.0f * q0q0 * q1 -
             q2q0 * ay - q4q1 + q8q1 * q1q1 + q8q1 * q2q2 + q4q1 * az;
        s2 = 4.0f * q0q0 * q2 + q2q0 * ax + q4q2 * q3q3 -
             q2q3 * ay - q4q2 + q8q2 * q1q1 + q8q2 * q2q2 + q4q2 * az;
        s3 = 4.0f * q1q1 * q3 - q2q1 * ax + 4.0f * q2q2 * q3 - two_q2 * ay;

        step_norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (step_norm > 0.0f)
        {
            recip_norm = madgwick_inv_sqrt(step_norm);
            q_dot0 -= filter->beta * s0 * recip_norm;
            q_dot1 -= filter->beta * s1 * recip_norm;
            q_dot2 -= filter->beta * s2 * recip_norm;
            q_dot3 -= filter->beta * s3 * recip_norm;
        }
    }

    q0 += q_dot0 * filter->dt;
    q1 += q_dot1 * filter->dt;
    q2 += q_dot2 * filter->dt;
    q3 += q_dot3 * filter->dt;

    recip_norm = madgwick_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (recip_norm == 0.0f)
    {
        madgwick_reset(filter);
        return;
    }

    filter->q[0] = q0 * recip_norm;
    filter->q[1] = q1 * recip_norm;
    filter->q[2] = q2 * recip_norm;
    filter->q[3] = q3 * recip_norm;
    madgwick_update_euler(filter);
}

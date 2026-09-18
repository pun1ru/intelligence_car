#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdint.h>

/* Euler angle indexes. Values are in radians. */
#define MADGWICK_ROLL  0U
#define MADGWICK_PITCH 1U
#define MADGWICK_YAW   2U

typedef struct
{
    /* Quaternion order is w, x, y, z. */
    float q[4];
    /* Euler angle order is roll, pitch, yaw, in radians. */
    float euler[3];
    float beta;
    float dt;
} madgwick_t;

/* Initialize the filter with the gradient descent gain beta. */
void madgwick_init(madgwick_t *filter, float beta);

/* Reset the attitude to identity while keeping the configured beta. */
void madgwick_reset(madgwick_t *filter);

/*
 * Update the attitude.
 *
 * gyro_*: angular rate in rad/s.
 * accel_*: acceleration vector; its magnitude is normalized internally.
 * dt: sample period in seconds.
 */
void madgwick_update(madgwick_t *filter,
                     float gyro_x, float gyro_y, float gyro_z,
                     float accel_x, float accel_y, float accel_z,
                     float dt);

/* Change beta without resetting the current attitude. */
void madgwick_set_beta(madgwick_t *filter, float beta);

#endif

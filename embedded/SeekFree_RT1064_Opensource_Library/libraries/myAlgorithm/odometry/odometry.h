#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

/* Heading and position are expressed in the vehicle frame defined by
 * docs/VEHICLE_CONVENTIONS.md: +X forward, +Y left, +Z up, so a positive
 * heading is a counter-clockwise turn seen from above.  The pose is relative
 * to the heading latched by odometry_reset, which lets a start line become the
 * origin without any absolute heading reference. */

#define ODOMETRY_PI_F 3.14159265358979323846f

typedef struct
{
    float x_m;
    float y_m;
    float heading_rad;
    /* Signed arc length travelled since odometry_reset. */
    float path_length_m;
    /* Heading that makes heading_rad equal zero; latched by odometry_reset. */
    float yaw_offset_rad;
    /* Encoder derived body yaw rate, for cross-checking the IMU yaw. */
    float yaw_rate_rad_s;
    float track_width_m;
    uint8_t origin_set;
} odometry_t;

/* Configures the geometry.  track_width_m must be finite and positive; an
 * invalid value is ignored and leaves the module unusable until a valid one is
 * supplied.  Clears origin_set. */
void odometry_init(odometry_t *odometry, float track_width_m);

/* Latches yaw_rad as the heading of the current position and clears x, y and
 * the travelled path.  Call it on the start line.  A non-finite yaw_rad is
 * ignored. */
void odometry_reset(odometry_t *odometry, float yaw_rad);

/* Integrates one interval.  left_delta_m and right_delta_m are the signed
 * wheel travel since the previous call, and dt_s is the interval length used
 * for the encoder yaw rate.  Returns 1 when the pose was updated; returns 0
 * for NULL, non-finite input, dt_s <= 0, or an unset origin. */
uint8_t odometry_step(odometry_t *odometry, float dt_s, float left_delta_m,
                      float right_delta_m, float yaw_rad);

#endif

#include "odometry.h"

#include <math.h>

/* Relative heading stays inside +/-pi so that crossing the yaw boundary takes
 * the short way round, matching the steering loop in app_balance.c. */
static float odometry_wrap_pi(float angle_rad)
{
    while (angle_rad > ODOMETRY_PI_F)
    {
        angle_rad -= 2.0f * ODOMETRY_PI_F;
    }
    while (angle_rad < -ODOMETRY_PI_F)
    {
        angle_rad += 2.0f * ODOMETRY_PI_F;
    }
    return angle_rad;
}

static uint8_t odometry_configured(const odometry_t *odometry)
{
    return ((odometry != 0) && (odometry->origin_set != 0U)) ? 1U : 0U;
}

void odometry_init(odometry_t *odometry, float track_width_m)
{
    if (odometry == 0)
    {
        return;
    }

    *odometry = (odometry_t){0};
    if (isfinite(track_width_m) && (track_width_m > 0.0f))
    {
        odometry->track_width_m = track_width_m;
    }
}

void odometry_reset(odometry_t *odometry, float yaw_rad)
{
    if ((odometry == 0) || (odometry->track_width_m <= 0.0f) ||
        !isfinite(yaw_rad))
    {
        return;
    }

    odometry->x_m = 0.0f;
    odometry->y_m = 0.0f;
    odometry->heading_rad = 0.0f;
    odometry->path_length_m = 0.0f;
    odometry->yaw_rate_rad_s = 0.0f;
    odometry->yaw_offset_rad = yaw_rad;
    odometry->origin_set = 1U;
}

uint8_t odometry_step(odometry_t *odometry, float dt_s, float left_delta_m,
                      float right_delta_m, float yaw_rad)
{
    float heading_rad;
    float delta_m;

    if ((odometry_configured(odometry) == 0U) || (dt_s <= 0.0f) ||
        !isfinite(dt_s) || !isfinite(left_delta_m) ||
        !isfinite(right_delta_m) || !isfinite(yaw_rad))
    {
        return 0U;
    }

    /* Differential drive: the right wheel leads on a left (positive) turn. */
    odometry->yaw_rate_rad_s = (right_delta_m - left_delta_m) /
                               (odometry->track_width_m * dt_s);

    /* Use the measured heading rather than the integrated one; the yaw source
     * carries the absolute direction and the encoders only supply travel. */
    heading_rad = odometry_wrap_pi(yaw_rad - odometry->yaw_offset_rad);
    delta_m = 0.5f * (left_delta_m + right_delta_m);

    odometry->x_m += delta_m * cosf(heading_rad);
    odometry->y_m += delta_m * sinf(heading_rad);
    odometry->path_length_m += delta_m;
    odometry->heading_rad = heading_rad;
    return 1U;
}

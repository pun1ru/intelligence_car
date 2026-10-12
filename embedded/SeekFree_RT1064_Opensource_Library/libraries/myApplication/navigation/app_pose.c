#include "app_pose.h"

#include <stddef.h>
#include <math.h>
#include "general_define.h"

#define APP_POSE_RAD_PER_DEG (ODOMETRY_PI_F / 180.0f)
#define APP_POSE_DEG_PER_RAD (180.0f / ODOMETRY_PI_F)

/* Wheel travel produced by one encoder count, from the geometry in
 * general_define.h.  Integrating counts rather than speed keeps the travelled
 * distance independent of the sampling interval. */
static float app_pose_metres_per_count(void)
{
    return WHEEL_TWO_PI_RAD / WHEEL_ENCODER_COUNTS_PER_REV * WHEEL_RADIUS_M;
}

void app_pose_init(app_pose_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    app_encoder_tracker_reset(&state->wheel_tracker);
    odometry_init(&state->odometry, WHEEL_TRACK_M);
    state->pose = (app_pose_t){0};
}

void app_pose_reset(app_pose_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    app_encoder_tracker_reset(&state->wheel_tracker);
    odometry_init(&state->odometry, WHEEL_TRACK_M);
    state->pose = (app_pose_t){0};
}

void app_pose_set_origin(app_pose_state_t *state, float yaw_deg)
{
    if ((state == NULL) || !isfinite(yaw_deg))
    {
        return;
    }

    app_encoder_tracker_reset(&state->wheel_tracker);
    odometry_reset(&state->odometry, yaw_deg * APP_POSE_RAD_PER_DEG);
    state->pose = (app_pose_t){0};
    state->pose.valid = state->odometry.origin_set;
}

uint8_t app_pose_step(app_pose_state_t *state,
                      const app_encoder_sample_t *encoder, float yaw_deg,
                      float dt_s)
{
    app_encoder_motion_t motion;
    float metres_per_count;

    if ((state == NULL) || (encoder == NULL) || !isfinite(yaw_deg) ||
        !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return 0U;
    }
    if (app_encoder_tracker_step(&state->wheel_tracker, encoder, dt_s,
                                 &motion) == 0U)
    {
        /* The first snapshot after a reset only records the baseline count. */
        return 0U;
    }

    metres_per_count = app_pose_metres_per_count();
    if (odometry_step(&state->odometry, dt_s,
                      (float)motion.left.delta_count * metres_per_count,
                      (float)motion.right.delta_count * metres_per_count,
                      yaw_deg * APP_POSE_RAD_PER_DEG) == 0U)
    {
        return 0U;
    }

    state->pose.x_m = state->odometry.x_m;
    state->pose.y_m = state->odometry.y_m;
    state->pose.heading_deg = state->odometry.heading_rad * APP_POSE_DEG_PER_RAD;
    state->pose.path_length_m = state->odometry.path_length_m;
    state->pose.yaw_rate_rad_s = state->odometry.yaw_rate_rad_s;
    state->pose.valid = 1U;
    return 1U;
}

void app_pose_snapshot(const app_pose_state_t *state, app_pose_t *pose)
{
    if ((state == NULL) || (pose == NULL))
    {
        return;
    }

    *pose = state->pose;
}

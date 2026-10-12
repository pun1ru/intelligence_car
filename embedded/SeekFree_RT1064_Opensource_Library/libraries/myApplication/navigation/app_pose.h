#ifndef APP_POSE_H
#define APP_POSE_H

#include <stdint.h>
#include "app_encoder.h"
#include "odometry.h"

/* Vehicle pose for the navigation layer, in the frame described by
 * docs/VEHICLE_CONVENTIONS.md: +X forward, +Y left, heading positive counter
 * clockwise.  The origin is latched with app_pose_set_origin, so the start line
 * becomes (0, 0) with heading 0. */
typedef struct
{
    float x_m;
    float y_m;
    float heading_deg;
    float path_length_m;
    float yaw_rate_rad_s;
    uint8_t valid;
} app_pose_t;

typedef struct
{
    app_encoder_tracker_t wheel_tracker;
    odometry_t odometry;
    app_pose_t pose;
} app_pose_state_t;

/* Configures the geometry from general_define.h and invalidates the pose. */
void app_pose_init(app_pose_state_t *state);

/* Drops the pose and the encoder baseline; the next app_pose_set_origin latches
 * a new origin. */
void app_pose_reset(app_pose_state_t *state);

/* Latches yaw_deg as the heading of the origin and clears the pose.  A
 * non-finite yaw_deg is ignored and leaves the pose invalid. */
void app_pose_set_origin(app_pose_state_t *state, float yaw_deg);

/* Integrates one interval from an absolute encoder snapshot.  dt_s is the
 * interval since the previous call.  Returns 1 when the pose was updated; the
 * first call after a reset only establishes the encoder baseline and returns 0. */
uint8_t app_pose_step(app_pose_state_t *state,
                      const app_encoder_sample_t *encoder, float yaw_deg,
                      float dt_s);

/* Copies the current pose; pose must not be NULL. */
void app_pose_snapshot(const app_pose_state_t *state, app_pose_t *pose);

#endif

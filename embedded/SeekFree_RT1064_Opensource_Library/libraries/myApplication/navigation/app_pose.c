#include "app_pose.h"

#include <stddef.h>
#include <math.h>
#include "general_define.h"

#define APP_POSE_RAD_PER_DEG (ODOMETRY_PI_F / 180.0f)
#define APP_POSE_DEG_PER_RAD (180.0f / ODOMETRY_PI_F)

/*
 * app_pose 实现：见 app_pose.h 开头的说明。
 * 注意这里对计数积分、而不是对速度积分：速度 = 计数/dt，一旦调度抖动
 * dt 就不准；直接累加计数则里程与采样间隔无关，只有角速度诊断量会被 dt 影响。
 */

/* 一个编码器计数对应多少米轮面行程：
 * 每计数弧度 = 2*pi / 每圈计数，再乘轮半径。计数与轮径取自 general_define.h。 */
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
    /* 轮距从 general_define.h 取；odometry_init 会校验合法性 */
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
    /* 重新 init 就顺带清掉了 origin_set，位姿随之失效 */
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
    /* 只有真正定上了原点，才把位姿标为可信 */
    state->pose.valid = state->odometry.origin_set;
    state->pose.yaw_offset_deg = yaw_deg;
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
        /* reset / set_origin 之后的第一帧只记录基线，差分不出增量 */
        return 0U;
    }

    /* 左右轮有各自的计数符号（WHEEL_*_COUNT_SIGN），motion 里的 delta_count
     * 已经统一成"前进为正"，所以这里直接乘即可，不需要再判方向。 */
    metres_per_count = app_pose_metres_per_count();
    if (odometry_step(&state->odometry, dt_s,
                      (float)motion.left.delta_count * metres_per_count,
                      (float)motion.right.delta_count * metres_per_count,
                      yaw_deg * APP_POSE_RAD_PER_DEG) == 0U)
    {
        /* 参数不合法或还没定原点：保持上一次的位姿不动 */
        return 0U;
    }

    /* 把算法层的弧度/内部状态翻译成对外发布的位姿 */
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

#include "app_nav.h"

#include <stddef.h>
#include <math.h>
#include "general_define.h"

/* 把角度收进 [-180, 180]，跨界走短边（与转向环的约定一致） */
static float app_nav_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void app_nav_init(app_nav_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->config.lookahead_m = NAV_TRACKING_LOOKAHEAD_M;
    state->config.speed_max_m_s = NAV_TRACKING_SPEED_MAX_M_S;
    state->config.speed_min_m_s = NAV_TRACKING_SPEED_MIN_M_S;
    state->config.slow_radius_m = NAV_TRACKING_SLOW_RADIUS_M;
    state->config.goal_radius_m = NAV_TRACKING_GOAL_RADIUS_M;
    pure_pursuit_init(&state->pursuer);
    state->command = (app_nav_command_t){0};
    state->enabled = (NAV_TRACKING_ENABLE_DEFAULT != 0U) ? 1U : 0U;
}

void app_nav_set_enabled(app_nav_state_t *state, uint8_t enabled)
{
    if (state == NULL)
    {
        return;
    }

    state->enabled = (enabled != 0U) ? 1U : 0U;
    if (state->enabled == 0U)
    {
        /* 关闭时立刻失效并清掉跟车进度，下次使能从头开始 */
        state->command = (app_nav_command_t){0};
        pure_pursuit_init(&state->pursuer);
    }
}

uint8_t app_nav_enabled(const app_nav_state_t *state)
{
    return ((state != NULL) && (state->enabled != 0U)) ? 1U : 0U;
}

void app_nav_reset(app_nav_state_t *state)
{
    if (state != NULL)
    {
        pure_pursuit_init(&state->pursuer);
        state->command = (app_nav_command_t){0};
    }
}

void app_nav_step(app_nav_state_t *state, const app_pose_t *pose)
{
    pure_pursuit_output_t output;
    uint16_t count;
    const pure_pursuit_point_t *points;

    if (state == NULL)
    {
        return;
    }
    /* 每个分支都先把指令清 0：只要有一项不满足，就不输出任何"看起来有效"的指令 */
    state->command = (app_nav_command_t){0};

    if (state->enabled == 0U)
    {
        return;
    }
    if ((pose == NULL) || (pose->valid == 0U))
    {
        /* 位姿不可信：宁可不跟踪，也不要用错的原点去算航向 */
        return;
    }
    count = app_nav_path_count();
    points = app_nav_path_points();
    if ((count == 0U) || (points == NULL))
    {
        /* 路径表为空：这是默认状态，表示"还没有可跑的路径" */
        return;
    }
    if (pure_pursuit_step(&state->config, &state->pursuer, points, count,
                          pose->x_m, pose->y_m, pose->heading_deg,
                          &output) == 0U)
    {
        return;
    }

    state->command.active = 1U;
    state->command.finished = output.finished;
    state->command.target_speed_m_s = output.target_speed_m_s;
    state->command.target_heading_deg = output.target_heading_deg;
    /* 路径坐标系 → IMU 坐标系：加上定原点时的 yaw 偏移，转向环才能直接用 */
    state->command.target_yaw_deg =
        app_nav_wrap_deg(pose->yaw_offset_deg + output.target_heading_deg);
    state->command.cross_track_m = output.cross_track_m;
    state->command.target_index = output.target_index;

    if (output.finished != 0U)
    {
        /* 到终点了：把目标速度压到 0，让车停在原地保持平衡，而不是继续往前冲。
         * 航向仍保持"路径末段的方向"，这样它是停稳的、不是歪的。
         * 末段方向由最后两个路径点算；路径只有 1 个点时保持当前朝向。 */
        state->command.target_speed_m_s = 0.0f;
        if (count >= 2U)
        {
            state->command.target_heading_deg = atan2f(
                points[count - 1U].y_m - points[count - 2U].y_m,
                points[count - 1U].x_m - points[count - 2U].x_m) *
                57.29577951308232f;
            state->command.target_yaw_deg =
                app_nav_wrap_deg(pose->yaw_offset_deg +
                                 state->command.target_heading_deg);
        }
    }
}

void app_nav_snapshot(const app_nav_state_t *state, app_nav_command_t *command)
{
    if ((state == NULL) || (command == NULL))
    {
        return;
    }

    *command = state->command;
}

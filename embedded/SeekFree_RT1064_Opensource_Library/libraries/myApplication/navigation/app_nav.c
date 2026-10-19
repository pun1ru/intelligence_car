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

static float app_nav_distance(float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;

    return sqrtf(dx * dx + dy * dy);
}

/* 沿路径量"从车当前位置到终点还剩多远"。
 * 用"车 → 下一个路径点 + 从下一个点到终点"而不是"车 → 最近点 + 最近点之后"，
 * 因为前者在最近点附近是连续变化的；后者会在每次最近点切换时跳一个段长，
 * 让终点减速曲线一顿一顿的（车身会前后晃）。 */
static float app_nav_remaining_m(const pure_pursuit_point_t *points,
                                 uint16_t count, uint16_t closest,
                                 float x_m, float y_m)
{
    uint16_t next = ((uint16_t)(closest + 1U) < count) ?
        (uint16_t)(closest + 1U) : closest;
    float total = app_nav_distance(x_m, y_m, points[next].x_m, points[next].y_m);
    uint16_t index;

    for (index = next; (uint16_t)(index + 1U) < count; index++)
    {
        total += app_nav_distance(points[index].x_m, points[index].y_m,
                                  points[index + 1U].x_m,
                                  points[index + 1U].y_m);
    }
    return total;
}

/* 沿最后一段的方向，车是否已越过终点 NAV_TRACKING_PAST_END_M 以上。
 * 冲过去就锁存"跑完"，不再掉头回来。 */
static uint8_t app_nav_past_end(const pure_pursuit_point_t *points,
                                uint16_t count, float x_m, float y_m)
{
    float sx;
    float sy;
    float length;
    float along;

    if (count < 2U)
    {
        return 0U;
    }
    sx = points[count - 1U].x_m - points[count - 2U].x_m;
    sy = points[count - 1U].y_m - points[count - 2U].y_m;
    length = sqrtf(sx * sx + sy * sy);
    if (length <= 0.0f)
    {
        return 0U;
    }
    /* (车 - 终点) 在最后一段方向上的投影：> 0 表示车已经走过终点 */
    along = ((x_m - points[count - 1U].x_m) * sx +
             (y_m - points[count - 1U].y_m) * sy) / length;
    return (along > NAV_TRACKING_PAST_END_M) ? 1U : 0U;
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
    state->finished_latched = 0U;
    /* 起始按最高速估前视距离，起步时不会因为速度还是 0 而只看眼前一点点 */
    state->last_speed_m_s = state->config.speed_max_m_s;
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
        state->finished_latched = 0U;
        state->last_speed_m_s = state->config.speed_max_m_s;
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
        state->finished_latched = 0U;
        state->last_speed_m_s = state->config.speed_max_m_s;
    }
}

void app_nav_step(app_nav_state_t *state, const app_pose_t *pose, float dt_s)
{
    pure_pursuit_output_t output;
    uint16_t count;
    const pure_pursuit_point_t *points;
    float lookahead;
    float speed_limit;
    float delta;
    float delta_max;

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
    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        dt_s = (float)DECISION_TASK_PERIOD_MS / 1000.0f;
    }

    /* 前视距离随速度：慢速时保持已验证的 0.18 m，快起来才看远一点。
     * 用上一拍的速度而不是本拍的：本拍的速度又要靠前视距离算，先有鸡还是先有蛋，
     * 差一拍（10 ms）对跟踪没有影响。 */
    lookahead = NAV_TRACKING_LOOKAHEAD_TIME_S * state->last_speed_m_s;
    if (lookahead < NAV_TRACKING_LOOKAHEAD_MIN_M)
    {
        lookahead = NAV_TRACKING_LOOKAHEAD_MIN_M;
    }
    if (lookahead > NAV_TRACKING_LOOKAHEAD_MAX_M)
    {
        lookahead = NAV_TRACKING_LOOKAHEAD_MAX_M;
    }
    state->config.lookahead_m = lookahead;

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

    /* 跑完就锁存：要么进入了终点判定半径，要么已经冲过终点。
     * 锁存后一直保持到 app_nav_reset（换路径/重新发车）。 */
    if ((output.finished != 0U) ||
        ((state->pursuer.index == (uint16_t)(count - 1U)) &&
         (app_nav_past_end(points, count, pose->x_m, pose->y_m) != 0U)))
    {
        state->finished_latched = 1U;
    }

    if (state->finished_latched != 0U)
    {
        /* 到终点了：把目标速度压到 0，让车停在原地保持平衡，而不是继续往前冲。
         * 航向仍保持"路径末段的方向"，这样它是停稳的、不是歪的。
         * 末段方向由最后两个路径点算；路径只有 1 个点时保持当前朝向。 */
        state->command.finished = 1U;
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
    else
    {
        /* 终点减速：剩余距离越短，允许的速度越低，到点时自然降到 0。
         * 这一步不受 speed_min 的下限约束——那是给弯道用的，停车必须能停。 */
        speed_limit = sqrtf(2.0f * NAV_TRACKING_END_DECEL_M_S2 *
                            app_nav_remaining_m(points, count,
                                                state->pursuer.index,
                                                pose->x_m, pose->y_m));
        if (speed_limit < state->command.target_speed_m_s)
        {
            state->command.target_speed_m_s = speed_limit;
        }

        /* 加速度限幅：曲率是分段算的，速度目标会一跳一跳；直接下发会让车
         * 反复加速/刹车（车身前后晃）。这里限制每拍的变化量。 */
        delta_max = NAV_TRACKING_ACCEL_M_S2 * dt_s;
        delta = state->command.target_speed_m_s - state->last_speed_m_s;
        if (delta > delta_max)
        {
            state->command.target_speed_m_s = state->last_speed_m_s + delta_max;
        }
        else if (delta < -delta_max)
        {
            state->command.target_speed_m_s = state->last_speed_m_s - delta_max;
        }
        if (state->command.target_speed_m_s < 0.0f)
        {
            state->command.target_speed_m_s = 0.0f;
        }
    }
    state->last_speed_m_s = state->command.target_speed_m_s;
}

void app_nav_snapshot(const app_nav_state_t *state, app_nav_command_t *command)
{
    if ((state == NULL) || (command == NULL))
    {
        return;
    }

    *command = state->command;
}

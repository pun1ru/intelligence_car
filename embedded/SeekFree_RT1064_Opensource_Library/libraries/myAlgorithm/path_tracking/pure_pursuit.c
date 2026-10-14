#include "pure_pursuit.h"

#include <stddef.h>
#include <math.h>

/*
 * pure_pursuit 实现：见 pure_pursuit.h 开头的说明。
 * 每一步做四件事：找最近点 → 取前视点 → 算期望航向 → 按曲率定速度。
 */

#define PURE_PURSUIT_RAD_PER_DEG 0.017453292519943295f
#define PURE_PURSUIT_DEG_PER_RAD 57.29577951308232f
#define PURE_PURSUIT_MIN_CURVATURE 1.0e-6f

/* 把角度收进 [-180, 180]，跨界时走短边 */
static float pure_pursuit_wrap_deg(float angle_deg)
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

static float pure_pursuit_distance(float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;

    return sqrtf(dx * dx + dy * dy);
}

/* 配置合法性：非法时宁可不算，也不要用奇怪的参数开车 */
static uint8_t pure_pursuit_config_valid(const pure_pursuit_config_t *config)
{
    return ((config != NULL) && isfinite(config->lookahead_m) &&
            (config->lookahead_m > 0.0f) && isfinite(config->speed_max_m_s) &&
            isfinite(config->speed_min_m_s) && isfinite(config->slow_radius_m) &&
            (config->slow_radius_m > 0.0f) && isfinite(config->goal_radius_m)) ? 1U : 0U;
}

void pure_pursuit_init(pure_pursuit_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->index = 0U;
    state->started = 0U;
}

/* 在 state->index 之后找离车最近的路径点。只往后找是为了让前视点单调前进：
 * 路径来回交叉时（比如绕锥桶回来），否则会退回到前一段上。 */
static uint16_t pure_pursuit_closest_index(const pure_pursuit_point_t *path,
                                           uint16_t count, uint16_t from,
                                           float x_m, float y_m)
{
    uint16_t index;
    uint16_t best_index = from;
    float best_distance = pure_pursuit_distance(x_m, y_m, path[from].x_m,
                                                path[from].y_m);

    for (index = (uint16_t)(from + 1U); index < count; index++)
    {
        float distance = pure_pursuit_distance(x_m, y_m, path[index].x_m,
                                               path[index].y_m);

        if (distance < best_distance)
        {
            best_distance = distance;
            best_index = index;
        }
    }
    return best_index;
}

/* 沿路径往前走，累积弧长到前视距离的那个点就是前视点；不够长就用最后一个点。
 * 注意是"沿路径量"而不是"车到点的直线距离"：后者在车略微越过最近点时会把
 * 前视点判到车身后面，于是算出一个要求倒退的航向。 */
static uint16_t pure_pursuit_lookahead_index(const pure_pursuit_point_t *path,
                                             uint16_t count, uint16_t closest,
                                             float lookahead_m)
{
    uint16_t index;
    uint16_t target = closest;
    float arc_length = 0.0f;

    for (index = (uint16_t)(closest + 1U); index < count; index++)
    {
        arc_length += pure_pursuit_distance(path[index - 1U].x_m,
                                            path[index - 1U].y_m,
                                            path[index].x_m, path[index].y_m);
        target = index;
        if (arc_length >= lookahead_m)
        {
            break;
        }
    }
    return target;
}

/* 横向偏差：车到"最近点→下一点"这条线的有符号距离，正表示车在路径左侧 */
static float pure_pursuit_cross_track(const pure_pursuit_point_t *path,
                                      uint16_t count, uint16_t closest,
                                      float x_m, float y_m)
{
    uint16_t next = ((uint16_t)(closest + 1U) < count) ?
        (uint16_t)(closest + 1U) : closest;
    float vx = path[next].x_m - path[closest].x_m;
    float vy = path[next].y_m - path[closest].y_m;
    float wx = x_m - path[closest].x_m;
    float wy = y_m - path[closest].y_m;
    float length = sqrtf(vx * vx + vy * vy);

    if (length <= 0.0f)
    {
        /* 只有一个点：方向未知，只给距离大小 */
        return sqrtf(wx * wx + wy * wy);
    }
    /* 二维叉积 v × w 的符号就是左右，除以线段长度得到垂直距离 */
    return (vx * wy - vy * wx) / length;
}

uint8_t pure_pursuit_step(const pure_pursuit_config_t *config,
                          pure_pursuit_state_t *state,
                          const pure_pursuit_point_t *path, uint16_t count,
                          float x_m, float y_m, float heading_deg,
                          pure_pursuit_output_t *output)
{
    uint16_t closest;
    uint16_t target;
    float dx;
    float dy;
    float target_heading_deg;
    float yaw_error_deg;
    float curvature;
    float radius;
    float speed;

    if ((pure_pursuit_config_valid(config) == 0U) || (state == NULL) ||
        (path == NULL) || (output == NULL) || (count == 0U) ||
        !isfinite(x_m) || !isfinite(y_m) || !isfinite(heading_deg))
    {
        return 0U;
    }
    if (state->index >= count)
    {
        /* 路径变短或状态过期：退回到最后一个点而不是越界 */
        state->index = (uint16_t)(count - 1U);
    }

    closest = pure_pursuit_closest_index(path, count, state->index, x_m, y_m);
    target = pure_pursuit_lookahead_index(path, count, closest,
                                          config->lookahead_m);

    /* 期望航向 = 车指向当前前视点的方向 */
    dx = path[target].x_m - x_m;
    dy = path[target].y_m - y_m;
    if ((fabsf(dx) < 1.0e-6f) && (fabsf(dy) < 1.0e-6f))
    {
        /* 车正好压在前视点上：保持当前航向，别把 atan2(0,0) 的噪声当指令 */
        target_heading_deg = heading_deg;
    }
    else
    {
        target_heading_deg = atan2f(dy, dx) * PURE_PURSUIT_DEG_PER_RAD;
    }
    yaw_error_deg = pure_pursuit_wrap_deg(target_heading_deg - heading_deg);

    /* 曲率估计：把前视点看成一段圆弧的终点，kappa = 2*sin(偏角)/前视距离。
     * 直线上偏角为 0、半径为无穷，所以速度取上限；弯越急半径越小。 */
    curvature = 2.0f * sinf(yaw_error_deg * PURE_PURSUIT_RAD_PER_DEG) /
                config->lookahead_m;
    radius = (fabsf(curvature) > PURE_PURSUIT_MIN_CURVATURE) ?
        (1.0f / fabsf(curvature)) : config->slow_radius_m;
    if (radius >= config->slow_radius_m)
    {
        speed = config->speed_max_m_s;
    }
    else
    {
        speed = config->speed_max_m_s * (radius / config->slow_radius_m);
        if (speed < config->speed_min_m_s)
        {
            speed = config->speed_min_m_s;
        }
    }
    if (speed > config->speed_max_m_s)
    {
        speed = config->speed_max_m_s;
    }

    output->target_speed_m_s = speed;
    output->target_heading_deg = target_heading_deg;
    output->yaw_error_deg = yaw_error_deg;
    output->cross_track_m = pure_pursuit_cross_track(path, count, closest,
                                                     x_m, y_m);
    output->target_index = target;
    output->finished =
        (((closest == (uint16_t)(count - 1U)) &&
          (pure_pursuit_distance(x_m, y_m, path[closest].x_m,
                                 path[closest].y_m) <= config->goal_radius_m)) ?
         1U : 0U);

    state->index = closest;
    state->started = 1U;
    return 1U;
}

#include "odometry.h"

#include <math.h>

/*
 * odometry 实现：见 odometry.h 开头的说明。
 * 这里的四个函数都只依赖入参和自身状态，同样的输入必然得到同样的输出，
 * 便于在电脑上做单元测试。
 */

/* 把角度收进 [-pi, pi]，这样跨越 yaw 边界时走短边，
 * 和 app_balance.c 里转向环 wrap_angle_deg 的处理保持一致。 */
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

    /* 先整体清零，再只在轮距合法时写入；轮距非法时保持 origin_set = 0，
     * 后面 odometry_reset / odometry_step 都会拒绝工作，避免拿错几何算出假位姿。 */
    *odometry = (odometry_t){0};
    if (isfinite(track_width_m) && (track_width_m > 0.0f))
    {
        odometry->track_width_m = track_width_m;
    }
}

void odometry_reset(odometry_t *odometry, float yaw_rad)
{
    /* track_width_m <= 0 说明 odometry_init 没拿到合法轮距，此时拒绝定原点 */
    if ((odometry == 0) || (odometry->track_width_m <= 0.0f) ||
        !isfinite(yaw_rad))
    {
        return;
    }

    odometry->x_m = 0.0f;
    odometry->y_m = 0.0f;
    /* 原点航向就是此刻的车头方向，所以相对航向从 0 开始 */
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

    /* 差速轮：左转（航向正方向）时右轮比左轮走得远，所以右减左。
     * track_width_m 已在 odometry_init 里校验为正数。 */
    odometry->yaw_rate_rad_s = (right_delta_m - left_delta_m) /
                               (odometry->track_width_m * dt_s);

    /* 航向直接取姿态解输出的 yaw（减去原点航向），不用编码器积分：
     * yaw 提供方向，编码器只提供走了多远，二者分工明确。 */
    heading_rad = odometry_wrap_pi(yaw_rad - odometry->yaw_offset_rad);
    /* 后轮轴向平均：左右轮行程的均值就是车体中心的前进距离 */
    delta_m = 0.5f * (left_delta_m + right_delta_m);

    odometry->x_m += delta_m * cosf(heading_rad);
    odometry->y_m += delta_m * sinf(heading_rad);
    odometry->path_length_m += delta_m;
    odometry->heading_rad = heading_rad;
    return 1U;
}

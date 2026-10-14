#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

/*
 * odometry：用左右轮行程 + 航向积分出车体位姿的纯算法模块。
 *
 * 这一层只做数学：不读硬件、不碰 RTOS、不包含逐飞或 FreeRTOS 头文件，
 * 所以能直接在电脑上用 gcc 编译测试（见 tests/host/odometry_test.c）。
 * 由 myApplication/navigation/app_pose.c 调用。
 *
 * 坐标系沿用 docs/VEHICLE_CONVENTIONS.md：x 向前(+X)，y 向左(+Y)，
 * 俯视逆时针为航向正方向。位姿是相对量：原点由 odometry_reset 当时传入的
 * 航向决定，所以起跑线可以直接当作 (0,0)、车头方向当作 0 度，
 * 不需要磁力计提供绝对航向。
 */

#define ODOMETRY_PI_F 3.14159265358979323846f

typedef struct
{
    float x_m;             /* 相对原点的前后位移，+X 向前，单位 m */
    float y_m;             /* 相对原点的左右位移，+Y 向左，单位 m */
    float heading_rad;     /* 相对原点航向的偏角，范围 [-pi, pi]，左转为正 */
    float path_length_m;   /* 自 odometry_reset 起的带符号累计行程，单位 m */
    float yaw_offset_rad;  /* 原点航向，heading_rad = yaw - yaw_offset_rad */
    float yaw_rate_rad_s;  /* 由左右轮行程差算出的车体角速度，用来和 IMU yaw 速率互校 */
    float track_width_m;   /* 轮距，取自 WHEEL_TRACK_M */
    uint8_t origin_set;    /* 0 表示还没定原点，此时 odometry_step 直接返回 0 */
} odometry_t;

/* 设定轮距并清空位姿。track_width_m 必须是有限正数，非法值会被忽略，
 * 此时模块保持不可用（origin_set 为 0）直到传入合法值。 */
void odometry_init(odometry_t *odometry, float track_width_m);

/* 把当前航向 yaw_rad 记为原点航向，并把 x、y、累计行程清零。
 * 发车线、上电第一次可用姿态、退出校准都调用它。yaw_rad 非法时不做任何改动。 */
void odometry_reset(odometry_t *odometry, float yaw_rad);

/* 积分一个采样间隔。left_delta_m / right_delta_m 是自上次调用以来左右轮的
 * 带符号行程（正为前进方向），dt_s 是间隔秒数，只用于换算编码器角速度。
 * 返回 1 表示位姿已更新；空指针、非有限值、dt_s <= 0 或尚未定原点时返回 0。 */
uint8_t odometry_step(odometry_t *odometry, float dt_s, float left_delta_m,
                      float right_delta_m, float yaw_rad);

#endif

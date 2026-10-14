#ifndef APP_POSE_H
#define APP_POSE_H

#include <stdint.h>
#include "app_encoder.h"
#include "odometry.h"

/*
 * app_pose：导航层的位姿接口，把"传感器数据"变成"车在哪、朝哪"。
 *
 * 数据链：receive_task 发布的绝对编码器计数 → app_encoder_tracker 做 16 位
 * 回绕差分得到每轮增量 → 按轮系尺寸换算成轮行程 → odometry 用行程和 yaw
 * 积分出位姿。换算和状态都由本模块持有，decision_task 每个周期调用一次。
 *
 * 坐标系见 docs/VEHICLE_CONVENTIONS.md：x 向前、y 向左、航向左转为正。
 * 原点是相对量，由 app_pose_set_origin 记下当时的航向，因此起跑线可以直接
 * 当作 (0,0)、车头方向当作 0 度。
 */
typedef struct
{
    float x_m;             /* 相对原点前后位移，+X 向前，m */
    float y_m;             /* 相对原点左右位移，+Y 向左，m */
    float heading_deg;     /* 相对原点航向偏角，[-180, 180]，左转为正 */
    float path_length_m;   /* 自原点起的带符号累计行程，m */
    float yaw_rate_rad_s;  /* 编码器推算的车体角速度，可与 IMU yaw 速率互校 */
    /* 定原点那一刻的 IMU yaw。路径跟踪算出的目标航向是"路径坐标系"里的，
     * 要加上这个偏移才能变成转向环使用的 IMU 绝对 yaw。 */
    float yaw_offset_deg;
    uint8_t valid;         /* 0 表示还没定原点或还没积分成功，此时数值不可信 */
} app_pose_t;

typedef struct
{
    app_encoder_tracker_t wheel_tracker;  /* 记住上次的绝对计数，负责差分 */
    odometry_t odometry;                  /* 纯算法：行程 + yaw → 位姿 */
    app_pose_t pose;                      /* 对外发布的当前位姿 */
} app_pose_state_t;

/* 用 general_define.h 里的轮系尺寸初始化，并清空位姿（valid 归 0）。 */
void app_pose_init(app_pose_state_t *state);

/* 丢掉当前位姿和编码器基线，等下一次 app_pose_set_origin 重新定原点。 */
void app_pose_reset(app_pose_state_t *state);

/* 把此刻的 yaw 记为原点航向，并把 x、y、累计行程清零。
 * 发车线、上电第一次姿态可用、退出校准都调用它；yaw_deg 非法时保持不变。 */
void app_pose_set_origin(app_pose_state_t *state, float yaw_deg);

/* 用一帧绝对编码器计数更新一次位姿。dt_s 是距上次调用的秒数。
 * 返回 1 表示位姿已更新；reset/set_origin 之后的第一帧只用来记录基线，返回 0。 */
uint8_t app_pose_step(app_pose_state_t *state,
                      const app_encoder_sample_t *encoder, float yaw_deg,
                      float dt_s);

/* 取当前位姿的快照（拷贝，避免调用方直接改内部状态）。 */
void app_pose_snapshot(const app_pose_state_t *state, app_pose_t *pose);

#endif

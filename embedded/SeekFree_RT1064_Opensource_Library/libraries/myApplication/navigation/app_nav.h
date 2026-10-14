#ifndef APP_NAV_H
#define APP_NAV_H

#include <stdint.h>
#include "app_nav_path.h"
#include "app_pose.h"
#include "pure_pursuit.h"

/*
 * app_nav：把"位姿 + 路径表"变成"目标速度 + 目标航向"。
 *
 * 它就是导航层缺的那一小段胶水：调用 myAlgorithm/path_tracking 的 Pure Pursuit
 * 算出路径坐标系下的期望航向，再加上定原点时的 IMU yaw 偏移，得到转向环能直接
 * 使用的 IMU 绝对航向。由 decision_task 每 10 ms 调用一次。
 *
 * 任何一步不满足（未使能、位姿不可信、路径表为空、输入非法）都会把 active 置 0，
 * 调用方据此回退到"不跟踪"的行为——宁可不动，也不要拿错误的指令去开车。
 */

typedef struct
{
    uint8_t active;          /* 1 = 本拍指令有效可用 */
    uint8_t finished;        /* 1 = 已进入终点判定范围 */
    float target_speed_m_s;  /* 目标线速度，给速度环 */
    float target_yaw_deg;    /* 目标航向，IMU 坐标系，可直接给转向环 */
    float target_heading_deg;/* 路径坐标系下的期望航向，仅用于调试观察 */
    float cross_track_m;     /* 到路径的横向偏差，正=在路径左侧 */
    uint16_t target_index;   /* 当前前视点在路径表中的序号 */
} app_nav_command_t;

typedef struct
{
    pure_pursuit_state_t pursuer;
    pure_pursuit_config_t config;
    app_nav_command_t command;
    uint8_t enabled;
} app_nav_state_t;

/* 用 general_define.h 里的参数配置跟踪器；默认使能状态取
 * NAV_TRACKING_ENABLE_DEFAULT（比赛禁用无线，所以默认关闭）。 */
void app_nav_init(app_nav_state_t *state);

/* 运行期开关，供上位机调试用。关闭时立刻把 active 清 0。 */
void app_nav_set_enabled(app_nav_state_t *state, uint8_t enabled);
uint8_t app_nav_enabled(const app_nav_state_t *state);

/* 换路径、重新发车时调用，让前视点从头开始。 */
void app_nav_reset(app_nav_state_t *state);

/* 算一拍。pose 不可信、路径为空或参数非法时只把 active 清 0。 */
void app_nav_step(app_nav_state_t *state, const app_pose_t *pose);

/* 取最近一次指令快照（拷贝）。 */
void app_nav_snapshot(const app_nav_state_t *state, app_nav_command_t *command);

#endif

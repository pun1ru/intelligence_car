#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <stdint.h>

/*
 * pure_pursuit：轨迹跟踪算法（Pure Pursuit）。
 *
 * 输入是"当前位姿 + 一条折线路径"，输出是"这一拍该朝哪、该走多快"。
 * 它只做几何计算，不读硬件、不碰 RTOS、不包含逐飞头文件，所以能在电脑上
 * 单测（见 tests/host/pure_pursuit_test.c）。由 myApplication/navigation 调用。
 *
 * 坐标系：路径点和位姿必须用同一套坐标——本工程约定发车点为原点、车头方向
 * 为 0 度、+X 向前、+Y 向左、航向逆时针为正（见 docs/VEHICLE_CONVENTIONS.md）。
 *
 * 工作原理：先在路径上找到离车最近的点，再从那里沿路径往前量 lookahead_m
 * 得到"前视点"，让车头朝着它走。前视距离越大走线越平缓、越小贴线越紧；
 * 曲率越大越减速。剩余路径不足一个前视距离时，前视点沿最后一段方向延伸到
 * 终点之外，这样车是"顺着末段方向"进终点，而不是斜插到终点再回正。
 */

typedef struct
{
    float x_m;
    float y_m;
} pure_pursuit_point_t;

typedef struct
{
    float lookahead_m;     /* 前视距离（沿路径量），典型 0.25~0.60 m */
    float speed_max_m_s;   /* 直线段速度上限 */
    float speed_min_m_s;   /* 大曲率时的速度下限。平衡车靠速度维持直立，不能给 0 */
    float slow_radius_m;   /* 曲率半径小于此值开始减速，典型 0.3~0.8 m */
    float goal_radius_m;   /* 距最后一个点小于此值即认为跑完 */
} pure_pursuit_config_t;

typedef struct
{
    float target_speed_m_s;    /* 输出：目标线速度，供速度环 */
    float target_heading_deg;  /* 输出：路径坐标系下的期望航向，供转向环（调用方加原点航向偏移） */
    float yaw_error_deg;       /* 输出：期望航向减当前航向，已回绕到 ±180，左转为正 */
    float cross_track_m;       /* 输出：到路径的横向偏差，正值表示车在路径左侧 */
    uint16_t target_index;     /* 输出：当前前视点在路径表中的序号，便于遥测观察 */
    uint8_t finished;          /* 输出：1 表示已进入终点判定范围 */
} pure_pursuit_output_t;

typedef struct
{
    uint16_t index;  /* 已经跟到第几个路径点。只前进不后退，避免在交叉处来回跳 */
    uint8_t started;
} pure_pursuit_state_t;

/* 清空跟车状态。换路径、重新发车时调用。 */
void pure_pursuit_init(pure_pursuit_state_t *state);

/* 算一拍。path/count 是路径表，x_m/y_m/heading_deg 是当前位姿（同一坐标系），
 * output 返回目标速度和目标航向。返回 1 表示输出有效；路径为空、指针为空、
 * 参数非有限值或 lookahead_m <= 0 时返回 0，此时不修改 output。 */
uint8_t pure_pursuit_step(const pure_pursuit_config_t *config,
                          pure_pursuit_state_t *state,
                          const pure_pursuit_point_t *path, uint16_t count,
                          float x_m, float y_m, float heading_deg,
                          pure_pursuit_output_t *output);

#endif

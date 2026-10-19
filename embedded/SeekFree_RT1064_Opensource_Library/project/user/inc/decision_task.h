#ifndef DECISION_TASK_H
#define DECISION_TASK_H

#include <stdint.h>
#include "app_nav.h"
#include "app_pose.h"

/* decision 任务：目前负责导航位姿，10 ms 一个周期（DECISION_TASK_PERIOD_MS）。
 * 后续的路径表与轨迹跟踪也会放在这里，见 docs/TODO_LIST.md。 */
void decision_task(void *pvParameters);

/* 读取 decision_task 最新算出的位姿（单元素队列，跨任务安全）。
 * 返回 0 表示队列为空或数据超过 VEHICLE_POSE_TIMEOUT_MS 没更新，
 * 此时不改动 *pose，调用方应把自己当作"没有位姿"处理。 */
uint8_t decision_task_get_pose(app_pose_t *pose);

/* 最新的路径跟踪指令（含 active 标志）。返回 0 表示没有新鲜快照，
 * 此时不改动 *nav。语义见 app_nav.h。 */
uint8_t decision_task_get_nav(app_nav_command_t *nav);

/* 请求把"此刻的位置/车头方向"重新定义为原点（x=y=0、heading=0），
 * 并让路径跟踪从第一个点重新开始。任何任务都可以调用，实际复位在
 * decision_task 的下一个周期完成。 */
void decision_task_request_pose_reset(void);

#endif

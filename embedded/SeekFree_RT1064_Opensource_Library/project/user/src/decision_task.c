#include "decision_task.h"

#include "FreeRTOS.h"
#include "app_nav.h"
#include "app_pose.h"
#include "general_define.h"
#include "queue.h"
#include "receive_task.h"
#include "state_task.h"
#include "task.h"
#include "task_metrics.h"

#include <math.h>

/*
 * decision_task：导航层目前唯一在跑的活 —— 维护位姿。
 *
 * 每个 10 ms 周期做三件事：
 *   1. 取三份快照：状态（当前是保护/平衡/导航/校准）、姿态（提供 yaw）、
 *      编码器绝对计数（提供走了多远）；
 *   2. 决定要不要重设原点。三种情况会重设：上电后第一次拿到有效姿态、
 *      进入导航状态（起跑线）、刚退出校准（台架轮速测试会让轮子空转数米，
 *      不重设会变成一次假位移）；
 *   3. 正常周期调用 app_pose_step 积分，并把位姿写进单元素队列，
 *      供 send_task（遥测）和以后的控制使用。
 *
 * 位姿的算法本身在 myAlgorithm/odometry 和 myApplication/navigation/app_pose，
 * 本文件只负责"什么时候算、什么时候重置、怎么发给别人"。
 */

typedef struct
{
    app_pose_t pose;
    app_nav_command_t nav;
    TickType_t publish_tick;  /* 写入时刻，用于判断快照是否已过期 */
} pose_snapshot_t;

static QueueHandle_t pose_queue;
static app_pose_state_t pose_state;
static app_nav_state_t nav_state;
/* 上位机"Reset pose"按钮的请求。单字节写在状态任务、读在 decision 任务，
 * 读写都是单字节，读后清零即可，不需要额外的锁。 */
static volatile uint8_t pose_reset_request;

void decision_task_request_pose_reset(void)
{
    pose_reset_request = 1U;
}

uint8_t decision_task_get_pose(app_pose_t *pose)
{
    pose_snapshot_t snapshot;

    /* xQueuePeek 不取走数据，任何任务都可以重复读同一份最新快照 */
    if ((pose == NULL) || (pose_queue == NULL) ||
        (xQueuePeek(pose_queue, &snapshot, 0U) != pdTRUE) ||
        ((xTaskGetTickCount() - snapshot.publish_tick) >
         pdMS_TO_TICKS(VEHICLE_POSE_TIMEOUT_MS)))
    {
        return 0U;
    }
    *pose = snapshot.pose;
    return 1U;
}

uint8_t decision_task_get_nav(app_nav_command_t *nav)
{
    pose_snapshot_t snapshot;

    if ((nav == NULL) || (pose_queue == NULL) ||
        (xQueuePeek(pose_queue, &snapshot, 0U) != pdTRUE) ||
        ((xTaskGetTickCount() - snapshot.publish_tick) >
         pdMS_TO_TICKS(VEHICLE_POSE_TIMEOUT_MS)))
    {
        return 0U;
    }
    *nav = snapshot.nav;
    return 1U;
}

void decision_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_pose_tick = last_wake_tick;
    app_vehicle_mode_t previous_vehicle_mode = APP_VEHICLE_PROTECT;
    uint8_t previous_calibrating = 0U;
    pose_snapshot_t snapshot;
    /* 1 = 还没定原点。上电时为 1，进入导航、退出校准时重新置 1，
     * 等下一个有效姿态样本把它清掉。上电就开始积分是为了能在台架上手推车
     * 核对里程标定（推 1 m 看 path_length），反正发车时原点会重设。 */
    uint8_t origin_pending = 1U;

    (void)pvParameters;

    pose_queue = xQueueCreate(1U, sizeof(pose_snapshot_t));
    if (pose_queue == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    app_pose_init(&pose_state);
    app_nav_init(&nav_state);

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        app_state_t state;
        app_attitude_t attitude;
        app_encoder_sample_t encoder;
        uint8_t state_valid = state_task_get_snapshot(&state);
        uint8_t attitude_valid = receive_attitude_snapshot(&attitude);
        uint8_t encoder_valid = receive_encoder_snapshot(&encoder);
        TickType_t now = xTaskGetTickCount();
        /* 距上次成功积分的间隔。用真实 tick 差而不是固定周期：
         * 这个任务优先级低，偶尔会被高优先级任务挤掉一两个周期。 */
        float dt_s = (float)(now - last_pose_tick) / (float)configTICK_RATE_HZ;

        /* 上位机要求复位：清位姿、把跟踪进度归零，下一拍用当前车头重新定原点。
         * 放在状态判断之外，这样保护状态下手推车也能复位。 */
        if (pose_reset_request != 0U)
        {
            pose_reset_request = 0U;
            app_pose_reset(&pose_state);
            app_nav_reset(&nav_state);
            origin_pending = 1U;
            last_pose_tick = now;
        }

        /* 状态快照拿不到时（超过 VEHICLE_STATE_TIMEOUT_MS 没更新）就整轮跳过，
         * 宁可少算一次，也不要凭猜测的车辆模式去重设原点。 */
        if (state_valid != 0U)
        {
            uint8_t navigating =
                (state.vehicle_mode == APP_VEHICLE_NAVIGATION) ? 1U : 0U;
            /* 只在"这一拍刚进导航"时重设原点，而不是整个导航期间每周期都重设 */
            uint8_t navigation_entered =
                ((navigating != 0U) &&
                 (previous_vehicle_mode != APP_VEHICLE_NAVIGATION)) ? 1U : 0U;
            uint8_t calibrating =
                (state.control_mode == APP_STATE_CALIBRATION) ? 1U : 0U;
            /* 校准模式不积分（那里会按住 C15/C14 让轮子空转），
             * 但退出时必须重建编码器基线，否则这段空转会被算成一大段位移。 */
            uint8_t calibration_left =
                ((calibrating == 0U) && (previous_calibrating != 0U)) ? 1U : 0U;

            if ((navigation_entered != 0U) || (calibration_left != 0U))
            {
                /* 比赛从起跑线前 1 m 内出发，所以"进入导航"就是这一趟的原点：
                 * 当前位置记为 (0,0)，此刻车头方向记为 0 度。 */
                app_pose_reset(&pose_state);
                origin_pending = 1U;
            }

            /* 第一分支：还没定原点，拿到有效 yaw 就定下来，这一拍不积分 */
            if ((origin_pending != 0U) && (attitude_valid != 0U) &&
                isfinite(attitude.yaw))
            {
                app_pose_set_origin(&pose_state, attitude.yaw);
                origin_pending = 0U;
                last_pose_tick = now;
            }
            /* 第二分支：正常积分。姿态或编码器快照不可用就跳过这一拍，
             * 因为差分用的是绝对计数，漏掉一拍不会丢里程，只是这次的
             * dt 会偏大、编码器角速度诊断量不准。 */
            else if ((origin_pending == 0U) && (calibrating == 0U) &&
                     (attitude_valid != 0U) && (encoder_valid != 0U) &&
                     (dt_s > 0.0f) && isfinite(attitude.yaw))
            {
                /* 里程来自计数积分，dt_s 只影响编码器角速度，所以偶发丢拍
                 * 不会让位置跑偏。 */
                (void)app_pose_step(&pose_state, &encoder, attitude.yaw, dt_s);
                last_pose_tick = now;
            }

            /* 记住这一拍的模式，下一拍才能识别"刚进导航""刚退出校准" */
            previous_vehicle_mode = state.vehicle_mode;
            previous_calibrating = calibrating;
        }

        /* 路径跟踪：算目标速度与目标航向。这一阶段只计算并发布，不驱动电机，
         * 所以未使能、位姿不可信、路径为空时它什么都不做（active 为 0）。 */
        /* 跟踪指令在每个周期都算一次，**不要求处于导航状态**：只有"是否用它驱动
         * 电机"才需要导航状态（判断在 app_control 里）。这样台架验证可以在保护
         * 状态下手推着车做——电机不工作，零风险，也不需要改任何开关。
         * 位姿不可信时 app_nav_step 自己会把指令标为无效。 */
        app_nav_step(&nav_state, &pose_state.pose);

        /* 不管这一拍算没算，都发布一次：valid 位会告诉上位机数据可不可信 */
        app_pose_snapshot(&pose_state, &snapshot.pose);
        app_nav_snapshot(&nav_state, &snapshot.nav);
        snapshot.publish_tick = xTaskGetTickCount();
        (void)xQueueOverwrite(pose_queue, &snapshot);
        task_metrics_end(TASK_METRIC_DECISION, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DECISION_TASK_PERIOD_MS));
    }
}

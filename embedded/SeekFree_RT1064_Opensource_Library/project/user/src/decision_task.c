#include "decision_task.h"

#include "FreeRTOS.h"
#include "app_pose.h"
#include "general_define.h"
#include "queue.h"
#include "receive_task.h"
#include "state_task.h"
#include "task.h"
#include "task_metrics.h"

#include <math.h>

typedef struct
{
    app_pose_t pose;
    TickType_t publish_tick;
} pose_snapshot_t;

static QueueHandle_t pose_queue;
static app_pose_state_t pose_state;

uint8_t decision_task_get_pose(app_pose_t *pose)
{
    pose_snapshot_t snapshot;

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

void decision_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_pose_tick = last_wake_tick;
    app_vehicle_mode_t previous_vehicle_mode = APP_VEHICLE_PROTECT;
    pose_snapshot_t snapshot;
    /* Set between entering navigation and latching the origin, which the first
     * valid attitude sample completes. */
    uint8_t origin_pending = 0U;

    (void)pvParameters;

    pose_queue = xQueueCreate(1U, sizeof(pose_snapshot_t));
    if (pose_queue == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    app_pose_init(&pose_state);

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
        /* Interval since the previous integration, so a skipped cycle is not
         * reported as a shorter one. */
        float dt_s = (float)(now - last_pose_tick) / (float)configTICK_RATE_HZ;

        if (state_valid != 0U)
        {
            uint8_t navigating =
                (state.vehicle_mode == APP_VEHICLE_NAVIGATION) ? 1U : 0U;
            uint8_t navigation_entered =
                ((navigating != 0U) &&
                 (previous_vehicle_mode != APP_VEHICLE_NAVIGATION)) ? 1U : 0U;
            uint8_t navigation_left =
                ((navigating == 0U) &&
                 (previous_vehicle_mode == APP_VEHICLE_NAVIGATION)) ? 1U : 0U;

            if (navigation_entered != 0U)
            {
                /* The rules start the run from the launch line, so entering
                 * navigation makes that position the origin with heading 0. */
                app_pose_reset(&pose_state);
                origin_pending = 1U;
            }
            else if (navigation_left != 0U)
            {
                /* The pose is only meaningful inside a run. */
                app_pose_reset(&pose_state);
                origin_pending = 0U;
            }

            if ((navigating != 0U) && (origin_pending != 0U) &&
                (attitude_valid != 0U) && isfinite(attitude.yaw))
            {
                app_pose_set_origin(&pose_state, attitude.yaw);
                origin_pending = 0U;
                last_pose_tick = now;
            }
            else if ((navigating != 0U) && (origin_pending == 0U) &&
                     (attitude_valid != 0U) && (encoder_valid != 0U) &&
                     (dt_s > 0.0f) && isfinite(attitude.yaw))
            {
                /* The travelled distance comes from encoder counts, so dt_s
                 * only scales the encoder yaw rate diagnostic. */
                (void)app_pose_step(&pose_state, &encoder, attitude.yaw, dt_s);
                last_pose_tick = now;
            }

            previous_vehicle_mode = state.vehicle_mode;
        }

        app_pose_snapshot(&pose_state, &snapshot.pose);
        snapshot.publish_tick = xTaskGetTickCount();
        (void)xQueueOverwrite(pose_queue, &snapshot);
        task_metrics_end(TASK_METRIC_DECISION, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DECISION_TASK_PERIOD_MS));
    }
}

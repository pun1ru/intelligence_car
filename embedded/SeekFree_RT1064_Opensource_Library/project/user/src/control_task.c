#include "control_task.h"

#include "app_control.h"
#include "general_define.h"
#include "receive_task.h"
#include "state_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "task_metrics.h"

typedef struct
{
    app_control_telemetry_t telemetry;
    TickType_t publish_tick;
} control_snapshot_t;

static QueueHandle_t control_queue;

uint8_t control_task_get_snapshot(app_control_telemetry_t *telemetry)
{
    control_snapshot_t snapshot;

    if ((telemetry == NULL) || (control_queue == NULL) ||
        (xQueuePeek(control_queue, &snapshot, 0U) != pdTRUE) ||
        ((xTaskGetTickCount() - snapshot.publish_tick) >
         pdMS_TO_TICKS(VEHICLE_CONTROL_TIMEOUT_MS)))
    {
        return 0U;
    }
    *telemetry = snapshot.telemetry;
    return 1U;
}

void control_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    control_queue = xQueueCreate(1U, sizeof(control_snapshot_t));
    if (control_queue == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    for (;;)
    {
        app_state_t state;
        app_attitude_t attitude;
        const app_state_t *state_snapshot =
            (state_task_get_snapshot(&state) != 0U) ? &state : NULL;
        const app_attitude_t *attitude_snapshot =
            (receive_attitude_snapshot(&attitude) != 0U) ? &attitude : NULL;
        uint32_t metric_start = task_metrics_begin();
        if ((state_snapshot != NULL) &&
            (state_snapshot->pid_update_requested != 0U))
        {
            (void)app_control_apply_pid_update(
                state_snapshot->pid_controller,
                state_snapshot->pid_parameter,
                state_snapshot->pid_value);
        }
        if (app_control_step(state_snapshot, attitude_snapshot) != 0U)
        {
            state_task_request_protection();
        }
        control_snapshot_t snapshot;
        app_control_get_telemetry(&snapshot.telemetry);
        snapshot.publish_tick = xTaskGetTickCount();
        (void)xQueueOverwrite(control_queue, &snapshot);
        task_metrics_end(TASK_METRIC_CONTROL, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

#include "control_task.h"

#include "app_control.h"
#include "general_define.h"
#include "receive_task.h"
#include "state_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_metrics.h"

void control_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    for (;;)
    {
        app_state_t state;
        app_attitude_t attitude;
        const app_state_t *state_snapshot =
            (state_task_get_snapshot(&state) != 0U) ? &state : NULL;
        const app_attitude_t *attitude_snapshot =
            (receive_attitude_snapshot(&attitude) != 0U) ? &attitude : NULL;
        uint32_t metric_start = task_metrics_begin();
        if (app_control_step(state_snapshot, attitude_snapshot) != 0U)
        {
            state_task_request_protection();
        }
        task_metrics_end(TASK_METRIC_CONTROL, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

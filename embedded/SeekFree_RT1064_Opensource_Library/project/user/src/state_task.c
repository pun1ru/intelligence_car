#include "general_include.h"

void state_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        task_metrics_end(TASK_METRIC_STATE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(STATE_TASK_PERIOD_MS));
    }
}

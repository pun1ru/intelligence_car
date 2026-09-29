#include "general_include.h"

void decision_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        task_metrics_end(TASK_METRIC_DECISION, metric_start);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

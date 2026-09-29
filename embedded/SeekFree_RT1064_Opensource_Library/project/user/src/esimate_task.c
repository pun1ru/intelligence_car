#include "general_include.h"

void estimate_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        task_metrics_end(TASK_METRIC_ESTIMATE, metric_start);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

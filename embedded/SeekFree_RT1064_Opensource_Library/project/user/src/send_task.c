#include "general_include.h"

void send_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

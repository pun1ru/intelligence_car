#include "send_task.h"

#include "app_serial.h"
#include "general_define.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_metrics.h"

void send_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        app_serial_send();
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SEND_TASK_PERIOD_MS));
    }
}

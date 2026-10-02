#include "app_system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "initial_task.h"
#include "task_metrics.h"

int main(void)
{
    app_system_platform_init();
    task_metrics_init();

    if (xTaskCreate(initial_task, "initial", 512U, NULL, 3U, NULL) == pdPASS)
    {
        vTaskStartScheduler();
    }
    for (;;)
    {
    }
}

#include "debug_task.h"

#include "app_display.h"
#include "general_define.h"
#include "receive_task.h"
#include "task_metrics.h"

void debug_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    app_display_init();
    for (;;)
    {
        app_attitude_t attitude;
        app_encoder_sample_t encoder;

        if (receive_display_snapshot(&attitude, &encoder) != 0U)
        {
            app_display_update(&attitude, &encoder);
        }
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DEBUG_TASK_PERIOD_MS));
    }
}

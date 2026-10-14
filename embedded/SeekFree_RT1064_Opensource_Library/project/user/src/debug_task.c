#include "debug_task.h"

#include "app_display.h"
#include "decision_task.h"
#include "general_define.h"
#include "receive_task.h"
#include "task_metrics.h"

void debug_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_sample_tick = last_wake_tick;

    (void)pvParameters;
    app_display_init();
    for (;;)
    {
        app_attitude_t attitude;
        app_encoder_sample_t encoder;
        app_pose_t pose;
        /* 位姿取不到位姿（没定原点或快照过期）就传 NULL，屏幕显示 "--" */
        const app_pose_t *pose_snapshot =
            (decision_task_get_pose(&pose) != 0U) ? &pose : NULL;

        if (receive_display_snapshot(&attitude, &encoder) != 0U)
        {
            TickType_t now = xTaskGetTickCount();
            app_display_update(&attitude, &encoder,
                               (float)(now - last_sample_tick) /
                               (float)configTICK_RATE_HZ,
                               pose_snapshot);
            last_sample_tick = now;
        }
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DEBUG_TASK_PERIOD_MS));
    }
}

#include "string.h"
#include "MIMXRT1064.h"
#include "zf_common_headfile.h"
#include "general_define.h"
#include "receive_task.h"
#include "task_metrics.h"

extern uint32_t SystemCoreClock;

task_runtime_metrics_t g_task_runtime;

void debug_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;

    tft180_init();
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_RED, RGB565_WHITE);
    tft180_show_string(0, 0, "IMU ANGLE");
    tft180_show_string(0, 16, "Roll:");
    tft180_show_string(0, 32, "Pitch:");
    tft180_show_string(0, 48, "Yaw:");
    tft180_show_string(0, 64, "EncL:");
    tft180_show_string(0, 80, "EncR:");

    for (;;)
    {
        float roll = g_imu_receive_data.roll;
        float pitch = g_imu_receive_data.pitch;
        float yaw = g_imu_receive_data.yaw;
        int16 left_count = g_encoder_receive_data.left_count;
        int16 right_count = g_encoder_receive_data.right_count;

        tft180_show_float(48, 16, roll, 3, 1);
        tft180_show_float(48, 32, pitch, 3, 1);
        tft180_show_float(48, 48, yaw, 3, 1);
        tft180_show_int(48, 64, left_count, 6);
        tft180_show_int(48, 80, right_count, 6);

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DEBUG_TASK_PERIOD_MS));
    }
}

void task_metrics_init(void)
{
    static const char *const task_names[TASK_METRIC_COUNT] =
    {
        "initial", "state", "decision", "estimate",
        "control", "receive", "send"
    };
    uint32_t index;

    memset(&g_task_runtime, 0, sizeof(g_task_runtime));
    for (index = 0; index < TASK_METRIC_COUNT; index++)
    {
        g_task_runtime.task[index].name = task_names[index];
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t task_metrics_begin(void)
{
    return DWT->CYCCNT;
}

void task_metrics_end(task_metric_id_t id, uint32_t start_cycle)
{
    task_runtime_metric_t *metric;
    TickType_t current_tick;
    TickType_t period_ticks;
    uint32_t elapsed_cycles;
    uint32_t cycles_per_us;
    uint32_t elapsed_us;

    if (id >= TASK_METRIC_COUNT)
    {
        return;
    }

    metric = &g_task_runtime.task[id];
    current_tick = xTaskGetTickCount();
    elapsed_cycles = DWT->CYCCNT - start_cycle;
    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }
    elapsed_us = elapsed_cycles / cycles_per_us;

    if (metric->run_count > 0U)
    {
        period_ticks = current_tick - metric->last_start_tick;
        metric->last_period_ticks = period_ticks;
        if (period_ticks > metric->max_period_ticks)
        {
            metric->max_period_ticks = period_ticks;
        }
    }
    metric->last_start_tick = current_tick;
    metric->last_elapsed_us = elapsed_us;
    if (elapsed_us > metric->max_elapsed_us)
    {
        metric->max_elapsed_us = elapsed_us;
    }
    metric->total_elapsed_us += elapsed_us;
    metric->run_count++;
    metric->handle = xTaskGetCurrentTaskHandle();
    metric->stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
}

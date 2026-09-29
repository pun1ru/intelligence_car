#ifndef TASK_METRICS_H_
#define TASK_METRICS_H_

#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    TASK_METRIC_INITIAL = 0,
    TASK_METRIC_STATE,
    TASK_METRIC_DECISION,
    TASK_METRIC_ESTIMATE,
    TASK_METRIC_CONTROL,
    TASK_METRIC_RECEIVE,
    TASK_METRIC_SEND,
    TASK_METRIC_COUNT
} task_metric_id_t;

typedef struct
{
    const char *name;
    TaskHandle_t handle;
    uint32_t last_elapsed_us;
    uint32_t max_elapsed_us;
    uint64_t total_elapsed_us;
    uint32_t run_count;
    TickType_t last_start_tick;
    TickType_t last_period_ticks;
    TickType_t max_period_ticks;
    UBaseType_t stack_high_water_mark;
} task_runtime_metric_t;

typedef struct
{
    task_runtime_metric_t task[TASK_METRIC_COUNT];
} task_runtime_metrics_t;

extern task_runtime_metrics_t g_task_runtime;

void task_metrics_init(void);
uint32_t task_metrics_begin(void);
void task_metrics_end(task_metric_id_t id, uint32_t start_cycle);

#endif

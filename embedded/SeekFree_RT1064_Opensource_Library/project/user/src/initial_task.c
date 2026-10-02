#include "initial_task.h"

#include "app_system.h"
#include "control_task.h"
#include "debug_task.h"
#include "decision_task.h"
#include "estimate_task.h"
#include "general_define.h"
#include "receive_task.h"
#include "send_task.h"
#include "state_task.h"
#include "task.h"
#include "task_metrics.h"

static TaskHandle_t state_task_handle;
static TaskHandle_t decision_task_handle;
static TaskHandle_t estimate_task_handle;
static TaskHandle_t control_task_handle;
static TaskHandle_t receive_task_handle;
static TaskHandle_t send_task_handle;
static TaskHandle_t debug_task_handle;

static void delete_created_tasks(void)
{
    TaskHandle_t *handles[] =
    {
        &state_task_handle, &decision_task_handle, &estimate_task_handle,
        &control_task_handle, &receive_task_handle, &send_task_handle,
        &debug_task_handle
    };
    uint8_t index;

    for (index = 0U; index < (sizeof(handles) / sizeof(handles[0])); index++)
    {
        if (*handles[index] != NULL)
        {
            vTaskDelete(*handles[index]);
            *handles[index] = NULL;
        }
    }
}

void initial_task(void *pvParameters)
{
    uint32_t metric_start;

    (void)pvParameters;
    metric_start = task_metrics_begin();
    app_system_devices_init(CONTROL_TASK_PERIOD_MS);
    if (receive_data_init() == 0U)
    {
        task_metrics_end(TASK_METRIC_INITIAL, metric_start);
        vTaskDelete(NULL);
        return;
    }

    if ((xTaskCreate(state_task, STATE_TASK_NAME, STATE_TASK_STACK_SIZE,
                     NULL, STATE_TASK_PRIORITY, &state_task_handle) != pdPASS) ||
        (xTaskCreate(decision_task, DECISION_TASK_NAME, DECISION_TASK_STACK_SIZE,
                     NULL, DECISION_TASK_PRIORITY, &decision_task_handle) != pdPASS) ||
        (xTaskCreate(estimate_task, ESTIMATE_TASK_NAME, ESTIMATE_TASK_STACK_SIZE,
                     NULL, ESTIMATE_TASK_PRIORITY, &estimate_task_handle) != pdPASS) ||
        (xTaskCreate(control_task, CONTROL_TASK_NAME, CONTROL_TASK_STACK_SIZE,
                     NULL, CONTROL_TASK_PRIORITY, &control_task_handle) != pdPASS) ||
        (xTaskCreate(receive_task, RECEIVE_TASK_NAME, RECEIVE_TASK_STACK_SIZE,
                     NULL, RECEIVE_TASK_PRIORITY, &receive_task_handle) != pdPASS) ||
        (xTaskCreate(send_task, SEND_TASK_NAME, SEND_TASK_STACK_SIZE,
                     NULL, SEND_TASK_PRIORITY, &send_task_handle) != pdPASS) ||
        (xTaskCreate(debug_task, DEBUG_TASK_NAME, DEBUG_TASK_STACK_SIZE,
                     NULL, DEBUG_TASK_PRIORITY, &debug_task_handle) != pdPASS))
    {
        delete_created_tasks();
    }

    task_metrics_end(TASK_METRIC_INITIAL, metric_start);
    vTaskDelete(NULL);
}

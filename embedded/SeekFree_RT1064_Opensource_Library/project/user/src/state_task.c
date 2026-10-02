#include "state_task.h"

#include "app_serial.h"
#include "app_state.h"
#include "estimate_task.h"
#include "FreeRTOS.h"
#include "general_define.h"
#include "queue.h"
#include "receive_task.h"
#include "task.h"
#include "task_metrics.h"

#define STATE_TASK_PROTECT_REQUEST 1U

typedef struct
{
    app_state_t state;
    TickType_t publish_tick;
} state_snapshot_t;

static TaskHandle_t state_handle;
static QueueHandle_t state_queue;

uint8_t state_task_get_snapshot(app_state_t *state)
{
    state_snapshot_t snapshot;

    if ((state == NULL) || (state_queue == NULL) ||
        (xQueuePeek(state_queue, &snapshot, 0U) != pdTRUE) ||
        ((xTaskGetTickCount() - snapshot.publish_tick) >
         pdMS_TO_TICKS(VEHICLE_STATE_TIMEOUT_MS)))
    {
        return 0U;
    }
    *state = snapshot.state;
    return 1U;
}

void state_task_request_protection(void)
{
    if (state_handle != NULL)
    {
        (void)xTaskNotify(state_handle, STATE_TASK_PROTECT_REQUEST, eSetBits);
    }
}

void state_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    app_state_t state;
    app_state_buttons_t buttons;
    app_attitude_t attitude;
    app_serial_command_t command;
    state_snapshot_t snapshot;
    app_control_mode_t previous_mode;
    uint8_t attitude_valid;
    uint8_t command_valid;
    uint32_t request;

    (void)pvParameters;
    state_handle = xTaskGetCurrentTaskHandle();
    state_queue = xQueueCreate(1U, sizeof(state_snapshot_t));
    if (state_queue == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    app_state_init(&state);

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        app_state_read_buttons(&buttons);
        attitude_valid = receive_attitude_snapshot(&attitude);
        command_valid = app_serial_poll_command(&command);
        previous_mode = state.control_mode;
        app_state_step(&state, &buttons, attitude_valid,
                       (attitude_valid != 0U) ? attitude.pitch : 0.0f,
                       STATE_TASK_PERIOD_MS,
                       (command_valid != 0U) ? &command : NULL);
        request = 0U;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &request, 0U);
        if ((request & STATE_TASK_PROTECT_REQUEST) != 0U)
        {
            app_state_force_protection(&state);
        }
        snapshot.state = state;
        snapshot.publish_tick = xTaskGetTickCount();
        (void)xQueueOverwrite(state_queue, &snapshot);
        if ((previous_mode == APP_STATE_CALIBRATION) &&
            (state.control_mode != APP_STATE_CALIBRATION))
        {
            estimate_task_request(ESTIMATE_REQUEST_CANCEL);
        }
        else if (state.calibrate_requested != 0U)
        {
            estimate_task_request(ESTIMATE_REQUEST_CALIBRATE);
        }
        else if (state.reset_ekf_requested != 0U)
        {
            estimate_task_request(ESTIMATE_REQUEST_RESET);
        }
        task_metrics_end(TASK_METRIC_STATE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(STATE_TASK_PERIOD_MS));
    }
}

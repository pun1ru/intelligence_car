#include "state_task.h"

#include "app_serial.h"
#include "app_state.h"
#include "estimate_task.h"
#include "FreeRTOS.h"
#include "general_define.h"
#include "receive_task.h"
#include "task.h"
#include "task_metrics.h"

void state_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    app_state_t state;
    app_state_buttons_t buttons;
    app_attitude_t attitude;
    app_serial_command_t command;
    app_control_mode_t previous_mode;
    uint8_t attitude_valid;
    uint8_t command_valid;

    (void)pvParameters;
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

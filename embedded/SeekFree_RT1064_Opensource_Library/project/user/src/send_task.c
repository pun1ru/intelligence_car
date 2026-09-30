#include "zf_common_headfile.h"
#include "general_include.h"

static const uint8 test_frame[] = {0xAA, 0x55, 0x01, 0x00, 0x01, 0xFE};

void send_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    uart_init(UART_8, 115200U, UART8_TX_D16, UART8_RX_D17);

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        uart_write_buffer(UART_8, test_frame, sizeof(test_frame));
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SEND_TASK_PERIOD_MS));
    }
}

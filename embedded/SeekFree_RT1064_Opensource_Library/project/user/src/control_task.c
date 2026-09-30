#include "zf_common_headfile.h"
#include "general_include.h"

void control_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        if (gpio_get_level(C12) == GPIO_LOW)
        {
            gpio_set_level(MOTOR_RIGHT_DIR, GPIO_HIGH);
            pwm_set_duty(MOTOR_RIGHT_PWM, MOTOR_TEST_DUTY);
        }
        else
        {
            pwm_set_duty(MOTOR_RIGHT_PWM, 0U);
            gpio_set_level(MOTOR_RIGHT_DIR, GPIO_LOW);
        }

        task_metrics_end(TASK_METRIC_CONTROL, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

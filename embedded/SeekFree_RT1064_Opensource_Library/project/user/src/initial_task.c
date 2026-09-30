#include "zf_common_headfile.h"
#include "general_define.h"
#include "general_include.h"
#include "receive_task.h"

TaskHandle_t stateTaskHandle;
TaskHandle_t decisionTaskHandle;
TaskHandle_t estimateTaskHnadle;
TaskHandle_t controlTaskHandle;
TaskHandle_t receiveTaskHandel;
TaskHandle_t sendTaskHandle;
TaskHandle_t debugTaskHandle;

static volatile uint8 buzzer_fast_running;

void buzzer_init(void)
{
    gpio_init(B11, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void buzzer_stop(void)
{
    buzzer_fast_running = 0U;
    gpio_set_level(B11, GPIO_LOW);
}

static void buzzer_pulse(void)
{
    gpio_set_level(B11, GPIO_HIGH);
    system_delay_ms(200U);
    gpio_set_level(B11, GPIO_LOW);
}

void buzzer_play(buzzer_mode_t mode)
{
    uint8 count;

    buzzer_init();
    if (mode == BUZZER_MODE_FAST_CONTINUOUS)
    {
        buzzer_fast_running = 1U;
        while (buzzer_fast_running != 0U)
        {
            gpio_set_level(B11, GPIO_HIGH);
            system_delay_ms(50U);
            gpio_set_level(B11, GPIO_LOW);
            system_delay_ms(50U);
        }
        return;
    }

    count = (uint8)mode;
    if (count > 4U)
    {
        count = 1U;
    }
    while (count-- > 0U)
    {
        buzzer_pulse();
        if (count > 0U)
        {
            system_delay_ms(100U);
        }
    }
}

void buzzer_beep_once(void) { buzzer_play(BUZZER_MODE_ONCE); }
void buzzer_beep_twice(void) { buzzer_play(BUZZER_MODE_TWICE); }
void buzzer_beep_three(void) { buzzer_play(BUZZER_MODE_THREE); }
void buzzer_beep_four(void) { buzzer_play(BUZZER_MODE_FOUR); }
void buzzer_beep_fast(void) { buzzer_play(BUZZER_MODE_FAST_CONTINUOUS); }

void initial_task(void *pvParameters)
{
    uint32_t metric_start;
    uint8 imu_init_status;

    (void)pvParameters;
    metric_start = task_metrics_begin();
    buzzer_beep_once();
    uart_init(UART_8, 115200U, UART8_TX_D16, UART8_RX_D17);
    imu_init_status = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
    g_imu_receive_data.imu_ready = (imu_init_status == 0U) ? 1U : 0U;
    encoder_quad_init(ENCODER_LEFT_INDEX, ENCODER_LEFT_CH1, ENCODER_LEFT_CH2);
    encoder_quad_init(ENCODER_RIGHT_INDEX, ENCODER_RIGHT_CH1, ENCODER_RIGHT_CH2);
    encoder_clear_count(ENCODER_LEFT_INDEX);
    encoder_clear_count(ENCODER_RIGHT_INDEX);
    g_imu_data_ready_sem = xSemaphoreCreateBinary();

    taskENTER_CRITICAL();
    xTaskCreate(state_task, "state_task", STATE_TASK_STACK_SIZE, NULL, STATE_TASK_PRIORITY, &stateTaskHandle);
    xTaskCreate(decision_task, "decision_task", DECISION_TASK_STACK_SIZE, NULL, DECISION_TASK_PRIORITY, &decisionTaskHandle);
    xTaskCreate(estimate_task, "estimate_task", ESTIMATE_TASK_STACK_SIZE, NULL, ESTIMATE_TASK_PRIORITY, &estimateTaskHnadle);
    xTaskCreate(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &controlTaskHandle);
    xTaskCreate(receive_task, "receive_task", RECEIVE_TASK_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, &receiveTaskHandel);
    xTaskCreate(send_task, "send_task", SEND_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY, &sendTaskHandle);
    xTaskCreate(debug_task, DEBUG_TASK_NAME, DEBUG_TASK_STACK_SIZE, NULL, DEBUG_TASK_PRIORITY, &debugTaskHandle);
    taskEXIT_CRITICAL();

    task_metrics_end(TASK_METRIC_INITIAL, metric_start);
    vTaskDelete(NULL);
}

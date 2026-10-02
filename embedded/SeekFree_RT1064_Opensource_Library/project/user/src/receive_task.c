#include "receive_task.h"

#include "app_calibration.h"
#include "app_sensor.h"
#include "general_define.h"
#include "queue.h"
#include "task.h"
#include "task_metrics.h"

SemaphoreHandle_t g_imu_data_ready_sem;

static QueueHandle_t imu_sample_queue;
static QueueHandle_t attitude_queue;
static QueueHandle_t encoder_queue;
static TickType_t attitude_publish_tick;

uint8_t receive_data_init(void)
{
    imu_sample_queue = xQueueCreate(1U, sizeof(app_imu_sample_t));
    attitude_queue = xQueueCreate(1U, sizeof(app_attitude_t));
    encoder_queue = xQueueCreate(1U, sizeof(app_encoder_sample_t));
    g_imu_data_ready_sem = xSemaphoreCreateBinary();

    if ((imu_sample_queue == NULL) || (attitude_queue == NULL) ||
        (encoder_queue == NULL) || (g_imu_data_ready_sem == NULL))
    {
        if (imu_sample_queue != NULL)
        {
            vQueueDelete(imu_sample_queue);
            imu_sample_queue = NULL;
        }
        if (attitude_queue != NULL)
        {
            vQueueDelete(attitude_queue);
            attitude_queue = NULL;
        }
        if (encoder_queue != NULL)
        {
            vQueueDelete(encoder_queue);
            encoder_queue = NULL;
        }
        if (g_imu_data_ready_sem != NULL)
        {
            vSemaphoreDelete(g_imu_data_ready_sem);
            g_imu_data_ready_sem = NULL;
        }
        return 0U;
    }
    return 1U;
}

uint8_t receive_imu_snapshot(app_imu_sample_t *sample)
{
    if ((sample == NULL) || (imu_sample_queue == NULL))
    {
        return 0U;
    }
    return (xQueuePeek(imu_sample_queue, sample, 0U) == pdTRUE) ? 1U : 0U;
}

uint8_t receive_display_snapshot(app_attitude_t *attitude,
                                 app_encoder_sample_t *encoder)
{
    if ((attitude == NULL) || (encoder == NULL) ||
        (attitude_queue == NULL) || (encoder_queue == NULL))
    {
        return 0U;
    }
    if (xQueuePeek(attitude_queue, attitude, 0U) != pdTRUE)
    {
        return 0U;
    }
    return (xQueuePeek(encoder_queue, encoder, 0U) == pdTRUE) ? 1U : 0U;
}

uint8_t receive_attitude_snapshot(app_attitude_t *attitude)
{
    if ((attitude == NULL) || (attitude_queue == NULL) ||
        (xQueuePeek(attitude_queue, attitude, 0U) != pdTRUE))
    {
        return 0U;
    }
    return ((xTaskGetTickCount() - attitude_publish_tick) <=
            pdMS_TO_TICKS(VEHICLE_ATTITUDE_TIMEOUT_MS)) ? 1U : 0U;
}

void receive_publish_attitude(const app_attitude_t *attitude)
{
    if ((attitude != NULL) && (attitude_queue != NULL))
    {
        (void)xQueueOverwrite(attitude_queue, attitude);
        attitude_publish_tick = xTaskGetTickCount();
    }
}

static void calibration_wait_for_imu(TickType_t *last_wake_tick)
{
    app_imu_sample_t sample;

    while ((app_sensor_imu_ready() == 0U) ||
           (receive_imu_snapshot(&sample) == 0U))
    {
        vTaskDelayUntil(last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }
}

void imu_accel_noise_calibrate(void)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t start_tick;
    app_imu_sample_t sample;
    uint32_t last_sample_count;

    app_calibration_accel_start();
    calibration_wait_for_imu(&last_wake_tick);
    (void)receive_imu_snapshot(&sample);
    last_sample_count = sample.sample_count;
    start_tick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(IMU_ACCEL_CALIBRATION_TIME_MS))
    {
        if ((receive_imu_snapshot(&sample) != 0U) &&
            (sample.sample_count != last_sample_count))
        {
            last_sample_count = sample.sample_count;
            app_calibration_accel_sample(&sample);
        }
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }
    app_calibration_accel_complete();
}

void imu_gyro_noise_calibrate(void)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t start_tick;
    TickType_t total_ticks = pdMS_TO_TICKS(IMU_GYRO_CALIBRATION_TIME_MS);
    TickType_t bias_ticks = total_ticks / 2U;
    app_imu_sample_t sample;
    uint32_t last_sample_count;

    app_calibration_gyro_start();
    calibration_wait_for_imu(&last_wake_tick);
    (void)receive_imu_snapshot(&sample);
    last_sample_count = sample.sample_count;
    start_tick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_tick) < bias_ticks)
    {
        if ((receive_imu_snapshot(&sample) != 0U) &&
            (sample.sample_count != last_sample_count))
        {
            last_sample_count = sample.sample_count;
            app_calibration_gyro_bias_sample(&sample);
        }
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    start_tick = xTaskGetTickCount();
    (void)receive_imu_snapshot(&sample);
    last_sample_count = sample.sample_count;
    while ((xTaskGetTickCount() - start_tick) < (total_ticks - bias_ticks))
    {
        if ((receive_imu_snapshot(&sample) != 0U) &&
            (sample.sample_count != last_sample_count))
        {
            last_sample_count = sample.sample_count;
            app_calibration_gyro_noise_sample(&sample);
        }
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }
    app_calibration_gyro_complete();
}

void receive_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    app_imu_sample_t imu;
    app_encoder_sample_t encoder;

    (void)pvParameters;
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        if ((imu_sample_queue != NULL) && (app_sensor_read_imu(&imu) != 0U))
        {
            imu.capture_tick = (uint32_t)xTaskGetTickCount();
            (void)xQueueOverwrite(imu_sample_queue, &imu);
            if (g_imu_data_ready_sem != NULL)
            {
                (void)xSemaphoreGive(g_imu_data_ready_sem);
            }
        }
        if (encoder_queue != NULL)
        {
            app_sensor_read_encoder(&encoder);
            (void)xQueueOverwrite(encoder_queue, &encoder);
        }
        task_metrics_end(TASK_METRIC_RECEIVE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }
}

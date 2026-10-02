#include "estimate_task.h"

#include "app_attitude.h"
#include "app_calibration.h"
#include "general_define.h"
#include "receive_task.h"
#include "task.h"
#include "task_metrics.h"
#include <math.h>

static TaskHandle_t estimate_handle;

void estimate_task_request(uint32_t request)
{
    if (estimate_handle != NULL)
    {
        (void)xTaskNotify(estimate_handle, request, eSetBits);
    }
}

static uint8_t calibration_sample_valid(const app_imu_sample_t *sample)
{
    float acc_norm2 = sample->acc_x_g * sample->acc_x_g +
                      sample->acc_y_g * sample->acc_y_g +
                      sample->acc_z_g * sample->acc_z_g;

    return (isfinite(acc_norm2) &&
            acc_norm2 >= IMU_CALIBRATION_ACC_MIN_G * IMU_CALIBRATION_ACC_MIN_G &&
            acc_norm2 <= IMU_CALIBRATION_ACC_MAX_G * IMU_CALIBRATION_ACC_MAX_G &&
            fabsf(sample->gyro_x_dps) <= IMU_CALIBRATION_GYRO_MAX_DPS &&
            fabsf(sample->gyro_y_dps) <= IMU_CALIBRATION_GYRO_MAX_DPS &&
            fabsf(sample->gyro_z_dps) <= IMU_CALIBRATION_GYRO_MAX_DPS) ? 1U : 0U;
}

void estimate_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    app_imu_sample_t sample;
    app_attitude_t attitude;
    app_gyro_calibration_result_t gyro_result;
    app_accel_calibration_result_t accel_result;
    TickType_t calibration_start = 0U;
    uint32_t last_calibration_sample = 0U;
    uint32_t last_estimated_sample = 0U;
    uint32_t last_sample_tick = 0U;
    uint32_t request;
    uint8_t calibration_active = 0U;
    uint8_t calibration_invalid = 0U;

    (void)pvParameters;
    estimate_handle = xTaskGetCurrentTaskHandle();
    app_attitude_init(pvPortMalloc);
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();
        request = 0U;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &request, 0U);
        if ((request & ESTIMATE_REQUEST_CANCEL) != 0U)
        {
            calibration_active = 0U;
        }
        else if ((request & ESTIMATE_REQUEST_RESET) != 0U)
        {
            calibration_active = 0U;
            app_attitude_reset();
        }
        else if ((request & ESTIMATE_REQUEST_CALIBRATE) != 0U)
        {
            app_calibration_gyro_start();
            app_calibration_accel_start();
            calibration_start = xTaskGetTickCount();
            last_calibration_sample = 0U;
            calibration_invalid = 0U;
            calibration_active = 1U;
        }
        if ((g_imu_data_ready_sem != NULL) &&
            (xSemaphoreTake(g_imu_data_ready_sem,
                            pdMS_TO_TICKS(ESTIMATE_TASK_PERIOD_MS)) == pdTRUE) &&
            (receive_imu_snapshot(&sample) != 0U))
        {
            if ((calibration_active != 0U) &&
                (sample.sample_count != last_calibration_sample))
            {
                TickType_t elapsed = xTaskGetTickCount() - calibration_start;
                last_calibration_sample = sample.sample_count;
                if (calibration_sample_valid(&sample) == 0U)
                {
                    calibration_invalid = 1U;
                }
                else
                {
                    app_calibration_accel_sample(&sample);
                    if (elapsed < pdMS_TO_TICKS(IMU_GYRO_CALIBRATION_TIME_MS / 2U))
                    {
                        app_calibration_gyro_bias_sample(&sample);
                    }
                    else
                    {
                        app_calibration_gyro_noise_sample(&sample);
                    }
                }
                if (elapsed >= pdMS_TO_TICKS(IMU_GYRO_CALIBRATION_TIME_MS))
                {
                    calibration_active = 0U;
                    app_calibration_gyro_complete();
                    app_calibration_accel_complete();
                    app_calibration_gyro_result(&gyro_result);
                    app_calibration_accel_result(&accel_result);
                    if ((calibration_invalid == 0U) &&
                        (gyro_result.bias_sample_count >= IMU_CALIBRATION_MIN_SAMPLES) &&
                        (gyro_result.noise_sample_count >= IMU_CALIBRATION_MIN_SAMPLES) &&
                        (accel_result.sample_count >= IMU_CALIBRATION_MIN_SAMPLES))
                    {
                        app_attitude_apply_calibration(&gyro_result, &accel_result);
                    }
                }
            }
            if (sample.sample_count != last_estimated_sample)
            {
                uint32_t elapsed_ticks = (last_estimated_sample == 0U) ?
                    pdMS_TO_TICKS(ESTIMATE_TASK_PERIOD_MS) :
                    sample.capture_tick - last_sample_tick;
                uint32_t max_ticks = pdMS_TO_TICKS(ATTITUDE_MAX_SAMPLE_GAP_MS);

                if (elapsed_ticks == 0U)
                {
                    elapsed_ticks = 1U;
                }
                if (elapsed_ticks > max_ticks)
                {
                    elapsed_ticks = max_ticks;
                }
                last_estimated_sample = sample.sample_count;
                last_sample_tick = sample.capture_tick;
                if (app_attitude_step(&sample,
                                      (float)elapsed_ticks / (float)configTICK_RATE_HZ,
                                      &attitude) != 0U)
                {
                    receive_publish_attitude(&attitude);
                }
            }
        }
        task_metrics_end(TASK_METRIC_ESTIMATE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(ESTIMATE_TASK_PERIOD_MS));
    }
}

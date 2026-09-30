#include "zf_common_headfile.h"
#include "general_include.h"
#include "receive_task.h"
#include <math.h>

volatile imu_receive_data_t g_imu_receive_data;
SemaphoreHandle_t g_imu_data_ready_sem;
volatile imu_gyro_calibration_t g_imu_gyro_calibration;
volatile imu_accel_calibration_t g_imu_accel_calibration;

void imu_accel_noise_calibrate(void)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t start_tick;
    TickType_t calibration_ticks = pdMS_TO_TICKS(IMU_ACCEL_CALIBRATION_TIME_MS);
    float mean[3] = {0.0f, 0.0f, 0.0f};
    float m2[3] = {0.0f, 0.0f, 0.0f};
    uint32_t source_sample_count;
    uint32_t last_source_sample_count;
    uint32_t sample_count = 0U;
    uint8 axis;

    g_imu_accel_calibration.active = 1U;
    g_imu_accel_calibration.complete = 0U;
    g_imu_accel_calibration.sample_count = 0U;
    for (axis = 0U; axis < 3U; axis++)
    {
        g_imu_accel_calibration.mean_norm[axis] = 0.0f;
        g_imu_accel_calibration.noise_variance_norm2[axis] = 0.0f;
        g_imu_accel_calibration.noise_stddev_norm[axis] = 0.0f;
    }

    while (g_imu_receive_data.imu_ready == 0U)
    {
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    start_tick = xTaskGetTickCount();
    last_source_sample_count = g_imu_receive_data.sample_count;
    while ((xTaskGetTickCount() - start_tick) < calibration_ticks)
    {
        source_sample_count = g_imu_receive_data.sample_count;
        if (source_sample_count != last_source_sample_count)
        {
            float norm = sqrtf(g_imu_receive_data.acc_x_g * g_imu_receive_data.acc_x_g +
                               g_imu_receive_data.acc_y_g * g_imu_receive_data.acc_y_g +
                               g_imu_receive_data.acc_z_g * g_imu_receive_data.acc_z_g);
            last_source_sample_count = source_sample_count;

            if (norm > 1.0e-6f && sample_count < 0xFFFFFFFFU)
            {
                float sample[3];
                sample[0] = g_imu_receive_data.acc_x_g / norm;
                sample[1] = g_imu_receive_data.acc_y_g / norm;
                sample[2] = g_imu_receive_data.acc_z_g / norm;
                sample_count++;

                for (axis = 0U; axis < 3U; axis++)
                {
                    float delta = sample[axis] - mean[axis];
                    mean[axis] += delta / (float)sample_count;
                    m2[axis] += delta * (sample[axis] - mean[axis]);
                }
            }
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    g_imu_accel_calibration.sample_count = sample_count;
    for (axis = 0U; axis < 3U; axis++)
    {
        g_imu_accel_calibration.mean_norm[axis] = mean[axis];
        if (sample_count > 1U)
        {
            g_imu_accel_calibration.noise_variance_norm2[axis] =
                m2[axis] / (float)(sample_count - 1U);
            g_imu_accel_calibration.noise_stddev_norm[axis] =
                sqrtf(g_imu_accel_calibration.noise_variance_norm2[axis]);
        }
    }
    g_imu_accel_calibration.active = 0U;
    g_imu_accel_calibration.complete = (sample_count > 1U) ? 1U : 0U;
}

void imu_gyro_noise_calibrate(void)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t start_tick;
    TickType_t calibration_ticks = pdMS_TO_TICKS(IMU_GYRO_CALIBRATION_TIME_MS);
    TickType_t bias_ticks = calibration_ticks / 2U;
    TickType_t noise_ticks = calibration_ticks - bias_ticks;
    float bias_mean[3] = {0.0f, 0.0f, 0.0f};
    float noise_mean[3] = {0.0f, 0.0f, 0.0f};
    float noise_m2[3] = {0.0f, 0.0f, 0.0f};
    uint32_t source_sample_count;
    uint32_t last_source_sample_count;
    uint32_t bias_sample_count = 0U;
    uint32_t noise_sample_count = 0U;
    uint8 axis;

    g_imu_gyro_calibration.active = 1U;
    g_imu_gyro_calibration.complete = 0U;
    g_imu_gyro_calibration.sample_count = 0U;
    g_imu_gyro_calibration.bias_sample_count = 0U;
    g_imu_gyro_calibration.noise_sample_count = 0U;
    for (axis = 0U; axis < 3U; axis++)
    {
        g_imu_gyro_calibration.gyro_bias_dps[axis] = 0.0f;
        g_imu_gyro_calibration.gyro_noise_variance_dps2[axis] = 0.0f;
        g_imu_gyro_calibration.gyro_noise_stddev_dps[axis] = 0.0f;
    }

    /* The receive task initializes the sensor.  Wait here so calibration never
     * treats the zero-initialized IMU data as a valid stationary sample. */
    while (g_imu_receive_data.imu_ready == 0U)
    {
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    start_tick = xTaskGetTickCount();
    last_source_sample_count = g_imu_receive_data.sample_count;
    while ((xTaskGetTickCount() - start_tick) < bias_ticks)
    {
        source_sample_count = g_imu_receive_data.sample_count;
        if (source_sample_count != last_source_sample_count)
        {
            float sample[3];

            sample[0] = g_imu_receive_data.gyro_x_dps;
            sample[1] = g_imu_receive_data.gyro_y_dps;
            sample[2] = g_imu_receive_data.gyro_z_dps;
            last_source_sample_count = source_sample_count;

            if (bias_sample_count < 0xFFFFFFFFU)
            {
                bias_sample_count++;
                for (axis = 0U; axis < 3U; axis++)
                {
                    float delta = sample[axis] - bias_mean[axis];
                    bias_mean[axis] += delta / (float)bias_sample_count;
                }
            }
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        g_imu_gyro_calibration.gyro_bias_dps[axis] = bias_mean[axis];
    }

    /* Use the measured bias to correct a second, independent noise window. */
    start_tick = xTaskGetTickCount();
    last_source_sample_count = g_imu_receive_data.sample_count;
    while ((xTaskGetTickCount() - start_tick) < noise_ticks)
    {
        source_sample_count = g_imu_receive_data.sample_count;
        if (source_sample_count != last_source_sample_count)
        {
            float sample[3];

            sample[0] = g_imu_receive_data.gyro_x_dps;
            sample[1] = g_imu_receive_data.gyro_y_dps;
            sample[2] = g_imu_receive_data.gyro_z_dps;
            last_source_sample_count = source_sample_count;

            if (noise_sample_count < 0xFFFFFFFFU)
            {
                noise_sample_count++;
                for (axis = 0U; axis < 3U; axis++)
                {
                    float bias_corrected_sample =
                        sample[axis] - g_imu_gyro_calibration.gyro_bias_dps[axis];
                    float delta = bias_corrected_sample - noise_mean[axis];

                    noise_mean[axis] += delta / (float)noise_sample_count;
                    noise_m2[axis] +=
                        delta * (bias_corrected_sample - noise_mean[axis]);
                }
            }
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }

    g_imu_gyro_calibration.bias_sample_count = bias_sample_count;
    g_imu_gyro_calibration.noise_sample_count = noise_sample_count;
    g_imu_gyro_calibration.sample_count = bias_sample_count + noise_sample_count;
    for (axis = 0U; axis < 3U; axis++)
    {
        if (noise_sample_count > 1U)
        {
            g_imu_gyro_calibration.gyro_noise_variance_dps2[axis] =
                noise_m2[axis] / (float)(noise_sample_count - 1U);
            g_imu_gyro_calibration.gyro_noise_stddev_dps[axis] =
                sqrtf(g_imu_gyro_calibration.gyro_noise_variance_dps2[axis]);
        }
    }
    g_imu_gyro_calibration.active = 0U;
    g_imu_gyro_calibration.complete =
        (bias_sample_count > 1U && noise_sample_count > 1U) ? 1U : 0U;
}

void receive_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    uint8 imu_init_status;

    (void)pvParameters;

    imu_init_status = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
    g_imu_receive_data.imu_ready = (imu_init_status == 0U) ? 1U : 0U;

    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        if (g_imu_receive_data.imu_ready != 0U)
        {
            imu660rc_get_acc();
            imu660rc_get_gyro();

            g_imu_receive_data.acc_x = imu660rc_acc_x;
            g_imu_receive_data.acc_y = imu660rc_acc_y;
            g_imu_receive_data.acc_z = imu660rc_acc_z;
            g_imu_receive_data.gyro_x = imu660rc_gyro_x;
            g_imu_receive_data.gyro_y = imu660rc_gyro_y;
            g_imu_receive_data.gyro_z = imu660rc_gyro_z;
            g_imu_receive_data.acc_x_g = imu660rc_acc_transition(imu660rc_acc_x);
            g_imu_receive_data.acc_y_g = imu660rc_acc_transition(imu660rc_acc_y);
            g_imu_receive_data.acc_z_g = imu660rc_acc_transition(imu660rc_acc_z);
            g_imu_receive_data.gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x);
            g_imu_receive_data.gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y);
            g_imu_receive_data.gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z);
            g_imu_receive_data.sample_count++;

            if (g_imu_data_ready_sem != NULL)
            {
                (void)xSemaphoreGive(g_imu_data_ready_sem);
            }
        }

        task_metrics_end(TASK_METRIC_RECEIVE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(RECEIVE_TASK_PERIOD_MS));
    }
}

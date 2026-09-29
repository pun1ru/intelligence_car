#include "zf_common_headfile.h"
#include "general_include.h"
#include "receive_task.h"

volatile imu_receive_data_t g_imu_receive_data;

void receive_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;

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
        }

        task_metrics_end(TASK_METRIC_RECEIVE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1U));
    }
}

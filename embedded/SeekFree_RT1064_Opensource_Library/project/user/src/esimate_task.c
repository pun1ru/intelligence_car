#include "zf_common_headfile.h"
#include "general_include.h"
#include "estimate_task.h"
#include "receive_task.h"
#include "ekf_quaternion.h"

#define IMU_G_TO_MPS2        (9.80665f)
#define IMU_DPS_TO_RAD       (0.017453292519943f)
#define ESTIMATE_DT_SECONDS  (0.001f)

static void estimate_attitude(void)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;

    gyro_x_dps = g_imu_receive_data.gyro_x_dps;
    gyro_y_dps = g_imu_receive_data.gyro_y_dps;
    gyro_z_dps = g_imu_receive_data.gyro_z_dps;
    gyro_x_dps -= IMU_INITIAL_GYRO_BIAS_X_DPS;
    gyro_y_dps -= IMU_INITIAL_GYRO_BIAS_Y_DPS;
    gyro_z_dps -= IMU_INITIAL_GYRO_BIAS_Z_DPS;

    gx = gyro_x_dps * IMU_DPS_TO_RAD;
    gy = gyro_y_dps * IMU_DPS_TO_RAD;
    gz = gyro_z_dps * IMU_DPS_TO_RAD;
    ax = g_imu_receive_data.acc_x_g * IMU_G_TO_MPS2;
    ay = g_imu_receive_data.acc_y_g * IMU_G_TO_MPS2;
    az = g_imu_receive_data.acc_z_g * IMU_G_TO_MPS2;


    /* Temporarily disabled for runtime isolation. */
    IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az, ESTIMATE_DT_SECONDS);

    g_imu_receive_data.quaternion[0] = QEKF_INS.q[0];
    g_imu_receive_data.quaternion[1] = QEKF_INS.q[1];
    g_imu_receive_data.quaternion[2] = QEKF_INS.q[2];
    g_imu_receive_data.quaternion[3] = QEKF_INS.q[3];
    g_imu_receive_data.roll = QEKF_INS.roll;
    g_imu_receive_data.pitch = QEKF_INS.pitch;
    g_imu_receive_data.yaw = QEKF_INS.yaw;

}

void estimate_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
		//imu_accel_noise_calibrate();
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f,
                           IMU_INITIAL_GYRO_NOISE_X_DPS2,
                           IMU_INITIAL_GYRO_NOISE_Y_DPS2,
                           IMU_INITIAL_GYRO_NOISE_Z_DPS2,
                           IMU_INITIAL_ACCEL_NOISE_X_NORM2,
                           IMU_INITIAL_ACCEL_NOISE_Y_NORM2,
                           IMU_INITIAL_ACCEL_NOISE_Z_NORM2);
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        if ((g_imu_data_ready_sem != NULL) &&
            (xSemaphoreTake(g_imu_data_ready_sem, portMAX_DELAY) == pdTRUE))
        {

            estimate_attitude();

        }

        task_metrics_end(TASK_METRIC_ESTIMATE, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(ESTIMATE_TASK_PERIOD_MS));
    }
}

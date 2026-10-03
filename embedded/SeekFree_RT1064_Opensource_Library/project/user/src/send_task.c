#include "send_task.h"

#include "app_serial.h"
#include "app_calibration.h"
#include "control_task.h"
#include "receive_task.h"
#include "state_task.h"
#include "general_define.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_metrics.h"

void send_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)pvParameters;
    for (;;)
    {
        app_serial_telemetry_t telemetry = {0};
        app_state_t state;
        app_attitude_t attitude;
        app_imu_sample_t imu;
        app_control_telemetry_t motion;
        app_gyro_calibration_result_t gyro_calibration;
        app_serial_calibration_t calibration_telemetry;
        uint32_t metric_start = task_metrics_begin();
        telemetry.uptime_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (state_task_get_snapshot(&state) != 0U)
        {
            telemetry.valid |= SERIAL_VALID_STATE;
            telemetry.control_mode = (uint8_t)state.control_mode;
            telemetry.vehicle_mode = (uint8_t)state.vehicle_mode;
            telemetry.target_speed_m_s = state.target_speed_m_s;
            telemetry.target_yaw_deg = state.target_yaw_deg;
        }
        if (receive_attitude_snapshot(&attitude) != 0U)
        {
            telemetry.valid |= SERIAL_VALID_ATTITUDE;
            telemetry.roll_deg = attitude.roll;
            telemetry.pitch_deg = attitude.pitch;
            telemetry.yaw_deg = attitude.yaw;
            telemetry.pitch_rate_dps = attitude.pitch_rate_dps;
        }
        if ((receive_imu_snapshot(&imu) != 0U) &&
            ((uint32_t)xTaskGetTickCount() - imu.capture_tick <=
             VEHICLE_ATTITUDE_TIMEOUT_MS / portTICK_PERIOD_MS))
        {
            telemetry.valid |= SERIAL_VALID_IMU;
            telemetry.gyro_x_dps = imu.gyro_x_dps;
            telemetry.gyro_y_dps = imu.gyro_y_dps;
            telemetry.gyro_z_dps = imu.gyro_z_dps;
            telemetry.acc_x_g = imu.acc_x_g;
            telemetry.acc_y_g = imu.acc_y_g;
            telemetry.acc_z_g = imu.acc_z_g;
        }
        if (control_task_get_snapshot(&motion) != 0U)
        {
            telemetry.valid |= SERIAL_VALID_CONTROL;
            telemetry.left_pwm = motion.left_pwm;
            telemetry.right_pwm = motion.right_pwm;
            telemetry.target_pitch_deg = motion.target_pitch_deg;
            if (motion.wheel_valid != 0U)
            {
                telemetry.valid |= SERIAL_VALID_MOTION;
                telemetry.left_speed_m_s = motion.left_speed_m_s;
                telemetry.right_speed_m_s = motion.right_speed_m_s;
                telemetry.left_delta_count = motion.left_delta_count;
                telemetry.right_delta_count = motion.right_delta_count;
            }
            if (motion.lift_detected != 0U)
            {
                telemetry.valid |= SERIAL_VALID_LIFT;
            }
        }
        app_serial_send(&telemetry);
        app_calibration_gyro_result(&gyro_calibration);
        calibration_telemetry.active = gyro_calibration.active;
        calibration_telemetry.complete = gyro_calibration.complete;
        calibration_telemetry.sample_count = gyro_calibration.bias_sample_count;
        calibration_telemetry.gyro_bias_x_dps = gyro_calibration.gyro_bias_dps[0];
        calibration_telemetry.gyro_bias_y_dps = gyro_calibration.gyro_bias_dps[1];
        calibration_telemetry.gyro_bias_z_dps = gyro_calibration.gyro_bias_dps[2];
        app_serial_send_calibration(&calibration_telemetry);
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SEND_TASK_PERIOD_MS));
    }
}

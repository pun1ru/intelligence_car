#include "send_task.h"

#include "app_serial.h"
#include "app_calibration.h"
#include "control_task.h"
#include "decision_task.h"
#include "receive_task.h"
#include "state_task.h"
#include "general_define.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_metrics.h"

/*
 * send_task：把三帧数据发给上位机，一轮 SEND_TELEMETRY_PERIOD_MS（100 ms）：
 *   时隙 0：52 字节车辆遥测帧（0x80）——状态、姿态、IMU、轮速、PWM、目标值；
 *   时隙 1：18 字节导航位姿帧（0x82）——位姿，仅用于调试观察；
 *   时隙 2：24 字节校准帧（0x81）——陀螺零偏采样进度与结果。
 * 三帧错开成三个时隙（每 20 ms 一个）而不是连发：实测无线模块缓冲装不下
 * 94 字节的连发，会整帧丢弃靠后的校准帧和位姿帧，而错开后一次最多 52 字节。
 * 帧率不变，只是把原本挤在一起的发送摊开。
 * 每帧只填自己有效的数据，并在 valid 位里标明，上位机据此决定是否显示 "--"。
 * 这里只负责"取谁的数据"，具体帧格式在 communication/app_serial。
 */

#if (SEND_TELEMETRY_PERIOD_MS % SEND_TASK_PERIOD_MS) != 0U
#error Send slot period must divide the frame period.
#endif

#define SEND_TELEMETRY_SLOT 0U
#define SEND_POSE_SLOT      1U
#define SEND_CALIBRATION_SLOT 2U
#define SEND_SLOT_COUNT (SEND_TELEMETRY_PERIOD_MS / SEND_TASK_PERIOD_MS)

void send_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    uint8_t slot = SEND_TELEMETRY_SLOT;

    (void)pvParameters;
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        if (slot == SEND_TELEMETRY_SLOT)
        {
            app_serial_telemetry_t telemetry = {0};
            app_state_t state;
            app_attitude_t attitude;
            app_imu_sample_t imu;
            app_control_telemetry_t motion;

            telemetry.uptime_ms =
                (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
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
        }
        else if (slot == SEND_POSE_SLOT)
        {
            app_serial_pose_t pose_telemetry = {0};
            app_state_t state;
            app_pose_t pose;

            /* 位姿由 decision_task 维护，这里只取快照。取不到（超过
             * VEHICLE_POSE_TIMEOUT_MS 没更新）就只发导航标记、数据位留空，
             * 上位机看到的就是 "--"，而不是冻结的旧数值。 */
            if (decision_task_get_pose(&pose) != 0U)
            {
                if (pose.valid != 0U)
                {
                    pose_telemetry.valid |= SERIAL_POSE_VALID_DATA;
                }
                pose_telemetry.x_m = pose.x_m;
                pose_telemetry.y_m = pose.y_m;
                pose_telemetry.heading_deg = pose.heading_deg;
                pose_telemetry.path_length_m = pose.path_length_m;
                pose_telemetry.yaw_rate_rad_s = pose.yaw_rate_rad_s;
            }
            if ((state_task_get_snapshot(&state) != 0U) &&
                (state.vehicle_mode == APP_VEHICLE_NAVIGATION))
            {
                /* bit1：此刻车辆确实处于导航状态，方便对着位姿判断场景 */
                pose_telemetry.valid |= SERIAL_POSE_VALID_NAVIGATION;
            }
            app_serial_send_pose(&pose_telemetry);
        }
        else if (slot == SEND_CALIBRATION_SLOT)
        {
            app_gyro_calibration_result_t gyro_calibration;
            app_serial_calibration_t calibration_telemetry;

            app_calibration_gyro_result(&gyro_calibration);
            calibration_telemetry.active = gyro_calibration.active;
            calibration_telemetry.complete = gyro_calibration.complete;
            calibration_telemetry.sample_count =
                gyro_calibration.bias_sample_count;
            calibration_telemetry.gyro_bias_x_dps =
                gyro_calibration.gyro_bias_dps[0];
            calibration_telemetry.gyro_bias_y_dps =
                gyro_calibration.gyro_bias_dps[1];
            calibration_telemetry.gyro_bias_z_dps =
                gyro_calibration.gyro_bias_dps[2];
            app_serial_send_calibration(&calibration_telemetry);
        }
        else
        {
            /* 空时隙：留给模块把上一帧送出去 */
        }

        slot = (uint8_t)((slot + 1U) % SEND_SLOT_COUNT);
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SEND_TASK_PERIOD_MS));
    }
}

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
 * send_task：把三帧数据发给上位机。以 SEND_TASK_PERIOD_MS（20 ms）为一个时隙：
 *   每 5 个时隙（100 ms）：52 字节车辆遥测帧（0x80）——状态、姿态、IMU、轮速、PWM；
 *   每 25 个时隙（500 ms）：18 字节导航位姿帧（0x82）——位姿，仅用于调试观察；
 *   每 25 个时隙（500 ms）：24 字节校准帧（0x81）——陀螺零偏采样进度与结果。
 *
 * 两点都是为了迁就无线模块实测吞吐（约 700 B/s，低于遥测+位姿+校准连发的
 * 940 B/s）：
 *   1. 一帧一个时隙、帧间留 20 ms，模块不用一次缓冲 94 字节；
 *   2. 位姿与校准这类"看得清就行"的数据降到 2 Hz，总流量降到约 604 B/s。
 * 遥测帧保持 100 ms 不变，因为它承载姿态与轮速，降频会直接影响看图。
 * 每帧只填自己有效的数据，并在 valid 位里标明，上位机据此决定是否显示 "--"。
 * 这里只负责"取谁的数据"，具体帧格式在 communication/app_serial。
 */

#if ((SEND_TELEMETRY_PERIOD_MS % SEND_TASK_PERIOD_MS) != 0U) || \
    ((SEND_SLOW_PERIOD_MS % SEND_TELEMETRY_PERIOD_MS) != 0U)
#error Send periods must be multiples: slot <= telemetry <= slow.
#endif

#define SEND_TELEMETRY_TICKS (SEND_TELEMETRY_PERIOD_MS / SEND_TASK_PERIOD_MS)
#define SEND_SLOW_TICKS      (SEND_SLOW_PERIOD_MS / SEND_TASK_PERIOD_MS)

void send_task(void *pvParameters)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    /* 时隙计数，用来把三帧摊到不同时隙上（见文件头的说明） */
    uint32_t tick = 0U;

    (void)pvParameters;
    for (;;)
    {
        uint32_t metric_start = task_metrics_begin();

        if ((tick % SEND_TELEMETRY_TICKS) == 0U)
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
                /* 目标速度/航向报"控制环实际跟着的值"：跟路线时是路径跟踪算出来的，
                 * 否则是状态机里的。否则上位机在跟踪期间会看到两个 0，以为是停了。 */
                telemetry.target_speed_m_s = motion.effective_speed_m_s;
                telemetry.target_yaw_deg = motion.effective_yaw_deg;
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
        else if ((tick % SEND_SLOW_TICKS) == 1U)
        {
            app_serial_pose_t pose_telemetry = {0};
            app_nav_command_t nav;
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
            /* 路径跟踪指令：只有 app_nav 认为本拍有效（active=1）才打上 bit2，
             * 否则上位机看到的就是 "--"，不会把过期的目标当成当前指令 */
            if ((decision_task_get_nav(&nav) != 0U) && (nav.active != 0U))
            {
                pose_telemetry.valid |= SERIAL_POSE_VALID_TRACKING;
                pose_telemetry.nav_target_speed_m_s = nav.target_speed_m_s;
                pose_telemetry.nav_target_yaw_deg = nav.target_yaw_deg;
                pose_telemetry.nav_cross_track_m = nav.cross_track_m;
                pose_telemetry.nav_target_index = (uint8_t)nav.target_index;
            }
            if ((state_task_get_snapshot(&state) != 0U) &&
                (state.vehicle_mode == APP_VEHICLE_NAVIGATION))
            {
                /* bit1：此刻车辆确实处于导航状态，方便对着位姿判断场景 */
                pose_telemetry.valid |= SERIAL_POSE_VALID_NAVIGATION;
            }
            app_serial_send_pose(&pose_telemetry);
        }
        else if ((tick % SEND_SLOW_TICKS) == 3U)
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

        tick++;
        task_metrics_end(TASK_METRIC_SEND, metric_start);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SEND_TASK_PERIOD_MS));
    }
}

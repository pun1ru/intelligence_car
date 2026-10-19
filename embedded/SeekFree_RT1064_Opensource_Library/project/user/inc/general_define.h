#ifndef GENERAL_DEFINE_H
#define GENERAL_DEFINE_H

#define STATE_TASK_PRIORITY 2
#define DECISION_TASK_PRIORITY 1
#define ESTIMATE_TASK_PRIORITY 3
#define CONTROL_TASK_PRIORITY 4
#define RECEIVE_TASK_PRIORITY 3
#define SEND_TASK_PRIORITY 1
#define DEBUG_TASK_PRIORITY 1

#define STATE_TASK_PERIOD_MS 10U
#define DECISION_TASK_PERIOD_MS 10U
#define ESTIMATE_TASK_PERIOD_MS 1U
#define CONTROL_TASK_PERIOD_MS 5U
#define RECEIVE_TASK_PERIOD_MS 1U
/* 发送时隙：发送任务每 SEND_TASK_PERIOD_MS 醒一次，一个时隙只发一帧。
 * 遥测帧每 SEND_TELEMETRY_PERIOD_MS 一帧，位姿帧与校准帧每 SEND_SLOW_PERIOD_MS
 * 一帧，且各自错开到不同时隙（见 send_task.c）。这样做的原因是实测无线模块
 * 吞吐有限（约 700 B/s）：三帧连发共 94 字节会被丢弃或截断，降到 604 B/s 并
 * 错开发送后才有余量。遥测帧的频率不能降，它承载姿态与轮速。 */
#define SEND_TASK_PERIOD_MS 20U
#define SEND_TELEMETRY_PERIOD_MS 100U
#define SEND_SLOW_PERIOD_MS 500U
#define DEBUG_TASK_PERIOD_MS 100U

#define IMU_GYRO_CALIBRATION_TIME_MS 20000U
//#define IMU_GYRO_CALIBRATION_ENABLE  1U
#define IMU_ACCEL_CALIBRATION_TIME_MS 20000U
#define ATTITUDE_ACCEL_LPF_ALPHA      0.95f
#define ATTITUDE_EKF_CHI_SQUARE_LIMIT 7.815f
#define ATTITUDE_EKF_CORRECTION_LIMIT 0.01f
#define ATTITUDE_MADGWICK_BETA        0.2f
#define ATTITUDE_MAX_SAMPLE_GAP_MS    20U
#define ATTITUDE_MIN_ACCEL_NORM_G     0.3f

#define STATE_TASK_STACK_SIZE 512
#define DECISION_TASK_STACK_SIZE 512  
#define ESTIMATE_TASK_STACK_SIZE 1024
#define CONTROL_TASK_STACK_SIZE 1024
#define RECEIVE_TASK_STACK_SIZE 1024
#define SEND_TASK_STACK_SIZE 1024
#define DEBUG_TASK_STACK_SIZE 1024

#define STATE_TASK_NAME "state_task"
#define DECISION_TASK_NAME "decision_task"
#define ESTIMATE_TASK_NAME "estimate_task"
#define CONTROL_TASK_NAME "control_task"
#define RECEIVE_TASK_NAME "receive_task"
#define SEND_TASK_NAME "send_task"
#define DEBUG_TASK_NAME "debug_task"

/* QTMR counts A rising edges only. Confirm 1024 is counts per encoder-shaft revolution in this mode. */
#define WHEEL_ENCODER_COUNTS_PER_ENCODER_REV 1024.0f
#define WHEEL_GEAR_TEETH                       68.0f
#define ENCODER_GEAR_TEETH                     30.0f
#define WHEEL_ENCODER_COUNTS_PER_REV           (WHEEL_ENCODER_COUNTS_PER_ENCODER_REV * \
                                               WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define WHEEL_DIAMETER_M                       0.064f
#define WHEEL_RADIUS_M                         (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_TRACK_M                          0.200f
#ifndef WHEEL_LEFT_COUNT_SIGN
#define WHEEL_LEFT_COUNT_SIGN        1
#endif
#ifndef WHEEL_RIGHT_COUNT_SIGN
#define WHEEL_RIGHT_COUNT_SIGN       -1
#endif
#define WHEEL_LEFT_PWM_FORWARD_LEVEL 0U
#define WHEEL_RIGHT_PWM_FORWARD_LEVEL 0U
#define MOTOR_PWM_FREQUENCY_HZ       17000U
#define MOTOR_TEST_TARGET_SPEED_MPS 1.0f
#define MOTOR_TEST_MAX_PWM_DUTY     5000U
#define MOTOR_LEFT_PWM_CHANNEL       PWM2_MODULE0_CHA_C6
#define MOTOR_LEFT_DIRECTION_PIN     C7
#define MOTOR_RIGHT_PWM_CHANNEL      PWM2_MODULE1_CHA_C8
#define MOTOR_RIGHT_DIRECTION_PIN    C9
#define ENCODER_LEFT_INDEX           QTIMER1_ENCODER1
#define ENCODER_LEFT_CHANNEL_1       QTIMER1_ENCODER1_CH1_C0
#define ENCODER_LEFT_CHANNEL_2       QTIMER1_ENCODER1_CH2_C1
#define ENCODER_RIGHT_INDEX          QTIMER1_ENCODER2
#define ENCODER_RIGHT_CHANNEL_1      QTIMER1_ENCODER2_CH1_C2
#define ENCODER_RIGHT_CHANNEL_2      QTIMER1_ENCODER2_CH2_C24
#define WHEEL_SPEED_PWM_POLARITY     1.0f
#define BALANCE_ANGLE_PWM_POLARITY   -1.0f
#define BALANCE_SPEED_TO_TILT_SIGN   1.0f

/* The wheel-speed test is independent of the balance cascade. */
#define WHEEL_SPEED_LOOP_PERIOD_MS   10U
#define WHEEL_SPEED_PWM_LIMIT        ((float)MOTOR_TEST_MAX_PWM_DUTY)
#define BALANCE_ANGLE_PERIOD_MS      5U
#define BALANCE_SPEED_PERIOD_MS      20U
#define BALANCE_UPRIGHT_PITCH_DEG    -90.0f
#define BALANCE_ANGLE_TARGET_PITCH_DEG -92.0f
#define BALANCE_SUPPORT_TARGET_PITCH_DEG -97.0f
#define BALANCE_ANGLE_PWM_LIMIT      3500.0f
#define ATTITUDE_ROLL_SIGN           -1.0f
#define BALANCE_TARGET_TILT_MAX_DEG  15.0f
#define BALANCE_TARGET_TILT_FORWARD_MAX_DEG  15.0f
#define BALANCE_TARGET_TILT_BACKWARD_MAX_DEG 5.0f
#define BALANCE_SPEED_TARGET_MAX_MPS 1.0f
#define WHEEL_SPEED_KP               5000.0f
#define WHEEL_SPEED_KI               0.0f
#define WHEEL_SPEED_KD               100.0f
#define WHEEL_SPEED_FEEDFORWARD_PWM_PER_MPS 0.0f
#define BALANCE_ANGLE_KP             600.0f
#define BALANCE_ANGLE_KI             0.0f
#define BALANCE_ANGLE_KD             30.0f
/* 以下三组增益是上位机在线调好、再写回这里的值（2026-10-12），
 * 目的是断电重启后不用重新调参。速度环与转向环的 Kp 曾用默认值 16 / 100。 */
#define BALANCE_SPEED_KP             24.0f
#define BALANCE_SPEED_KI             3.0f
#define BALANCE_SPEED_KD             0.0f
/* 转向环：PID 输出先限幅 ±BALANCE_STEERING_PWM_LIMIT(1000)，
 * 再叠加前馈后与角度环一起按 ±BALANCE_ANGLE_PWM_LIMIT(3500) 限幅，
 * 所以大角度转向主要靠 BALANCE_STEERING_FEEDFORWARD_PWM_PER_DEG。 */
#define BALANCE_STEERING_KP          30.0f
#define BALANCE_STEERING_KI          2.0f
#define BALANCE_STEERING_KD          0.0f
#define BALANCE_STEERING_PWM_LIMIT   1000.0f
#define BALANCE_STEERING_PWM_POLARITY 1.0f
#define BALANCE_STEERING_FEEDFORWARD_PWM_PER_DEG 10.0f
#define BALANCE_STEERING_ENABLE_DEFAULT 0U
#define CONTROL_PID_D_FILTER_ALPHA   0.6f

#define VEHICLE_BUTTON_DEBOUNCE_MS   30U
#define VEHICLE_BUTTON_CALIBRATE_PIN C12
#define VEHICLE_BUTTON_NAV_PIN       C13
#define VEHICLE_BUTTON_ARM_PIN       C14
#define VEHICLE_BUTTON_PROTECT_PIN   C15
#define VEHICLE_MODE_SWITCH_1_PIN    C26
#define VEHICLE_MODE_SWITCH_2_PIN    C27
#define VEHICLE_SWITCH_ON_LEVEL      0U
#define IMU_CALIBRATION_GYRO_MAX_DPS 3.0f
#define IMU_CALIBRATION_ACC_MIN_G    0.9f
#define IMU_CALIBRATION_ACC_MAX_G    1.1f
#define IMU_CALIBRATION_MIN_SAMPLES  100U
#define IMU_CALIBRATION_GYRO_NOISE_FLOOR_DPS2 0.000001f
#define IMU_CALIBRATION_ACC_NOISE_FLOOR_NORM2 0.000001f
#define VEHICLE_ATTITUDE_TIMEOUT_MS  30U
#define VEHICLE_STATE_TIMEOUT_MS     30U
#define VEHICLE_CONTROL_TIMEOUT_MS   30U
#define VEHICLE_POSE_TIMEOUT_MS      50U

/* 导航路径跟踪（Pure Pursuit）。速度与半径都是保守初值，上车后按表现调整。
 * NAV_TRACKING_ENABLE_DEFAULT 是跟踪总开关：正式比赛禁用无线，开关必须在这里
 * 定好（设 1），否则上位机没法把它打开。 */
#define NAV_TRACKING_ENABLE_DEFAULT  1U
/* 是否真的用跟踪指令驱动电机。
 *   1 = 正常（车会自己跟路线走，比赛用这个）
 *   0 = 只算不动：照常计算并上报导航指令，但控制环忽略它（车只原地平衡）。
 *       台架验证"路径 → 目标速度/航向"是否正确时用它，可以手推着车看数字，
 *       既不用怕车自己跑，也不用让电机和你抢方向盘。 */
#define NAV_TRACKING_DRIVE_ENABLE    1U
/* 前视距离（"车看多远"）：决定目标航向算在哪，是"抄不抄内圈"的主因。
 * 演进：0.40 时车明显提前打方向抄内圈；0.28 变化不大（那次的偏差主要被终点
 * 判定半径盖住了）。现在与"弧上路径点加密"配套调到 0.18——前视距离必须
 * ≥ 2 倍点间距，否则前视点在点与点之间跳变，车会发抖画龙；点加密后间距
 * 0.078 m，0.18 ≈ 2.3 倍，是安全余量。 */
#define NAV_TRACKING_LOOKAHEAD_M     0.18f
/* 功能验证阶段先用 0.20；导航层确认无误后（2026-10-19）加到 0.40 试探。
 * 弯里按曲率自动降速：曲率半径 0.4 m 时目标速度 = 上限 × (0.4/0.5) ≈ 0.32 m/s，
 * 直道上才是 0.40。上限还受 BALANCE_SPEED_TARGET_MAX_MPS（1.0）约束。
 * 若出现左右画龙/发抖，先回 0.30，再考虑把前视距离随速度放大。 */
#define NAV_TRACKING_SPEED_MAX_M_S   0.40f
#define NAV_TRACKING_SPEED_MIN_M_S   0.15f
#define NAV_TRACKING_SLOW_RADIUS_M   0.50f
#define NAV_TRACKING_GOAL_RADIUS_M   0.10f
#define VEHICLE_ARM_PITCH_MAX_DEG    10.0f
#define VEHICLE_SUPPORT_ARM_PITCH_MAX_DEG 4.0f
#define VEHICLE_SUPPORT_AUTO_MIN_PITCH_DEG -100.0f
#define VEHICLE_SUPPORT_AUTO_MAX_PITCH_DEG -96.0f
#define VEHICLE_SUPPORT_AUTO_ENTER_MS 500U
#define VEHICLE_SUPPORT_EXIT_PITCH_DEG -96.0f
#define VEHICLE_SUPPORT_EXIT_MS 100U
#define VEHICLE_BALANCE_ENTRY_KICK_SPEED_MPS 0.15f
#define VEHICLE_BALANCE_ENTRY_KICK_MS         100U
#define VEHICLE_FALL_PITCH_MAX_DEG   45.0f
#define VEHICLE_SERIAL_TIMEOUT_MS    500U

/* Encoder-only lift diagnostic.  This is deliberately a telemetry flag and
 * does not arm a protection transition: wheel speed alone cannot prove that
 * the chassis is airborne while the vehicle is commanded to move. */
#define VEHICLE_LIFT_MAX_WHEEL_SPEED_MPS       2.0f
#define VEHICLE_LIFT_FREE_SPIN_SPEED_MPS       0.75f
#define VEHICLE_LIFT_SPEED_DIFF_MPS            1.0f
#define VEHICLE_LIFT_MIN_SPEED_MPS             0.25f
#define VEHICLE_LIFT_TARGET_SPEED_EPS_MPS      0.05f
#define VEHICLE_LIFT_MIN_PWM_DUTY              1000.0f
#define VEHICLE_LIFT_DETECT_MS                 100U
#define VEHICLE_LIFT_CLEAR_MS                  200U

/* Downlink command: A5 5A command data[4] xor(command,data). */
#define SERIAL_FRAME_SYNC_0          0xA5U
#define SERIAL_FRAME_SYNC_1          0x5AU
#define SERIAL_FRAME_LENGTH          8U
#define SERIAL_COMMAND_HEARTBEAT     0x00U
#define SERIAL_COMMAND_STATE         0x01U
#define SERIAL_COMMAND_SPEED         0x02U
#define SERIAL_COMMAND_YAW           0x03U
#define SERIAL_COMMAND_MOTION        0x04U
#define SERIAL_COMMAND_FEEDFORWARD   0x05U
#define SERIAL_COMMAND_CALIBRATE     0x06U
#define SERIAL_COMMAND_STEERING      0x07U
#define SERIAL_COMMAND_STEERING_FEEDFORWARD 0x08U
/* 把当前位置重置为原点、当前车头方向记为 0°；上位机 Trajectory 页的
 * "Reset pose" 按钮发它。方便原地重测，不必断电重做陀螺零偏校准。 */
#define SERIAL_COMMAND_POSE_RESET    0x09U
#define SERIAL_COMMAND_PID_BASE      0x50U
#define SERIAL_COMMAND_PID_LAST      0x5BU
#define SERIAL_PID_CONTROLLER_WHEEL  0U
#define SERIAL_PID_CONTROLLER_ANGLE  1U
#define SERIAL_PID_CONTROLLER_SPEED  2U
#define SERIAL_PID_CONTROLLER_STEERING 3U
#define SERIAL_PID_GAIN_KP            0U
#define SERIAL_PID_GAIN_KI            1U
#define SERIAL_PID_GAIN_KD            2U
#define SERIAL_PID_MAX_VALUE          10000.0f
#define SERIAL_STATE_PROTECT         0x00U
#define SERIAL_STATE_BALANCE         0x01U
#define SERIAL_STATE_NAVIGATION      0x02U
#define SERIAL_STATE_SUPPORT         0x03U
#define SERIAL_SPEED_SCALE_MPS       0.001f
#define SERIAL_YAW_SCALE_DEG         0.01f
#define SERIAL_FEEDFORWARD_SCALE     1.0f
#define SERIAL_FEEDFORWARD_MAX_PWM_PER_MPS 10000.0f
#define SERIAL_STEERING_FEEDFORWARD_SCALE 1.0f
#define SERIAL_STEERING_FEEDFORWARD_MAX_PWM_PER_DEG 1000.0f
#define SERIAL_TELEMETRY_SYNC_0      0x5AU
#define SERIAL_TELEMETRY_SYNC_1      0xA5U
#define SERIAL_TELEMETRY_VERSION     0x01U
#define SERIAL_TELEMETRY_TYPE        0x80U
#define SERIAL_TELEMETRY_LENGTH      52U
#define SERIAL_CALIBRATION_TYPE      0x81U
#define SERIAL_CALIBRATION_LENGTH    24U
/* 导航位姿单独用一种上行帧，这样 52 字节定长遥测帧一个字节都不用动，
 * 旧固件/旧上位机仍能按原帧长解析。帧格式见 docs/SERIAL_PROTOCOL.md。 */
#define SERIAL_NAV_POSE_TYPE         0x82U
#define SERIAL_NAV_POSE_LENGTH       25U
/* 位姿帧 valid 字节的位定义 */
#define SERIAL_POSE_VALID_DATA       0x01U  /* 位姿已定原点并成功积分，数值可信 */
#define SERIAL_POSE_VALID_NAVIGATION 0x02U  /* 发送时刻车辆处于导航状态 */
#define SERIAL_POSE_VALID_TRACKING   0x04U  /* 路径跟踪正在输出有效指令 */
/* 位姿帧里导航指令字段的缩放：速度 0.001 m/s、横向偏差 0.001 m，
 * 目标航向复用 SERIAL_ANGLE_UNITS_PER_DEG（0.01 deg）。 */
#define SERIAL_POSE_NAV_SPEED_UNITS_PER_MPS 1000.0f
#define SERIAL_POSE_NAV_OFFSET_UNITS_PER_M  1000.0f
/* 位姿帧各字段的定点缩放：位置用 0.001 m，角速度用 0.001 rad/s；
 * 航向复用 SERIAL_ANGLE_UNITS_PER_DEG（0.01 deg）。 */
#define SERIAL_POSE_POSITION_UNITS_PER_M  1000.0f
#define SERIAL_POSE_RATE_UNITS_PER_RAD_S  1000.0f
#define SERIAL_TELEMETRY_CRC_POLY    0x1021U
#define SERIAL_TELEMETRY_CRC_INIT    0xFFFFU
#define SERIAL_VALID_STATE           0x01U
#define SERIAL_VALID_ATTITUDE        0x02U
#define SERIAL_VALID_IMU             0x04U
#define SERIAL_VALID_MOTION          0x08U
#define SERIAL_VALID_CONTROL         0x10U
#define SERIAL_VALID_LIFT            0x20U
#define SERIAL_ANGLE_UNITS_PER_DEG   100.0f
#define SERIAL_RATE_UNITS_PER_DPS    100.0f
#define SERIAL_ACCEL_UNITS_PER_G     1000.0f
#define SERIAL_SPEED_UNITS_PER_MPS   1000.0f
#define SERIAL_PWM_LIMIT             10000
#define WHEEL_TWO_PI_RAD             6.283185307179586f
#define VEHICLE_UART_INDEX           UART_8
#define VEHICLE_UART_BAUD            115200U
#define VEHICLE_UART_RX_BUFFER_SIZE  128U
#define VEHICLE_UART_TX_PIN          UART8_TX_D16
#define VEHICLE_UART_RX_PIN          UART8_RX_D17

#endif 

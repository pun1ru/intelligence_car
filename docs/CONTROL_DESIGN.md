# 双轮平衡控制设计与当前接入状态

支撑平衡是独立的机体状态，不属于 PID 调参流程。该状态可由上位机发送 `STATE=3` 请求，也可由状态机自动进入：当车辆处于普通平衡状态，Pitch 连续 500 ms 位于 `-100~-96 deg` 且没有按住 C15 时，自动切换到支撑平衡；保护状态不会自动进入。进入条件为姿态有效且 Pitch 位于 `-97 ±4 deg`；进入后默认速度目标为 0、姿态目标为 `-97 deg`，用于车体由挡板支撑时保持稳定。支撑状态仍接受速度目标，正速度用于让车体向前运动；Pitch 连续超过 `-96 deg` 100 ms 后自动切换回普通平衡并清零速度目标，由普通平衡的 `-92 deg` 目标接管。普通平衡和导航仍使用各自的 `-92 deg` 姿态目标。在线修改 PID 时车端单独进入保护，修改完成后再选择所需机体状态。

物理坐标、轮径、编码器正方向和引脚见 [车体约定](VEHICLE_CONVENTIONS.md)，串口帧见 [串口协议](SERIAL_PROTOCOL.md)。本页说明已有控制算法与尚待标定的执行方向。

## 轮速反馈

`app_encoder_convert` 接受每一批左右增量计数和实际间隔 `dt_s`，返回角位移 rad、角速度 rad/s、轮沿线速度 m/s 及累计里程 m。左轮正计数表示从 `+Y` 看向原点时逆时针运动；右轮原始计数反号后与左轮使用相同运动符号。轮径 64 mm、轮半径 `0.032 m`、轮距 `0.200 m`。QTMR 当前只计 A 相上升沿；暂按该模式下编码器轴每圈 1024 计数，轮轴一圈计数为 `1024*(68/30)=2321.0667`。若 1024 是四倍频总计数，当前驱动对应值应为 256，须修改 `WHEEL_ENCODER_COUNTS_PER_ENCODER_REV`。

屏幕现在每 100 ms 对接收队列中的**绝对计数快照**做 16 位回绕差分，并用实际显示间隔计算左/右轮角速度和线速度。首次快照只建立基线。相邻显示间隔内的真实计数变化需小于 32768，避免回绕方向歧义。控制链独立读取每 10 ms 的增量计数，取相邻两次有符号轮速的平均值，供 20 ms 速度外环使用。

## 控制算法

| 接口 | 设计周期 | 输入 | 输出 |
| --- | --- | --- | --- |
| `app_balance_wheel_speed_step` | 10 ms | 左右目标/实测轮速 m/s | 左右有符号 PWM，供独立轮速测试 |
| `app_balance_angle_step` | 5 ms | 目标 pitch、实测 pitch、pitch 角速度 | 左右有符号 PWM |
| `app_balance_step` 速度外环 | 20 ms | 目标/平均线速度 m/s | 目标倾角 |
| `app_balance_step` 角度内环 | 5 ms | 目标倾角、姿态、角速度 | 左右有符号 PWM |

直立 pitch 安全基准为 `-90 deg`，当前角度目标为 `-92 deg`。C14 起立或串口进入平衡/导航要求相对安全基准误差不超过 `10 deg`；运行中误差超过 `45 deg`，或姿态无效/过期，立即清零电机 PWM 并锁存保护，只能再次请求起立。`control_task` 每 5 ms 调用 `app_balance_step`；速度 PI 每 20 ms 根据左右轮平均线速度修正目标倾角，角度 PD 每 5 ms 以校准后的 X 轴陀螺仪角速度作为阻尼反馈。正轮速偏大时，`BALANCE_SPEED_TO_TILT_SIGN=+1` 使目标 pitch 减小，让车体先向后倾，再通过姿态环减速；形成后倾时轮子可能短暂向前推。两电机收到相同的有符号 PWM，角度环输出暂限幅 `±3000/10000`。按钮和导航模式目标速度为零；串口平衡模式采用收到的目标线速度。角度环极性与轮速测试的 `WHEEL_SPEED_PWM_POLARITY` 独立，实车增益仍需调试。

当前使用 Madgwick 四元数姿态估计，应用层 `pitch` 对应算法输出的 X 轴欧拉角，范围 `[-180 deg, 180 deg]`。`-90 deg` 并非该角度的回绕边界；若静止时仍跳到 `±180 deg`，需同时检查原始陀螺仪、加速度、四元数和其他欧拉角。屏幕显示 `Gx dps`，便于观察角度跳变时 X 轴陀螺仪是否也出现大角速度。角度环遇到这种输出会触发上述跌倒保护，不会继续驱动电机。

## 临时电机测试与状态

C26/C27 双开时进入校准模式；C12 采样校准，C13 重置当前姿态滤波器。轮速测试由控制任务每 10 ms 调用 `app_control_step`：按住 C15 测左轮、按住 C14 测右轮，有符号正 PWM 经 `WHEEL_*_PWM_FORWARD_LEVEL=0` 将对应 DIR 设为低电平。控制器根据独立读取的增量编码器计数和轮系尺寸，按轮速绝对值闭环跟踪 `MOTOR_TEST_TARGET_SPEED_MPS=1.0 m/s`；PWM 上限为 `MOTOR_TEST_MAX_PWM_DUTY=5000/10000`。轮速 P 增益为 `4000`。松开对应按键或退出双开模式，该轮立即归零并重置 PID。C26/C27 只开一个时同样输出零 PWM。

正常模式中 C14 申请平衡、C13 申请导航、C15 保护；串口可请求相同状态并保存速度、yaw 目标，500 ms 无合法帧后锁存保护。导航暂不执行路径动作。姿态算法依据实际 IMU 样本间隔积分；显示 roll 的符号已按 `+X` 观察方向修正。
The steering defaults are `Kp=100`, `Ki=2`, `Kd=0`, with a separate angle
feedforward default of `10 PWM/deg`. The steering feedforward is the shortest
Yaw error multiplied by this gain. The control loop wraps
the target minus measured Yaw into `[-180, 180]`, so crossing the angle
boundary always follows the shorter turn. Encoder speed anomalies are exposed
as a telemetry diagnostic (`valid bit5`) after 100 ms and clear after 200 ms;
the flag does not independently enter protection.

Support mode forces the steering loop off and clears its integral. When Support
returns to Balance, the state task seeds the target Yaw from the current measured
Yaw. The speed loop default Kp is `16`; its pitch target limit is asymmetric:
`+15 deg` forward and `-5 deg` backward, while the angle PWM limit is `3500`.

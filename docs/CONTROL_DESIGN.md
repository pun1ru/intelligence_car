# 双轮平衡控制接口

当前模块已加入 Keil 工程，但没有接入任务调度或串口接收中断。现有 C12 电机测试动作保持原样。

## 标定与方向

在 `project/user/inc/general_define.h` 中设置 `WHEEL_ENCODER_COUNTS_PER_REV`（轮轴一圈对应的实际四倍频计数，含减速比）和 `WHEEL_RADIUS_M`。两者目前为 0，`app_encoder_convert` 会返回失败，避免把未标定计数当成 m/s。`WHEEL_LEFT_COUNT_SIGN`、`WHEEL_RIGHT_COUNT_SIGN` 定义车体前进时编码器计数的符号；两个 `WHEEL_*_PWM_FORWARD_LEVEL` 定义前进时方向引脚电平。`CONTROL_PWM_POLARITY` 与 `BALANCE_SPEED_TO_TILT_SIGN` 需结合实际 pitch 正方向确认。增益也只是初始值，必须上板调参。

`drv_motion_read_encoder` 保持原始计数语义，供现有显示和接收任务使用。新增 `drv_motion_read_encoder_delta` 每次返回左右轮相对上次增量读取的有符号计数，支持 16 位计数器回绕；单次真实增量须小于 32768 计数。`app_encoder_convert` 要对每一批增量调用一次，`dt_s` 必须是该批计数的实际时间间隔。输出包含角位移 rad、角速度 rad/s、轮沿线速度 m/s 与累计里程 m。长度为 1 的接收队列可能覆盖采样，不能直接拿偶尔读取的最新增量配一个较长控制周期。

## 控制周期

| 接口 | 周期 | 输入 | 输出 |
| --- | --- | --- | --- |
| `app_balance_wheel_speed_step` | 10 ms | 左右目标/实测轮速 m/s | 左右有符号 PWM，用于独立轮速测试 |
| `app_balance_angle_step` | 5 ms | 目标 pitch、实测 pitch、pitch 角速度 | 左右有符号 PWM，用于独立倾角控制 |
| `app_balance_step` 角度内环 | 5 ms | 目标速度、pitch deg、pitch 角速度 deg/s、左右轮速 | 左右有符号 PWM |
| `app_balance_step` 速度外环 | 每 4 次角度调用，即 20 ms | 目标车速与平均轮速 m/s | 目标 pitch deg，供角度内环使用 |

平衡模式中速度外环只改变目标倾角，角度内环独占电机 PWM。独立轮速测试接口与平衡接口不能同时驱动电机。调用方在保护状态应传 `enabled=0` 并将零输出写到电机；`app_balance_apply` 只在被显式调用时写硬件。导航状态目前应传零目标速度。起立和跌倒判定共用 `VEHICLE_ARM_PITCH_MAX_DEG` 范围。

## 状态与串口

`app_state_step` 接收已消抖前的按钮电平、姿态有效标记、pitch 和可选串口命令。默认按钮调试/保护；C14 在起立角范围内进入平衡，C13 进入导航站立，C15 进入保护。跌倒、姿态无效或串口超时进入锁存保护，必须重新按键或收到串口启动帧。导航只保留站立。串口 yaw 目标仅保存，不参与控制。

暂定接收帧共 8 字节：`A5 5A CMD D0 D1 D2 D3 XOR`，校验为 `CMD` 到 `D3` 的异或。`CMD=00` 心跳，`01` 状态（`D0=0` 保护、`1` 平衡、`2` 导航），`02` 速度（`D0,D1` 为小端有符号 mm/s），`03` yaw（小端有符号 0.01 deg）。`D2,D3` 暂保留。只要收到合法帧即进入串口调试模式，500 ms 无合法帧则回到按钮调试并进入保护。`app_serial_poll_command` 可从 UART 非阻塞读取完整命令，但目前没有任务调用它。

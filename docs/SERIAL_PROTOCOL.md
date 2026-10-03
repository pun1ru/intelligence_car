# 无线串口通信协议 v1

车端 UART8：TX=D16、RX=D17；115200 baud、8N1、无硬件流控。无线模块使用透明传输。车端以中断接收 UART8 字节并放入 128 字节环形缓冲区；缓冲区满时丢弃新字节，校验失败的命令不会刷新在线计时。多字节数均为小端；有符号量使用二进制补码。下行沿用现有 8 字节命令格式，上行采用 v1 遥测帧。两个方向使用不同帧头；接收端必须按帧头、固定长度和校验重同步。

## 电脑到车：命令帧

固定 8 字节：`A5 5A CMD D0 D1 D2 D3 XOR`，其中 `XOR=CMD^D0^D1^D2^D3`。`D0:D1`、`D2:D3` 均为 int16。除 `MOTION` 外，`D2:D3` 必须为零。无效帧、未知命令、非法状态或非零保留字节被丢弃，且不刷新车端串口存活时间。

| CMD | 名称 | value | value2 | 效果 |
| --- | --- | --- | --- | --- |
| `00` | HEARTBEAT | 0 | 0 | 刷新串口存活时间；上位机每 100 ms 发送 |
| `01` | STATE | 0=保护，1=平衡，2=导航，3=支撑平衡 | 0 | 请求状态转换 |
| `02` | SPEED | 目标速度，mm/s | 0 | 平衡状态下更新速度；车端限幅 ±1000 mm/s |
| `03` | YAW | 目标航向，0.01 deg | 0 | 平衡状态下保存目标；启用转向环后用于左右轮差速 |
| `04` | MOTION | 目标速度，mm/s | 目标航向，0.01 deg | 平衡状态下同时更新两者 |
| `50..5B` | PID | 命令号编码控制环和增益；见下表 | IEEE-754 float32，小端 | 写入一个 PID 增益并立即进入保护 |

PID 命令号为 `0x50 + controller * 3 + gain`。`controller=0` 轮速环，`1` 姿态角度环，`2` 速度环，`3` 转向环；`gain=0` Kp，`1` Ki，`2` Kd。PID 帧的 `D0..D3` 是 float32，不再按 int16 解码。当前允许范围为 `0..10000`。轮速环参数同时写入左右轮。每次合法 PID 命令都会清零速度目标、清零电机输出并进入保护，即使车辆当前已经处于保护也会重置对应 PID 积分项；修改完成后必须重新发送 `STATE(BALANCE)` 或 `STATE(NAVIGATION)`，且仍需满足姿态和按钮安全条件。

保护命令始终可用。进入平衡或导航要求姿态有效、Pitch 在 `-90 ±10 deg`、C15 未按住，C26/C27 均关闭。支撑平衡是独立机体状态，进入要求 Pitch 在 `-97 ±4 deg`；进入后默认目标 Pitch 为 `-97 deg`、速度目标为 0，用于挡板支撑时的稳定站立。车辆处于普通平衡状态时，如果 Pitch 连续 `500 ms` 位于 `-100~-96 deg`，且 C15 未按住，会自动切换到支撑平衡；保护状态不会自动进入支撑平衡。支撑平衡可以接受速度目标，正速度会驱动车体向前；当 Pitch 连续超过 `-96 deg` 达到 `100 ms`，状态机切回普通平衡并将速度目标清零，之后由普通平衡目标 `-92 deg` 接管。支撑平衡不等同于 PID 调参，PID 修改仍通过独立的 PID 命令处理。跌倒超过 `-90 ±45 deg` 或姿态失效会进入保护，不能自动恢复。C26/C27 双开时进入校准；只开一个时强制保护。合法命令使车进入串口调试模式；500 ms 内没有合法命令时返回按钮模式并进入保护，运动目标清零。导航目前仅执行基础平衡。车端不提供命令 ACK；上位机必须依据后续遥测中的状态和目标值判断是否执行。命令帧没有序号或重传保证，急停后也应确认遥测状态；失联时依赖车端 500 ms 保护。

## 车到电脑：遥测帧

每 100 ms 一帧，固定 52 字节，序号 uint8 循环。CRC 为 CRC-16/CCITT-FALSE：初值 `FFFF`、多项式 `1021`、不反射、无末尾异或，覆盖字节 0..49；帧尾 CRC 小端。没有有效位的测量字段填零，上位机不得当作真实读数。固定长度为 52 字节，线路占用约 4.5 ms/帧。

| 偏移 | 长度 | 字段 | 单位/说明 |
| --- | --- | --- | --- |
| 0 | 2 | `5A A5` | 上行帧头 |
| 2 | 1 | version=`01` | 协议版本 |
| 3 | 1 | type=`80` | 车辆遥测 |
| 4 | 1 | sequence | 逐帧加一，模 256 |
| 5 | 1 | valid | bit0 状态、bit1 姿态、bit2 IMU、bit3 轮速/编码器、bit4 控制输出 |
| 6 | 4 | uptime_ms | uint32 车端运行时间 ms，回绕 |
| 10 | 1 | control_mode | 0 按钮、1 串口、2 校准；需 bit0 |
| 11 | 1 | vehicle_mode | 0 保护、1 平衡、2 导航、3 支撑平衡；需 bit0 |
| 12/14/16 | 各 2 | Roll/Pitch/Yaw | int16，0.01 deg；需 bit1 |
| 18 | 2 | Pitch 角速度 | int16，0.01 deg/s；需 bit1 |
| 20/22/24 | 各 2 | Gyro X/Y/Z | int16，0.01 deg/s；需 bit2 |
| 26/28/30 | 各 2 | Accel X/Y/Z | int16，0.001 g；需 bit2 |
| 32/34 | 各 2 | 左/右轮速 | int16，0.001 m/s，带方向；需 bit3 |
| 36/38 | 各 2 | 左/右编码器增量 | int16，最近一次 10 ms 控制采样计数；需 bit3 |
| 40/42 | 各 2 | 左/右实际 PWM | int16，±10000 对应 ±100% 占空比；0 为停机；需 bit4 |
| 44 | 2 | 目标速度 | int16，0.001 m/s；需 bit0 |
| 46 | 2 | 目标 Yaw | int16，0.01 deg；需 bit0；当前不参与控制 |
| 48 | 2 | 目标 Pitch | int16，0.01 deg；控制器当前目标；需 bit4 |
| 50 | 2 | CRC16 | 小端 |

编码器增量是最新一次采样值，不是 100 ms 遥测周期内的累计值。PWM 数值是已限幅的电机命令，正负号为工程定义的前进方向；不是实际测得的占空比。遥测没有加密或认证，控制链路应使用可信无线模块与物理隔离。上位机读取 CRC 错误或序号丢帧时只显示有效帧；500 ms 未收到有效遥测时显示超时。车辆保护以其下行命令超时逻辑为准。
## Upper monitor additions

Telemetry `valid` bit `0x20` reports the vehicle-side encoder speed anomaly or lift diagnostic. The fixed 52-byte frame is unchanged; the diagnostic does not itself enter protection. The upper monitor displays the flag with the vehicle state, offers four selectable realtime charts, and fuses signed wheel speed with measured yaw into a metre-scaled trajectory view. W/S sets configurable forward or reverse speed and A/D changes a wrapped yaw target; the vehicle controller follows the shortest angular error.

Command `05` sets the wheel speed feedforward gain in `PWM/(m/s)` and enters
protection. Command `08` sets the steering feedforward gain in `PWM/deg` and
also enters protection. The steering feedforward default is `10 PWM/deg`; the
steering PID defaults are `Kp=100`, `Ki=2`, `Kd=0`. The upper panel exposes
both gains independently.

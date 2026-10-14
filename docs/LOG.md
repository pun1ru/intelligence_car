# 项目日志

按日期追加记录实际变更、验证结果和未解决事项；仅记录已完成的工作，未验证的内容应明确说明。

## 2026-10-04

- 变更：新增根目录 `AGENTS.md`、`CLAUDE.md`，约定 agent 阅读并同步维护 `docs/`、记录日志及遵守代码规范；建立本日志。当前控制环调节目标已记录于 `TODO_LIST.md`。
- 验证：仅涉及文档，已检查文件内容；未进行编译或实车测试。
- 未解决：转向环、角度环和速度环的调节仍待完成。

- 变更：新增 `.gitignore`，将 clangd 缓存、Keil 编译输出与日志、Python 字节码、编译数据库和本机 Keil 会话文件排除出 Git 跟踪；从 Git 索引中移除已跟踪的对应生成文件，保留本地文件。
- 验证：检查 Git 忽略规则、索引状态及本地文件保留情况；未运行编译。
- 未解决：无。

## 2026-10-12

- 变更：把上位机在线调好的控制参数写回 `general_define.h`：速度环 `Kp` 由 `16` 改为 `24`（`Ki=3`），转向环 `Kp` 由 `100` 改为 `30`（`Ki=2`、前馈 `10 PWM/deg`）；角度环 `600/0/30` 原本与在线值一致。同步更新 `CONTROL_DESIGN.md`、`SERIAL_PROTOCOL.md` 和 `upper/README.md` 中的默认值说明，并修正文档中与代码不符的旧值（角度环限幅 `3500`、轮速环 `WHEEL_SPEED_KP=5000`、`KD=100`）。
- 变更：新增导航层位姿估计。`myAlgorithm/odometry/odometry.h/.c` 为纯算法模块（编码器里程 + yaw 积分得到 `x`、`y`、`heading`，航向相对进入导航时的原点并回绕到 `[-180°, 180°]`，同时输出编码器角速度供与 IMU yaw 速率互校）；`myApplication/navigation/app_pose.h/.c` 把 `receive_task` 的绝对编码器快照按轮系尺寸换算为轮行程并维护位姿；`decision_task` 由空任务框架改为每 10 ms 更新位姿，进入导航时以当前位置和航向为原点、退出导航时位姿失效，并通过单元素队列发布快照（新增 `decision_task_get_pose`）。`receive_task` 新增只读接口 `receive_encoder_snapshot`。两个新目录已加入 `rt1064.uvprojx` 的分组和 include path。位姿约定补充到 `VEHICLE_CONVENTIONS.md`，模块结构更新到 `ARCHITECTURE.md`。
- 变更：新增主机侧单元测试 `tests/host/odometry_test.c`，覆盖直线、左转、航向原点偏置、跨 ±180° 回绕、非法输入（空指针、非正 `dt`、NaN/Inf、无效轮距）与倒车。
- 验证：`gcc -std=c99 -Wall -Wextra` 编译测试无告警，`25 checks, 0 failure(s)`。Keil `UV4 -b rt1064.uvprojx` 全量编译 `0 Error(s), 0 Warning(s)`，`Code=102952 RO-data=11392 RW-data=216 ZI-data=78616`（改动前为 `Code=99544 RO-data=11360 RW-data=216 ZI-data=78500`）。
- 未解决：位姿未在实车验证，也未标定轮径/轮距比例误差与漂移速度；位姿只能通过内部接口读取，没有遥测或屏幕输出，车上无法直接观察；路径表与轨迹跟踪（Pure Pursuit）未实现；转向环 PID 输出 `±1000` 限幅导致大角度饱和的问题未处理（当前靠前馈提供主要转向力矩）。

## 2026-10-12（下午：位姿可观测性）

- 变更：车端新增 `0x82` 导航位姿上行帧（固定 18 字节：x、y、heading、累计里程、编码器推算角速度，`valid` bit0 位姿有效、bit1 处于导航）。沿用同一帧头、版本、序号与 CRC，52 字节遥测帧与 24 字节校准帧未改动，因此不破坏既有解析。`app_serial` 新增 `app_serial_encode_pose`/`app_serial_send_pose`，`send_task` 每 100 ms 在遥测帧之后发送一帧，位姿来自 `decision_task_get_pose`。
- 变更：上位机 `protocol.py` 增加 `NavPoseTelemetry` 与 `decode_nav_pose`，解析器改为按帧类型查表取帧长与解码函数；`main.py` 在轨迹图上把车端位姿（橙色）与主机端推算（蓝色）叠加显示，新增“Vehicle pose (x, y)”与“Vehicle path length”读数；同时把面板里过期的默认 PID 数值改为与固件一致（速度环 `Kp=24`、转向环 `Kp=30`）。
- 变更：位姿积分时机从“仅导航状态”改为“上电后第一个有效姿态样本起持续积分”，进入导航时重新确定原点、退出导航后保留最后位姿；校准模式不积分且退出校准时重置原点（轮速台架测试会让轮子空转数米，否则会被记成一次位移）。这样在上电、保护或平衡状态下都能手推车辆核对里程标定，不必先把车立起来进入导航。
- 验证：`python -m py_compile` 通过；`python -m unittest test_protocol` 5 项通过，含 C 端编码字节与 Python 解码的交叉核对用例——`tests/host/pose_frame_dump.c` 用固件里的 `app_serial_encode_pose` 生成 `5aa5018203032efb37022823290983ff5d74`，与 Python 手工构造的帧逐字节相同，解码值一致。Keil 全量编译 `0 Error(s), 0 Warning(s)`，`Code=103624 RO-data=11392 RW-data=216 ZI-data=78616`。
- 验证：`tests/host/odometry_test.c` 25 项检查通过。
- 未解决：位姿帧本身未在实车链路上核对（需要烧录后接上位机确认）；位姿准确度与漂移仍未标定；路径表与轨迹跟踪（Pure Pursuit）未实现。

## 2026-10-12（补注释）

- 变更：为本次新增和改动的代码补充中文注释，便于队友直接阅读：`odometry.h/.c`、`app_pose.h/.c`、`decision_task.h/.c`、`app_serial.h/.c`（位姿帧部分）、`send_task.c`、`receive_task.h/.c`（快照接口）、`general_define.h`（位姿帧宏与写回的增益值）、`tests/host/*.c`、`upper/protocol.py`、`upper/main.py`、`upper/test_protocol.py`。注释语言遵循仓库既有做法（`kalman`、`EKF`、`madgwick` 本来就用中文注释），标识符保持英文。
- 变更：`C_CODE_STYLE.md` 明确新增源码保存为 UTF-8 无 BOM（含中文注释时同样，不用 GBK），并补充注释语言与文件头说明的建议。
- 验证：Keil 全量编译 `0 Error(s), 0 Warning(s)`，`Code=103624 RO-data=11392 RW-data=216 ZI-data=78616`，与加注释前逐字节相同，说明只改了注释；`tests/host/odometry_test.exe` 25 项通过；`tests/host/pose_frame_dump.exe` 输出的固定向量未变；`python -m py_compile` 与 `python -m unittest test_protocol`（5 项）通过。已核对全部改动文件为合法 UTF-8 且无 BOM。
- 未解决：无新增；此前关于实车验证、标定与路径跟踪的待办不变。

## 2026-10-12（上位机读数被裁）

- 问题：位姿读数在 Live values 里看不到。实测确认不是版本问题（运行中的窗口标题带有版本标记、进程命令行为 `python.exe main.py`），而是**布局裁剪**：窗口内容实测需要约 1020 px，默认窗口仅 820 px，而 `main.py` 中只有 Notebook 所在行设置了 `weight=1`，窗口偏矮时全部压缩都由 Notebook 承担，Live values 的两个读数框因此只分到 240 px，超出部分被父框裁掉。
- 影响范围：不只新增的两行。逐层裁剪检查显示，改动前的最后一行 `Trajectory`，以及左栏的 `cal_status`、`bias_x`、`bias_y`、`bias_z`、`cal_samples`（校准读数）同样一直不可见。
- 变更：`Vehicle pose (x, y)` 与 `Vehicle path length` 移到窗口顶部 `Vehicle state` 框（该框按内容取高，不会被压缩）；窗口默认几何由 `1180x820` 改为 `1180x1000`；窗口标题加入 `UI_VERSION` 标记，便于区分多份仓库副本与旧进程。
- 验证：用脚本逐层比对每个读数与其父控件的高度，`pose`、`pose_path`、`roll`、`cal_samples`、`bias_z`、`trajectory`、`target_pitch`、`left_speed` 均为 VISIBLE。另外把一帧真实 0x82 数据喂给界面，读数正确显示为 `-1.23, 0.57` 与 `2.35`，轨迹点正常追加；`python -m py_compile` 通过。未在实车链路上复测。
- 未解决：位姿帧的实车链路验证与里程标定仍待完成。

## 2026-10-14

- 实车（台架）验证：在保护状态下手推车辆，格子实测 92 cm，上位机 `Vehicle path length` 读数 0.92（当时朝车尾方向推，故为负号），比例误差约 0.1%。结论：`WHEEL_ENCODER_COUNTS_PER_REV`（2321.07）与轮径常数可用，不需要修正；此前担心的 1.2% 偏差没有出现。位姿帧（0x82）在无线链路上收发正常，上位机读数与轨迹曲线均已可见。
- 变更：新增轨迹跟踪的算法与路径表。`myAlgorithm/path_tracking/pure_pursuit` 实现 Pure Pursuit：先找最近点，再沿路径量取前视距离得到前视点，由前视点方向给出目标航向，并按曲率估计把速度在上限与下限之间缩放；输出还包含横向偏差与前视点序号便于调试。`myApplication/navigation/app_nav_path` 为锥桶间隙点表（相对发车点、+X 向前、+Y 向左），当前为空表并附填表说明与示例，`app_nav_path_count()` 返回 0。
- 变更：新增主机侧测试 `tests/host/pure_pursuit_test.c`（29 项），覆盖直线跟踪、横向偏差左右符号、急弯减速、终点判定、前视点只前进不后退、单点路径、车压在前视点上的退化情形与非法输入。算法实现中特意把前视距离改为"沿路径量取"而不是"车到点的直线距离"——后者在车略微越过最近点时会算出要求倒退的航向。
- 变更：两个新模块加入 `rt1064.uvprojx` 的分组与 include path；文档同步（`ARCHITECTURE.md` 说明模块未接入控制环、`TODO_LIST.md` 更新路径表示与跟踪接入两项、`C_CODE_STYLE.md` 前一日已补编码约定）。
- 验证：`gcc -std=c99 -Wall -Wextra` 编译无告警，`pure_pursuit_test` 29 项、`odometry_test` 25 项全部通过。Keil 全量编译 `0 Error(s), 0 Warning(s)`，`Code=103624` 与加注释前一致——新模块尚无调用者，链接器未将其纳入固件，车的现有行为未改变。
- 未解决：路径表尚未填入实际锥桶坐标；Pure Pursuit 尚未接入控制环（需要换算到 IMU 航向坐标系、定前视距离与速度上下限、并加"总开关与安全退出"策略）；位姿的航向积分与转向符号尚未做方框闭合验证。

## 2026-10-14（修丢帧计数）

- 问题：上位机 `Telemetry` 栏的 `lost` 计数会在链路完好时持续增长，看起来像无线模块一直在丢帧，影响"这次标定结果可不可信"的判断。
- 原因：三种上行帧共用同一个序号，车端每 100 ms 依次发遥测、校准、位姿各一帧，序号是连续的；而旧实现只在遥测帧（0x80）里更新序号并计算差值，于是相邻两个遥测帧之间隔着校准帧和位姿帧，每帧固定误报丢失 2 帧（每秒约 20）。这是加入 0x82 帧时引入的，与无线模块无关。
- 变更：把丢帧统计抽成 `protocol.SequenceTracker`，对**每一帧**都调用，并保留"间隔大于半圈视为重新同步、不计丢帧"的处理；`main.py` 在帧分发处统一调用。同时补上该计数器的单元测试（连续、丢一帧、丢多帧、序号回绕、间隔过大）。
- 验证：`python -m unittest test_protocol` 6 项通过；用构造的字节流模拟车端循环——5 个完整循环（15 帧）显示 `lost 0 / bad 0`，故意去掉中间一帧则 `lost 1`。`py_compile` 通过。
- 未解决：无线链路的实际稳定性未评估（若 `bad` 增长说明 CRC 出错，属链路干扰或波特率/接线问题；必要时台架改用 USB-TTL 有线直连）。

## 2026-10-14（错开发送，解决无线丢帧）

- 现象：上位机持续 timeout，`lost` 与 `bad` 同时持续增长；新增的帧间隔读数约 125/312 ms，说明遥测帧基本能到达。
- 定位：车端发送先排除——`uart_write_buffer` 是逐字节阻塞写（`LPUART_WriteBlocking`），94 字节每周期都按顺序写入硬件，不存在车端缓冲溢出。剩下的是"模块 + 电脑"这一段：连发的三帧共 94 字节，模块缓冲装不下时会把靠后的帧整帧丢弃（`lost`）或截断成半帧导致 CRC 失败（`bad`）；遥测帧在最前面所以能活下来，于是帧间隔看着正常而丢帧集中在后两帧。
- 变更：`send_task` 由"每 100 ms 连发三帧"改为"每 20 ms 发一帧，三帧错开在三个时隙"（遥测时隙 0、位姿时隙 1、校准时隙 2，其余空闲），一轮仍 100 ms，帧率与序号规则不变。新增 `SEND_TELEMETRY_PERIOD_MS` 与编译期检查，`SEND_TASK_PERIOD_MS` 由 100 改为 20。
- 变更：上位机新增 `Frame period now/max (ms)` 读数与 `SequenceTracker` 丢帧统计（后者修掉"三种帧共用序号却只对遥测帧计数"导致的误报）。
- 验证：Keil 全量编译 `0 Error(s), 0 Warning(s)`，`Code=103696`（此前 103624）。尚未烧录复测，无线链路需实机确认 `lost`/`bad` 是否停止增长。
- 未解决：若错开后仍丢帧，说明模块的持续吞吐不足，下一步降低位姿帧频率（如 300~500 ms）或台架改用 USB-TTL 有线直连。

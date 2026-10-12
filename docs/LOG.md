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

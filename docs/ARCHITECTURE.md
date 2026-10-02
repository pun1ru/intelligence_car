# 嵌入式软件架构规范

适用范围：`embedded/SeekFree_RT1064_Opensource_Library` 下的自有嵌入式代码。本规范描述新增代码的目标结构；现有代码按功能迭代逐步迁移，不要求为遵守目录形式而一次性重写。

## 1. 目录与职责

```text
embedded/SeekFree_RT1064_Opensource_Library/
├── project/user/
│   ├── inc/                     任务头文件、启动和中断入口声明
│   └── src/                     任务实现、main.c、isr.c
└── libraries/
    ├── myAlgorithm/             自有算法层：PID、滤波、估计、数学运算等
    ├── myApplication/           自有应用层，按业务模块分目录
    │   ├── system/              系统启动和模块初始化
    │   ├── sensor/              IMU、编码器采样及姿态估计、校准
    │   ├── motion/              轮电机控制流程
    │   ├── interaction/         蜂鸣器和屏幕显示
    │   ├── state/               车辆状态机
    │   └── communication/       串口通信
    ├── myDriver/                自有驱动层：板级设备与逐飞接口适配
    ├── zf_common/               逐飞公共库
    ├── zf_driver/               逐飞芯片外设驱动
    ├── zf_device/               逐飞设备驱动
    ├── zf_components/           逐飞组件
    ├── Freertos/                第三方 RTOS
    └── sdk/、components/等       平台或第三方代码
```

`my*` 目录归项目维护；逐飞目录作为外部依赖使用，原则上不改其源码。确有逐飞库缺陷或硬件差异时，优先在 `myDriver` 做适配；必须修改上游文件时，单独记录原因、影响和升级时的合并方法。

| 层 | 负责 | 不负责 |
| --- | --- | --- |
| `project/user` 的 Task | 周期调度、任务同步、任务间通信、调用 Application 接口；`main` 负责启动入口，`isr` 负责中断转发 | 具体业务规则、复杂算法、直接编排 GPIO/PWM 等设备细节 |
| `myApplication` | 完整业务动作与状态转换；组合算法和驱动接口；定义安全策略与应用输入输出 | 创建任务、直接使用 FreeRTOS 队列/信号量、依赖任务文件 |
| `myAlgorithm` | 纯计算、滤波、估计、控制律；显式输入、输出和算法状态 | 读取硬件、处理任务同步、包含 FreeRTOS 或逐飞设备头 |
| `myDriver` | 板级引脚/外设配置、采样和执行器操作；隔离逐飞 API 与硬件差异 | 业务决策、控制律、创建任务或调用 Application |
| 逐飞/SDK 库 | 提供底层能力 | 承载项目自有业务代码 |

## 2. 依赖方向

```text
Task（project/user） ──调用──> Application ──调用──> Algorithm
                                    │
                                    └────────调用──> myDriver ──调用──> 逐飞库 / SDK

Task ──使用──> FreeRTOS（调度与通信）
```

- 箭头表示源码依赖和调用方向。`Task -> Application`、`Application -> Algorithm / myDriver`、`myDriver -> 逐飞库 / SDK` 是默认允许路径；同层模块也应保持无环依赖。下层不得包含上层头文件、调用上层函数或引用上层全局变量。
- `myAlgorithm` 优先只依赖 C 标准库；确需 CMSIS-DSP 等纯计算库时，在算法模块内明确该依赖，保持与板级硬件及 FreeRTOS 解耦。若算法实现需要 RTOS 分配器或设备类型，应改为由调用者提供内存/普通数值，或将适配代码移到 Application/Driver。
- Task 的正常业务入口只调用 Application。启动时的任务创建、中断通知和任务通信属于 `user` 的例外；底层初始化应封装为 Application/Driver 初始化接口，由启动流程调用。不要以 Task 直接调用逐飞驱动作为长期结构。
- `myApplication` 可以按业务拆模块，但模块之间的调用关系应写清所有权并保持有向无环；公共数据类型只放在确实共同使用的头文件，不通过巨型公共头传递整个系统状态。
- 不允许通过函数指针、`extern` 全局变量或包含 `.c` 文件绕开上述方向。回调若必须从底层通知上层，由接口注入并记录调用上下文，避免形成编译依赖；中断唤醒由 `user` 侧适配到 FreeRTOS。

## 3. 数据流与执行边界

一个控制周期建议按以下顺序组织：

1. Task 根据周期或通知取得最新输入快照，并通过队列/通知处理跨任务通信。
2. Task 调用一个或少量 Application 接口，例如 `app_vehicle_control_step(&input, &output)`；不在 Task 中计算 PID、判断行驶状态或直接操作 PWM。
3. Application 完成状态转换、安全判断，调用 Algorithm 计算，必要时调用 Driver 采样或写执行器。
4. Driver 将物理设备和逐飞 API 的细节封装在本层；接口返回数据、状态或错误码。

跨任务共享数据由生产者、消费者和同步方式共同定义，尽量传递值或不可变快照。Application 和 Algorithm 不持有 `TaskHandle_t`、队列句柄或信号量。算法更新使用调用方传入的 `dt_s`、采样值和状态，不自己读取 RTOS tick 或传感器。同步调用需要标出超时和失败后的安全行为。

## 4. 文件划分与新增模块

- 一个模块集中管理一个职责，并拥有同名 `.h/.c`。例如 `myDriver/drv_motion.h/.c` 封装电机与编码器，`myApplication/motion/app_control.h/.c` 负责控制动作，`myAlgorithm/pid/pid.h/.c` 实现 PID。Application 仅按上述业务域分目录，暂不为 IMU、编码器、蜂鸣器等设备再建子目录。
- 对外头文件只暴露该层接口；私有状态和逐飞头文件尽量留在 `.c`。Application 头文件使用普通 C 数据类型，避免迫使 Task 间接包含整个逐飞库；Algorithm 头文件保持可在主机环境编译。
- 新建自有 `.c/.h` 后，把源码加入 `project/mdk/rt1064.uvprojx` 对应分组，并更新必要的 include path；仅创建文件不会自动加入 Keil 构建。分组名与目录层次保持一致。
- `project/user/inc` 放任务入口、任务配置以及任务通信声明；任务实现放 `project/user/src`。当前工程统一将可调设定量放在 `general_define.h`。该头文件只定义常量，不得引入 Task 接口或运行时依赖；Driver/Application 可以读取这些配置宏。

## 5. 当前实现与后续检查

当前板级外设访问集中在 `myDriver`。`myApplication` 的系统、传感器、运动、交互外设、通信和状态目录均已有实现。屏幕输出由 `interaction/app_display` 管理，串口帧由 `communication/app_serial` 管理。`receive_task` 用长度为 1 的 FreeRTOS 队列发布 IMU、编码器和姿态快照；`estimate_task` 由信号量唤醒。任务运行统计独立放在 `project/user/src/task_metrics.c`。Kalman 算法通过调用方提供的分配器获取内存，算法头文件不再包含 FreeRTOS。

`state_task` 按周期读取按键、拨码、姿态及串口命令，调用 `app_state` 更新状态，并通过单元素队列发布状态快照；`estimate_task` 负责 Madgwick 姿态计算、校准与滤波器重置。`decision_task` 仍为空任务框架。`control_task` 每 5 ms 读取状态和姿态并调用 Application 控制入口：平衡/导航状态运行 20 ms 速度外环和 5 ms pitch 角度内环，当前零速角度基准为 `-92 deg`；C26/C27 双开校准模式下，每 10 ms 执行 C15 左轮、C14 右轮的 1 m/s 轮速闭环测试。控制任务发现姿态无效或偏离直立安全基准 `-90 deg` 超过 `45 deg` 时，当周期清零 PWM 并通知状态任务锁存保护。控制、IMU 接收和姿态解算任务优先级高于 TFT 显示任务。yaw 外环尚未接入电机调度。

校准在 `estimate_task` 中按样本非阻塞执行，结果仅在 RAM 中保存；`receive_task` 持续发布 IMU 和编码器快照。姿态估计按实际样本 tick 间隔积分，屏幕按相邻编码器绝对计数差分计算轮速。车体坐标、轮系正方向、引脚和拨码操作见 [车体约定](VEHICLE_CONVENTIONS.md)，上位机帧草案见 [串口协议](SERIAL_PROTOCOL.md)。

新增模块评审时检查：文件是否位于所属层、头文件是否泄漏上层/第三方类型、依赖图是否有环、任务是否仅处理调度与通信、异常时执行器是否进入安全状态。

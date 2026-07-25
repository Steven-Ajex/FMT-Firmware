# FMT Firmware Agent / Skills 顶层设计报告

生成日期：`2026-03-17`

## 0. 结论摘要

- 建议先建立 `1` 个顶层 agent，`15` 个原子 `code-reading skills`，`6` 个 `workflow skills`。
- 代表目标固定为 `target/sieon/s1`，因为它同时覆盖 `SConstruct`、`BuildLists.py`、`config/*.py`、`board.c`、`link.lds`、target-local tasks，能完整反映 FMT 的真实装配方式。
- 模型层默认深挖路径固定为 `Multicopter + 非 HIL/SIH`，即 `ins/cf_ins + fms/mc_fms + control/mc_controller`。这是 `target/sieon/s1/config/model.py:7-21` 的默认组合。
- 必须把“接口层 skill”和“模型内部 skill”分开。
- 接口层回答：谁订阅什么 topic、谁发布什么 topic、step 周期是什么、参数和日志怎么绑定。
- 内部层回答：`FMS` 状态机怎么转、控制指令怎么生成、`Controller` 的环路结构是什么、`INS` 的导航数据如何解算。
- 当前阶段不需要从网上安装任何额外 skill。本地已有 skill 足够支撑架构阅读与 skill 拆分设计，避免引入供应链和信息泄露风险。

## 1. 分析范围与抽样策略

- 目标：为后续落地 FMT 领域 agent / sub-skills 提供一套自顶向下、边界清晰、能够深入模型内部的设计。
- 抽样范围：
- 板级与装配层：`target/sieon/s1/*`
- 运行时骨架：`src/startup.c`、`src/module/task_manager/*`、`src/module/ipc/*`、`src/module/log/*`、`src/module/sensor/*`
- 控制主链：`src/task/vehicle/normal/task_vehicle.c`
- 模型接口层：`src/model/*/*_interface.c`
- 模型内部层：`src/model/ins/cf_ins/lib/INS.c`、`src/model/fms/mc_fms/lib/FMS.c`、`src/model/control/mc_controller/lib/Controller.c`
- 变体证据：`src/model/ins/px4_ecl/*`、`target/sieon/s1/config/model.py`
- 事实与推断约束：
- 高置信度事实只基于代码。
- 关于不同 target 的共性，只在 `src/` 共享骨架处给出高置信度结论；凡是 `board.c`、`config/model.py`、`tasks/*` 一类内容，都明确标注为 `s1` 代表目标事实。

## 2. FMT-Firmware 的关键架构事实

### 2.1 装配方式是 `target` 驱动，不是硬编码主工程

- `target/sieon/s1/SConstruct:21-58` 通过 `--vehicle`、`--airframe`、`--sim` 选择构建宏与仿真模式。
- `target/sieon/s1/BuildLists.py:3-9` 汇总 `driver / hal / module / protocol / library / model / task` 七类配置。
- `target/sieon/s1/config/model.py:7-83` 选择 `plant / ins / fms / control` 组合，默认路径是 `cf_ins + mc_fms + mc_controller`，同时支持 `VTOL`。
- `target/sieon/s1/config/task.py:3-8` 指定通用任务集。
- `target/sieon/s1/tasks/SConscript` 说明 `s1` 还会额外编译 target-local tasks。
- `target/sieon/s1/link.lds:55-80` 保留 `ParamTab`、`MlogTab`、`TaskTab`，说明参数、日志总线、任务采用“声明即注册”的静态装配模式。

结论：

- FMT 的第一层认知不是“某个板子怎么初始化”，而是“target 目录如何装配共享 `src` 骨架”。
- 这决定了 `build/assembly` 必须是独立 skill，而不是附属于板级 skill。

### 2.2 启动链是 `BSP + task manager + post init` 的叠加

- `src/startup.c:57-66` 的主启动链依次调用 `bsp_initialize()`、`task_manager_init()`、`bsp_post_initialize()`、`task_manager_start()`。
- `target/sieon/s1/board/board.c:368-411` 的 `bsp_early_initialize()` 完成 CPU、heap、HAL、clock、GPIO、USART、systick、I2C、SPI、PWM、FDCAN、RC、sys stat 初始化。
- `target/sieon/s1/board/board.c:414-495` 的 `bsp_initialize()` 完成 `mcn_init()`、workqueue、storage、FS、param、mavproxy、USB、ADC、`rt_workqueue`、`lwIP`、ETH、板载传感器 bring-up 与注册。
- `target/sieon/s1/board/board.c:497-537` 的 `bsp_post_initialize()` 再加载 TOML、初始化 `pilot_cmd / gcs_cmd / auto_cmd / mission_data / actuator`、启动 `devmq`、LED、PMU、init script。
- `src/module/task_manager/task_manager.c:85-175` 先扫描 `TaskTab`，按依赖执行 init/create，再启动 `auto_start` 任务。

补充：

- `target/sieon/s1/tasks/task_dual_imu_attitude.c:633-641`
- `target/sieon/s1/tasks/task_bridge_mlog.c:134-142`
- `target/sieon/s1/tasks/task_can_bridge.c:354-362`

结论：

- `s1` 代表目标下，不能只看共享 `src/task/*`，还要把 target-local runtime extensions 纳入认知。

### 2.3 `task_vehicle` 是控制主循环

- `src/task/vehicle/normal/task_vehicle.c:108-117` 启动 `1ms` 周期定时器。
- `src/task/vehicle/normal/task_vehicle.c:67-86` 每次更新依次执行：
- `sensor_collect()`
- `pilot_cmd_collect()`
- `gcs_cmd_collect()`
- `mission_data_collect()`
- `plant_interface_step()`（SIH 时）
- `ins_interface_step()`
- `fms_interface_step()`
- `control_interface_step()`
- `send_actuator_cmd()`

结论：

- 这是后续理解 `INS -> FMS -> Controller -> actuator` 的主骨架。
- 任何模型 skill 在进入 `*_interface.c` 前，都应先知道这条调度顺序。

### 2.4 `uMCN`、`mlog`、sensor hub 是运行时基础设施

- `src/module/ipc/uMCN.h:40-99` 与 `src/module/ipc/uMCN.c:171-223,408-458` 说明 `uMCN` 不是简单队列，而是带 `renewal / event / callback` 语义的 topic bus。
- `src/module/log/mlog.h:70-137` 与 `src/module/log/mlog.c:290-370,606-651` 说明 `mlog` 会扫描 `MlogTab` 和参数表，形成结构化日志 schema。
- `src/task/logger/task_logger.c:109-178` 与 `src/module/param/sys_param.c:27-32` 说明 `logger task` 与 `SYSTEM.MLOG_MODE` 共同决定日志记录策略。
- `target/sieon/s1/board/board.c:464-480` 与 `src/module/sensor/sensor_hub.c:543-757` 说明传感器链路是 `driver init -> register_sensor_* -> sensor_collect -> topic publish`。

结论：

- FMT 里的“控制栈”不是只有模型代码；topic bus、sensor hub、logger 都是理解控制行为的前置条件。

### 2.5 模型接口层是 firmware 与生成模型的稳定边界

- `src/model/ins/cf_ins/lib/INS.h:8-17`、`src/model/fms/mc_fms/lib/FMS.h:4-13`、`src/model/control/mc_controller/lib/Controller.h:8-17` 都明确这些模型来自 Simulink 生成代码。
- `src/model/ins/cf_ins/ins_interface.c:543-616,667-758` 从 sensors / ext pos 组装 `INS_U`，调用 `INS_step()`，发布 `ins_output`，记录 `mlog`。
- `src/model/fms/mc_fms/fms_interface.c:292-380` 从 `pilot_cmd / gcs_cmd / auto_cmd / mission_data / ins_output / control_output` 组装 `FMS_U`，调用 `FMS_step()`，发布 `fms_output`。
- `src/model/control/mc_controller/control_interface.c:106-183` 从 `fms_output / ins_output` 组装 `Controller_U`，调用 `Controller_step()`，发布 `control_output`。

结论：

- `*_interface.c` 是 firmware 侧的“反腐层”。
- 但仅仅停在接口层，不足以回答你现在关心的“状态机转移、控制指令生成、导航解算算法”。

### 2.6 `FMS` 内部确实需要单独深挖技能

- `src/model/fms/mc_fms/lib/FMS.c:2052-2078` 的 `FMS_Mode()` 直接基于 `INS_Out.flag` 做模式合法性与 degrade 判定。
- `src/model/fms/mc_fms/lib/FMS.c:2484-2741` 的 `FMS_Mission()` 管 mission command、`wp_index`、`sp_waypoint`、`set_speed`。
- `src/model/fms/mc_fms/lib/FMS.c:2800-3370` 包含 `FMS_enter_internal_Auto()`、`FMS_enter_internal_Arm()`、`FMS_SubMode()`、`FMS_Arm()`、`FMS_Vehicle()`。
- `src/model/fms/mc_fms/lib/FMS.c:3690-3853` 的 `FMS_c11_FMS()` 是总状态机，内部有 `Command_Listener`、`Combo_Stick`、`Lost_Return`、`Vehicle` 等状态域。
- `src/model/fms/mc_fms/lib/FMS.c:4306-4486` 说明 `FMS` 还会结合 `Control_Out.actuator_cmd` 平均值和姿态判断 `on_ground`，再驱动状态机时间条件。
- `src/model/fms/mc_fms/lib/FMS.c:4998-5046`、`8880-8976`、`10750-10783` 说明 `FMS_Out` 不是单一路径生成，而是按控制模式分别输出 `u/v/w_cmd`、`p/q/r_cmd`、`phi/theta/throttle_cmd` 等字段。

结论：

- `fmt-fms-interface-reader` 只够回答“FMS 接什么、出什么”。
- 但“状态机怎么转”和“控制指令怎么生成”必须拆成更深的 sibling skills。

### 2.7 `Controller` 内部也需要单独深挖技能

- `src/model/control/mc_controller/lib/Controller.c:162-344` 一开始就基于 `INS_Out` 做导航系到机体系的速度转换，并处理 `FMS_Out.reset` 引发的积分器复位。
- `src/model/control/mc_controller/lib/Controller.c:431-505` 说明姿态指令会受 `ROLL_PITCH_CMD_LIM` 限幅。
- `src/model/control/mc_controller/lib/Controller.c:724-793` 说明不同模式下，`rate_cmd_B_radPs` 既可能直接取自 `FMS_Out.p/q/r_cmd`，也可能由姿态误差和协调项间接生成。
- `src/model/control/mc_controller/lib/Controller.c:796-1050` 说明速率环包含积分、微分、前馈、限幅，且 `RATE_I_MAX` 等参数参与约束。
- `src/model/control/mc_controller/lib/Controller.c:3001-3163` 说明位置/速度指令 `u_cmd / v_cmd / w_cmd` 会进一步转成姿态/油门与 `actuator_cmd`。
- `src/model/control/mc_controller/lib/Controller.c:3274-3441` 展示各积分器在固定周期下的离散更新。

结论：

- `fmt-controller-interface-reader` 只能解释 topic 边界。
- “控制器结构、环路层级、命令生成与限幅/积分器逻辑”必须进入单独 deep skill。

### 2.8 `INS` 导航解算算法同样需要单独深挖技能

- `src/model/ins/cf_ins/lib/INS.c:1387-1460` 的 `INS_step()` 本身就是一个大体量导航解算入口，而不是轻薄 wrapper。
- `src/model/ins/cf_ins/lib/INS.c:2363-2455` 说明磁场处理会把测量转换为 `uT`，再结合 `WMM` 查表和地理位置做地磁相关计算。
- `src/model/ins/cf_ins/lib/INS.c:2750-2838` 说明磁力计并不是“有数据就融合”，而是有有效性、超时、quality 积分等质量门控。
- `src/model/ins/cf_ins/lib/INS.c:6953-7033` 说明 GPS 速度观测存在显式 delay line。
- `src/model/ins/cf_ins/lib/INS.c:9218-9351` 说明空速同样经过 delay、差压到空速换算、限幅与补偿链路。
- `src/model/ins/cf_ins/lib/INS.c:10280-10310` 最终把 `vn / ve / vd / airspeed / lat / lon / alt / x_R / y_R` 等导航结果写入 `INS_Out`。

结论：

- “INS 需要哪些传感器”与“INS 的导航数据怎么解算”是两个不同问题。
- 前者属于 interface skill，后者属于 navigation deep skill。

### 2.9 `px4_ecl` 是不同方法论的 INS 变体，不应混进默认 `cf_ins` 技能

- `src/model/ins/px4_ecl/lib/INS.c:24-98` 说明 `px4_ecl` 先创建 EKF 实例，再下发参数，再运行 EKF 包装层。
- `src/model/ins/px4_ecl/ecl/EKF/estimator_interface.cpp:49-180` 说明它有自己的 IMU downsample、buffer、delay、rest detection 逻辑。
- `src/model/ins/px4_ecl/ecl/EKF/control.cpp:291-340` 说明位置/速度/heading 融合是 PX4 ECL 内部的显式 fusion control 流程。

结论：

- `cf_ins` 与 `px4_ecl` 的分析方法不同。
- 当前核心技能应围绕 `s1` 默认 `cf_ins` 路径设计；若后续 target 切到 `px4_ecl`，应新增 sibling skill，而不是把 `cf_ins` skill 扩成“大而全 INS”。

## 3. 第一性原理定义

### 3.1 不可再约简的任务单元

“回答一个边界明确的 FMT 固件架构问题，并给出代码证据、关键数据流或控制流、边界说明与未覆盖项。”

### 3.2 输入 / 输出 / Definition of Done

- 输入：
- 仓库路径
- `target / vehicle / sim` 组合
- 问题焦点，例如“FMS 状态机”“Controller 结构”“INS 导航解算”
- 输出：
- 架构说明
- 关键文件与调用链
- 关键 topic / 状态 / 参数 / 输出字段
- 事实、推断、未覆盖项
- Definition of Done：
- 指出入口文件或注册点
- 指出运行时数据流或控制流
- 说清负责边界与不负责边界
- 给出至少一组代码锚点

### 3.3 不可跳过的前置依赖

- 读任何模型前，先确认 `target/sieon/s1/config/model.py` 选择的是哪条模型组合。
- 读模型接口前，先确认 `task_vehicle` 中 `sensor -> command -> INS -> FMS -> Controller -> actuator` 的调度顺序。
- 读 `FMS` 前，先确认 `pilot_cmd / gcs_cmd / auto_cmd / mission_data` 的入口路径。
- 读 `Controller` 前，先确认 `FMS_Out` 字段语义。
- 读 `INS` 算法前，先确认 `sensor_hub` 发布的 topic 和有效性/时间戳语义。

### 3.4 为什么必须把接口层和模型内部层拆开

- 触发边界不同：
- “INS 订阅哪些 topic” 与 “INS 如何做导航解算” 不是同一类问题。
- “FMS 接什么输入” 与 “FMS 状态机怎么转” 不是同一类问题。
- “Controller 输出去哪” 与 “Controller 内部几层环路、怎么限幅” 不是同一类问题。
- 方法论不同：
- 接口层主要读 `*_interface.c`、`*_EXPORT`、topic 依赖、日志绑定。
- 内部层主要读生成代码的状态机、控制律、离散积分器、限幅、观测融合、delay chain。
- 完成标准不同：
- 接口层 DoD 是解释边界。
- 内部层 DoD 是解释算法或状态演化。

## 4. 拆分结论

- 是否建议拆分：`是`
- 核心原因：
- FMT 的真实边界天然分成 `build / platform / runtime / io / model-interface / model-internal`
- 用户触发语句明显不同：问 `TaskTab`、`uMCN`、`FMS 状态机`、`INS 算法` 时，不应误触发同一个大 skill
- 内部模型阅读已经不是普通接口阅读，必须单独建深挖 skill
- 如果不拆：
- 用户问“FMS 状态机如何切换”时，会被一个只会解释 `fms_interface.c` 的 skill 误处理
- 用户问“INS 解算算法”时，会被一个只会罗列传感器输入的 skill 误处理

## 5. Agent 结构设计

### 5.1 顶层 Agent

`fmt-firmware-architect`

- 角色：
- FMT 固件代码阅读唯一默认入口
- 先确认 `target / vehicle / sim`
- 再把请求路由到最合适的原子 skill 或 workflow skill
- 负责：
- 选择阅读顺序
- 汇总跨 skill 结论
- 明确当前代表路径是否为 `s1 + mc + cf_ins + mc_fms + mc_controller`
- 不负责：
- 自己吞掉所有细节分析
- 在没有必要时跨越多个专业阶段

### 5.2 Workflow Skills

#### `fmt-system-topdown-reader`

- 用途：从 `target` 装配一路走到控制主链，建立系统级心智模型。
- 调用顺序：
- `fmt-build-assembly-reader`
- `fmt-board-bootstrap-reader`
- `fmt-task-scheduling-reader`
- `fmt-topic-bus-reader`
- `fmt-sensor-pipeline-reader`
- `fmt-command-ingress-reader`
- `fmt-ins-interface-reader`
- `fmt-fms-interface-reader`
- `fmt-controller-interface-reader`
- `fmt-actuator-output-reader`

#### `fmt-runtime-infrastructure-reader`

- 用途：只看运行时基础设施，不钻模型内部。
- 调用顺序：
- `fmt-task-scheduling-reader`
- `fmt-topic-bus-reader`
- `fmt-logging-pipeline-reader`

#### `fmt-control-stack-reader`

- 用途：只看控制主链的端到端数据流，不钻状态机和控制律内部。
- 调用顺序：
- `fmt-command-ingress-reader`
- `fmt-sensor-pipeline-reader`
- `fmt-ins-interface-reader`
- `fmt-fms-interface-reader`
- `fmt-controller-interface-reader`
- `fmt-actuator-output-reader`

#### `fmt-ins-deep-dive-reader`

- 用途：详细分析 INS，从传感器入口一路进入导航解算。
- 调用顺序：
- `fmt-sensor-pipeline-reader`
- `fmt-ins-interface-reader`
- `fmt-ins-cf-navigation-reader`

#### `fmt-fms-deep-dive-reader`

- 用途：详细分析 FMS，从指令入口一路进入状态机和命令生成。
- 调用顺序：
- `fmt-command-ingress-reader`
- `fmt-fms-interface-reader`
- `fmt-fms-state-machine-reader`
- `fmt-fms-command-generation-reader`

#### `fmt-controller-deep-dive-reader`

- 用途：详细分析 Controller，从 `FMS_Out` 进入控制器结构与执行输出。
- 调用顺序：
- `fmt-fms-interface-reader`
- `fmt-controller-interface-reader`
- `fmt-controller-structure-reader`
- `fmt-actuator-output-reader`

### 5.3 原子 Skills

#### `fmt-build-assembly-reader`

- 主目标：解释 `target/sieon/s1` 如何装配共享 `src` 骨架。
- 不负责：运行时调度、模型内部算法。
- 证据锚点：
- `target/sieon/s1/SConstruct:21-58`
- `target/sieon/s1/BuildLists.py:3-9`
- `target/sieon/s1/config/model.py:7-83`
- `target/sieon/s1/config/task.py:3-8`
- `target/sieon/s1/tasks/SConscript`
- `target/sieon/s1/link.lds:55-80`
- DoD：能说清 target 宏、模型选择、任务选择、静态注册段。

#### `fmt-board-bootstrap-reader`

- 主目标：解释 `s1` 的 BSP bring-up 与 post init。
- 不负责：`task_vehicle` 内部控制逻辑。
- 证据锚点：
- `target/sieon/s1/board/board.c:368-537`
- DoD：能说清 early/init/post 三阶段职责。

#### `fmt-task-scheduling-reader`

- 主目标：解释 `TaskTab` 注册、任务依赖、auto start 与 `vehicle` 主循环。
- 不负责：topic 内容语义、控制器内部算法。
- 证据锚点：
- `src/startup.c:57-66`
- `src/module/task_manager/task_manager.h:19-41`
- `src/module/task_manager/task_manager.c:85-175`
- `src/task/vehicle/normal/task_vehicle.c:67-117`
- DoD：能说清任务是如何从静态注册走到运行的。

#### `fmt-topic-bus-reader`

- 主目标：解释 `uMCN` 的 topic、renewal、event、copy 语义。
- 不负责：具体 topic 的业务字段解释。
- 证据锚点：
- `src/module/ipc/uMCN.h:40-99`
- `src/module/ipc/uMCN.c:171-223`
- `src/module/ipc/uMCN.c:408-458`
- DoD：能说明 publish/subscribe/copy 的运行语义。

#### `fmt-logging-pipeline-reader`

- 主目标：解释 `mlog + ulog + logger task` 的结构化日志链路。
- 不负责：飞行日志性能结论、调参报告。
- 证据锚点：
- `src/module/log/mlog.h:70-137`
- `src/module/log/mlog.c:290-370`
- `src/module/log/mlog.c:606-651`
- `src/task/logger/task_logger.c:109-178`
- `src/module/param/sys_param.c:27-32`
- DoD：能说清 `MlogTab`、参数快照、logger task 的关系。

#### `fmt-sensor-pipeline-reader`

- 主目标：解释 `driver -> register_sensor_* -> sensor_collect -> sensor topics`。
- 不负责：INS 导航解算。
- 证据锚点：
- `target/sieon/s1/board/board.c:464-480`
- `src/module/sensor/sensor_hub.c:543-647`
- `src/module/sensor/sensor_hub.c:654-757`
- DoD：能讲清传感器数据从驱动到 topic 的完整路径。

#### `fmt-command-ingress-reader`

- 主目标：解释 `pilot_cmd / gcs_cmd / auto_cmd / mission_data` 如何成为 FMS 输入。
- 不负责：FMS 内部状态机和控制律。
- 证据锚点：
- `target/sieon/s1/board/board.c:505-518`
- `src/module/sysio/pilot_cmd.c:46-49`
- `src/module/sysio/pilot_cmd.c:301-345`
- `src/model/fms/mc_fms/fms_interface.c:292-319`
- DoD：能讲清 RC/GCS/auto/mission 输入如何进入 `FMS_U`。

#### `fmt-ins-interface-reader`

- 主目标：解释 `ins_interface.c` 的 topic 边界、`INS_U` 组装、`ins_output` 发布与日志绑定。
- 不负责：`INS.c` 内部导航算法。
- 证据锚点：
- `src/model/ins/cf_ins/lib/INS.h:8-17`
- `src/model/ins/cf_ins/ins_interface.c:543-616`
- `src/model/ins/cf_ins/ins_interface.c:667-705`
- `src/model/ins/cf_ins/ins_interface.c:708-758`
- DoD：能列出 INS 的输入 topics、输出 topic、mlog buses。

#### `fmt-ins-cf-navigation-reader`

- 主目标：详细解释默认 `cf_ins` 的导航数据解算算法。
- 不负责：传感器 wiring、`px4_ecl` 变体。
- 典型问题：
- “INS 里速度、位置、地理坐标、空速是怎么解出来的”
- “磁场/GPS/空速在 INS 里怎么参与解算”
- 证据锚点：
- `src/model/ins/cf_ins/lib/INS.c:1387-1460`
- `src/model/ins/cf_ins/lib/INS.c:2363-2455`
- `src/model/ins/cf_ins/lib/INS.c:2750-2838`
- `src/model/ins/cf_ins/lib/INS.c:6953-7033`
- `src/model/ins/cf_ins/lib/INS.c:9218-9351`
- `src/model/ins/cf_ins/lib/INS.c:10280-10310`
- DoD：能说清观测质量门控、delay chain、主要解算输出，以及它们如何进入 `INS_Out`。

#### `fmt-fms-interface-reader`

- 主目标：解释 `fms_interface.c` 如何组装 `FMS_U` 并发布 `fms_output`。
- 不负责：穷举 `FMS.c` 内部 chart 分支。
- 证据锚点：
- `src/model/fms/mc_fms/lib/FMS.h:4-13`
- `src/model/fms/mc_fms/fms_interface.c:26-35`
- `src/model/fms/mc_fms/fms_interface.c:292-363`
- `src/model/fms/mc_fms/fms_interface.c:366-380`
- DoD：能说清 `FMS_U` 来源、`fms_output` 去向、日志绑定点。

#### `fmt-fms-state-machine-reader`

- 主目标：详细解释 `mc_fms` 的状态机结构、状态域、转移条件和 degrade 逻辑。
- 不负责：每个状态下所有控制指令的数值生成细节。
- 典型问题：
- “FMS 怎么从 Disarm 到 Arm 到 Auto”
- “模式切换依赖哪些 INS 标志、命令、时间条件”
- “lost return / combo stick / command listener 分别做什么”
- 证据锚点：
- `src/model/fms/mc_fms/lib/FMS.c:2052-2078`
- `src/model/fms/mc_fms/lib/FMS.c:2484-2741`
- `src/model/fms/mc_fms/lib/FMS.c:2800-3370`
- `src/model/fms/mc_fms/lib/FMS.c:3690-3853`
- `src/model/fms/mc_fms/lib/FMS.c:4428-4486`
- DoD：能画出主要状态域、关键转移条件、输入依赖和输出状态字段。

#### `fmt-fms-command-generation-reader`

- 主目标：详细解释 `FMS_Out` 中 `ctrl_mode / u_cmd / v_cmd / w_cmd / p_cmd / q_cmd / r_cmd / throttle_cmd` 的生成路径。
- 不负责：Controller 内部如何消费这些命令。
- 典型问题：
- “不同模式下 FMS 到底输出哪一类控制指令”
- “mission / auto / pilot 输入如何变成 setpoint”
- “sp_waypoint / set_speed / yaw rate 怎么影响输出”
- 证据锚点：
- `src/model/fms/mc_fms/lib/FMS.c:2593-2640`
- `src/model/fms/mc_fms/lib/FMS.c:4998-5046`
- `src/model/fms/mc_fms/lib/FMS.c:8880-8976`
- `src/model/fms/mc_fms/lib/FMS.c:10750-10783`
- `src/model/fms/mc_fms/lib/FMS.c:12957-13019`
- DoD：能按控制模式说明 `FMS_Out` 字段来源和限幅逻辑。

#### `fmt-controller-interface-reader`

- 主目标：解释 `control_interface.c` 如何把 `fms_output + ins_output` 送入 Controller。
- 不负责：控制器内部环路与控制律细节。
- 证据锚点：
- `src/model/control/mc_controller/lib/Controller.h:8-17`
- `src/model/control/mc_controller/control_interface.c:25-30`
- `src/model/control/mc_controller/control_interface.c:106-143`
- `src/model/control/mc_controller/control_interface.c:145-183`
- DoD：能说明 Controller 的 topic 依赖、参数绑定、输出 topic。

#### `fmt-controller-structure-reader`

- 主目标：详细解释 `mc_controller` 的控制器结构、命令生成路径、积分器/限幅/前馈逻辑和 `actuator_cmd` 输出骨架。
- 不负责：底层 PWM 寄存器或电机驱动实现。
- 典型问题：
- “Controller 有几层环路”
- “`FMS_Out` 怎么变成 `rate_cmd`、姿态指令和 `actuator_cmd`”
- “复位、积分器、限幅和前馈在哪里发生”
- 证据锚点：
- `src/model/control/mc_controller/lib/Controller.c:162-344`
- `src/model/control/mc_controller/lib/Controller.c:431-505`
- `src/model/control/mc_controller/lib/Controller.c:724-793`
- `src/model/control/mc_controller/lib/Controller.c:796-1050`
- `src/model/control/mc_controller/lib/Controller.c:3001-3163`
- `src/model/control/mc_controller/lib/Controller.c:3274-3441`
- DoD：能说明环路层级、模式分支、关键参数限幅、输出生成路径。

#### `fmt-actuator-output-reader`

- 主目标：解释 `control_output` 如何依配置映射到执行机构。
- 不负责：底层外设寄存器配置细节。
- 证据锚点：
- `src/module/sysio/actuator_cmd.c:79-149`
- `src/module/sysio/actuator_cmd.c:152-190`
- `target/sieon/s1/config/sysconfig.toml`
- DoD：能说清 topic 来源、from/to 映射、TOML 绑定关系。

### 5.4 变体扩展规则

- 若 `config/model.py` 选择 `ins/px4_ecl`，新增 `fmt-ins-px4-ecl-reader`，不要把 `fmt-ins-cf-navigation-reader` 扩成多算法混合 skill。
- 若 `vehicle` 切到 `VTOL`，新增 `fmt-vtol-fms-state-machine-reader` 与 `fmt-vtol-controller-structure-reader`，不要把 `mc_*` 默认技能污染成多机型混合 skill。
- 若后续需要分析 `external_ins`，同样新增 sibling skill，而不是塞进现有 `ins-interface` 或 `ins-cf-navigation`。

## 6. 推荐仓库组织方案

```text
fmt-firmware/
├─ workflows/
│  ├─ fmt-firmware-architect/
│  ├─ fmt-system-topdown-reader/
│  ├─ fmt-runtime-infrastructure-reader/
│  ├─ fmt-control-stack-reader/
│  ├─ fmt-ins-deep-dive-reader/
│  ├─ fmt-fms-deep-dive-reader/
│  └─ fmt-controller-deep-dive-reader/
├─ build/
│  └─ fmt-build-assembly-reader/
├─ platform/
│  └─ fmt-board-bootstrap-reader/
├─ runtime/
│  ├─ fmt-task-scheduling-reader/
│  ├─ fmt-topic-bus-reader/
│  └─ fmt-logging-pipeline-reader/
├─ io/
│  ├─ fmt-sensor-pipeline-reader/
│  ├─ fmt-command-ingress-reader/
│  └─ fmt-actuator-output-reader/
└─ models/
   ├─ interface/
   │  ├─ fmt-ins-interface-reader/
   │  ├─ fmt-fms-interface-reader/
   │  └─ fmt-controller-interface-reader/
   ├─ internal/
   │  ├─ fmt-ins-cf-navigation-reader/
   │  ├─ fmt-fms-state-machine-reader/
   │  ├─ fmt-fms-command-generation-reader/
   │  └─ fmt-controller-structure-reader/
   └─ variants/
      ├─ fmt-ins-px4-ecl-reader/                # future extension
      ├─ fmt-vtol-fms-state-machine-reader/     # future extension
      └─ fmt-vtol-controller-structure-reader/  # future extension
```

命名规则：

- 统一使用 `fmt-<object>-<action>`
- `workflow` 只做编排，不吞掉原子 skill 的专业边界
- `reader` 只表示代码阅读技能；未来若加日志解码或调参报告，使用 `decoder / analyzer / reporter`

## 7. 迁移计划

### Phase 1：先落顶层入口与模型深挖核心链

- 先实现：
- `fmt-firmware-architect`
- `fmt-build-assembly-reader`
- `fmt-task-scheduling-reader`
- `fmt-topic-bus-reader`
- `fmt-command-ingress-reader`
- `fmt-sensor-pipeline-reader`
- `fmt-ins-interface-reader`
- `fmt-ins-cf-navigation-reader`
- `fmt-fms-interface-reader`
- `fmt-fms-state-machine-reader`
- `fmt-fms-command-generation-reader`
- `fmt-controller-interface-reader`
- `fmt-controller-structure-reader`

原因：

- 这是最直接覆盖你当前关注点的最小可用集合。
- 它已经能回答 `FMS` 状态机、控制命令生成、`Controller` 结构、`INS` 导航解算算法。

### Phase 2：补板级、日志与执行出口

- 再实现：
- `fmt-board-bootstrap-reader`
- `fmt-logging-pipeline-reader`
- `fmt-actuator-output-reader`

### Phase 3：补 workflows 与变体扩展

- 实现：
- `fmt-system-topdown-reader`
- `fmt-runtime-infrastructure-reader`
- `fmt-control-stack-reader`
- `fmt-ins-deep-dive-reader`
- `fmt-fms-deep-dive-reader`
- `fmt-controller-deep-dive-reader`
- 视 target 需要再增加：
- `fmt-ins-px4-ecl-reader`
- `fmt-vtol-fms-state-machine-reader`
- `fmt-vtol-controller-structure-reader`

## 8. 风险与控制

### 风险 1：过度拆分

- 表现：用户只想“理解 FMS”，却被迫在多个细粒度 skill 之间自己拼装。
- 控制：
- 用 `fmt-firmware-architect` 作为默认入口。
- 用 `fmt-fms-deep-dive-reader / fmt-ins-deep-dive-reader / fmt-controller-deep-dive-reader` 承担常见端到端深挖场景。

### 风险 2：接口层与内部层重叠

- 典型重叠：
- `fmt-ins-interface-reader` 与 `fmt-ins-cf-navigation-reader`
- `fmt-fms-interface-reader` 与 `fmt-fms-state-machine-reader`
- `fmt-fms-command-generation-reader` 与 `fmt-controller-structure-reader`
- 控制：
- interface skill 默认只读 `*_interface.c`、`*_EXPORT`、topic 与日志绑定。
- internal skill 默认从生成代码入口函数、chart、环路、delay、积分器、限幅开始。

### 风险 3：把 `s1` 事实误当作全局事实

- 控制：
- 所有板级结论都显式标注为 `s1` 代表目标事实。
- 顶层 agent 在进入 skill 前强制确认 `target / vehicle / sim`。

### 风险 4：把多算法或多机型变体塞进一个 skill

- 控制：
- `cf_ins` 与 `px4_ecl` 分技能。
- `mc_fms / mc_controller` 与 `vtol_*` 分技能。

## 9. 质量评分

- Activation Precision：`5/5`
- 原因：用户问 `FMS 状态机`、`Controller 结构`、`INS 算法` 时，都能命中独立 skill。
- Boundary Clarity：`5/5`
- 原因：接口层与内部层已经正式拆开。
- Professional Depth：`5/5`
- 原因：新方案已经覆盖状态机、控制律、导航解算，不再停留在接口层。
- Usability：`4.5/5`
- 原因：原子 skill 数量变多，但通过顶层 agent 和 model deep-dive workflows 控制使用复杂度。
- Maintainability：`4.5/5`
- 原因：未来新加 `px4_ecl`、`VTOL` 只需挂 sibling，不需要重写整棵树。
- Verifiability：`5/5`
- 原因：每个 skill 都有明确代码锚点和 DoD。

总评：

- `通过`
- 这套结构已经能覆盖“正确认识 FMT 固件骨架”与“细粒度分析 FMS / Controller / INS 内部行为”两个目标。

## 10. 最终建议

不要把 FMT 做成一个“大而全”的单体 skill。

更稳妥的方案是：

1. 用 `fmt-firmware-architect` 做唯一总入口。
2. 第一层先覆盖 `build / runtime / io / model-interface`。
3. 第二层明确补上 `FMS 状态机`、`FMS 控制命令生成`、`Controller 结构`、`INS 导航解算` 四个 deep skills。
4. 对 `px4_ecl`、`VTOL` 这类方法论不同的变体，后续用 sibling skills 扩展，不污染默认 `s1 + mc` 路径。

## 11. 下一步实施清单

- [ ] 你确认这份 agent 树与 skill 边界
- [ ] 我按这份报告创建各目录与 `SKILL.md`
- [ ] 先实现 Phase 1 的核心 skills
- [ ] 再补 workflows
- [ ] 最后按 target 需要补 `px4_ecl / VTOL` 变体 skills

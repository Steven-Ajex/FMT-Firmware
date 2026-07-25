# AGENTS.md for `e:\SUST\FMT-Firmware`

## Skills

This repository provides local FMT-Firmware skills under `skills/fmt-firmware/`.

### Default routing

- If the user does not explicitly name a more specific `fmt-*` skill, start with `fmt-firmware-architect`.
- Default representative path for code reading:
  - `target/sieon/s1`
  - `Multicopter`
  - `非 HIL/SIH`
  - `ins/cf_ins + fms/mc_fms + control/mc_controller`
- If the actual `target / vehicle / sim / model` differs from the default path, state that clearly before using any deep-dive model skill.

### Available repo-local skills

- `fmt-firmware-architect`: FMT-Firmware 总入口编排技能。用于用户想理解 FMT 飞控架构、但未明确具体子模块时，先确认 target、vehicle、sim，再路由到装配、板级 bring-up、任务调度、topic 总线、日志链路、传感器链路、INS、FMS、Controller、actuator 等更细技能；也适用于需要跨多个 FMT 子技能汇总结论的场景。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-firmware-architect/SKILL.md`)
- `fmt-system-topdown-reader`: FMT-Firmware 自顶向下总览编排技能。用于用户想系统建立对 FMT 固件的正确认识时，从 target 装配、板级 bring-up、任务调度、topic 总线、传感器链路一路走到 INS、FMS、Controller 与 actuator，形成完整心智模型。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-system-topdown-reader/SKILL.md`)
- `fmt-runtime-infrastructure-reader`: FMT-Firmware 运行时基础设施编排技能。用于用户主要关心任务调度、topic 总线、日志系统等 shared runtime 机制，而不是控制模型内部状态机或算法时使用。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-runtime-infrastructure-reader/SKILL.md`)
- `fmt-control-stack-reader`: FMT-Firmware 控制主链编排技能。用于用户只关心从传感器和指令入口到 INS、FMS、Controller、actuator 的端到端数据流，而暂时不需要深入生成模型内部状态机和控制律时使用。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-control-stack-reader/SKILL.md`)
- `fmt-ins-deep-dive-reader`: FMT-Firmware INS 深挖编排技能。用于需要细致分析 INS 的传感器输入、interface 边界以及默认 cf_ins 导航数据解算算法时使用，适合回答 INS 需要哪些观测、如何做质量门控、延迟处理以及如何形成导航输出。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-ins-deep-dive-reader/SKILL.md`)
- `fmt-fms-deep-dive-reader`: FMT-Firmware FMS 深挖编排技能。用于需要细致分析 FMS 的输入来源、状态机转换、模式切换条件以及控制指令生成路径时使用，尤其适合回答 arm、auto、mission、return、manual 等模式如何转移和产生命令。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-fms-deep-dive-reader/SKILL.md`)
- `fmt-controller-deep-dive-reader`: FMT-Firmware Controller 深挖编排技能。用于需要细致分析 mc_controller 的控制器结构、命令生成路径、积分器与限幅逻辑以及 actuator 输出骨架时使用，适合回答 FMS 输出如何进入控制器并变成执行命令。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/workflows/fmt-controller-deep-dive-reader/SKILL.md`)
- `fmt-build-assembly-reader`: FMT-Firmware 构建与装配阅读技能。用于分析 target 如何通过 SConstruct、BuildLists、config、link.lds 和静态注册段装配共享 src 骨架，适合回答控制板适配、模型选择、任务选择以及 TaskTab、ParamTab、MlogTab 的装配问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/build/fmt-build-assembly-reader/SKILL.md`)
- `fmt-board-bootstrap-reader`: FMT-Firmware 板级 bring-up 阅读技能。用于分析 target/sieon/s1 的 early init、bsp_initialize、bsp_post_initialize 三阶段，以及 FDCAN、RC、lwIP、ETH、板载传感器初始化与 target-local tasks 等板级扩展行为。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/platform/fmt-board-bootstrap-reader/SKILL.md`)
- `fmt-task-scheduling-reader`: FMT-Firmware 任务调度阅读技能。用于分析 startup、TaskTab、task_manager 与 vehicle 主循环如何配合启动和驱动系统，适合回答任务注册、依赖初始化、auto_start、周期执行和控制主循环顺序的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/runtime/fmt-task-scheduling-reader/SKILL.md`)
- `fmt-topic-bus-reader`: FMT-Firmware topic 总线阅读技能。用于分析 uMCN 的 topic、publish、subscribe、copy、renewal、event、callback 语义，以及模块之间如何通过 topic 而不是直接函数调用通信。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/runtime/fmt-topic-bus-reader/SKILL.md`)
- `fmt-logging-pipeline-reader`: FMT-Firmware 日志链路阅读技能。用于分析 mlog、ulog、logger task、MlogTab 和参数快照的关系，适合回答结构化日志 bus schema、自动录制策略和日志系统初始化流程的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/runtime/fmt-logging-pipeline-reader/SKILL.md`)
- `fmt-sensor-pipeline-reader`: FMT-Firmware 传感器链路阅读技能。用于分析板级驱动初始化、register_sensor、sensor_collect 以及 sensor topics 的完整数据路径，适合回答传感器驱动如何接入系统以及采样、校准、滤波、发布过程的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/io/fmt-sensor-pipeline-reader/SKILL.md`)
- `fmt-command-ingress-reader`: FMT-Firmware 指令入口阅读技能。用于分析 pilot_cmd、gcs_cmd、auto_cmd、mission_data 如何进入系统并最终组装成 FMS_U，适合回答 RC、GCS、mission 和自动指令入口、映射与汇聚问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/io/fmt-command-ingress-reader/SKILL.md`)
- `fmt-actuator-output-reader`: FMT-Firmware 执行输出阅读技能。用于分析 control_output 如何通过 actuator_cmd 和 sysconfig.toml 的 from、to 映射落到具体执行机构，适合回答输出通道绑定、HIL 与实机路径差异以及执行器出口配置问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/io/fmt-actuator-output-reader/SKILL.md`)
- `fmt-ins-interface-reader`: FMT-Firmware INS 接口阅读技能。用于分析 ins_interface.c 如何把 sensor 和外部位置相关 topics 组装为 INS_U、调用 INS_step 并发布 ins_output，适合回答 INS 订阅什么、输出什么、周期是什么以及日志如何绑定的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/interface/fmt-ins-interface-reader/SKILL.md`)
- `fmt-fms-interface-reader`: FMT-Firmware FMS 接口阅读技能。用于分析 fms_interface.c 如何把 pilot_cmd、gcs_cmd、auto_cmd、mission_data、ins_output、control_output 组装成 FMS_U、调用 FMS_step 并发布 fms_output，适合回答 FMS 输入输出边界与周期问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/interface/fmt-fms-interface-reader/SKILL.md`)
- `fmt-controller-interface-reader`: FMT-Firmware Controller 接口阅读技能。用于分析 control_interface.c 如何把 fms_output 和 ins_output 组装成 Controller_U、调用 Controller_step 并发布 control_output，适合回答 Controller 的输入 topic、输出 topic、参数绑定和周期问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/interface/fmt-controller-interface-reader/SKILL.md`)
- `fmt-ins-cf-navigation-reader`: FMT-Firmware 默认 cf_ins 导航解算阅读技能。用于细致分析 INS.c 中的导航数据解算过程，包括磁场处理、观测质量门控、GPS 和空速延迟链路以及 INS_Out 的生成，适合回答 INS 中速度、位置、地理坐标和空速如何被解算的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/internal/fmt-ins-cf-navigation-reader/SKILL.md`)
- `fmt-fms-state-machine-reader`: FMT-Firmware mc_fms 状态机阅读技能。用于细致分析 FMS.c 中的状态机结构、状态域、模式切换、arm 和 auto 转换、mission 驱动以及 degrade 和 lost return 等逻辑，适合回答 FMS 状态如何转移以及转移条件来自哪里的问答。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/internal/fmt-fms-state-machine-reader/SKILL.md`)
- `fmt-fms-command-generation-reader`: FMT-Firmware mc_fms 控制命令生成阅读技能。用于细致分析 FMS_Out 中 ctrl_mode、u_cmd、v_cmd、w_cmd、p_cmd、q_cmd、r_cmd、phi_cmd、theta_cmd、throttle_cmd 等字段如何在不同模式和 mission 场景下生成，适合回答 FMS 中控制指令生成路径的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/internal/fmt-fms-command-generation-reader/SKILL.md`)
- `fmt-controller-structure-reader`: FMT-Firmware mc_controller 结构阅读技能。用于细致分析 Controller.c 中的控制器层级、命令生成路径、姿态和速率环、积分器、微分、前馈、限幅以及 actuator_cmd 输出骨架，适合回答 Controller 结构和控制指令如何变成执行输出的问题。 (file: `E:/SUST/FMT-Firmware/skills/fmt-firmware/models/internal/fmt-controller-structure-reader/SKILL.md`)

### How to use repo-local skills

- Discovery: Repo-local skills live under `skills/fmt-firmware/`.
- Trigger rules: If the user names a `fmt-*` skill explicitly, use it directly. If not, prefer `fmt-firmware-architect` as the default entry.
- Scope discipline:
  - `models/interface/*` only explains firmware/model boundaries, topics, periods, and logging bindings.
  - `models/internal/*` explains generated-model internals such as state machines, control command generation, controller structure, and navigation algorithms.
- Workflow vs atomic:
  - Prefer `workflows/*` for broad or end-to-end questions.
  - Prefer atomic skills for narrow questions with a clear boundary.
- Variant handling:
  - Default deep-dive model skills cover the `s1 + Multicopter + cf_ins + mc_fms + mc_controller` path.
  - If the actual code path is `px4_ecl`, `VTOL`, or other non-default variants, say so explicitly before continuing. Do not silently assume the default path still applies.

---
name: fmt-firmware-architect
description: FMT-Firmware 总入口编排技能。用于用户想理解 FMT 飞控架构、但未明确具体子模块时，先确认 target、vehicle、sim，再路由到装配、板级 bring-up、任务调度、topic 总线、日志链路、传感器链路、INS、FMS、Controller、actuator 等更细技能；也适用于需要跨多个 FMT 子技能汇总结论的场景。
---

# Fmt Firmware Architect

## Overview

这是 FMT-Firmware 代码阅读的唯一默认入口。先确认 `target / vehicle / sim`，再把问题路由到最合适的原子 skill 或 workflow skill。

当用户的问题比较宽、边界不清，或者需要跨多个 `fmt-*` skills 汇总时，读取 `references/architect-routing-playbook.md`。

默认代表路径：

- `target/sieon/s1`
- `Multicopter`
- `非 HIL/SIH`
- `ins/cf_ins + fms/mc_fms + control/mc_controller`

## Workflow

1. 先判断用户是在问 `build / platform / runtime / io / models` 哪一层。
2. 若用户只说“帮我理解 FMT”，优先走 `$fmt-system-topdown-reader`。
3. 若用户只关心控制主链，走 `$fmt-control-stack-reader`。
4. 若用户明确点名 `INS / FMS / Controller` 且要细节，分别走对应 deep-dive workflow。
5. 若用户已经点名某个原子 skill，就不要再做额外编排。

## Routing Rules

- `target/config/SConstruct/link.lds/TaskTab` 问题：路由到对应装配或调度 skill。
- `uMCN / topic / mlog / logger` 问题：路由到 runtime skills。
- `pilot_cmd / gcs_cmd / mission / actuator` 问题：路由到 io skills。
- `*_interface.c` 边界问题：路由到 models/interface。
- `FMS 状态机 / 控制命令生成 / Controller 结构 / INS 导航解算`：路由到 models/internal 或 deep-dive workflows。

## Output

输出应包含：

- 当前假定的 `target / vehicle / sim`
- 选中的 skills 与顺序
- 跨 skill 汇总结论
- 仍需进一步深挖的点

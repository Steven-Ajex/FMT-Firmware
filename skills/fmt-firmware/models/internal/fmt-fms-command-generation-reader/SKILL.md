---
name: fmt-fms-command-generation-reader
description: FMT-Firmware mc_fms 控制命令生成阅读技能。用于细致分析 FMS_Out 中 ctrl_mode、u_cmd、v_cmd、w_cmd、p_cmd、q_cmd、r_cmd、phi_cmd、theta_cmd、throttle_cmd 等字段如何在不同模式和 mission 场景下生成，适合回答 FMS 中控制指令生成路径的问题。
---

# Fmt Fms Command Generation Reader

## Overview

本技能聚焦 `FMS_Out` 具体命令字段的来源和分支，不重新解释状态机结构。

当用户需要默认 `s1 + mc_fms` 路径下的命令生成阅读顺序、输出字段锚点和模式到命令的映射时，读取 `references/mc-fms-command-map.md`。

## Preconditions

先确认：

- 已理解 `$fmt-fms-interface-reader`
- 已理解 `$fmt-fms-state-machine-reader`

## Read Order

1. `Cmd_In.sp_waypoint / cur_waypoint / set_speed` 的更新段
2. 各种 `FMS_Y.FMS_Out.*` 赋值段
3. 与 `VEL_XY_LIM / VEL_Z_LIM / YAW_RATE_LIM` 相关的限幅段
4. pilot direct control 段

## Focus

- 不同 `ctrl_mode` 下到底输出哪类命令
- mission、auto、pilot 输入如何影响 `FMS_Out`
- setpoint 和限幅如何形成

## Not In Scope

- `Controller` 如何消费这些命令
- `vtol_fms`

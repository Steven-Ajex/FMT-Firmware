---
name: fmt-fms-state-machine-reader
description: FMT-Firmware mc_fms 状态机阅读技能。用于细致分析 FMS.c 中的状态机结构、状态域、模式切换、arm 和 auto 转换、mission 驱动以及 degrade 和 lost return 等逻辑，适合回答 FMS 状态如何转移以及转移条件来自哪里的问答。
---

# Fmt Fms State Machine Reader

## Overview

本技能聚焦 `mc_fms` 的状态机，不负责解释所有控制量的数值计算。

当用户需要默认 `s1 + mc_fms` 路径下的状态机阅读顺序、状态域分层和关键代码锚点时，读取 `references/mc-fms-state-machine-map.md`。

## Preconditions

先确认：

- 已理解 `$fmt-command-ingress-reader`
- 已理解 `$fmt-fms-interface-reader`

## Read Order

1. `FMS_Mode()`
2. `FMS_Mission()`
3. `FMS_enter_internal_Auto()` 与 `FMS_enter_internal_Arm()`
4. `FMS_SubMode()`、`FMS_Arm()`、`FMS_Vehicle()`
5. `FMS_c11_FMS()`

## Focus

- 状态域如何分层
- `INS_Out.flag`、命令输入、时间条件如何影响转移
- `Command_Listener`、`Combo_Stick`、`Lost_Return`、`Vehicle` 各自负责什么
- `degrade`、`on_ground` 如何进入状态机

## Not In Scope

- `FMS_Out.u_cmd / p_cmd / throttle_cmd` 的生成细节
- `vtol_fms`

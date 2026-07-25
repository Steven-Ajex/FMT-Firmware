---
name: fmt-board-bootstrap-reader
description: FMT-Firmware 板级 bring-up 阅读技能。用于分析 target/sieon/s1 的 early init、bsp_initialize、bsp_post_initialize 三阶段，以及 FDCAN、RC、lwIP、ETH、板载传感器初始化与 target-local tasks 等板级扩展行为。
---

# Fmt Board Bootstrap Reader

## Overview

本技能聚焦 `target/sieon/s1/board/board.c` 及其 target-local 扩展。

## Read Order

1. `target/sieon/s1/board/board.c`
2. `target/sieon/s1/tasks/task_dual_imu_attitude.c`
3. `target/sieon/s1/tasks/task_bridge_mlog.c`
4. `target/sieon/s1/tasks/task_can_bridge.c`

## Focus

- early/init/post 三阶段职责
- 板级外设与网络 bring-up
- 传感器驱动初始化与注册
- target-local runtime extension

## Not In Scope

- `TaskTab` 启动机制
- `uMCN` 语义
- `FMS / Controller / INS` 内部算法


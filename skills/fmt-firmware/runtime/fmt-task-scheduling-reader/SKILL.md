---
name: fmt-task-scheduling-reader
description: FMT-Firmware 任务调度阅读技能。用于分析 startup、TaskTab、task_manager 与 vehicle 主循环如何配合启动和驱动系统，适合回答任务注册、依赖初始化、auto_start、周期执行和控制主循环顺序的问题。
---

# Fmt Task Scheduling Reader

## Overview

本技能解释 FMT 自己的 task manager 如何叠加在 RTOS 之上。

## Read Order

1. `src/startup.c`
2. `src/module/task_manager/task_manager.h`
3. `src/module/task_manager/task_manager.c`
4. `src/task/vehicle/normal/task_vehicle.c`

## Focus

- `TASK_EXPORT` 如何落入 `TaskTab`
- `task_manager_init/start` 如何处理依赖和自启动
- `task_vehicle` 如何形成 `sensor -> command -> INS -> FMS -> Controller -> actuator` 主循环

## Not In Scope

- `uMCN` 结构细节
- 各模型内部状态机


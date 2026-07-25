---
name: fmt-logging-pipeline-reader
description: FMT-Firmware 日志链路阅读技能。用于分析 mlog、ulog、logger task、MlogTab 和参数快照的关系，适合回答结构化日志 bus schema、自动录制策略和日志系统初始化流程的问题。
---

# Fmt Logging Pipeline Reader

## Overview

本技能解释 FMT 的结构化日志链路，不做飞行日志性能归因。

## Read Order

1. `src/module/log/mlog.h`
2. `src/module/log/mlog.c`
3. `src/task/logger/task_logger.c`
4. `src/module/param/sys_param.c`

## Focus

- `MLOG_BUS_DEFINE` 与 `MlogTab`
- `mlog_init` 如何扫描 bus 和参数
- `logger task` 如何异步刷写
- `SYSTEM.MLOG_MODE` 如何影响录制

## Not In Scope

- 日志文件离线解码
- 飞行后分析报告


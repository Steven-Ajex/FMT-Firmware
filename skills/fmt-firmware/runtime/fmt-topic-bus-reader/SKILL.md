---
name: fmt-topic-bus-reader
description: FMT-Firmware topic 总线阅读技能。用于分析 uMCN 的 topic、publish、subscribe、copy、renewal、event、callback 语义，以及模块之间如何通过 topic 而不是直接函数调用通信。
---

# Fmt Topic Bus Reader

## Overview

本技能聚焦 `uMCN` 的运行语义。

## Read Order

1. `src/module/ipc/uMCN.h`
2. `src/module/ipc/uMCN.c`

## Focus

- `McnHub` 内部结构
- `mcn_publish` 的数据写入和事件唤醒
- `mcn_copy` 与 `mcn_copy_from_hub` 的区别

## Not In Scope

- 具体 topic 字段业务含义
- 日志刷盘策略


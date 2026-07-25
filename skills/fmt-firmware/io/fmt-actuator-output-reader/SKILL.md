---
name: fmt-actuator-output-reader
description: FMT-Firmware 执行输出阅读技能。用于分析 control_output 如何通过 actuator_cmd 和 sysconfig.toml 的 from、to 映射落到具体执行机构，适合回答输出通道绑定、HIL 与实机路径差异以及执行器出口配置问题。
---

# Fmt Actuator Output Reader

## Overview

本技能解释 `control_output` 如何映射到执行器，不分析底层寄存器驱动。

## Read Order

1. `src/module/sysio/actuator_cmd.c`
2. `target/sieon/s1/config/sysconfig.toml`

## Focus

- `control_output` 与 `rc_trim_channels` 的来源
- `from / to` 映射关系
- target 配置如何决定执行器出口

## Not In Scope

- 电调或 PWM 设备底层实现
- 控制器内部控制律


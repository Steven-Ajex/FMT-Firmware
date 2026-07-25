---
name: fmt-controller-interface-reader
description: FMT-Firmware Controller 接口阅读技能。用于分析 control_interface.c 如何把 fms_output 和 ins_output 组装成 Controller_U、调用 Controller_step 并发布 control_output，适合回答 Controller 的输入 topic、输出 topic、参数绑定和周期问题。
---

# Fmt Controller Interface Reader

## Overview

默认按 `target/sieon/s1 + mc_controller` 路径阅读。

## Read Order

1. `src/model/control/mc_controller/lib/Controller.h`
2. `src/model/control/mc_controller/control_interface.c`

## Focus

- `Controller_U` 的输入来源
- `Controller_step()` 的调用边界
- `control_output` 的发布与 mlog 记录
- `CONTROL_EXPORT.period` 与参数绑定

## Not In Scope

- `Controller.c` 内部环路结构
- `vtol_controller` 变体


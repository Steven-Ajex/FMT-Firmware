---
name: fmt-ins-interface-reader
description: FMT-Firmware INS 接口阅读技能。用于分析 ins_interface.c 如何把 sensor 和外部位置相关 topics 组装为 INS_U、调用 INS_step 并发布 ins_output，适合回答 INS 订阅什么、输出什么、周期是什么以及日志如何绑定的问题。
---

# Fmt Ins Interface Reader

## Overview

默认按 `target/sieon/s1 + Multicopter + 非 HIL/SIH + cf_ins` 路径阅读。

## Read Order

1. `src/model/ins/cf_ins/lib/INS.h`
2. `src/model/ins/cf_ins/ins_interface.c`

## Focus

- `INS_U` 由哪些 topics 组装而成
- `INS_step()` 在接口层如何被调用
- `ins_output` 与 mlog buses 如何发布
- `INS_EXPORT.period` 与 `model_info` 如何暴露

## Not In Scope

- `INS.c` 内部导航解算算法
- `px4_ecl` 或其他 INS 变体


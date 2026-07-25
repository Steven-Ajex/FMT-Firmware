---
name: fmt-ins-cf-navigation-reader
description: FMT-Firmware 默认 cf_ins 导航解算阅读技能。用于细致分析 INS.c 中的导航数据解算过程，包括磁场处理、观测质量门控、GPS 和空速延迟链路以及 INS_Out 的生成，适合回答 INS 中速度、位置、地理坐标和空速如何被解算的问题。
---

# Fmt Ins Cf Navigation Reader

## Overview

本技能只分析默认 `cf_ins`，不混入 `px4_ecl`。

当用户需要默认 `s1 + Multicopter + cf_ins` 路径下的详细代码锚点、阅读顺序和边界说明时，读取 `references/cf-ins-navigation-map.md`。

## Preconditions

先确认：

- `target/sieon/s1/config/model.py` 选中的 INS 是 `ins/cf_ins`
- 已理解 `$fmt-sensor-pipeline-reader`
- 已理解 `$fmt-ins-interface-reader`

## Read Order

1. `src/model/ins/cf_ins/lib/INS.c` 的 `INS_step()` 主入口
2. 磁场与 WMM 相关段
3. 质量门控与 timeout 段
4. GPS delay 与 airspeed delay 段
5. `INS_Out` 赋值段

## Focus

- 主要观测是如何进入解算链的
- 磁力计质量和时效性如何控制
- GPS 和空速为什么要走 delay line
- 最终哪些导航量被写入 `INS_Out`

## Not In Scope

- `ins_interface.c` 的 topic 组装
- `px4_ecl` 的 EKF 结构

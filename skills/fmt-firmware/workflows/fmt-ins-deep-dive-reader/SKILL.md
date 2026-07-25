---
name: fmt-ins-deep-dive-reader
description: FMT-Firmware INS 深挖编排技能。用于需要细致分析 INS 的传感器输入、interface 边界以及默认 cf_ins 导航数据解算算法时使用，适合回答 INS 需要哪些观测、如何做质量门控、延迟处理以及如何形成导航输出。
---

# Fmt Ins Deep Dive Reader

## Overview

默认按 `target/sieon/s1 + Multicopter + 非 HIL/SIH + cf_ins` 路径分析。

当用户给出的是一个端到端 INS 深挖问题，而不是单一局部问题时，读取 `references/ins-deep-dive-playbook.md` 作为编排手册。

## Sequence

1. `$fmt-sensor-pipeline-reader`
2. `$fmt-ins-interface-reader`
3. `$fmt-ins-cf-navigation-reader`

## Boundary

- 本技能默认不分析 `px4_ecl`。
- 若实际 target 选择 `ins/px4_ecl`，停止并提示需要单独的变体 skill。

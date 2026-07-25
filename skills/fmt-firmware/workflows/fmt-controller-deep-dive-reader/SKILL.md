---
name: fmt-controller-deep-dive-reader
description: FMT-Firmware Controller 深挖编排技能。用于需要细致分析 mc_controller 的控制器结构、命令生成路径、积分器与限幅逻辑以及 actuator 输出骨架时使用，适合回答 FMS 输出如何进入控制器并变成执行命令。
---

# Fmt Controller Deep Dive Reader

## Overview

默认按 `target/sieon/s1 + mc_controller` 路径分析。

当用户需要端到端分析 `FMS_Out -> Controller -> control_output -> actuator`，而不是只看 Controller 的局部函数时，读取 `references/controller-deep-dive-playbook.md` 作为编排手册。

## Sequence

1. `$fmt-fms-interface-reader`
2. `$fmt-controller-interface-reader`
3. `$fmt-controller-structure-reader`
4. `$fmt-actuator-output-reader`

## Boundary

- 本技能默认不分析 `vtol_controller`。
- 重点放在控制器结构和命令路径，不讨论底层寄存器输出。

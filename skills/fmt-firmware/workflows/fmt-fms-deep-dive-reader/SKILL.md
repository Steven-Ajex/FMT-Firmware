---
name: fmt-fms-deep-dive-reader
description: FMT-Firmware FMS 深挖编排技能。用于需要细致分析 FMS 的输入来源、状态机转换、模式切换条件以及控制指令生成路径时使用，尤其适合回答 arm、auto、mission、return、manual 等模式如何转移和产生命令。
---

# Fmt Fms Deep Dive Reader

## Overview

默认按 `target/sieon/s1 + mc_fms` 路径分析。

当用户需要端到端分析 FMS 的输入来源、状态机和控制命令生成，而不是单问其中一段时，读取 `references/fms-deep-dive-playbook.md` 作为编排手册。

## Sequence

1. `$fmt-command-ingress-reader`
2. `$fmt-fms-interface-reader`
3. `$fmt-fms-state-machine-reader`
4. `$fmt-fms-command-generation-reader`

## Boundary

- 本技能默认不分析 `vtol_fms`。
- 若 `vehicle` 为 `VTOL`，先标明当前 skill 只覆盖 `mc_fms` 默认路径。

---
name: fmt-system-topdown-reader
description: FMT-Firmware 自顶向下总览编排技能。用于用户想系统建立对 FMT 固件的正确认识时，从 target 装配、板级 bring-up、任务调度、topic 总线、传感器链路一路走到 INS、FMS、Controller 与 actuator，形成完整心智模型。
---

# Fmt System Topdown Reader

## Overview

当目标是“先看清整机架构，再看局部模块”时使用本技能。

当用户要的是系统级总览、阅读顺序和最终总图，而不是某一局部 deep dive 时，读取 `references/system-topdown-playbook.md`。

## Sequence

1. `$fmt-build-assembly-reader`
2. `$fmt-board-bootstrap-reader`
3. `$fmt-task-scheduling-reader`
4. `$fmt-topic-bus-reader`
5. `$fmt-sensor-pipeline-reader`
6. `$fmt-command-ingress-reader`
7. `$fmt-ins-interface-reader`
8. `$fmt-fms-interface-reader`
9. `$fmt-controller-interface-reader`
10. `$fmt-actuator-output-reader`

## Output

输出应形成一条完整骨架：

- `target` 如何装配
- 系统如何启动
- `task_vehicle` 如何驱动控制主链
- 指令和传感器如何进入模型
- `INS -> FMS -> Controller -> actuator` 如何串起来

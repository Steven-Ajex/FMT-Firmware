---
name: fmt-control-stack-reader
description: FMT-Firmware 控制主链编排技能。用于用户只关心从传感器和指令入口到 INS、FMS、Controller、actuator 的端到端数据流，而暂时不需要深入生成模型内部状态机和控制律时使用。
---

# Fmt Control Stack Reader

## Overview

本技能用于快速拉通控制主链，但默认停在接口层。

## Sequence

1. `$fmt-command-ingress-reader`
2. `$fmt-sensor-pipeline-reader`
3. `$fmt-ins-interface-reader`
4. `$fmt-fms-interface-reader`
5. `$fmt-controller-interface-reader`
6. `$fmt-actuator-output-reader`

## Output

输出应回答：

- 指令从哪里进入
- 传感器从哪里进入
- 三个模型接口分别吃什么、吐什么
- 执行器最终怎么收到命令


---
name: fmt-runtime-infrastructure-reader
description: FMT-Firmware 运行时基础设施编排技能。用于用户主要关心任务调度、topic 总线、日志系统等 shared runtime 机制，而不是控制模型内部状态机或算法时使用。
---

# Fmt Runtime Infrastructure Reader

## Overview

本技能只看 shared runtime，不钻 `FMS.c`、`Controller.c`、`INS.c` 内部算法。

当用户要的是 `TaskTab / uMCN / mlog` 这一层的系统级阅读顺序与汇总输出，而不是单个局部机制时，读取 `references/runtime-infrastructure-playbook.md`。

## Sequence

1. `$fmt-task-scheduling-reader`
2. `$fmt-topic-bus-reader`
3. `$fmt-logging-pipeline-reader`

## Output

输出应说明：

- 任务如何注册、启动、按周期运行
- topic 如何发布、订阅、拷贝、唤醒
- 结构化日志如何落地

---
name: fmt-command-ingress-reader
description: FMT-Firmware 指令入口阅读技能。用于分析 pilot_cmd、gcs_cmd、auto_cmd、mission_data 如何进入系统并最终组装成 FMS_U，适合回答 RC、GCS、mission 和自动指令入口、映射与汇聚问题。
---

# Fmt Command Ingress Reader

## Overview

本技能解释控制指令从外部入口进入 FMS 之前的路径。

## Read Order

1. `target/sieon/s1/board/board.c`
2. `src/module/sysio/pilot_cmd.c`
3. `src/model/fms/mc_fms/fms_interface.c`

## Focus

- `pilot_cmd / gcs_cmd / auto_cmd / mission_data` 的初始化和采集点
- RC 与 GCS 指令如何进入 `FMS_U`
- mission 数据如何成为状态机输入

## Not In Scope

- `FMS.c` 内部 chart 细节
- `Controller` 对命令的消费方式


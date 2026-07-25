---
name: fmt-fms-interface-reader
description: FMT-Firmware FMS 接口阅读技能。用于分析 fms_interface.c 如何把 pilot_cmd、gcs_cmd、auto_cmd、mission_data、ins_output、control_output 组装成 FMS_U、调用 FMS_step 并发布 fms_output，适合回答 FMS 输入输出边界与周期问题。
---

# Fmt Fms Interface Reader

## Overview

默认按 `target/sieon/s1 + mc_fms` 路径阅读。

## Read Order

1. `src/model/fms/mc_fms/lib/FMS.h`
2. `src/model/fms/mc_fms/fms_interface.c`

## Focus

- `FMS_U` 的输入来源
- `FMS_step()` 的调用边界
- `fms_output` 的发布与日志记录
- `FMS_EXPORT.period` 与 `model_info`

## Not In Scope

- `FMS.c` 内部状态机 chart
- 各模式下控制命令的数值生成细节


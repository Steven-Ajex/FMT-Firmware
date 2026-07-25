---
name: fmt-sensor-pipeline-reader
description: FMT-Firmware 传感器链路阅读技能。用于分析板级驱动初始化、register_sensor、sensor_collect 以及 sensor topics 的完整数据路径，适合回答传感器驱动如何接入系统以及采样、校准、滤波、发布过程的问题。
---

# Fmt Sensor Pipeline Reader

## Overview

本技能解释传感器数据如何从板级驱动流入 topic。

## Read Order

1. `target/sieon/s1/board/board.c`
2. `src/module/sensor/sensor_hub.c`

## Focus

- `register_sensor_*` 如何连接设备与 sensor hub
- `sensor_collect` 如何做采样、旋转补偿、校准、滤波、发布
- 默认 `s1` 上 IMU、Mag、Baro、GPS、空速计的注册方式

## Not In Scope

- `INS.c` 内部导航融合算法


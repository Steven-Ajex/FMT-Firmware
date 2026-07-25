---
name: fmt-controller-structure-reader
description: FMT-Firmware mc_controller 结构阅读技能。用于细致分析 Controller.c 中的控制器层级、命令生成路径、姿态和速率环、积分器、微分、前馈、限幅以及 actuator_cmd 输出骨架，适合回答 Controller 结构和控制指令如何变成执行输出的问题。
---

# Fmt Controller Structure Reader

## Overview

本技能只分析默认 `mc_controller`。

当用户需要默认 `s1 + mc_controller` 路径下的环路结构、命令传递链和关键代码锚点时，读取 `references/mc-controller-structure-map.md`。

## Preconditions

先确认：

- 已理解 `$fmt-fms-interface-reader`
- 已理解 `$fmt-controller-interface-reader`

## Read Order

1. `Controller_step()` 入口和模式判断段
2. 速度和姿态误差相关段
3. `rate_cmd_B_radPs` 生成段
4. 速率环积分、微分、前馈和限幅段
5. `Control_Out.actuator_cmd` 赋值段

## Focus

- 控制器是几层环路
- `FMS_Out` 如何分模式变成姿态、速率和推力命令
- 积分器 reset、限幅、前馈在何处发生
- 最终 `actuator_cmd` 如何形成

## Not In Scope

- `control_interface.c` 的 topic 组装
- 底层 PWM 或电机驱动寄存器
- `vtol_controller`

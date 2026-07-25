# Controller Deep Dive Playbook

## Use this playbook when

- The user asks a broad controller question, such as:
- “Controller 怎么从 FMS 输出一路变成执行命令？”
- “Controller 的结构、环路、限幅和 actuator 输出怎么串起来？”
- “控制器内部命令路径和执行机构映射怎么配合？”

## Default path

- Representative target: `target/sieon/s1`
- Model path: `Multicopter + 非 HIL/SIH + control/mc_controller`
- If actual controller path is `vtol_controller`, stop and say this workflow only covers `mc_controller`.

## Workflow intent

This workflow combines:

1. upstream command boundary from FMS
2. controller firmware/model boundary
3. controller internal loop structure
4. downstream actuator mapping

Keep controller internals separate from actuator mapping. The controller produces `control_output`; actuator mapping is a later stage.

## Execution order

1. Run `$fmt-fms-interface-reader`
2. Run `$fmt-controller-interface-reader`
3. Run `$fmt-controller-structure-reader`
4. Run `$fmt-actuator-output-reader`

## What each child skill contributes

### `$fmt-fms-interface-reader`

- Explains the upstream command source that becomes part of `Controller_U`
- Use it when the user asks what the controller consumes from FMS

### `$fmt-controller-interface-reader`

- Explains topic boundary, period, and model invocation
- Use it to define where firmware ends and generated control law begins

### `$fmt-controller-structure-reader`

- Explains loop hierarchy, rate-command generation, rate loop, limiters, and `Control_Out.actuator_cmd`
- For exact structural anchors, load:
- `fmt-controller-structure-reader/references/mc-controller-structure-map.md`

### `$fmt-actuator-output-reader`

- Explains mapping from `control_output` to configured actuators
- Use it to avoid conflating controller law with hardware-output mapping

## Recommended output structure

1. Assumptions
- current `target / vehicle / sim / controller variant`

2. Upstream boundary
- which `FMS_Out` fields matter

3. Controller boundary
- `Controller_U`
- period
- `control_output`

4. Controller internals
- loop hierarchy
- direct vs indirect rate-command paths
- integrators, feedforward, saturation

5. Downstream mapping
- how `control_output` reaches actuators

6. Residual boundaries
- what still belongs to FMS or hardware drivers

## Typical question routing

- “Controller 是几层环路？”:
- prioritize the internal structure stage
- “为什么 FMS 的命令最后变成这些 actuator 值？”:
- explain upstream boundary, then internal structure, then actuator mapping
- “限幅和积分器在哪？”:
- go directly to the internal structure stage

## Stop conditions

- If the active controller is `vtol_controller`, stop after stating the mismatch.
- If the user asks why FMS chose a specific command family, hand off to `$fmt-fms-deep-dive-reader`.


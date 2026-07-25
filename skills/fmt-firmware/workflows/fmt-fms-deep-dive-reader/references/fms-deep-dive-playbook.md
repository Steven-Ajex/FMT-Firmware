# FMS Deep Dive Playbook

## Use this playbook when

- The user asks a broad FMS question, such as:
- “FMS 里的状态机和控制指令生成怎么串起来？”
- “FMS 从 RC/mission 输入到输出控制命令的完整链路是什么？”
- “FMS 模式切换和 setpoint 生成如何配合？”

## Default path

- Representative target: `target/sieon/s1`
- Model path: `Multicopter + 非 HIL/SIH + fms/mc_fms`
- If actual vehicle/model path is `VTOL` or `vtol_fms`, stop and say this workflow only covers `mc_fms`.

## Workflow intent

This workflow must keep three layers separate:

1. command ingress before the model
2. interface boundary around `FMS_U` and `fms_output`
3. generated-model internals:
- state machine
- command generation

Do not merge state transition logic and command-generation logic into a single explanation block.

## Execution order

1. Run `$fmt-command-ingress-reader`
2. Run `$fmt-fms-interface-reader`
3. Run `$fmt-fms-state-machine-reader`
4. Run `$fmt-fms-command-generation-reader`

## What each child skill contributes

### `$fmt-command-ingress-reader`

- Explains RC, GCS, auto, and mission ingress
- Use it to answer:
- 哪些外部输入真正进入 `FMS_U`

### `$fmt-fms-interface-reader`

- Explains the firmware-side model boundary
- Use it to answer:
- `FMS_U` 来源
- `fms_output` 去向

### `$fmt-fms-state-machine-reader`

- Explains chart structure and transition conditions
- Use it to answer:
- why a mode is active
- why a transition occurs

For exact state-machine anchors, load:

- `fmt-fms-state-machine-reader/references/mc-fms-state-machine-map.md`

### `$fmt-fms-command-generation-reader`

- Explains `FMS_Out` field generation
- Use it to answer:
- which command family is emitted in each mode
- how mission and auto setpoints turn into output commands

For exact command-generation anchors, load:

- `fmt-fms-command-generation-reader/references/mc-fms-command-map.md`

## Recommended output structure

1. Assumptions
- current `target / vehicle / sim / FMS variant`

2. Input ingress
- RC, GCS, auto, mission

3. Interface boundary
- what becomes `FMS_U`
- what leaves as `fms_output`

4. State machine
- main state domains
- transition conditions

5. Command generation
- `ctrl_mode`
- direct rate / attitude / translational command families

6. Boundaries and next hop
- what the controller will consume next

## Typical question routing

- “FMS 为什么切到 Return 或 Land？”:
- prioritize state machine first, then mission ingress
- “FMS 为什么输出 u/v/w 而不是 p/q/r？”:
- prioritize command generation after a short state recap
- “FMS 从任务点怎么变成控制命令？”:
- walk mission ingress -> mission state -> command generation

## Stop conditions

- If the active model is `vtol_fms`, stop after stating the mismatch.
- If the user asks controller internals, hand off to `$fmt-controller-deep-dive-reader`.


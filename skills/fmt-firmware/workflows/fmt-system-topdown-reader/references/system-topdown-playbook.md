# System Topdown Playbook

## Use this playbook when

- The user wants a whole-system understanding before local details
- The user is new to FMT-Firmware
- The user wants a report that starts at target assembly and ends at the control chain

## Default path

- Representative target: `target/sieon/s1`
- Model path:
- `Multicopter`
- `非 HIL/SIH`
- `ins/cf_ins + fms/mc_fms + control/mc_controller`

## Workflow intent

This workflow should build the mental model in layers:

1. how the target is assembled
2. how the board and runtime start
3. how the main loop is scheduled
4. how data enters the control stack
5. how model interfaces connect the chain
6. how outputs leave the chain

Avoid jumping directly into generated-model internals unless the user explicitly asks.

## Execution order and purpose

### `$fmt-build-assembly-reader`

- Purpose:
- establish `target / vehicle / sim / model` assumptions
- explain `TaskTab / ParamTab / MlogTab` assembly context

### `$fmt-board-bootstrap-reader`

- Purpose:
- explain `s1` board bring-up, post init, and target-local extensions

### `$fmt-task-scheduling-reader`

- Purpose:
- explain startup, `task_manager`, and the `task_vehicle` control loop

### `$fmt-topic-bus-reader`

- Purpose:
- explain how modules communicate through `uMCN`

### `$fmt-sensor-pipeline-reader`

- Purpose:
- explain how sensors become topics before entering INS

### `$fmt-command-ingress-reader`

- Purpose:
- explain how RC, GCS, auto, and mission inputs become `FMS_U`

### `$fmt-ins-interface-reader`

- Purpose:
- explain the INS firmware boundary without yet diving into solver internals

### `$fmt-fms-interface-reader`

- Purpose:
- explain the FMS firmware boundary without yet diving into state machine internals

### `$fmt-controller-interface-reader`

- Purpose:
- explain the controller firmware boundary without yet diving into loop internals

### `$fmt-actuator-output-reader`

- Purpose:
- explain how `control_output` is mapped to configured actuators

## Recommended output structure

1. assumptions
- current `target / vehicle / sim / model path`

2. assembly layer
- how the target selects modules and tasks

3. runtime layer
- startup, task registration, `task_vehicle`, topic bus, logging context

4. control-chain overview
- sensor ingress
- command ingress
- INS interface
- FMS interface
- controller interface
- actuator egress

5. system picture
- a concise end-to-end chain from target assembly to actuator output

6. optional next dives
- where to go next for FMS, Controller, or INS internals

## Handoff rules

- If the user asks for generated-model internals after the overview, hand off to:
- `$fmt-ins-deep-dive-reader`
- `$fmt-fms-deep-dive-reader`
- `$fmt-controller-deep-dive-reader`

## Stop conditions

- If the user only wants a local answer about `TaskTab`, `uMCN`, or one model boundary, stop using this workflow and route directly to the narrower skill.


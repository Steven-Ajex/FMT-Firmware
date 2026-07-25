# Architect Routing Playbook

## Use this playbook when

- The user says “帮我理解 FMT”
- The user gives a large question that spans multiple layers
- The user does not name a specific `fmt-*` skill
- The user asks for a combined explanation instead of a single local answer

## Default assumptions

- Representative target: `target/sieon/s1`
- Default model path:
- `Multicopter`
- `非 HIL/SIH`
- `ins/cf_ins + fms/mc_fms + control/mc_controller`

Always state these assumptions before going deep. If the real code path differs, say so and reroute.

## Routing decision tree

### Broad system understanding

- Trigger:
- “先帮我整体理解这个飞控”
- “FMT 的整体架构是什么”
- Route to:
- `$fmt-system-topdown-reader`

### Shared runtime mechanisms

- Trigger:
- `TaskTab`
- `task_manager`
- `uMCN`
- `mlog`
- `logger`
- Route to:
- `$fmt-runtime-infrastructure-reader`
- or direct runtime atomic skills when the question is narrow

### Control-stack overview without deep internals

- Trigger:
- “从传感器到执行器怎么串起来”
- “指令和传感器怎么进控制链”
- Route to:
- `$fmt-control-stack-reader`

### INS deep dive

- Trigger:
- “INS 怎么解算”
- “INS 为什么有 delay / quality gate”
- Route to:
- `$fmt-ins-deep-dive-reader`

### FMS deep dive

- Trigger:
- “FMS 状态机怎么转”
- “FMS 怎么生成控制命令”
- Route to:
- `$fmt-fms-deep-dive-reader`

### Controller deep dive

- Trigger:
- “Controller 的结构是什么”
- “FMS 输出怎么变成 actuator 命令”
- Route to:
- `$fmt-controller-deep-dive-reader`

## Atomic fallback routing

- build and assembly only:
- `$fmt-build-assembly-reader`
- board bring-up only:
- `$fmt-board-bootstrap-reader`
- scheduling only:
- `$fmt-task-scheduling-reader`
- topic bus only:
- `$fmt-topic-bus-reader`
- logging only:
- `$fmt-logging-pipeline-reader`
- sensor ingress only:
- `$fmt-sensor-pipeline-reader`
- command ingress only:
- `$fmt-command-ingress-reader`
- actuator mapping only:
- `$fmt-actuator-output-reader`

## Output contract

When acting as the top-level router, always return:

1. assumptions
2. selected skills and why
3. ordered execution path
4. consolidated conclusion
5. remaining deep-dive options

## Stop conditions

- If the user already named a specific `fmt-*` skill, stop rerouting and use it directly.
- If the user asks for offline log decoding, tuning advice, or runtime behavior that needs evidence beyond source code, say that this repository skill set is focused on source-code understanding.


# Runtime Infrastructure Playbook

## Use this playbook when

- The user asks a broad runtime-mechanism question, such as:
- “FMT 的任务、消息和日志系统怎么协作？”
- “TaskTab、uMCN、mlog 是怎么串起来的？”
- “运行时骨架是什么，不看控制算法内部”

## Workflow intent

This workflow should explain the shared runtime skeleton in three layers:

1. task registration and execution order
2. topic-based communication semantics
3. structured logging and persistence path

Keep those three layers separate in the explanation. Do not blur scheduling, IPC, and logging into one generic “framework”.

## Execution order

1. Run `$fmt-task-scheduling-reader`
2. Run `$fmt-topic-bus-reader`
3. Run `$fmt-logging-pipeline-reader`

## What each child skill contributes

### `$fmt-task-scheduling-reader`

- Explains `startup -> task_manager -> TaskTab -> auto_start -> task_vehicle`
- Use it to answer:
- how runtime execution begins
- where the control loop sits inside the scheduler

### `$fmt-topic-bus-reader`

- Explains `uMCN` topic semantics
- Use it to answer:
- how modules exchange data without direct function coupling
- what `publish / subscribe / renewal / event / callback` mean

### `$fmt-logging-pipeline-reader`

- Explains `MlogTab`, `mlog_init`, `logger task`, and `SYSTEM.MLOG_MODE`
- Use it to answer:
- how structured logs are registered
- how they are collected and written

## Recommended output structure

1. Assumptions
- current `target / vehicle / sim`
- whether the question stays in shared runtime scope

2. Scheduling layer
- static registration
- init order
- auto-start behavior
- where `task_vehicle` fits

3. IPC layer
- topic hubs
- publish/copy semantics
- event-driven wakeups

4. Logging layer
- bus registration
- parameter snapshot
- logger task flush path

5. Runtime synthesis
- how scheduling, IPC, and logging interact in one system picture

6. Boundaries
- what belongs to model interfaces
- what belongs to generated-model internals

## Typical question mapping

- “TaskTab 怎么让任务跑起来？”:
- prioritize scheduling layer
- “uMCN 和普通队列有什么区别？”:
- prioritize IPC layer
- “mlog 为什么知道 bus schema 和参数？”:
- prioritize logging layer
- “vehicle 主循环和 topic、日志怎么协同？”:
- summarize all three layers in order

## Handoff rules

- If the user asks about sensor data before it enters INS, hand off to `$fmt-sensor-pipeline-reader`.
- If the user asks about `*_interface.c`, hand off to the corresponding interface reader.
- If the user asks about `FMS / Controller / INS` generated-model internals, stop this workflow and hand off to the corresponding deep-dive workflow.

## Stop conditions

- If the question is only about one narrow runtime mechanism, use the atomic runtime skill directly instead of the workflow.
- If the user asks for runtime performance conclusions that need traces or profiling, say this workflow only covers source-code structure.


# INS Deep Dive Playbook

## Use this playbook when

- The user asks an end-to-end INS question, such as:
- “INS 怎么从传感器一路解到导航输出？”
- “INS 的输入、边界和算法怎么串起来看？”
- “INS 为什么要做质量门控和延迟处理？”

## Default path

- Representative target: `target/sieon/s1`
- Model path: `Multicopter + 非 HIL/SIH + ins/cf_ins`
- If the actual model path is not `cf_ins`, stop and say that this workflow only covers the default path.

## Workflow intent

This workflow combines three distinct layers:

1. sensor ingress before the INS model
2. firmware/model interface boundary
3. `cf_ins` navigation algorithm internals

Keep those three layers separate in the explanation. Do not collapse them into a single undifferentiated “INS module”.

## Execution order

1. Run `$fmt-sensor-pipeline-reader`
2. Run `$fmt-ins-interface-reader`
3. Run `$fmt-ins-cf-navigation-reader`

## What each child skill contributes

### `$fmt-sensor-pipeline-reader`

- Explains `driver init -> register_sensor_* -> sensor_collect -> sensor topics`
- Use it to answer:
- INS 之前的数据是怎么来的
- 哪些 topic 对 INS 可见

### `$fmt-ins-interface-reader`

- Explains how topics are packed into `INS_U`
- Use it to answer:
- INS 的 firmware 边界是什么
- INS 发布什么、记录什么、周期是什么

### `$fmt-ins-cf-navigation-reader`

- Explains the generated algorithm internals for `cf_ins`
- Use it to answer:
- 为什么有 WMM、质量门控、GPS delay、airspeed delay
- 最终导航量怎样落到 `INS_Out`

For exact code anchors, load the child skill reference:

- `fmt-ins-cf-navigation-reader/references/cf-ins-navigation-map.md`

## Recommended output structure

1. Assumptions
- current `target / vehicle / sim / INS variant`

2. Sensor ingress
- what reaches INS before the model boundary

3. Interface boundary
- how `INS_U` is formed
- what `INS_Out` publishes

4. Navigation algorithm
- observation groups
- quality gating
- delay chains
- final navigation outputs

5. Residual unknowns
- anything that needs a variant-specific skill or runtime evidence

## Typical question routing

- “INS 需要哪些传感器？”:
- prioritize sensor pipeline, then interface
- “INS 为什么输出漂移/质量差？”:
- prioritize interface sanity, then `cf_ins` quality and delay logic
- “INS 里经纬度、局部坐标、空速是怎么出来的？”:
- go directly to the algorithm stage after a short boundary recap

## Stop conditions

- If actual code path is `px4_ecl`, stop after explaining the mismatch.
- If the user asks for offline log diagnosis or estimator tuning conclusions, stop and say this workflow only covers source-code understanding.


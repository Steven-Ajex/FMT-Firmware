# CF INS Navigation Map

## Default path

- Representative target: `target/sieon/s1`
- Default model path: `Multicopter + 非 HIL/SIH + ins/cf_ins`
- Source of truth for this assumption:
- `target/sieon/s1/config/model.py:7-21`
- `src/task/vehicle/normal/task_vehicle.c:67-86`

## Read this reference when

- You need exact code anchors for `cf_ins`
- You need a fixed reading order instead of free exploration
- You need to explain how sensor observations turn into `INS_Out`

## Recommended reading order

1. Confirm the default model selection in `target/sieon/s1/config/model.py:7-21`.
2. Confirm where `ins_interface_step()` sits in the vehicle loop in `src/task/vehicle/normal/task_vehicle.c:67-86`.
3. Read the firmware boundary in `src/model/ins/cf_ins/ins_interface.c:543-616`.
4. Enter the algorithm at `src/model/ins/cf_ins/lib/INS.c:1387-1460`.
5. Follow observation-specific branches:
   - magnetometer and WMM
   - magnetometer validity and quality
   - GPS delays
   - airspeed delays
6. End at the `INS_Out` assignment block in `src/model/ins/cf_ins/lib/INS.c:10280-10324`.

## Anchor map

### Interface handoff

- `src/model/ins/cf_ins/ins_interface.c:543-616`
- Purpose: shows which sensor and external-position topics are packed into `INS_U`
- Use this to separate firmware-side wiring from algorithm internals

### Algorithm entry

- `src/model/ins/cf_ins/lib/INS.c:1387-1460`
- Purpose: `INS_step()` entry and major working-state variables
- Use this to establish that `cf_ins` is a large generated navigation algorithm, not a thin wrapper

### Magnetometer and WMM path

- `src/model/ins/cf_ins/lib/INS.c:2363-2455`
- Purpose: convert magnetic measurements, compute lookup indices, query WMM tables
- Questions it answers:
- how magnetic measurements are normalized
- where geo-dependent magnetic reference enters the solver

### Magnetometer validity and quality gate

- `src/model/ins/cf_ins/lib/INS.c:2750-2838`
- Purpose: sensor range check, timeout check, quality integration
- Questions it answers:
- why mag is not always fused just because data exists
- where quality and timeout logic live

### GPS delay chain

- `src/model/ins/cf_ins/lib/INS.c:6953-7033`
- Purpose: maintain delayed GPS velocity observations
- Questions it answers:
- where GPS observations are buffered
- why delayed observation alignment exists in the generated model

### Airspeed delay and compensation

- `src/model/ins/cf_ins/lib/INS.c:9218-9351`
- Purpose: convert differential pressure to airspeed, apply delay, compensation, and history update
- Questions it answers:
- how airspeed enters the navigation estimate
- where airspeed timing alignment is handled

### Navigation output assembly

- `src/model/ins/cf_ins/lib/INS.c:10280-10324`
- Purpose: assign `vn / ve / vd / airspeed / lat / lon / alt / x_R / y_R / h_R / h_AGL`
- Use this block to close the explanation from observations to final navigation outputs

## Boundary with adjacent skills

- Use `$fmt-sensor-pipeline-reader` for driver-to-topic wiring before `INS_U`
- Use `$fmt-ins-interface-reader` for topic, period, and logging boundaries
- Do not use this skill to explain `px4_ecl`; that requires a separate variant skill

## Typical question mapping

- “INS 用了哪些观测量？”:
- start from `ins_interface.c:543-616`, then jump to the observation anchors in `INS.c`
- “INS 为什么会有延迟链？”:
- go directly to `INS.c:6953-7033` and `INS.c:9218-9351`
- “INS 最终输出哪些导航量？”:
- go directly to `INS.c:10280-10324`


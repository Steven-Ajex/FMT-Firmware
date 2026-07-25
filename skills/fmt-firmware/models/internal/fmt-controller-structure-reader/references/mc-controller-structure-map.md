# MC Controller Structure Map

## Default path

- Representative target: `target/sieon/s1`
- Default model path: `Multicopter + 非 HIL/SIH + control/mc_controller`
- Source of truth:
- `target/sieon/s1/config/model.py:7-21`
- `src/task/vehicle/normal/task_vehicle.c:67-86`

## Read this reference when

- You need a fixed reading order for `mc_controller`
- You need to explain loop structure instead of only topic boundaries
- You need to connect `FMS_Out` to `Control_Out.actuator_cmd`

## Recommended reading order

1. Read `src/model/control/mc_controller/control_interface.c:106-183` to confirm `Controller_U`.
2. Read `src/model/control/mc_controller/lib/Controller.c:162-344` to establish entry conditions, frame conversion, and reset handling.
3. Read `src/model/control/mc_controller/lib/Controller.c:431-505` for attitude-command saturation.
4. Read `src/model/control/mc_controller/lib/Controller.c:706-805` for `ctrl_mode -> rate_cmd_B_radPs` generation.
5. Read `src/model/control/mc_controller/lib/Controller.c:796-1050` for rate-loop integrator, derivative, feedforward, and limits.
6. Read `src/model/control/mc_controller/lib/Controller.c:3001-3163` for translational-command to actuator-output conversion.
7. Read `src/model/control/mc_controller/lib/Controller.c:3274-3441` for discrete-state updates and confirm timing assumptions.

## Anchor map

### Interface handoff

- `src/model/control/mc_controller/control_interface.c:106-183`
- Purpose: identify how `fms_output` and `ins_output` become `Controller_U`

### Entry, frame conversion, and reset

- `src/model/control/mc_controller/lib/Controller.c:162-344`
- Purpose: establish mode checks, navigation-to-body conversion, and reset-sensitive integrator initialization

### Attitude-command limiting

- `src/model/control/mc_controller/lib/Controller.c:431-505`
- Purpose: bound roll and pitch commands with `ROLL_PITCH_CMD_LIM`
- Use this when the question is about outer-loop saturation

### `ctrl_mode` to rate-command generation

- `src/model/control/mc_controller/lib/Controller.c:706-805`
- Purpose: select between direct `p/q/r` commands and indirectly generated rate commands
- Questions it answers:
- when `FMS_Out.p/q/r_cmd` pass straight through
- when attitude or coordinated terms generate `rate_cmd_B_radPs`

### Rate-loop core

- `src/model/control/mc_controller/lib/Controller.c:796-1050`
- Purpose: compute rate error, integrator state, derivative term, feedforward, and bounded outputs
- Questions it answers:
- where `RATE_I_MAX` applies
- where rate-loop dynamics live

### Translational and vertical command path

- `src/model/control/mc_controller/lib/Controller.c:3001-3163`
- Purpose: use `u_cmd / v_cmd / w_cmd` and internal states to form final `Control_Out.actuator_cmd`
- Use this when the user asks how the controller reaches actuator outputs

### Discrete update section

- `src/model/control/mc_controller/lib/Controller.c:3274-3441`
- Purpose: show explicit fixed-period state updates
- Use this to confirm the controller is implemented as a discrete-time design

## Boundary with adjacent skills

- Use `$fmt-controller-interface-reader` for topic boundaries and model period
- Use `$fmt-fms-command-generation-reader` to understand the meaning of upstream `FMS_Out` fields
- Use `$fmt-actuator-output-reader` for post-controller mapping from `control_output` to configured actuators

## Typical question mapping

- “Controller 有几层环路？”:
- read entry block, attitude limiting block, rate-command generation block, then rate-loop core
- “FMS 的命令怎么变成执行量？”:
- start at `706-805`, then `796-1050`, then `3001-3163`
- “Controller 的积分器和限幅在哪里？”:
- read `162-344`, `431-505`, `796-1050`, and `3274-3441`


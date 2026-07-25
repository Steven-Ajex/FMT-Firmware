# MC FMS Command Map

## Default path

- Representative target: `target/sieon/s1`
- Default model path: `Multicopter + 非 HIL/SIH + fms/mc_fms`
- Source of truth:
- `target/sieon/s1/config/model.py:7-21`
- `src/task/vehicle/normal/task_vehicle.c:67-86`

## Read this reference when

- You need to explain how `FMS_Out` fields are produced
- You need a stable entry order through `mc_fms`
- You want to map `ctrl_mode` to output field families

## Recommended reading order

1. Confirm `FMS_U` in `src/model/fms/mc_fms/fms_interface.c:292-363`.
2. If state logic matters, skim `src/model/fms/mc_fms/lib/FMS.c:2484-2741` first to understand `Cmd_In`.
3. Read `src/model/fms/mc_fms/lib/FMS.c:4998-5046` for one velocity-command output path.
4. Read `src/model/fms/mc_fms/lib/FMS.c:8880-8976` for auto-command and masked direct-command output paths.
5. Read `src/model/fms/mc_fms/lib/FMS.c:10750-10783` for pilot direct `p/q/r/throttle` generation.
6. Read `src/model/fms/mc_fms/lib/FMS.c:12957-13019` for another velocity-command output branch with limit handling.

## Anchor map

### Mission setpoint preparation

- `src/model/fms/mc_fms/lib/FMS.c:2484-2660`
- Purpose: update `Cmd_In.sp_waypoint`, `cur_waypoint`, and `set_speed`
- Use this block when explaining where later command-generation branches get their setpoints

### Velocity command branch

- `src/model/fms/mc_fms/lib/FMS.c:4998-5046`
- Purpose: write `status`, `state`, `ctrl_mode`, `u_cmd`, `v_cmd`, `w_cmd`
- Questions it answers:
- where one `ctrl_mode` branch emits translational commands
- how saturation is applied before writing `FMS_Out`

### Auto command with masks

- `src/model/fms/mc_fms/lib/FMS.c:8880-8976`
- Purpose: forward `Auto_Cmd` fields and apply limits to `u/v/w_cmd`
- Questions it answers:
- how `cmd_mask` interacts with generated outputs
- how direct auto commands and bounded velocity commands coexist

### Pilot direct acro branch

- `src/model/fms/mc_fms/lib/FMS.c:10750-10783`
- Purpose: generate `p_cmd`, `q_cmd`, `r_cmd`, `throttle_cmd` from sticks
- Use this when the question is about manual-rate command generation

### Additional bounded velocity branch

- `src/model/fms/mc_fms/lib/FMS.c:12957-13019`
- Purpose: another `u/v/w_cmd` generation path with explicit `VEL_XY_LIM` and `VEL_Z_LIM`
- Use this to compare how different modes still converge on the same output bus fields

## Practical output grouping

- Direct rate and throttle family:
- `p_cmd`, `q_cmd`, `r_cmd`, `throttle_cmd`
- Attitude family:
- `phi_cmd`, `theta_cmd`, `psi_rate_cmd`
- Translational family:
- `u_cmd`, `v_cmd`, `w_cmd`

## Boundary with adjacent skills

- Use `$fmt-fms-state-machine-reader` first if the user asks why a mode branch is active
- Use `$fmt-controller-structure-reader` next if the user asks how these commands are consumed

## Typical question mapping

- “FMS 在这个模式下输出的是速度指令还是角速度指令？”:
- compare the `FMS_Out` assignment branches in `4998-5046`, `8880-8976`, and `10750-10783`
- “Mission waypoint 怎么变成控制指令？”:
- start from `2484-2660`, then move to the later `FMS_Out` assignment blocks


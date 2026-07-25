# MC FMS State Machine Map

## Default path

- Representative target: `target/sieon/s1`
- Default model path: `Multicopter + 非 HIL/SIH + fms/mc_fms`
- Source of truth:
- `target/sieon/s1/config/model.py:7-21`
- `src/task/vehicle/normal/task_vehicle.c:67-86`

## Read this reference when

- You need an exact entry order for `mc_fms` state-machine analysis
- You want to explain `arm / auto / mission / return / lost link` transitions
- You want to separate state transitions from output command generation

## Recommended reading order

1. Confirm where `fms_interface_step()` sits in `src/task/vehicle/normal/task_vehicle.c:67-86`.
2. Read `src/model/fms/mc_fms/fms_interface.c:292-363` to understand `FMS_U`.
3. Read `src/model/fms/mc_fms/lib/FMS.c:2052-2078` for top-level mode legality and degrade checks.
4. Read `src/model/fms/mc_fms/lib/FMS.c:2484-2741` for mission-driven state updates.
5. Read `src/model/fms/mc_fms/lib/FMS.c:2800-3370` for the internal `Auto / Arm / Vehicle` state decomposition.
6. Read `src/model/fms/mc_fms/lib/FMS.c:3690-3853` for the composite state machine orchestration.
7. Read `src/model/fms/mc_fms/lib/FMS.c:4306-4486` for `on_ground`, timing conditions, and state-machine timebase support.

## Anchor map

### Firmware boundary

- `src/model/fms/mc_fms/fms_interface.c:292-363`
- Purpose: identify which external inputs become `FMS_U`
- Use this before entering generated code

### Mode legality and degrade

- `src/model/fms/mc_fms/lib/FMS.c:2052-2078`
- Function: `FMS_Mode()`
- Purpose: check whether requested modes are legal based on `INS_Out.flag`
- Questions it answers:
- how mode degradation is detected
- why some requested modes are rejected

### Mission state progression

- `src/model/fms/mc_fms/lib/FMS.c:2484-2741`
- Function: `FMS_Mission()`
- Purpose: update `wp_index`, `nav_cmd`, `sp_waypoint`, `set_speed`, mission substate
- Questions it answers:
- how mission data advances waypoint state
- where `Return`, `Land`, `Takeoff`, `Waypoint`, and `SetSpeed` branches begin

### Internal Auto and Arm decomposition

- `src/model/fms/mc_fms/lib/FMS.c:2800-3370`
- Functions: `FMS_enter_internal_Auto()`, `FMS_enter_internal_Arm()`, `FMS_SubMode()`, `FMS_Arm()`, `FMS_Vehicle()`
- Purpose: show how the generated chart is layered instead of being a single flat switch

### Composite chart orchestrator

- `src/model/fms/mc_fms/lib/FMS.c:3690-3853`
- Function: `FMS_c11_FMS()`
- Purpose: coordinate `Command_Listener`, `Combo_Stick`, `Lost_Return`, `Vehicle`
- Questions it answers:
- which state domains evolve in parallel
- where command listening and stick combos are handled

### Timebase and on-ground support

- `src/model/fms/mc_fms/lib/FMS.c:4306-4486`
- Purpose: compute `on_ground`, maintain chart counters, update transition timing references
- Use this when explaining land/disarm timing or why `Control_Out` affects state evolution

## Boundary with adjacent skills

- Use `$fmt-command-ingress-reader` for RC, GCS, auto, and mission ingress
- Use `$fmt-fms-interface-reader` for `FMS_U` and `fms_output` boundaries
- Use `$fmt-fms-command-generation-reader` for `FMS_Out` field generation after state transitions are known

## Typical question mapping

- “FMS 怎么从 Disarm 切到 Arm？”:
- read `FMS_Mode()`, then `FMS_Arm()`, then `FMS_c11_FMS()`
- “Mission 状态为什么跳到 Return 或 Land？”:
- read `FMS_Mission()`
- “Lost link 和 combo stick 在哪里？”:
- read `FMS_c11_FMS()`


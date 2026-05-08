/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_state.h
 *
 * Internal EKF state container.  Holds:
 *   - the nominal state (full-precision quaternion, velocity, position)
 *   - the error-state covariance P
 *   - the WGS84 origin (double precision) for NED <-> LLA conversion
 *   - sensor health / readiness book-keeping
 *
 * The error-state vector layout is given by ekf_state_idx_t in INS.h.
 */

#ifndef EKF_STATE_H__
#define EKF_STATE_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* P stored as a flat row-major NxN buffer to keep arm_math compatibility */
typedef struct {
    /* ---------- nominal state ---------- */
    real32_T q[4];        /* body-to-NED quaternion, [w x y z], unit norm  */
    real32_T v_NED[3];    /* velocity in NED frame [m/s]                   */
    real32_T p_NED[3];    /* position in NED frame [m] (relative to origin)*/
    real32_T b_g[3];      /* gyro bias [rad/s]                             */
    real32_T b_a[3];      /* accel bias [m/s^2]                            */
    real32_T baro_b;      /* baro height bias [m]                          */
    real32_T terr_d;      /* terrain height in NED Down [m]                */

    /* ---------- error-state covariance ---------- */
    real32_T P[EKF_NSTATES * EKF_NSTATES];

    /* ---------- WGS84 origin (high precision) ---------- */
    real_T   lat0_rad;
    real_T   lon0_rad;
    real_T   alt0_m;
    real_T   dx_dlat;     /* m / rad latitude  */
    real_T   dy_dlon;     /* m / rad longitude */
    uint8_T  origin_set;

    /* ---------- timing ---------- */
    uint32_T t_prev_us;   /* last predict timestamp, microseconds */
    real32_T dt;          /* nominal step period [s]              */

    /* ---------- readiness / status book-keeping ---------- */
    uint32_T flag;        /* mirrors INS_Out_Bus.flag    */
    uint32_T status;      /* mirrors INS_Out_Bus.status  */
    uint8_T  init_done;
    uint8_T  baro_seeded; /* baro_b snapped at least once after origin */
} ekf_t;

extern ekf_t ekf;

void ekf_state_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_STATE_H__ */

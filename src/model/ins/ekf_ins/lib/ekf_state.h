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
 *   - the error-state covariance, stored in Bierman-Thornton UDU' form:
 *       P = U * diag(D) * U^T,
 *     where U is unit upper triangular (U[i,i] = 1, U[i,j] only
 *     meaningful for j > i) and D is the diagonal of variances.
 *     The factorisation is positive-semi-definite by construction so
 *     the symmetrize / clamp_floor passes used by the conventional
 *     form are no longer needed for stability.
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

typedef struct {
    /* ---------- nominal state ---------- */
    real32_T q[4];        /* body-to-NED quaternion, [w x y z], unit norm  */
    real32_T v_NED[3];    /* velocity in NED frame [m/s]                   */
    real32_T p_NED[3];    /* position in NED frame [m] (relative to origin)*/
    real32_T b_g[3];      /* gyro bias [rad/s]                             */
    real32_T b_a[3];      /* accel bias [m/s^2]                            */
    real32_T baro_b;      /* baro height bias [m]                          */
    real32_T terr_d;      /* terrain height in NED Down [m]                */

    /* Previous-step bias-corrected IMU samples, kept for the two-sample
     * coning / sculling corrections in ekf_predict (Savage 1998).  Zero
     * on first step => zero correction, which is mathematically correct
     * (just no improvement over single-sample).                          */
    real32_T omega_prev[3];
    real32_T acc_prev[3];

    /* ---------- error-state covariance, UDU' factored ---------------- */
    /* U: unit upper triangular NxN.  U[i*N+j] is meaningful only for    *
     *    j >= i; U[i*N+i] = 1; lower triangle is unused (kept at 0).    *
     * D: diagonal of variances (length N).                              *
     * Invariant:  P = U * diag(D) * U^T                                 */
    real32_T U[EKF_NSTATES * EKF_NSTATES];
    real32_T D[EKF_NSTATES];

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

/* P[i,i] = D[i] + sum_{k > i} U[i,k]^2 * D[k]   (since U[i,i] = 1).      */
static inline real32_T ekf_P_diag(int i)
{
    real32_T s = ekf.D[i];
    for (int k = i + 1; k < EKF_NSTATES; k++) {
        real32_T u = ekf.U[i * EKF_NSTATES + k];
        s += u * u * ekf.D[k];
    }
    return s;
}

#ifdef __cplusplus
}
#endif

#endif /* EKF_STATE_H__ */

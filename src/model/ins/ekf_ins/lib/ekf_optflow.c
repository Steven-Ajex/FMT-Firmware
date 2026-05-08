/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_optflow.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

#define OPF_MIN_QUALITY  100U   /* tunable; promoted to a param if needed */

/* ------------------------------------------------------------------ */
/*  Body-frame horizontal velocity observation                         */
/*                                                                     */
/*    z_vx_body = R^T(q)[0,:] * v_NED                                  */
/*    z_vy_body = R^T(q)[1,:] * v_NED                                  */
/*                                                                     */
/*  H rows w.r.t. velocity error (the dominant term):                  */
/*    H_vx[VN..VD] = R^T row 0 = (R[0], R[3], R[6])                    */
/*    H_vy[VN..VD] = R^T row 1 = (R[1], R[4], R[7])                    */
/*                                                                     */
/*  Attitude coupling is second-order (small dtheta times v) and       */
/*  omitted in this version; it can be added later as -R^T*[v_NED]_x.  */
/* ------------------------------------------------------------------ */
int ekf_update_optflow(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_OPF))               return 0;
    if (INS_U.Optical_Flow.timestamp == 0U)                    return 0;
    if (INS_U.Optical_Flow.quality < OPF_MIN_QUALITY)          return 0;

    real32_T R[9];
    quat_to_dcm(ekf.q, R);

    real32_T h_vx = R[0] * ekf.v_NED[0] + R[3] * ekf.v_NED[1] + R[6] * ekf.v_NED[2];
    real32_T h_vy = R[1] * ekf.v_NED[0] + R[4] * ekf.v_NED[1] + R[7] * ekf.v_NED[2];

    real32_T innov_x = INS_U.Optical_Flow.vx - h_vx;
    real32_T innov_y = INS_U.Optical_Flow.vy - h_vy;

    real32_T R_meas = INS_PARAM.EKF_OPF_NSE * INS_PARAM.EKF_OPF_NSE;
    real32_T H[N];
    real32_T dx[N];
    int n_ok = 0;

    ekf_set_innov_tag("opf_x");
    /* x */
    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_VN] = R[0];
    H[EKF_X_VE] = R[3];
    H[EKF_X_VD] = R[6];
    if (ekf_update_scalar(H, innov_x, R_meas, INS_PARAM.EKF_OPF_GATE, dx)) {
        ekf_inject_error(dx); n_ok++;
    }

    ekf_set_innov_tag("opf_y");
    /* y -- recompute R since q may have changed */
    quat_to_dcm(ekf.q, R);
    h_vy = R[1] * ekf.v_NED[0] + R[4] * ekf.v_NED[1] + R[7] * ekf.v_NED[2];
    innov_y = INS_U.Optical_Flow.vy - h_vy;

    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_VN] = R[1];
    H[EKF_X_VE] = R[4];
    H[EKF_X_VD] = R[7];
    if (ekf_update_scalar(H, innov_y, R_meas, INS_PARAM.EKF_OPF_GATE, dx)) {
        ekf_inject_error(dx); n_ok++;
    }
    return n_ok;
}

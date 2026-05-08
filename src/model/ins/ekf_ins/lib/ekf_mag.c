/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_mag.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

/* ------------------------------------------------------------------ */
/*  Initial alignment from accel (tilt) + mag (heading)                */
/* ------------------------------------------------------------------ */
void ekf_mag_align_initial(void)
{
    real32_T ax = INS_U.IMU.acc_x;
    real32_T ay = INS_U.IMU.acc_y;
    real32_T az = INS_U.IMU.acc_z;
    real32_T n  = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1.0f) {
        return;     /* free-fall / sensor not ready */
    }

    real32_T phi   = atan2f(-ay, -az);
    real32_T theta = atan2f(ax, sqrtf(ay * ay + az * az));
    real32_T psi   = 0.0f;

    if (INS_U.MAG.timestamp != 0U) {
        real32_T mx = INS_U.MAG.mag_x;
        real32_T my = INS_U.MAG.mag_y;
        real32_T mz = INS_U.MAG.mag_z;
        real32_T cphi = cosf(phi),   sphi = sinf(phi);
        real32_T cthe = cosf(theta), sthe = sinf(theta);
        /* tilt-compensated horizontal field, yaw assumed zero */
        real32_T mag_n =  mx * cthe + my * sthe * sphi + mz * sthe * cphi;
        real32_T mag_e =  my * cphi - mz * sphi;
        psi = atan2f(-mag_e, mag_n);
    }

    quat_from_euler(ekf.q, phi, theta, psi);

    /* P0 diagonal */
    for (int i = 0; i < N * N; i++) ekf.P[i] = 0.0f;

    real32_T pv = INS_PARAM.EKF_P0_POS  * INS_PARAM.EKF_P0_POS;
    real32_T vv = INS_PARAM.EKF_P0_VEL  * INS_PARAM.EKF_P0_VEL;
    real32_T av = INS_PARAM.EKF_P0_ATT  * INS_PARAM.EKF_P0_ATT;
    real32_T bg = INS_PARAM.EKF_P0_BG   * INS_PARAM.EKF_P0_BG;
    real32_T ba = INS_PARAM.EKF_P0_BA   * INS_PARAM.EKF_P0_BA;
    real32_T bb = INS_PARAM.EKF_P0_BARO * INS_PARAM.EKF_P0_BARO;
    real32_T tv = INS_PARAM.EKF_P0_TERR * INS_PARAM.EKF_P0_TERR;

    for (int i = 0; i < 3; i++) {
        ekf.P[(EKF_X_PN   + i) * N + (EKF_X_PN   + i)] = pv;
        ekf.P[(EKF_X_VN   + i) * N + (EKF_X_VN   + i)] = vv;
        ekf.P[(EKF_X_DTHX + i) * N + (EKF_X_DTHX + i)] = av;
        ekf.P[(EKF_X_BGX  + i) * N + (EKF_X_BGX  + i)] = bg;
        ekf.P[(EKF_X_BAX  + i) * N + (EKF_X_BAX  + i)] = ba;
    }
    ekf.P[EKF_X_BARO_B * N + EKF_X_BARO_B] = bb;
    ekf.P[EKF_X_TERR   * N + EKF_X_TERR]   = tv;

    ekf.init_done = 1;
}

/* ------------------------------------------------------------------ */
/*  Magnetic heading update                                            */
/*                                                                     */
/*  Convention: q is injected as q <- Exp(0.5*dtheta_NED) * q.  A      */
/*  positive dtheta_z therefore increases yaw, so the linearized       */
/*  observation of yaw becomes:                                        */
/*                                                                     */
/*      h(x + dx) = psi(q) + dtheta_z + ...                            */
/*                                                                     */
/*  Hence H[EKF_X_DTHZ] = +1.                                          */
/* ------------------------------------------------------------------ */
int ekf_update_mag_heading(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_MAG)) return 0;
    if (INS_U.MAG.timestamp == 0U) return 0;

    real32_T mb[3] = { INS_U.MAG.mag_x, INS_U.MAG.mag_y, INS_U.MAG.mag_z };
    real32_T mn[3];
    quat_rotate_vec(ekf.q, mb, mn);

    real32_T psi_meas = atan2f(-mn[1], mn[0]);
    real32_T phi, theta, psi_pred;
    quat_to_euler(ekf.q, &phi, &theta, &psi_pred);

    real32_T innov = ekf_wrap_pi(psi_meas - psi_pred);

    real32_T H[N] = { 0.0f };
    H[EKF_X_DTHZ] = 1.0f;

    real32_T R = INS_PARAM.EKF_MAG_NSE * INS_PARAM.EKF_MAG_NSE;
    real32_T dx[N];
    if (ekf_update_scalar(H, innov, R, INS_PARAM.EKF_MAG_GATE, dx)) {
        ekf_inject_error(dx);
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Gravity observation (tilt update)                                  */
/*                                                                     */
/*  When linear acceleration is small, the specific-force vector       */
/*  observed by the accelerometer is the negative gravity vector       */
/*  expressed in body:                                                 */
/*                                                                     */
/*      f_B  =  -R^T(q) * g_NED      with  g_NED = [0, 0, +9.80665]    */
/*                                                                     */
/*  Linearizing in the left-perturbation convention                    */
/*  (R_true = Exp(dtheta_NED) * R_nominal) gives                       */
/*                                                                     */
/*      H_theta = -R^T_nominal * [g_NED]_x                             */
/*                                                                     */
/*  The two horizontal components of f_B carry the tilt information;   */
/*  the vertical component is dominated by the magnitude check.  We    */
/*  apply the update only when |f_B| is close to g, gating large       */
/*  manoeuvres out by a magnitude window.                              */
/* ------------------------------------------------------------------ */
int ekf_update_gravity(void)
{
    real32_T fx = INS_U.IMU.acc_x - ekf.b_a[0];
    real32_T fy = INS_U.IMU.acc_y - ekf.b_a[1];
    real32_T fz = INS_U.IMU.acc_z - ekf.b_a[2];
    real32_T fn = sqrtf(fx * fx + fy * fy + fz * fz);

    /* magnitude window: 1 m/s^2 around g */
    if (fabsf(fn - 9.80665f) > 1.0f) return 0;

    /* predicted f_B = -R^T * g_NED.  We write R^T as columns of R. */
    real32_T R[9];
    quat_to_dcm(ekf.q, R);
    /* g_NED = [0,0,g], so R^T*g_NED = [R[6], R[7], R[8]] * g (column 3 of R) */
    const real32_T g = 9.80665f;
    real32_T h_x = -R[6] * g;       /* row 0 of R^T = column 0 of R */
    real32_T h_y = -R[7] * g;
    real32_T h_z = -R[8] * g;
    (void)h_z;                      /* applied only as magnitude check */

    /* Innovations in body x and y */
    real32_T innov_x = fx - h_x;
    real32_T innov_y = fy - h_y;

    /* H_theta = -R^T * [g_NED]_x.  With g_NED = [0,0,g]:
     *   [g_NED]_x = [[0,-g,0],[g,0,0],[0,0,0]]
     *   H_theta = -R^T * [g_NED]_x =
     *     row0 of -R^T * [...] = [ R[7]*g, -R[6]*g, 0 ]
     *     row1                 = [ R[4]*g, -R[3]*g, 0 ]
     *     row2                 = [ R[1]*g, -R[0]*g, 0 ]
     *
     * Wait — we need rows of (-R^T * [g_NED]_x).  R^T entries: R^T[i,j] = R[j,i].
     *   ( -R^T * [g_NED]_x )[i, j] = - sum_k R[k,i] * [g_NED]_x[k,j]
     * For j=0: [g]_x[:,0] = [0, g, 0]'  -> sum = R[1,i]*g, so entry = -R[1,i]*g
     * For j=1: [g]_x[:,1] = [-g,0, 0]'  -> sum = -R[0,i]*g, so entry = R[0,i]*g
     * For j=2: 0
     *
     * So row i of H_theta:
     *   [ -R[1,i]*g,  R[0,i]*g,  0 ]
     *
     * R is stored row-major: R[i*3+j] = R[i,j]. R[1,i] = R[1*3+i] = R[3+i].
     */
    real32_T H[N];

    /* --- innov_x (body x component) --- */
    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_DTHX] = -R[3 + 0] * g;
    H[EKF_X_DTHY] =  R[0 + 0] * g;
    /* dependence on accel bias: ∂f_b/∂ba_x = -1 */
    H[EKF_X_BAX]  = -1.0f;

    /* Loose noise (sigma = 0.5 m/s^2) so the tilt observation only nudges
     * the state; precise value tuned in Phase 5. */
    const real32_T Rxy = 0.25f;
    real32_T dx[N];
    int n_ok = 0;
    if (ekf_update_scalar(H, innov_x, Rxy, 5.0f, dx)) {
        ekf_inject_error(dx); n_ok++;
    }

    /* --- innov_y (body y component), recompute R as q changed --- */
    quat_to_dcm(ekf.q, R);
    h_y = -R[7] * g;
    innov_y = (INS_U.IMU.acc_y - ekf.b_a[1]) - h_y;

    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_DTHX] = -R[3 + 1] * g;
    H[EKF_X_DTHY] =  R[0 + 1] * g;
    H[EKF_X_BAY]  = -1.0f;

    if (ekf_update_scalar(H, innov_y, Rxy, 5.0f, dx)) {
        ekf_inject_error(dx); n_ok++;
    }

    return n_ok;
}

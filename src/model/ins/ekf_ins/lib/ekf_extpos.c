/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_extpos.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

/* ------------------------------------------------------------------ */
/*  External position update                                           */
/*                                                                     */
/*  Treated as direct observations of p_NED with the noise from        */
/*  EKF_EXT_POS_NSE.  We require that an absolute reference be set     */
/*  (origin_set == 1) so the external frame and the EKF frame align;   */
/*  in practice the external module is expected to publish in the      */
/*  same NED-relative-to-origin convention that the EKF maintains.     */
/* ------------------------------------------------------------------ */
int ekf_update_extpos(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_EXT_POS))   return 0;
    if (INS_U.External_Pos.timestamp == 0U)            return 0;

    uint32_T fv = INS_U.External_Pos.field_valid;
    if (!(fv & (EXTPOS_BIT_XY | EXTPOS_BIT_Z)))        return 0;

    real32_T R_meas = INS_PARAM.EKF_EXT_POS_NSE * INS_PARAM.EKF_EXT_POS_NSE;
    real32_T H[N];
    real32_T dx[N];
    int n_ok = 0;

    if (fv & EXTPOS_BIT_XY) {
        ekf_set_innov_tag("ext_x");
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_PN] = 1.0f;
        if (ekf_update_scalar(H,
                              INS_U.External_Pos.x - ekf.p_NED[0],
                              R_meas, 0.0f, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
        ekf_set_innov_tag("ext_y");
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_PE] = 1.0f;
        if (ekf_update_scalar(H,
                              INS_U.External_Pos.y - ekf.p_NED[1],
                              R_meas, 0.0f, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
    }

    if (fv & EXTPOS_BIT_Z) {
        ekf_set_innov_tag("ext_z");
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_PD] = 1.0f;
        if (ekf_update_scalar(H,
                              INS_U.External_Pos.z - ekf.p_NED[2],
                              R_meas, 0.0f, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
    }

    return n_ok;
}

/* ------------------------------------------------------------------ */
/*  External attitude update                                           */
/*                                                                     */
/*  Roll / pitch are fused as small-angle observations directly on     */
/*  the body x / y components of the attitude error (so the linear     */
/*  Jacobian is identity; the cf_ins behaviour for level platforms).   */
/*                                                                     */
/*  Yaw fusion is gated by EKF_EXTPOS_PSI_MODE.                        */
/* ------------------------------------------------------------------ */
int ekf_update_extatt(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_EXT_ATT))   return 0;
    if (INS_U.External_Pos.timestamp == 0U)            return 0;

    uint32_T fv = INS_U.External_Pos.field_valid;

    real32_T R_meas = INS_PARAM.EKF_EXT_ATT_NSE * INS_PARAM.EKF_EXT_ATT_NSE;
    real32_T H[N];
    real32_T dx[N];
    int n_ok = 0;

    real32_T phi, theta, psi;
    quat_to_euler(ekf.q, &phi, &theta, &psi);

    if (fv & EXTPOS_BIT_RP) {
        ekf_set_innov_tag("ext_phi");
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_DTHX] = 1.0f;
        if (ekf_update_scalar(H,
                              ekf_wrap_pi(INS_U.External_Pos.phi - phi),
                              R_meas, 0.0f, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
        ekf_set_innov_tag("ext_theta");
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_DTHY] = 1.0f;
        if (ekf_update_scalar(H,
                              ekf_wrap_pi(INS_U.External_Pos.theta - theta),
                              R_meas, 0.0f, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
    }

    if (fv & EXTPOS_BIT_PSI) {
        real32_T psi_ref = INS_U.External_Pos.psi;
        switch (INS_PARAM.EKF_EXTPOS_PSI_MODE) {
        case 0U: /* do not fuse */
            psi_ref = 0.0f;
            break;
        case 1U: /* as-is */
            break;
        case 2U: /* with offset */
            psi_ref += INS_PARAM.EKF_EXTPOS_PSI;
            break;
        case 3U: /* alias of 0 — disabled */
        default:
            psi_ref = 0.0f;
            break;
        }
        if (INS_PARAM.EKF_EXTPOS_PSI_MODE == 1U
         || INS_PARAM.EKF_EXTPOS_PSI_MODE == 2U) {
            ekf_set_innov_tag("ext_psi");
            for (int i = 0; i < N; i++) H[i] = 0.0f;
            H[EKF_X_DTHZ] = 1.0f;
            if (ekf_update_scalar(H,
                                  ekf_wrap_pi(psi_ref - psi),
                                  R_meas, 0.0f, dx)) {
                ekf_inject_error(dx); n_ok++;
            }
        }
    }
    return n_ok;
}

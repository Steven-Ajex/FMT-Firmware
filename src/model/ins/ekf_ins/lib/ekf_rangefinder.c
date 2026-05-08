/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_rangefinder.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

/* Hard-coded validity envelope.  Promoted to params when needed. */
#define RF_MIN_DIST_M    0.05f
#define RF_MAX_DIST_M    40.0f
#define RF_MIN_COS_TILT  0.866f      /* ~30 deg */

/* ------------------------------------------------------------------ */
/*  Range finder height observation                                    */
/*                                                                     */
/*  Measurement model:                                                 */
/*      z_meas * cos_tilt = terr_d - p_NED[2]                          */
/*                                                                     */
/*  We rearrange to z_pred = (terr_d - p_NED[2]) / cos_tilt, so the    */
/*  innovation has natural meter units and the H row is independent    */
/*  of cos_tilt for small angles.                                      */
/*                                                                     */
/*  Linearised in the dominant terms:                                  */
/*      dH/d(p_d)    = -1 / cos_tilt                                   */
/*      dH/d(terr_d) = +1 / cos_tilt                                   */
/* ------------------------------------------------------------------ */
int ekf_update_rangefinder(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_RF))     return 0;
    if (INS_U.Rangefinder.timestamp == 0U)          return 0;

    real32_T z = INS_U.Rangefinder.distance;
    if (z < RF_MIN_DIST_M || z > RF_MAX_DIST_M)     return 0;

    /* cos of body-Z to NED-Down angle, i.e. R[8] = R_zz */
    real32_T R[9];
    quat_to_dcm(ekf.q, R);
    real32_T cos_tilt = R[8];
    if (cos_tilt < RF_MIN_COS_TILT)                 return 0;

    real32_T inv_c   = 1.0f / cos_tilt;
    real32_T h_pred  = (ekf.terr_d - ekf.p_NED[2]) * inv_c;
    real32_T innov   = z - h_pred;

    real32_T H[N] = { 0.0f };
    H[EKF_X_PD]   = -inv_c;
    H[EKF_X_TERR] =  inv_c;

    real32_T R_meas = INS_PARAM.EKF_RF_NSE * INS_PARAM.EKF_RF_NSE;
    real32_T dx[N];
    ekf_set_innov_tag("rf");
    if (ekf_update_scalar(H, innov, R_meas, INS_PARAM.EKF_RF_GATE, dx)) {
        ekf_inject_error(dx);
        return 1;
    }
    return 0;
}

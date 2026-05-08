/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_baro.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"
#include <math.h>

#define N EKF_NSTATES

/* ------------------------------------------------------------------ */
/*  Pressure -> ISA height (m above MSL).  Standard atmosphere.        */
/*    h = 44330.77 * (1 - (P/101325)^(1/5.255876))                    */
/* ------------------------------------------------------------------ */
real32_T ekf_baro_height_from_pressure(real32_T pressure_pa)
{
    if (pressure_pa < 1.0f) return 0.0f;
    return 44330.77f * (1.0f - powf(pressure_pa / 101325.0f, 0.190284f));
}

/* ------------------------------------------------------------------ */
/*  Height update                                                      */
/*                                                                     */
/*  Predicted measurement:                                             */
/*    h_pred = -p_NED[2] + alt_0 + baro_b                              */
/*                                                                     */
/*  H w.r.t. the error state:                                          */
/*    dH/d(p_d)    = -1                                                */
/*    dH/d(baro_b) = +1                                                */
/* ------------------------------------------------------------------ */
int ekf_update_baro(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_BARO))            return 0;
    if (INS_U.Barometer.timestamp == 0U)                     return 0;
    if (INS_PARAM.EKF_HGT_MODE != EKF_HGT_SRC_BARO)          return 0;

    real32_T h_meas = ekf_baro_height_from_pressure(INS_U.Barometer.pressure);

    /* If no absolute reference is set yet, treat the very first baro
     * sample as the local height origin.  This lets the variant operate
     * baro-only (no GPS) and still output meaningful relative altitude. */
    if (!ekf.origin_set) {
        ekf.alt0_m     = h_meas;
        ekf.lat0_rad   = 0.0;
        ekf.lon0_rad   = 0.0;
        ekf.dx_dlat    = 0.0;       /* signals "no horizontal origin yet" */
        ekf.dy_dlon    = 0.0;
        ekf.p_NED[2]   = 0.0f;
        ekf.baro_b     = 0.0f;
        ekf.baro_seeded  = 1;
        return 0;       /* nothing to fuse — we just defined the origin */
    }

    /* On first usable baro sample after the WGS84 origin has been set
     * (e.g. by GPS first fix), snap baro_b so the initial innovation is
     * zero.  Otherwise the EKF would have to absorb a possibly large
     * step (GPS height vs ISA height frequently differ by 10-100 m). */
    if (!ekf.baro_seeded) {
        ekf.baro_b    = h_meas - ((real32_T)ekf.alt0_m - ekf.p_NED[2]);
        ekf.baro_seeded = 1;
        return 0;
    }

    real32_T h_pred = -ekf.p_NED[2] + (real32_T)ekf.alt0_m + ekf.baro_b;
    real32_T innov  = h_meas - h_pred;

    real32_T H[N] = { 0.0f };
    H[EKF_X_PD]     = -1.0f;
    H[EKF_X_BARO_B] =  1.0f;

    real32_T R = INS_PARAM.EKF_BARO_NSE * INS_PARAM.EKF_BARO_NSE;
    real32_T dx[N];
    ekf_set_innov_tag("baro");
    if (ekf_update_scalar(H, innov, R, INS_PARAM.EKF_BARO_GATE, dx)) {
        ekf_inject_error(dx);
        return 1;
    }
    return 0;
}

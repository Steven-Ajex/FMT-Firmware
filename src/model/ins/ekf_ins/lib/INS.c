/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * INS.c
 *
 * Top-level entry of the ekf_ins variant.  Wires the prediction step and
 * the measurement front-ends (currently mag heading + gravity tilt) and
 * publishes the result into INS_Y.
 *
 * Each phase plugs additional updates in here without touching the
 * predict / inject machinery.
 */

#include "INS.h"
#include "ekf_state.h"
#include "ekf_math.h"
#include "ekf_core.h"
#include "ekf_mag.h"

INS_U_T      INS_U;
INS_Y_T      INS_Y;
INS_PARAM_T  INS_PARAM;
INS_EXPORT_T INS_EXPORT = {
    2U,                     /* 2 ms = 500 Hz step                   */
    { 'E', 'K', 'F', '_', 'I', 'N', 'S', ' ',
      'v', '0', '.', '2', 0, 0, 0, 0, 0, 0, 0, 0 }
};

ekf_t ekf;

/* ------------------------------------------------------------------ */
/*  Default parameter values.  Re-applied by INS_init() so the firmware*/
/*  always starts from a sane configuration even before the param      */
/*  layer overwrites them via param_link_variable().                   */
/* ------------------------------------------------------------------ */
static void ekf_load_defaults(void)
{
    INS_PARAM.EKF_GYR_NOISE     = 1.5e-2f;
    INS_PARAM.EKF_ACC_NOISE     = 3.5e-1f;
    INS_PARAM.EKF_BG_NOISE      = 1.0e-4f;
    INS_PARAM.EKF_BA_NOISE      = 1.0e-3f;
    INS_PARAM.EKF_BARO_B_NOISE  = 1.0e-2f;
    INS_PARAM.EKF_TERR_NOISE    = 5.0e-2f;

    INS_PARAM.EKF_P0_POS  = 10.0f;
    INS_PARAM.EKF_P0_VEL  = 1.0f;
    INS_PARAM.EKF_P0_ATT  = 0.35f;
    INS_PARAM.EKF_P0_BG   = 0.05f;
    INS_PARAM.EKF_P0_BA   = 0.5f;
    INS_PARAM.EKF_P0_BARO = 5.0f;
    INS_PARAM.EKF_P0_TERR = 5.0f;

    INS_PARAM.EKF_GPS_POS_NSE = 0.5f;
    INS_PARAM.EKF_GPS_VEL_NSE = 0.3f;
    INS_PARAM.EKF_GPS_ALT_NSE = 1.5f;
    INS_PARAM.EKF_BARO_NSE    = 2.0f;
    INS_PARAM.EKF_MAG_NSE     = 0.05f;
    INS_PARAM.EKF_RF_NSE      = 0.1f;
    INS_PARAM.EKF_OPF_NSE     = 0.2f;
    INS_PARAM.EKF_EXT_POS_NSE = 0.05f;
    INS_PARAM.EKF_EXT_ATT_NSE = 0.05f;

    INS_PARAM.EKF_GPS_GATE   = 5.0f;
    INS_PARAM.EKF_MAG_GATE   = 5.0f;
    INS_PARAM.EKF_BARO_GATE  = 5.0f;
    INS_PARAM.EKF_RF_GATE    = 5.0f;
    INS_PARAM.EKF_OPF_GATE   = 5.0f;

    INS_PARAM.EKF_AID_MASK = EKF_AID_GPS | EKF_AID_MAG | EKF_AID_BARO;
    INS_PARAM.EKF_HGT_MODE = EKF_HGT_SRC_BARO;
    INS_PARAM.EKF_EXTPOS_PSI_MODE = 3;
    INS_PARAM.EKF_EXTPOS_PSI = 0.0f;

    INS_PARAM.EKF_GPS_DELAY  = 100;
    INS_PARAM.EKF_BARO_DELAY = 10;
    INS_PARAM.EKF_MAG_DELAY  = 0;
    INS_PARAM.EKF_RF_DELAY   = 10;
    INS_PARAM.EKF_OPF_DELAY  = 10;
    INS_PARAM.EKF_EXT_DELAY  = 20;

    INS_PARAM.EKF_GPS_X_OFFSET = 0.0f;
    INS_PARAM.EKF_GPS_Y_OFFSET = 0.0f;
    INS_PARAM.EKF_GPS_Z_OFFSET = 0.0f;
}

void ekf_state_reset(void)
{
    memset(&ekf, 0, sizeof(ekf));
    ekf.q[0] = 1.0f;
    ekf.dt   = (real32_T)INS_EXPORT.period * 1.0e-3f;
}

/* ------------------------------------------------------------------ */
/*  Output assembly                                                    */
/* ------------------------------------------------------------------ */
static void publish_output(void)
{
    INS_Out_Bus* y = &INS_Y.INS_Out;

    real32_T phi, theta, psi;
    quat_to_euler(ekf.q, &phi, &theta, &psi);

    y->timestamp = INS_U.IMU.timestamp;
    y->phi       = phi;
    y->theta     = theta;
    y->psi       = psi;
    y->quat[0]   = ekf.q[0];
    y->quat[1]   = ekf.q[1];
    y->quat[2]   = ekf.q[2];
    y->quat[3]   = ekf.q[3];

    /* angular rate / specific force after bias correction */
    y->p = INS_U.IMU.gyr_x - ekf.b_g[0];
    y->q = INS_U.IMU.gyr_y - ekf.b_g[1];
    y->r = INS_U.IMU.gyr_z - ekf.b_g[2];
    y->ax = INS_U.IMU.acc_x - ekf.b_a[0];
    y->ay = INS_U.IMU.acc_y - ekf.b_a[1];
    y->az = INS_U.IMU.acc_z - ekf.b_a[2];

    /* velocity / position (NED) — populated in later phases */
    y->vn = ekf.v_NED[0];
    y->ve = ekf.v_NED[1];
    y->vd = ekf.v_NED[2];

    y->x_R = ekf.p_NED[0];
    y->y_R = ekf.p_NED[1];
    y->h_R = -ekf.p_NED[2];
    y->h_AGL = -(ekf.p_NED[2] - ekf.terr_d);

    y->airspeed = 0.0f;

    /* WGS84 LLA — populated in Phase 2 once GPS is fused */
    y->lat = y->lon = y->alt = 0.0;
    y->lat_0 = y->lon_0 = y->alt_0 = 0.0;
    y->dx_dlat = y->dy_dlon = 0.0;

    y->status = ekf.status | 1U;        /* IMU available bit */
    y->flag   = ekf.flag;
    if (ekf.init_done) {
        y->flag |= (1U << 0);            /* ready    */
        y->flag |= (1U << 2);            /* att_valid */
    }
}

void INS_init(void)
{
    ekf_load_defaults();
    ekf_state_reset();
    INS_Y.INS_Out.quat[0] = 1.0f;
}

/* ------------------------------------------------------------------ */
/*  Top-level step                                                     */
/* ------------------------------------------------------------------ */
void INS_step(void)
{
    if (!ekf.init_done) {
        ekf_mag_align_initial();
        publish_output();
        return;
    }

    /* ---- prediction ---- */
    real32_T omega[3] = { INS_U.IMU.gyr_x, INS_U.IMU.gyr_y, INS_U.IMU.gyr_z };
    real32_T accel[3] = { INS_U.IMU.acc_x, INS_U.IMU.acc_y, INS_U.IMU.acc_z };
    ekf_predict(omega, accel, ekf.dt);

    /* ---- gravity tilt update (per-step, gated by |f| ≈ g) ---- */
    ekf_update_gravity();

    /* ---- mag heading update (only when MAG bus ticks) ---- */
    static uint32_T last_mag_ts = 0;
    if (INS_U.MAG.timestamp != last_mag_ts) {
        last_mag_ts = INS_U.MAG.timestamp;
        ekf_update_mag_heading();
    }

    /* ---- assemble output bus ---- */
    publish_output();
}

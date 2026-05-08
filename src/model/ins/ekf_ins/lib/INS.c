/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * INS.c
 *
 * Entry point of the ekf_ins variant.  Phase 0 provides a working stub:
 *   - INS_init    : sets defaults, zeroes state
 *   - INS_step    : passes IMU through, derives tilt from gravity, holds yaw,
 *                   forwards everything else as zero.  This unblocks the
 *                   firmware build / mlog pipeline while later phases plug in
 *                   the full EKF.
 */

#include "INS.h"
#include "ekf_state.h"
#include "ekf_math.h"

INS_U_T      INS_U;
INS_Y_T      INS_Y;
INS_PARAM_T  INS_PARAM;
INS_EXPORT_T INS_EXPORT = {
    2U,                     /* 2 ms = 500 Hz step                  */
    { 'E', 'K', 'F', '_', 'I', 'N', 'S', ' ',
      'v', '0', '.', '1', 0, 0, 0, 0, 0, 0, 0, 0 }
};

ekf_t ekf;

/* ------------------------------------------------------------------ */
/*  default parameter values (overridden by param_link_variable)       */
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
/*  Phase-0 step: derive tilt from accel + gyro pass-through.          */
/*  Real EKF prediction / update is added in Phase 1+.                 */
/* ------------------------------------------------------------------ */
static void phase0_attitude_from_accel(void)
{
    real32_T ax = INS_U.IMU.acc_x;
    real32_T ay = INS_U.IMU.acc_y;
    real32_T az = INS_U.IMU.acc_z;
    real32_T n  = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1.0f) {
        return;     /* free-fall or invalid sample */
    }

    /* roll/pitch from gravity assumption (a measures -g in body) */
    real32_T phi   = atan2f(-ay, -az);
    real32_T theta = atan2f(ax, sqrtf(ay * ay + az * az));
    real32_T psi   = 0.0f;
    quat_from_euler(ekf.q, phi, theta, psi);
}

void INS_init(void)
{
    ekf_load_defaults();
    ekf_state_reset();
    INS_Y.INS_Out.quat[0] = 1.0f;
}

void INS_step(void)
{
    INS_Out_Bus* y = &INS_Y.INS_Out;

    /* phase 0: tilt-only attitude */
    phase0_attitude_from_accel();

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

    y->p = INS_U.IMU.gyr_x;
    y->q = INS_U.IMU.gyr_y;
    y->r = INS_U.IMU.gyr_z;

    y->ax = INS_U.IMU.acc_x;
    y->ay = INS_U.IMU.acc_y;
    y->az = INS_U.IMU.acc_z;

    y->vn = 0.0f;
    y->ve = 0.0f;
    y->vd = 0.0f;
    y->airspeed = 0.0f;

    y->lat = y->lon = y->alt = 0.0;
    y->lat_0 = y->lon_0 = y->alt_0 = 0.0;
    y->dx_dlat = y->dy_dlon = 0.0;

    y->x_R = y->y_R = y->h_R = y->h_AGL = 0.0f;

    /* status: at least IMU available; flags: attitude valid (tilt only) */
    y->status = 1U;             /* imu1_available  */
    y->flag   = (1U << 2);      /* att_valid        */
}

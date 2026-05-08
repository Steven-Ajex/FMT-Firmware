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
#include "ekf_geo.h"
#include "ekf_gps.h"
#include "ekf_baro.h"
#include "ekf_rangefinder.h"
#include "ekf_optflow.h"
#include "ekf_extpos.h"
#include "ekf_health.h"

INS_U_T      INS_U;
INS_Y_T      INS_Y;
INS_PARAM_T  INS_PARAM;
INS_EXPORT_T INS_EXPORT = {
    2U,                     /* 2 ms = 500 Hz step                   */
    { 'E', 'K', 'F', '_', 'I', 'N', 'S', ' ',
      'v', '0', '.', '2', 0, 0, 0, 0, 0, 0, 0, 0 }
};

ekf_t ekf;

/* Last seen bus timestamps used to detect a fresh sample.  Kept at file
 * scope (rather than function-local statics) so ekf_state_reset() can
 * zero them and unit tests can reinitialise the EKF cleanly. */
static uint32_T s_last_mag_ts;
static uint32_T s_last_gps_ts;
static uint32_T s_last_baro_ts;
static uint32_T s_last_rf_ts;
static uint32_T s_last_opf_ts;
static uint32_T s_last_ext_ts;

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
    s_last_mag_ts = s_last_gps_ts = s_last_baro_ts = 0U;
    s_last_rf_ts  = s_last_opf_ts = s_last_ext_ts  = 0U;
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

    /* WGS84 LLA — radians for lat/lon, meters for alt, m/rad for d*_d* */
    if (ekf.origin_set) {
        real_T   lat_rad, lon_rad, alt_m;
        real32_T ned[3] = { ekf.p_NED[0], ekf.p_NED[1], ekf.p_NED[2] };
        ekf_geo_ned_to_lla(ned, &lat_rad, &lon_rad, &alt_m);
        y->lat     = lat_rad;
        y->lon     = lon_rad;
        y->alt     = alt_m;
        y->lat_0   = ekf.lat0_rad;
        y->lon_0   = ekf.lon0_rad;
        y->alt_0   = ekf.alt0_m;
        y->dx_dlat = ekf.dx_dlat;
        y->dy_dlon = ekf.dy_dlon;
    } else {
        y->lat = y->lon = y->alt = 0.0;
        y->lat_0 = y->lon_0 = y->alt_0 = 0.0;
        y->dx_dlat = y->dy_dlon = 0.0;
    }

    /* status bits - mirror INS_Status.bit (see ins_interface.h):
     *   0 imu1   1 imu2   2 mag    3 baro
     *   4 gps    5 sonar  6 optflow                                    */
    uint32_T status = 0U;
    if (ekf_health_available(EKF_SENS_IMU))  status |= (1U << 0);
    if (ekf_health_available(EKF_SENS_MAG))  status |= (1U << 2);
    if (ekf_health_available(EKF_SENS_BARO)) status |= (1U << 3);
    if (ekf_health_available(EKF_SENS_GPS))  status |= (1U << 4);
    if (ekf_health_available(EKF_SENS_RF))   status |= (1U << 5);
    if (ekf_health_available(EKF_SENS_OPF))  status |= (1U << 6);
    y->status = status;

    /* flag bits - mirror INS_Flag.bit:
     *   0 ready          1 standstill  2 att_valid  3 head_valid
     *   4 vel_valid      5 WGS84_pos_valid
     *   6 xy_R_valid     7 h_R_valid   8 h_AGL_valid                   */
    uint32_T flag = 0U;
    if (ekf.init_done) {
        flag |= (1U << 0);                  /* ready     */
        flag |= (1U << 2);                  /* att_valid */
        flag |= (1U << 3);                  /* head_valid */
    }
    if (ekf_h.standstill)                       flag |= (1U << 1);
    if (ekf.origin_set || ekf.baro_seeded)      flag |= (1U << 4);  /* vel_valid       */
    if (ekf.origin_set)                         flag |= (1U << 5);  /* WGS84_pos_valid */
    if (ekf.origin_set)                         flag |= (1U << 6);  /* xy_R_valid      */
    if (ekf.origin_set || ekf.baro_seeded)      flag |= (1U << 7);  /* h_R_valid       */
    if (ekf_health_available(EKF_SENS_RF))      flag |= (1U << 8);  /* h_AGL_valid     */
    y->flag = flag;
}

void INS_init(void)
{
    ekf_load_defaults();
    ekf_state_reset();
    ekf_health_init();
    INS_Y.INS_Out.quat[0] = 1.0f;
}

/* ------------------------------------------------------------------ */
/*  Top-level step                                                     */
/* ------------------------------------------------------------------ */
void INS_step(void)
{
    /* Health bookkeeping must run every step so timeouts trip even
     * when the EKF itself is paused (e.g. during the initial align). */
    ekf_health_update(INS_U.IMU.timestamp);

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

    /* ---- async measurement updates: only when the bus timestamp ticks
     *      and the corresponding sensor is currently healthy. ---- */
    if (INS_U.MAG.timestamp != s_last_mag_ts) {
        s_last_mag_ts = INS_U.MAG.timestamp;
        if (ekf_health_available(EKF_SENS_MAG)) ekf_update_mag_heading();
    }
    if (INS_U.GPS_uBlox.timestamp != s_last_gps_ts) {
        s_last_gps_ts = INS_U.GPS_uBlox.timestamp;
        if (ekf_health_available(EKF_SENS_GPS)) {
            ekf_update_gps_pos();
            ekf_update_gps_vel();
        }
    }
    if (INS_U.Barometer.timestamp != s_last_baro_ts) {
        s_last_baro_ts = INS_U.Barometer.timestamp;
        if (ekf_health_available(EKF_SENS_BARO)) ekf_update_baro();
    }
    if (INS_U.Rangefinder.timestamp != s_last_rf_ts) {
        s_last_rf_ts = INS_U.Rangefinder.timestamp;
        if (ekf_health_available(EKF_SENS_RF)) ekf_update_rangefinder();
    }
    if (INS_U.Optical_Flow.timestamp != s_last_opf_ts) {
        s_last_opf_ts = INS_U.Optical_Flow.timestamp;
        if (ekf_health_available(EKF_SENS_OPF)) ekf_update_optflow();
    }
    if (INS_U.External_Pos.timestamp != s_last_ext_ts) {
        s_last_ext_ts = INS_U.External_Pos.timestamp;
        if (ekf_health_available(EKF_SENS_EXT)) {
            ekf_update_extpos();
            ekf_update_extatt();
        }
    }

    /* ---- assemble output bus ---- */
    publish_output();
}

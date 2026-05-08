/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_gps.h"
#include "ekf_geo.h"
#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

/* GPS uBlox lat/lon are int32 in degrees * 1e7, height in mm. */
static inline real_T ublox_to_rad(int32_t deg_e7)
{
    return ((real_T)deg_e7) * 1.0e-7 * EKF_DEG2RAD;
}

int ekf_gps_available(void)
{
    return (INS_U.GPS_uBlox.fixType >= 3) && (INS_U.GPS_uBlox.timestamp != 0U);
}

int ekf_gps_first_fix(void)
{
    if (ekf.origin_set)            return 0;
    if (!ekf_gps_available())      return 0;

    real_T lat = ublox_to_rad(INS_U.GPS_uBlox.lat);
    real_T lon = ublox_to_rad(INS_U.GPS_uBlox.lon);
    real_T alt = (real_T)INS_U.GPS_uBlox.height * 1.0e-3;

    ekf_geo_set_origin(lat, lon, alt);

    /* place the vehicle at the local origin */
    v3_zero(ekf.p_NED);
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Position update                                                    */
/* ------------------------------------------------------------------ */
int ekf_update_gps_pos(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_GPS)) return 0;
    if (!ekf_gps_available())                    return 0;

    if (!ekf.origin_set) {
        ekf_gps_first_fix();
        return 0;
    }

    real_T lat = ublox_to_rad(INS_U.GPS_uBlox.lat);
    real_T lon = ublox_to_rad(INS_U.GPS_uBlox.lon);
    real_T alt = (real_T)INS_U.GPS_uBlox.height * 1.0e-3;

    real32_T z[3];
    ekf_geo_lla_to_ned(lat, lon, alt, z);

    /* lever arm: p_imu = p_gps - R(q) * offset_B */
    real32_T off_B[3] = {
        INS_PARAM.EKF_GPS_X_OFFSET,
        INS_PARAM.EKF_GPS_Y_OFFSET,
        INS_PARAM.EKF_GPS_Z_OFFSET,
    };
    real32_T off_N[3];
    quat_rotate_vec(ekf.q, off_B, off_N);
    z[0] -= off_N[0];
    z[1] -= off_N[1];
    z[2] -= off_N[2];

    real32_T R_h = INS_PARAM.EKF_GPS_POS_NSE * INS_PARAM.EKF_GPS_POS_NSE;
    real32_T R_v = INS_PARAM.EKF_GPS_ALT_NSE * INS_PARAM.EKF_GPS_ALT_NSE;
    real32_T H[N];
    real32_T dx[N];
    int n_ok = 0;

    /* ---- x (north) ---- */
    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_PN] = 1.0f;
    if (ekf_update_scalar(H, z[0] - ekf.p_NED[0], R_h, INS_PARAM.EKF_GPS_GATE, dx)) {
        ekf_inject_error(dx); n_ok++;
    }

    /* ---- y (east) ---- */
    for (int i = 0; i < N; i++) H[i] = 0.0f;
    H[EKF_X_PE] = 1.0f;
    if (ekf_update_scalar(H, z[1] - ekf.p_NED[1], R_h, INS_PARAM.EKF_GPS_GATE, dx)) {
        ekf_inject_error(dx); n_ok++;
    }

    /* ---- z (down): only if GPS is the active height source ---- */
    if (INS_PARAM.EKF_HGT_MODE == EKF_HGT_SRC_GPS) {
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_PD] = 1.0f;
        if (ekf_update_scalar(H, z[2] - ekf.p_NED[2], R_v, INS_PARAM.EKF_GPS_GATE, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
    }

    return n_ok;
}

/* ------------------------------------------------------------------ */
/*  Velocity update                                                    */
/* ------------------------------------------------------------------ */
int ekf_update_gps_vel(void)
{
    if (!(INS_PARAM.EKF_AID_MASK & EKF_AID_GPS)) return 0;
    if (!ekf_gps_available())                    return 0;
    if (!ekf.origin_set)                         return 0;

    real32_T v_meas[3] = {
        (real32_T)INS_U.GPS_uBlox.velN * 1.0e-3f,
        (real32_T)INS_U.GPS_uBlox.velE * 1.0e-3f,
        (real32_T)INS_U.GPS_uBlox.velD * 1.0e-3f,
    };

    /* lever-arm rate correction:  v_imu = v_gps - R * (omega_b x offset_B) */
    real32_T omega_b[3] = {
        INS_U.IMU.gyr_x - ekf.b_g[0],
        INS_U.IMU.gyr_y - ekf.b_g[1],
        INS_U.IMU.gyr_z - ekf.b_g[2],
    };
    real32_T off_B[3] = {
        INS_PARAM.EKF_GPS_X_OFFSET,
        INS_PARAM.EKF_GPS_Y_OFFSET,
        INS_PARAM.EKF_GPS_Z_OFFSET,
    };
    real32_T omega_x_off_B[3];
    v3_cross(omega_x_off_B, omega_b, off_B);
    real32_T omega_x_off_N[3];
    quat_rotate_vec(ekf.q, omega_x_off_B, omega_x_off_N);

    real32_T innov[3] = {
        v_meas[0] - omega_x_off_N[0] - ekf.v_NED[0],
        v_meas[1] - omega_x_off_N[1] - ekf.v_NED[1],
        v_meas[2] - omega_x_off_N[2] - ekf.v_NED[2],
    };

    real32_T R = INS_PARAM.EKF_GPS_VEL_NSE * INS_PARAM.EKF_GPS_VEL_NSE;
    real32_T H[N];
    real32_T dx[N];
    int n_ok = 0;
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < N; i++) H[i] = 0.0f;
        H[EKF_X_VN + axis] = 1.0f;
        if (ekf_update_scalar(H, innov[axis], R, INS_PARAM.EKF_GPS_GATE, dx)) {
            ekf_inject_error(dx); n_ok++;
        }
    }
    return n_ok;
}

/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_health.h"
#include "ekf_state.h"
#include "ekf_math.h"

ekf_health_t ekf_h;

/* Per-sensor timeout defaults [ms].  Promoted to params later if a
 * specific airframe needs different windows. */
static const uint32_T DEFAULT_TIMEOUT[EKF_SENS_COUNT] = {
    [EKF_SENS_IMU]  = 50U,
    [EKF_SENS_MAG]  = 250U,
    [EKF_SENS_BARO] = 250U,
    [EKF_SENS_GPS]  = 2000U,
    [EKF_SENS_RF]   = 500U,
    [EKF_SENS_OPF]  = 250U,
    [EKF_SENS_EXT]  = 500U,
};

/* Standstill detection thresholds.  Tuned by inspection; revisit in
 * Phase 5 with replay data. */
#define STILL_GYR_THRESH        0.05f       /* rad/s norm                 */
#define STILL_ACC_TOL_MPS2      0.6f        /* |a - g| tolerance          */
#define MOTION_ACC_TOL_MPS2     1.5f        /* sustained linear accel     */
#define HYST_COUNT_STILL        100U        /* ~200 ms @ 500 Hz           */
#define HYST_COUNT_MOTION       50U         /* ~100 ms                    */
#define LP_ALPHA                0.05f

void ekf_health_init(void)
{
    memset(&ekf_h, 0, sizeof(ekf_h));
    for (int i = 0; i < EKF_SENS_COUNT; i++) {
        ekf_h.s[i].timeout_ms = DEFAULT_TIMEOUT[i];
    }
}

/* Lookup the bus timestamp for a given sensor, in firmware ms. */
static uint32_T sensor_ts(ekf_sensor_id_t s)
{
    switch (s) {
    case EKF_SENS_IMU:  return INS_U.IMU.timestamp;
    case EKF_SENS_MAG:  return INS_U.MAG.timestamp;
    case EKF_SENS_BARO: return INS_U.Barometer.timestamp;
    case EKF_SENS_GPS:  return INS_U.GPS_uBlox.timestamp;
    case EKF_SENS_RF:   return INS_U.Rangefinder.timestamp;
    case EKF_SENS_OPF:  return INS_U.Optical_Flow.timestamp;
    case EKF_SENS_EXT:  return INS_U.External_Pos.timestamp;
    default:            return 0U;
    }
}

void ekf_health_update(uint32_T now_ms)
{
    /* ---------- per-sensor liveness ---------- */
    for (int i = 0; i < EKF_SENS_COUNT; i++) {
        ekf_sensor_health_t* h = &ekf_h.s[i];
        uint32_T ts = sensor_ts((ekf_sensor_id_t)i);
        if (ts != 0U && ts != h->last_ts) {
            h->last_ts   = ts;
            h->ever_seen = 1U;
        }
        /* fresh if a sample arrived within the timeout window */
        if (h->ever_seen) {
            uint32_T age = (now_ms >= h->last_ts) ? (now_ms - h->last_ts) : 0U;
            h->available = (age <= h->timeout_ms) ? 1U : 0U;
        } else {
            h->available = 0U;
        }
    }

    /* ---------- motion classification ---------- */
    real32_T gx = INS_U.IMU.gyr_x - ekf.b_g[0];
    real32_T gy = INS_U.IMU.gyr_y - ekf.b_g[1];
    real32_T gz = INS_U.IMU.gyr_z - ekf.b_g[2];
    real32_T ax = INS_U.IMU.acc_x - ekf.b_a[0];
    real32_T ay = INS_U.IMU.acc_y - ekf.b_a[1];
    real32_T az = INS_U.IMU.acc_z - ekf.b_a[2];

    real32_T g_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    real32_T a_norm = sqrtf(ax * ax + ay * ay + az * az);

    ekf_h.gyr_norm_lp += LP_ALPHA * (g_norm - ekf_h.gyr_norm_lp);
    ekf_h.acc_norm_lp += LP_ALPHA * (a_norm - ekf_h.acc_norm_lp);

    int still_now = (ekf_h.gyr_norm_lp < STILL_GYR_THRESH)
                 && (fabsf(ekf_h.acc_norm_lp - 9.80665f) < STILL_ACC_TOL_MPS2);
    int moving_now = fabsf(ekf_h.acc_norm_lp - 9.80665f) > MOTION_ACC_TOL_MPS2;

    if (still_now) {
        ekf_h.still_count = (ekf_h.still_count < UINT32_MAX) ? ekf_h.still_count + 1U : ekf_h.still_count;
        ekf_h.moving_count = 0U;
    } else if (moving_now) {
        ekf_h.moving_count = (ekf_h.moving_count < UINT32_MAX) ? ekf_h.moving_count + 1U : ekf_h.moving_count;
        ekf_h.still_count = 0U;
    }

    if (ekf_h.still_count >= HYST_COUNT_STILL) {
        ekf_h.standstill = 1U;
        ekf_h.in_air     = 0U;
    } else if (ekf_h.moving_count >= HYST_COUNT_MOTION) {
        ekf_h.standstill = 0U;
        ekf_h.in_air     = 1U;
    }
}

/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_health.h
 *
 * Sensor liveness, standstill / in-air detection and the bit-fields
 * for INS_Out_Bus.flag / .status.
 *
 * Design notes:
 *   - "Available" is a soft bit derived from the bus timestamp.  When
 *     no new sample has arrived for `timeout_ms` the flag drops, the
 *     EKF stops fusing the corresponding measurement and the output
 *     mirror in INS_Out_Bus.status clears.
 *   - Standstill = low gyro AND accel close to g for a sustained
 *     window.  It opens the gravity update unconditionally and
 *     (eventually) gates IMU-bias learning.
 *   - In-air     = sustained linear motion or velocity.  It frees the
 *     mag fusion when MAG_FLY_EN-style gating is implemented.
 *
 * The module is independent of the EKF: it only reads INS_U / INS_Y
 * timestamps and writes a small in-memory snapshot of health state.
 * The actual flag / status assembly happens in publish_output().
 */

#ifndef EKF_HEALTH_H__
#define EKF_HEALTH_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EKF_SENS_IMU = 0,
    EKF_SENS_MAG,
    EKF_SENS_BARO,
    EKF_SENS_GPS,
    EKF_SENS_RF,
    EKF_SENS_OPF,
    EKF_SENS_EXT,
    EKF_SENS_COUNT
} ekf_sensor_id_t;

typedef struct {
    uint32_T last_ts;     /* most recent bus timestamp [ms]    */
    uint32_T timeout_ms;  /* drop available after this idle    */
    uint8_T  available;   /* 1 if a fresh sample is in window  */
    uint8_T  ever_seen;   /* 1 if at least one sample arrived  */
} ekf_sensor_health_t;

typedef struct {
    ekf_sensor_health_t s[EKF_SENS_COUNT];

    /* Motion classification.  Each is a sticky bit guarded by a
     * count-based hysteresis to avoid chattering. */
    uint8_T  standstill;
    uint8_T  in_air;
    uint32_T still_count;
    uint32_T moving_count;

    /* Low-pass gyro / accel norms for the motion check */
    real32_T gyr_norm_lp;
    real32_T acc_norm_lp;
} ekf_health_t;

extern ekf_health_t ekf_h;

void ekf_health_init(void);
void ekf_health_update(uint32_T now_ms);

static inline int ekf_health_available(ekf_sensor_id_t s) {
    return (s < EKF_SENS_COUNT) ? ekf_h.s[s].available : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* EKF_HEALTH_H__ */

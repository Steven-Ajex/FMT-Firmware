/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_gps.h
 *
 * GPS measurement front-end:
 *
 *   ekf_gps_first_fix    sets the WGS84 origin from the first usable
 *                        fix and zeroes p_NED so the vehicle starts at
 *                        the local origin.
 *
 *   ekf_update_gps_pos   3-axis position update with body lever-arm
 *                        compensation.  The vertical channel is fused
 *                        only when EKF_HGT_MODE selects GPS altitude.
 *
 *   ekf_update_gps_vel   3-axis NED velocity update with the
 *                        omega x offset rate correction.
 *
 *   ekf_gps_available    returns 1 when the latest sample carries a
 *                        usable fix (3D or better).
 */

#ifndef EKF_GPS_H__
#define EKF_GPS_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

int ekf_gps_available(void);
int ekf_gps_first_fix(void);
int ekf_update_gps_pos(void);
int ekf_update_gps_vel(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_GPS_H__ */

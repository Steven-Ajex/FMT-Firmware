/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_mag.h
 *
 * Magnetic heading and tilt observation models.
 *
 *   ekf_mag_align_initial - one-shot startup alignment from accel + mag.
 *                           Sets the initial quaternion + diagonal P0.
 *
 *   ekf_update_mag_heading - tilt-compensated yaw update.  Scalar.
 *
 *   ekf_update_gravity     - tilt update from the accelerometer assuming
 *                           low specific force (free of large maneuvers).
 *                           Sequential 2-axis update; gated on |f| ~ g.
 */

#ifndef EKF_MAG_H__
#define EKF_MAG_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

void ekf_mag_align_initial(void);
int  ekf_update_mag_heading(void);
int  ekf_update_gravity(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_MAG_H__ */

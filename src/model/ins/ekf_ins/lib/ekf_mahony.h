/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_mahony.h
 *
 * Stand-alone explicit-complementary-filter (Mahony 2008) for attitude
 * only.  Runs in parallel with the main EKF as a "reference observer"
 * and never participates in the EKF state - its purpose is purely
 * diagnostic / safety: a Mahony filter cannot starve out the way an
 * EKF can (no innovation gating; fixed Kp/Ki gains pull the quaternion
 * toward the accel/mag observation every step), so when the two
 * estimators diverge by more than a few degrees the EKF is the one
 * that is wrong.
 *
 * Reference: Mahony, Hamel, Pflimlin 2008 "Nonlinear Complementary
 * Filters on the Special Orthogonal Group" IEEE Trans. Automatic
 * Control 53(5).
 */

#ifndef EKF_MAHONY_H__
#define EKF_MAHONY_H__

#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Reset the Mahony state to identity quaternion and zero integral.    */
void ekf_mahony_reset(void);

/* Advance the Mahony filter by dt seconds using the latest INS_U.IMU  *
 * (and INS_U.MAG, if has_mag != 0 and the timestamp is non-zero).     *
 * Idempotent w.r.t. the main EKF - call once per INS_step.            */
void ekf_mahony_step(real32_T dt);

/* Current Mahony attitude quaternion (body->NED, w x y z, unit norm). */
const real32_T* ekf_mahony_quat(void);

/* Current Mahony Euler angles, radians (for cheap diagnostics).       */
void ekf_mahony_euler(real32_T* phi, real32_T* theta, real32_T* psi);

/* Angle (radians) between the Mahony quaternion and the EKF nominal   *
 * quaternion - small when both agree, large when one diverges.        */
real32_T ekf_mahony_disagreement_rad(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_MAHONY_H__ */

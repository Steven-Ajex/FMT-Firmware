/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_math.h
 *
 * Light-weight math helpers used by the EKF.  Vector / quaternion ops are
 * inlined; matrix routines wrap arm_math when available, and fall back to
 * plain loops otherwise (so the file still builds in unit tests on host).
 */

#ifndef EKF_MATH_H__
#define EKF_MATH_H__

#include "rtwtypes.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EKF_PI
#define EKF_PI 3.14159265358979323846f
#endif

/* ---------- scalar ---------- */
static inline real32_T ekf_clampf(real32_T x, real32_T lo, real32_T hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline real32_T ekf_wrap_pi(real32_T a)
{
    while (a >  EKF_PI) a -= 2.0f * EKF_PI;
    while (a < -EKF_PI) a += 2.0f * EKF_PI;
    return a;
}

/* ---------- 3-vector ---------- */
static inline void v3_zero(real32_T v[3])
{
    v[0] = v[1] = v[2] = 0.0f;
}

static inline void v3_copy(real32_T dst[3], const real32_T src[3])
{
    dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
}

static inline real32_T v3_norm(const real32_T v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static inline void v3_cross(real32_T r[3], const real32_T a[3], const real32_T b[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

/* ---------- quaternion (w, x, y, z) ---------- */
void  quat_normalize(real32_T q[4]);
void  quat_from_euler(real32_T q[4], real32_T phi, real32_T theta, real32_T psi);
void  quat_to_euler(const real32_T q[4], real32_T* phi, real32_T* theta, real32_T* psi);
/* DCM body-to-NED, row-major 3x3 */
void  quat_to_dcm(const real32_T q[4], real32_T R[9]);
/* Rotate a body-frame vector into NED:  v_NED = R(q) * v_B          */
void  quat_rotate_vec(const real32_T q[4], const real32_T v_B[3], real32_T v_N[3]);
/* Compose: q <- q * exp(0.5 * omega * dt)  (small-angle integration) */
void  quat_integrate(real32_T q[4], const real32_T omega[3], real32_T dt);
/* Compose: q <- q * exp(0.5 * alpha) where alpha is a body-frame rotation
 * vector for one step (already containing any coning correction).        */
void  quat_apply_rotvec(real32_T q[4], const real32_T alpha[3]);
/* Apply small-angle attitude error: q <- exp(0.5 * dtheta) * q       */
void  quat_inject_error(real32_T q[4], const real32_T dtheta[3]);

#ifdef __cplusplus
}
#endif

#endif /* EKF_MATH_H__ */

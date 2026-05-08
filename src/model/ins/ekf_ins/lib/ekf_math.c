/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_math.h"

void quat_normalize(real32_T q[4])
{
    real32_T n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1e-9f) {
        real32_T inv = 1.0f / n;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    }
}

void quat_from_euler(real32_T q[4], real32_T phi, real32_T theta, real32_T psi)
{
    real32_T cphi   = cosf(0.5f * phi),   sphi   = sinf(0.5f * phi);
    real32_T ctheta = cosf(0.5f * theta), stheta = sinf(0.5f * theta);
    real32_T cpsi   = cosf(0.5f * psi),   spsi   = sinf(0.5f * psi);

    q[0] = cphi * ctheta * cpsi + sphi * stheta * spsi;
    q[1] = sphi * ctheta * cpsi - cphi * stheta * spsi;
    q[2] = cphi * stheta * cpsi + sphi * ctheta * spsi;
    q[3] = cphi * ctheta * spsi - sphi * stheta * cpsi;
    quat_normalize(q);
}

void quat_to_euler(const real32_T q[4], real32_T* phi, real32_T* theta, real32_T* psi)
{
    real32_T qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    *phi   = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    real32_T s = 2.0f * (qw * qy - qz * qx);
    s = ekf_clampf(s, -1.0f, 1.0f);
    *theta = asinf(s);
    *psi   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

void quat_to_dcm(const real32_T q[4], real32_T R[9])
{
    real32_T qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    real32_T qxx = qx * qx, qyy = qy * qy, qzz = qz * qz;
    real32_T qxy = qx * qy, qxz = qx * qz, qyz = qy * qz;
    real32_T qwx = qw * qx, qwy = qw * qy, qwz = qw * qz;

    R[0] = 1.0f - 2.0f * (qyy + qzz);
    R[1] = 2.0f * (qxy - qwz);
    R[2] = 2.0f * (qxz + qwy);
    R[3] = 2.0f * (qxy + qwz);
    R[4] = 1.0f - 2.0f * (qxx + qzz);
    R[5] = 2.0f * (qyz - qwx);
    R[6] = 2.0f * (qxz - qwy);
    R[7] = 2.0f * (qyz + qwx);
    R[8] = 1.0f - 2.0f * (qxx + qyy);
}

void quat_rotate_vec(const real32_T q[4], const real32_T v_B[3], real32_T v_N[3])
{
    real32_T R[9];
    quat_to_dcm(q, R);
    v_N[0] = R[0] * v_B[0] + R[1] * v_B[1] + R[2] * v_B[2];
    v_N[1] = R[3] * v_B[0] + R[4] * v_B[1] + R[5] * v_B[2];
    v_N[2] = R[6] * v_B[0] + R[7] * v_B[1] + R[8] * v_B[2];
}

void quat_integrate(real32_T q[4], const real32_T omega[3], real32_T dt)
{
    /* First-order quaternion integration via the rotation vector formula:
     *   q <- q * dq, where dq = [cos(|w|dt/2), sin(|w|dt/2)/|w| * w] */
    real32_T wx = omega[0], wy = omega[1], wz = omega[2];
    real32_T wn = sqrtf(wx * wx + wy * wy + wz * wz);
    real32_T dqw, dqx, dqy, dqz;

    if (wn * dt < 1e-7f) {
        dqw = 1.0f;
        dqx = 0.5f * wx * dt;
        dqy = 0.5f * wy * dt;
        dqz = 0.5f * wz * dt;
    } else {
        real32_T half = 0.5f * wn * dt;
        real32_T s    = sinf(half) / wn;
        dqw = cosf(half);
        dqx = s * wx;
        dqy = s * wy;
        dqz = s * wz;
    }

    real32_T qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    q[0] = qw * dqw - qx * dqx - qy * dqy - qz * dqz;
    q[1] = qw * dqx + qx * dqw + qy * dqz - qz * dqy;
    q[2] = qw * dqy - qx * dqz + qy * dqw + qz * dqx;
    q[3] = qw * dqz + qx * dqy - qy * dqx + qz * dqw;
    quat_normalize(q);
}

void quat_inject_error(real32_T q[4], const real32_T dtheta[3])
{
    /* q <- exp(0.5 * dtheta) (X) q,  with the small-angle approximation
     * dq = [1, 0.5 * dtheta] then renormalize. */
    real32_T dqw = 1.0f;
    real32_T dqx = 0.5f * dtheta[0];
    real32_T dqy = 0.5f * dtheta[1];
    real32_T dqz = 0.5f * dtheta[2];

    real32_T qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    q[0] = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
    q[1] = dqw * qx + dqx * qw + dqy * qz - dqz * qy;
    q[2] = dqw * qy - dqx * qz + dqy * qw + dqz * qx;
    q[3] = dqw * qz + dqx * qy - dqy * qx + dqz * qw;
    quat_normalize(q);
}

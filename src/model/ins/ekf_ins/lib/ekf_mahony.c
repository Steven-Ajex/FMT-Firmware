/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_mahony.c
 *
 * Explicit complementary filter on SO(3) (Mahony, Hamel, Pflimlin 2008),
 * attitude only, running as a background reference observer alongside
 * the main EKF.  See ekf_mahony.h for rationale.
 *
 * Algorithm (per step, dt seconds):
 *   1. omega_b   = IMU gyro
 *   2. e_acc     = a_meas (normalised) x R^T(q) * [0,0,-1]    (tilt error)
 *   3. e_mag     = m_meas (horiz proj) x R^T(q) * b_mag_NED  (heading)
 *   4. e         = e_acc + e_mag
 *   5. b_int    += -Ki * e * dt                              (bias integ)
 *   6. omega_corr= omega_b + Kp * e + b_int
 *   7. q        <- q * Exp(0.5 * omega_corr * dt)            (rotvec)
 *
 * Tuning: Kp = 2.0, Ki = 0.005 (classic Madgwick reset defaults).
 * Both gains are scalar and global; no per-axis tuning - this is a
 * coarse reference, not a precise estimator.
 */

#include "ekf_mahony.h"
#include "ekf_state.h"
#include "ekf_math.h"
#include "INS.h"

#include <math.h>

#define MAHONY_KP   2.0f
#define MAHONY_KI   0.005f
#define MAHONY_G    9.80665f

static real32_T s_q[4]    = { 1.0f, 0.0f, 0.0f, 0.0f };
static real32_T s_bias[3] = { 0.0f, 0.0f, 0.0f };

void ekf_mahony_reset(void)
{
    s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
}

void ekf_mahony_step(real32_T dt)
{
    if (dt <= 0.0f) return;

    real32_T omega_b[3] = {
        INS_U.IMU.gyr_x, INS_U.IMU.gyr_y, INS_U.IMU.gyr_z
    };

    real32_T e[3] = { 0.0f, 0.0f, 0.0f };

    /* ---------- accel tilt feedback ---------- */
    real32_T ax = INS_U.IMU.acc_x;
    real32_T ay = INS_U.IMU.acc_y;
    real32_T az = INS_U.IMU.acc_z;
    real32_T an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 1.0f) {
        /* Body-frame expected gravity = R^T(q) * [0, 0, +g].  With body- *
         * to-NED q, R^T's columns are the body axes in NED; row k of    *
         * R^T is column k of R, and we only need the last column ([6..8]*
         * in our row-major storage):                                    */
        real32_T R[9];
        quat_to_dcm(s_q, R);
        /* g_body_predicted = R^T * [0, 0, g]; equivalently the third    *
         * column of R scaled by g (since R^T row k = R col k):          */
        real32_T gx_pred = R[2] * MAHONY_G;     /* R[0*3+2] */
        real32_T gy_pred = R[5] * MAHONY_G;     /* R[1*3+2] */
        real32_T gz_pred = R[8] * MAHONY_G;     /* R[2*3+2] */
        /* Measured specific force in body points opposite gravity in    *
         * steady state, i.e. -[ax,ay,az] should align with g_body_pred. *
         * Use the cross product of UNIT vectors as the small-angle      *
         * error proxy (Mahony Eq. 32).                                  */
        real32_T inv_n = 1.0f / an;
        real32_T mx =  -ax * inv_n;
        real32_T my =  -ay * inv_n;
        real32_T mz =  -az * inv_n;
        real32_T gn = sqrtf(gx_pred*gx_pred + gy_pred*gy_pred + gz_pred*gz_pred);
        if (gn > 1.0e-6f) {
            real32_T inv_gn = 1.0f / gn;
            gx_pred *= inv_gn;
            gy_pred *= inv_gn;
            gz_pred *= inv_gn;
            e[0] += my * gz_pred - mz * gy_pred;
            e[1] += mz * gx_pred - mx * gz_pred;
            e[2] += mx * gy_pred - my * gx_pred;
        }
    }

    /* ---------- mag heading feedback (only if mag sample present) ---------- */
    if (INS_U.MAG.timestamp != 0U) {
        real32_T mx_b = INS_U.MAG.mag_x;
        real32_T my_b = INS_U.MAG.mag_y;
        real32_T mz_b = INS_U.MAG.mag_z;
        real32_T mn = sqrtf(mx_b*mx_b + my_b*my_b + mz_b*mz_b);
        if (mn > 1.0e-6f) {
            real32_T inv_mn = 1.0f / mn;
            mx_b *= inv_mn; my_b *= inv_mn; mz_b *= inv_mn;
            /* Reference field in NED: horizontal, magnitude 1, along    *
             * the X-East-of-North axis after subtracting declination.   *
             * For simplicity assume the projection onto the NED         *
             * horizontal plane should be aligned with +N (= [1, 0, 0]). */
            real32_T R[9];
            quat_to_dcm(s_q, R);
            /* mag in NED predicted from body sample: R * m_body         */
            real32_T mn_x = R[0]*mx_b + R[1]*my_b + R[2]*mz_b;
            real32_T mn_y = R[3]*mx_b + R[4]*my_b + R[5]*mz_b;
            real32_T mn_z = R[6]*mx_b + R[7]*my_b + R[8]*mz_b;
            /* Reference: horizontal projection forced to point along +N */
            real32_T hor = sqrtf(mn_x*mn_x + mn_y*mn_y);
            if (hor > 1.0e-6f) {
                real32_T ref_x = hor;       /* + sign: bring heading to N */
                real32_T ref_y = 0.0f;
                real32_T ref_z = mn_z;
                /* Cross product in NED of measured-projected x reference */
                real32_T e_n_x = mn_y * ref_z - mn_z * ref_y;
                real32_T e_n_y = mn_z * ref_x - mn_x * ref_z;
                real32_T e_n_z = mn_x * ref_y - mn_y * ref_x;
                /* Rotate the NED error back into body frame (R^T * e_n) */
                real32_T e_b_x = R[0]*e_n_x + R[3]*e_n_y + R[6]*e_n_z;
                real32_T e_b_y = R[1]*e_n_x + R[4]*e_n_y + R[7]*e_n_z;
                real32_T e_b_z = R[2]*e_n_x + R[5]*e_n_y + R[8]*e_n_z;
                e[0] += e_b_x;
                e[1] += e_b_y;
                e[2] += e_b_z;
            }
        }
    }

    /* ---------- integral bias and gyro correction ---------- */
    s_bias[0] -= MAHONY_KI * e[0] * dt;
    s_bias[1] -= MAHONY_KI * e[1] * dt;
    s_bias[2] -= MAHONY_KI * e[2] * dt;
    real32_T omega_corr[3] = {
        omega_b[0] + MAHONY_KP * e[0] + s_bias[0],
        omega_b[1] + MAHONY_KP * e[1] + s_bias[1],
        omega_b[2] + MAHONY_KP * e[2] + s_bias[2],
    };

    /* ---------- quaternion integration ---------- */
    quat_integrate(s_q, omega_corr, dt);
}

const real32_T* ekf_mahony_quat(void) { return s_q; }

void ekf_mahony_euler(real32_T* phi, real32_T* theta, real32_T* psi)
{
    quat_to_euler(s_q, phi, theta, psi);
}

real32_T ekf_mahony_disagreement_rad(void)
{
    /* angle between two unit quaternions:  2 * acos(|<q1, q2>|)        */
    real32_T dot = s_q[0]*ekf.q[0] + s_q[1]*ekf.q[1]
                 + s_q[2]*ekf.q[2] + s_q[3]*ekf.q[3];
    if (dot < 0.0f) dot = -dot;
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * acosf(dot);
}

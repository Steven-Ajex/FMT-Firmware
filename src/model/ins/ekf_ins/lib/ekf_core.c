/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"

#define N EKF_NSTATES

/* Two scratch matrices (BSS, ~2.3 KB total) avoid pressure on the model
 * thread's stack, which is sized for ~4 KB on most targets. */
static real32_T s_F[N * N];
static real32_T s_T[N * N];

/* ------------------------------------------------------------------ */
/*  Maintenance                                                        */
/* ------------------------------------------------------------------ */
void ekf_symmetrize(void)
{
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            real32_T a = 0.5f * (ekf.P[i * N + j] + ekf.P[j * N + i]);
            ekf.P[i * N + j] = a;
            ekf.P[j * N + i] = a;
        }
    }
}

void ekf_clamp_diag(real32_T floor_value)
{
    for (int i = 0; i < N; i++) {
        if (ekf.P[i * N + i] < floor_value) {
            ekf.P[i * N + i] = floor_value;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Predict                                                            */
/*                                                                     */
/*  Nominal state propagation (Sola error-state KF formulation):       */
/*      omega_b   = omega_meas - bg                                    */
/*      a_b       = acc_meas   - ba                                    */
/*      a_NED     = R(q) * a_b + g_NED       (g_NED = [0,0,+9.80665])  */
/*      v_NED    += a_NED * dt                                         */
/*      p_NED    += v_NED * dt + 0.5 * a_NED * dt^2                    */
/*      q        <- q ⊕ exp(0.5 * omega_b * dt)                        */
/*                                                                     */
/*  Covariance propagation with the continuous-time error dynamics     */
/*  Fc, discretized as F = I + Fc*dt:                                  */
/*      Fc[POS, VEL] =  I_3                                            */
/*      Fc[VEL, ATT] = -[a_NED]_x                                      */
/*      Fc[VEL, BA ] = -R(q)                                           */
/*      Fc[ATT, ATT] = -[omega_b]_x                                    */
/*      Fc[ATT, BG ] = -I_3                                            */
/*  All other Fc blocks are zero.  P <- F*P*F' + Q*dt.                 */
/* ------------------------------------------------------------------ */
static void skew(real32_T S[9], const real32_T v[3])
{
    S[0] = 0.0f;   S[1] = -v[2]; S[2] =  v[1];
    S[3] =  v[2];  S[4] =  0.0f; S[5] = -v[0];
    S[6] = -v[1];  S[7] =  v[0]; S[8] =  0.0f;
}

void ekf_predict(const real32_T omega_meas[3],
                 const real32_T acc_meas[3],
                 real32_T dt)
{
    if (dt <= 0.0f) {
        return;
    }

    /* ---------- 1. nominal propagation ---------- */
    real32_T omega_b[3] = {
        omega_meas[0] - ekf.b_g[0],
        omega_meas[1] - ekf.b_g[1],
        omega_meas[2] - ekf.b_g[2],
    };
    real32_T a_b[3] = {
        acc_meas[0] - ekf.b_a[0],
        acc_meas[1] - ekf.b_a[1],
        acc_meas[2] - ekf.b_a[2],
    };

    real32_T R_b2n[9];
    quat_to_dcm(ekf.q, R_b2n);

    real32_T a_NED[3];
    a_NED[0] = R_b2n[0] * a_b[0] + R_b2n[1] * a_b[1] + R_b2n[2] * a_b[2];
    a_NED[1] = R_b2n[3] * a_b[0] + R_b2n[4] * a_b[1] + R_b2n[5] * a_b[2];
    a_NED[2] = R_b2n[6] * a_b[0] + R_b2n[7] * a_b[1] + R_b2n[8] * a_b[2] + 9.80665f;

    real32_T v_prev[3];
    v3_copy(v_prev, ekf.v_NED);

    ekf.v_NED[0] += a_NED[0] * dt;
    ekf.v_NED[1] += a_NED[1] * dt;
    ekf.v_NED[2] += a_NED[2] * dt;

    /* trapezoidal position update for second-order accuracy */
    ekf.p_NED[0] += 0.5f * (v_prev[0] + ekf.v_NED[0]) * dt;
    ekf.p_NED[1] += 0.5f * (v_prev[1] + ekf.v_NED[1]) * dt;
    ekf.p_NED[2] += 0.5f * (v_prev[2] + ekf.v_NED[2]) * dt;

    quat_integrate(ekf.q, omega_b, dt);

    /* ---------- 2. build F = I + Fc*dt ---------- */
    real32_T* F = s_F;
    for (int i = 0; i < N * N; i++) F[i] = 0.0f;
    for (int i = 0; i < N; i++) F[i * N + i] = 1.0f;

    /* dPos/dVel = I */
    for (int i = 0; i < 3; i++) F[(EKF_X_PN + i) * N + (EKF_X_VN + i)] = dt;

    /* dVel/dTheta = -[a_NED]_x */
    real32_T S[9];
    skew(S, a_NED);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[(EKF_X_VN + i) * N + (EKF_X_DTHX + j)] = -S[i * 3 + j] * dt;
        }
    }

    /* dVel/dBa = -R(q) */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[(EKF_X_VN + i) * N + (EKF_X_BAX + j)] = -R_b2n[i * 3 + j] * dt;
        }
    }

    /* dTheta/dTheta = I - [omega_b]_x * dt */
    skew(S, omega_b);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[(EKF_X_DTHX + i) * N + (EKF_X_DTHX + j)] -= S[i * 3 + j] * dt;
        }
    }

    /* dTheta/dBg = -I */
    for (int i = 0; i < 3; i++) F[(EKF_X_DTHX + i) * N + (EKF_X_BGX + i)] = -dt;

    /* ---------- 3. P <- F * P * F' ---------- */
    real32_T* T = s_T;
    /* T = F * P */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            real32_T s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += F[i * N + k] * ekf.P[k * N + j];
            }
            T[i * N + j] = s;
        }
    }
    /* P = T * F' */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            real32_T s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += T[i * N + k] * F[j * N + k];
            }
            ekf.P[i * N + j] = s;
        }
    }

    /* ---------- 4. add discrete process noise Q*dt ---------- */
    real32_T sg2  = INS_PARAM.EKF_GYR_NOISE    * INS_PARAM.EKF_GYR_NOISE    * dt;
    real32_T sa2  = INS_PARAM.EKF_ACC_NOISE    * INS_PARAM.EKF_ACC_NOISE    * dt;
    real32_T sbg2 = INS_PARAM.EKF_BG_NOISE     * INS_PARAM.EKF_BG_NOISE     * dt;
    real32_T sba2 = INS_PARAM.EKF_BA_NOISE     * INS_PARAM.EKF_BA_NOISE     * dt;
    real32_T sb2  = INS_PARAM.EKF_BARO_B_NOISE * INS_PARAM.EKF_BARO_B_NOISE * dt;
    real32_T st2  = INS_PARAM.EKF_TERR_NOISE   * INS_PARAM.EKF_TERR_NOISE   * dt;

    for (int i = 0; i < 3; i++) {
        ekf.P[(EKF_X_VN + i)   * N + (EKF_X_VN + i)]   += sa2;   /* mapped via R yet diag-equivalent */
        ekf.P[(EKF_X_DTHX + i) * N + (EKF_X_DTHX + i)] += sg2;
        ekf.P[(EKF_X_BGX + i)  * N + (EKF_X_BGX + i)]  += sbg2;
        ekf.P[(EKF_X_BAX + i)  * N + (EKF_X_BAX + i)]  += sba2;
    }
    ekf.P[EKF_X_BARO_B * N + EKF_X_BARO_B] += sb2;
    ekf.P[EKF_X_TERR   * N + EKF_X_TERR]   += st2;

    ekf_symmetrize();
    ekf_clamp_diag(1.0e-9f);
}

/* ------------------------------------------------------------------ */
/*  Scalar Kalman update                                               */
/* ------------------------------------------------------------------ */
int ekf_update_scalar(const real32_T H[N],
                      real32_T innov,
                      real32_T R,
                      real32_T gate,
                      real32_T dx_out[N])
{
    real32_T HP[N];     /* 1 x N row vector  H * P                */
    real32_T K[N];      /* N x 1 Kalman gain                       */

    /* HP[j] = sum_k H[k] * P[k,j] */
    for (int j = 0; j < N; j++) {
        real32_T s = 0.0f;
        for (int k = 0; k < N; k++) {
            s += H[k] * ekf.P[k * N + j];
        }
        HP[j] = s;
    }

    /* S = H * P * H' + R = HP * H' + R */
    real32_T S = R;
    for (int k = 0; k < N; k++) S += HP[k] * H[k];

    if (S <= 0.0f) {
        return 0;
    }

    /* gate */
    if (gate > 0.0f) {
        if (innov * innov > gate * gate * S) {
            return 0;
        }
    }

    real32_T inv_S = 1.0f / S;

    /* K = P * H' / S */
    for (int i = 0; i < N; i++) {
        real32_T s = 0.0f;
        for (int k = 0; k < N; k++) {
            s += ekf.P[i * N + k] * H[k];
        }
        K[i] = s * inv_S;
    }

    /* P -= K * HP   (rank-1) */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            ekf.P[i * N + j] -= K[i] * HP[j];
        }
    }
    ekf_symmetrize();
    ekf_clamp_diag(1.0e-9f);

    if (dx_out != NULL) {
        for (int i = 0; i < N; i++) dx_out[i] = K[i] * innov;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Inject error-state into nominal state                              */
/* ------------------------------------------------------------------ */
void ekf_inject_error(const real32_T dx[N])
{
    /* position / velocity (additive) */
    ekf.p_NED[0] += dx[EKF_X_PN];
    ekf.p_NED[1] += dx[EKF_X_PE];
    ekf.p_NED[2] += dx[EKF_X_PD];
    ekf.v_NED[0] += dx[EKF_X_VN];
    ekf.v_NED[1] += dx[EKF_X_VE];
    ekf.v_NED[2] += dx[EKF_X_VD];

    /* attitude (small-angle injection) */
    real32_T dtheta[3] = { dx[EKF_X_DTHX], dx[EKF_X_DTHY], dx[EKF_X_DTHZ] };
    quat_inject_error(ekf.q, dtheta);

    /* biases (additive) */
    ekf.b_g[0] += dx[EKF_X_BGX];
    ekf.b_g[1] += dx[EKF_X_BGY];
    ekf.b_g[2] += dx[EKF_X_BGZ];
    ekf.b_a[0] += dx[EKF_X_BAX];
    ekf.b_a[1] += dx[EKF_X_BAY];
    ekf.b_a[2] += dx[EKF_X_BAZ];

    /* augmented states */
    ekf.baro_b += dx[EKF_X_BARO_B];
    ekf.terr_d += dx[EKF_X_TERR];
}

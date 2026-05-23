/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_core.h"
#include "ekf_state.h"
#include "ekf_math.h"
#include <math.h>

#define N EKF_NSTATES

/* Two NxN scratch matrices in BSS (~2.3 KB).  Used by the Thornton
 * temporal update in ekf_predict:
 *   s_W : holds W = F * U (initially), then transformed in place by the
 *         Modified Weighted Gram-Schmidt to produce the new U factor.
 *   s_V : holds the augmented identity matrix tracking the diagonal
 *         process-noise contributions through the MWGS, also updated
 *         in place.  Together (W | V) play the role of the augmented
 *         [F*U | I] matrix in the standard Thornton algorithm.        */
static real32_T s_W[N * N];
static real32_T s_V[N * N];

/* Innovation hook (NULL in firmware; replay binary installs a writer). */
static ekf_innov_cb_t s_innov_cb  = NULL;
static const char*    s_innov_tag = "?";

/* Forward decl: defined after k_innov_tags.  Used by ekf_set_innov_tag */
int ekf_innov_tag_to_id(const char* tag);

/* Per-tag rejection counters for adaptive-gating logic (Path A).        *
 * Sized for the tag table below; helpers are defined further down.     */
#define EKF_FAIL_SOFT_THRESH    30   /* widen gate 2x past this           */
#define EKF_FAIL_MED_THRESH     60   /* widen 4x past this                */
#define EKF_FAIL_HARD_THRESH   100   /* widen 10x past this; flag fault   */
#define EKF_FAIL_GATE_MULT_MAX  10.0f

static int      s_fail_count[19];   /* must match EKF_INNOV_TAG_COUNT     */
static uint32_T s_fault_mask = 0u;
static int      s_current_tag_id = -1;

void ekf_set_innov_cb(ekf_innov_cb_t cb) { s_innov_cb = cb; }
void ekf_set_innov_tag(const char* tag)  {
    s_innov_tag = tag ? tag : "?";
    s_current_tag_id = ekf_innov_tag_to_id(s_innov_tag);
}

/* Order here defines the integer id that the INS_Innov mlog bus stores.
 * Append-only: do not insert in the middle, downstream parsers depend
 * on these slot numbers.                                               */
static const char* const k_innov_tags[] = {
    "mag",          /*  0 */
    "grav_x",       /*  1 */
    "grav_y",       /*  2 */
    "gps_pos_n",    /*  3 */
    "gps_pos_e",    /*  4 */
    "gps_pos_d",    /*  5 */
    "gps_vel_n",    /*  6 */
    "gps_vel_e",    /*  7 */
    "gps_vel_d",    /*  8 */
    "baro",         /*  9 */
    "rf",           /* 10 */
    "opf_x",        /* 11 */
    "opf_y",        /* 12 */
    "ext_x",        /* 13 */
    "ext_y",        /* 14 */
    "ext_z",        /* 15 */
    "ext_phi",      /* 16 */
    "ext_theta",    /* 17 */
    "ext_psi",      /* 18 */
};
#define EKF_INNOV_TAG_COUNT ((int)(sizeof(k_innov_tags) / sizeof(k_innov_tags[0])))

int ekf_innov_tag_count(void) { return EKF_INNOV_TAG_COUNT; }

int ekf_innov_tag_to_id(const char* tag)
{
    if (tag == NULL) return -1;
    for (int i = 0; i < EKF_INNOV_TAG_COUNT; i++) {
        if (strcmp(k_innov_tags[i], tag) == 0) return i;
    }
    return -1;
}

const char* ekf_innov_tag_from_id(int id)
{
    if (id < 0 || id >= EKF_INNOV_TAG_COUNT) return "?";
    return k_innov_tags[id];
}

int ekf_innov_fail_count(int tag_id)
{
    if (tag_id < 0 || tag_id >= EKF_INNOV_TAG_COUNT) return 0;
    return s_fail_count[tag_id];
}

uint32_T ekf_fault_mask(void) { return s_fault_mask; }

void ekf_fault_clear(int tag_id)
{
    if (tag_id < 0 || tag_id >= EKF_INNOV_TAG_COUNT) return;
    s_fail_count[tag_id] = 0;
    s_fault_mask &= ~(1u << tag_id);
}

static real32_T adaptive_gate(real32_T base_gate, int fail_count)
{
    if (base_gate <= 0.0f) return base_gate;     /* gating disabled */
    real32_T mult = 1.0f;
    if (fail_count > EKF_FAIL_HARD_THRESH)      mult = EKF_FAIL_GATE_MULT_MAX;
    else if (fail_count > EKF_FAIL_MED_THRESH)  mult = 4.0f;
    else if (fail_count > EKF_FAIL_SOFT_THRESH) mult = 2.0f;
    return base_gate * mult;
}

/* ------------------------------------------------------------------ */
/*  Maintenance                                                        */
/*                                                                     */
/*  In the UDU' formulation the covariance is structurally symmetric   */
/*  (P = U*diag(D)*U^T) and positive-semi-definite (D >= 0 for any     */
/*  numerical perturbation that keeps D non-negative).  The old        */
/*  symmetrize / clamp_floor scaffolding therefore reduces to ensuring */
/*  D itself stays positive.  ekf_symmetrize is retained as a no-op    */
/*  so external callers (if any) keep linking.                         */
/* ------------------------------------------------------------------ */
void ekf_symmetrize(void)
{
    /* UDU' factorisation is symmetric by construction. */
}

void ekf_clamp_diag(real32_T floor_value)
{
    for (int i = 0; i < N; i++) {
        if (ekf.D[i] < floor_value) ekf.D[i] = floor_value;
    }
}

/* Per-state-type covariance floors.  These guard against pathological
 * driving of D[i] into the noise level (e.g. an overconfident sensor
 * fusion chain) and mirror the same units (sigma^2) as the previous
 * P-diagonal-based implementation.  See ekf_state_idx_t for indices.
 *
 *   pos      sigma >= 1   cm   (var 1e-4 m^2)
 *   vel      sigma >= 1   cm/s (var 1e-4 m^2/s^2)
 *   dtheta   sigma >= 0.1 mrad (var 1e-8 rad^2)  - mostly avoids zeros
 *   bg       sigma >= 0.03 mrad/s
 *   ba       sigma >= 1   mm/s^2
 *   baro_b   sigma >= 10  cm
 *   terr_d   sigma >= 20  cm
 */
void ekf_clamp_floor(void)
{
    static const real32_T FLOOR[EKF_NSTATES] = {
        1.0e-4f, 1.0e-4f, 1.0e-4f,    /* pos    */
        1.0e-4f, 1.0e-4f, 1.0e-4f,    /* vel    */
        1.0e-8f, 1.0e-8f, 1.0e-8f,    /* dtheta */
        1.0e-9f, 1.0e-9f, 1.0e-9f,    /* bg     */
        1.0e-6f, 1.0e-6f, 1.0e-6f,    /* ba     */
        1.0e-2f,                      /* baro_b */
        4.0e-2f,                      /* terr_d */
    };
    for (int i = 0; i < N; i++) {
        if (ekf.D[i] < FLOOR[i]) ekf.D[i] = FLOOR[i];
    }
}

/* ------------------------------------------------------------------ */
/*  Predict                                                            */
/*                                                                     */
/*  Nominal state propagation (Sola error-state KF formulation with    */
/*  Savage two-sample coning / sculling, Savage 1998 Parts 1-2):       */
/*    Inputs at step k:                                                */
/*      omega_b   = omega_meas - bg                                    */
/*      a_b       = acc_meas   - ba                                    */
/*    Two-sample rotation increment with coning correction:            */
/*      alpha = 0.5*(omega_b_prev + omega_b)*dt                        */
/*            + (dt^2/12) * (omega_b_prev x omega_b)                   */
/*    Two-sample body specific-force increment with sculling + the     */
/*    rotation/translation (scrolling) cross term:                     */
/*      dv_b  = 0.5*(a_b_prev + a_b)*dt                                */
/*            + (dt^2/12) * (omega_b_prev x a_b + a_b_prev x omega_b)  */
/*            + 0.5 * (alpha x dv_b_trapz)                             */
/*    Then v_NED += R(q)*dv_b + g_NED*dt,  q <- q * exp(0.5*alpha).    */
/*    p_NED uses the NED trapezoidal rule, which is already 2nd order  */
/*    given the new v_NED.                                             */
/*                                                                     */
/*  Covariance propagation:  F = I + Fc*dt + 0.5*Fc^2*dt^2 (2nd-order  */
/*  Taylor of the matrix exponential).  Fc is sparse:                  */
/*      Fc[POS, VEL] =  I_3                                            */
/*      Fc[VEL, ATT] = -[a_NED]_x                                      */
/*      Fc[VEL, BA ] = -R(q)                                           */
/*      Fc[ATT, ATT] = -[omega_b]_x                                    */
/*      Fc[ATT, BG ] = -I_3                                            */
/*  Fc^2 adds three new block couplings: POS<->ATT, POS<->BA,          */
/*  VEL<->BG, plus diagonal corrections inside ATT<->ATT and ATT<->BG. */
/*  All non-zero blocks of F still live in rows POS/VEL/ATT and        */
/*  columns POS/VEL/ATT (after F1' is applied), so the block-sparse    */
/*  P <- F*P*F' propagation has the same data-flow as the first-order  */
/*  variant - only the inner formulas grow.   P <- F*P*F' + Q*dt.      */
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

    /* ---------- 1. nominal propagation (Savage two-sample) ---------- */
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

    /* DCM at the start of the step - used both to rotate the body-frame
     * specific-force increment into NED and as the linearisation point   *
     * for the F Jacobian below. */
    real32_T R_b2n[9];
    quat_to_dcm(ekf.q, R_b2n);

    /* Two-sample rotation increment alpha = integral omega dt with the
     * Savage coning correction:                                          *
     *   alpha = 0.5*(omega_prev + omega) * dt + (dt^2/12) * omega_prev x omega
     * On the very first step omega_prev is zero, so the correction term  *
     * vanishes - we degrade gracefully to first-order, never wrong.      */
    real32_T omega_cross[3];
    v3_cross(omega_cross, ekf.omega_prev, omega_b);
    real32_T dt2_12 = dt * dt * (1.0f / 12.0f);
    real32_T alpha[3];
    for (int i = 0; i < 3; i++) {
        alpha[i] = 0.5f * (ekf.omega_prev[i] + omega_b[i]) * dt
                 + dt2_12 * omega_cross[i];
    }

    /* Body-frame specific-force increment:                                *
     *   dv_b = trapezoidal_part + sculling + scrolling                    *
     * - trapezoidal:  0.5*(a_prev + a)*dt                                 *
     * - sculling:     (dt^2/12)*(omega_prev x a + a_prev x omega)         *
     *   (corrects coupling between rotation and translation over the step)*
     * - scrolling:    0.5 * alpha x trapezoidal_part                      *
     *   (corrects the fact that the frame rotates during the step) */
    real32_T dv_trapz[3];
    for (int i = 0; i < 3; i++) {
        dv_trapz[i] = 0.5f * (ekf.acc_prev[i] + a_b[i]) * dt;
    }
    real32_T tmp1[3], tmp2[3], dv_scull[3], dv_scroll[3];
    v3_cross(tmp1, ekf.omega_prev, a_b);
    v3_cross(tmp2, ekf.acc_prev,   omega_b);
    for (int i = 0; i < 3; i++) {
        dv_scull[i] = dt2_12 * (tmp1[i] + tmp2[i]);
    }
    v3_cross(dv_scroll, alpha, dv_trapz);
    real32_T dv_body[3];
    for (int i = 0; i < 3; i++) {
        dv_body[i] = dv_trapz[i] + dv_scull[i] + 0.5f * dv_scroll[i];
    }

    /* Rotate the body-frame increment into NED using the start-of-step
     * DCM, then add gravity contribution.                                 */
    real32_T dv_NED[3];
    dv_NED[0] = R_b2n[0]*dv_body[0] + R_b2n[1]*dv_body[1] + R_b2n[2]*dv_body[2];
    dv_NED[1] = R_b2n[3]*dv_body[0] + R_b2n[4]*dv_body[1] + R_b2n[5]*dv_body[2];
    dv_NED[2] = R_b2n[6]*dv_body[0] + R_b2n[7]*dv_body[1] + R_b2n[8]*dv_body[2]
              + 9.80665f * dt;

    real32_T v_prev[3];
    v3_copy(v_prev, ekf.v_NED);
    ekf.v_NED[0] += dv_NED[0];
    ekf.v_NED[1] += dv_NED[1];
    ekf.v_NED[2] += dv_NED[2];

    /* Trapezoidal position update for second-order accuracy */
    ekf.p_NED[0] += 0.5f * (v_prev[0] + ekf.v_NED[0]) * dt;
    ekf.p_NED[1] += 0.5f * (v_prev[1] + ekf.v_NED[1]) * dt;
    ekf.p_NED[2] += 0.5f * (v_prev[2] + ekf.v_NED[2]) * dt;

    /* Apply the coning-corrected rotation vector to the quaternion */
    quat_apply_rotvec(ekf.q, alpha);

    /* a_NED used only as the linearisation point for the F Jacobian -
     * the leading-order specific force in NED.  Coning / sculling terms
     * are higher-order corrections in dt and are absorbed into the
     * process noise Q rather than tracked through F.                     */
    real32_T a_NED[3];
    a_NED[0] = R_b2n[0] * a_b[0] + R_b2n[1] * a_b[1] + R_b2n[2] * a_b[2];
    a_NED[1] = R_b2n[3] * a_b[0] + R_b2n[4] * a_b[1] + R_b2n[5] * a_b[2];
    a_NED[2] = R_b2n[6] * a_b[0] + R_b2n[7] * a_b[1] + R_b2n[8] * a_b[2] + 9.80665f;

    /* Store current samples for next step's two-sample integration */
    v3_copy(ekf.omega_prev, omega_b);
    v3_copy(ekf.acc_prev,   a_b);

    /* ---------- 2. precompute the 3x3 dense blocks of F1 = F - I ----- */
    /* F = I + Fc*dt + 0.5*Fc^2*dt^2.  F1 = F - I has 8 non-zero blocks  */
    /* (with A = dt*[a]_x, Wsk = dt*[w]_x, Rd = dt*R):                   */
    /*   B_PV = dt*I                  (POS, VEL)  - inline scalar         */
    /*   B_PA = -(dt/2) * A           (POS, ATT)  - NEW from Fc^2         */
    /*   B_PB = -(dt/2) * Rd          (POS, BA )  - NEW from Fc^2         */
    /*   B_VA = -A + 0.5 * (A*Wsk)    (VEL, ATT)  - augmented from Fc^2   */
    /*   B_VG = +(dt/2) * A           (VEL, BG )  - NEW from Fc^2         */
    /*   B_VB = -Rd                   (VEL, BA )                          */
    /*   B_AA = -Wsk + 0.5 * (Wsk*Wsk)(ATT, ATT)  - augmented from Fc^2   */
    /*   B_AG = -dt*I + (dt/2)*Wsk    (ATT, BG )  - off-diag from Fc^2    */
    real32_T A[9], Wsk[9], Rd[9];
    skew(A,   a_NED);
    skew(Wsk, omega_b);
    for (int k = 0; k < 9; k++) {
        A[k]   *= dt;
        Wsk[k] *= dt;
        Rd[k]   = dt * R_b2n[k];
    }

    real32_T AW[9], WW[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            real32_T sa = 0.0f, sw = 0.0f;
            for (int k = 0; k < 3; k++) {
                sa += A[i * 3 + k]   * Wsk[k * 3 + j];
                sw += Wsk[i * 3 + k] * Wsk[k * 3 + j];
            }
            AW[i * 3 + j] = sa;
            WW[i * 3 + j] = sw;
        }
    }

    real32_T half_dt = 0.5f * dt;
    real32_T B_PA[9], B_PB[9], B_VA[9], B_VG[9], B_VB[9], B_AA[9], B_AG[9];
    for (int k = 0; k < 9; k++) {
        B_PA[k] = -half_dt * A[k];
        B_PB[k] = -half_dt * Rd[k];
        B_VA[k] = -A[k]    + 0.5f * AW[k];
        B_VG[k] =  half_dt * A[k];
        B_VB[k] = -Rd[k];
        B_AA[k] = -Wsk[k]  + 0.5f * WW[k];
        B_AG[k] =  half_dt * Wsk[k];
    }
    B_AG[0] -= dt; B_AG[4] -= dt; B_AG[8] -= dt;

    /* ---------- 3. UDU' temporal update (Thornton MWGS) -------------- */
    /* Build W = F * U using block-sparse F (the inner formulas are the  */
    /* same as the F*P block-sparse Phase A from the conventional form,  */
    /* just with U taking the place of P).                               */
    real32_T* W = s_W;
    for (int j = 0; j < N; j++) {
        for (int a = 0; a < 3; a++) {
            real32_T s = ekf.U[(EKF_X_PN + a) * N + j]
                       + dt * ekf.U[(EKF_X_VN + a) * N + j];
            for (int b = 0; b < 3; b++) {
                s += B_PA[a * 3 + b] * ekf.U[(EKF_X_DTHX + b) * N + j];
                s += B_PB[a * 3 + b] * ekf.U[(EKF_X_BAX  + b) * N + j];
            }
            W[(EKF_X_PN + a) * N + j] = s;
        }
        for (int a = 0; a < 3; a++) {
            real32_T s = ekf.U[(EKF_X_VN + a) * N + j];
            for (int b = 0; b < 3; b++) {
                s += B_VA[a * 3 + b] * ekf.U[(EKF_X_DTHX + b) * N + j];
                s += B_VG[a * 3 + b] * ekf.U[(EKF_X_BGX  + b) * N + j];
                s += B_VB[a * 3 + b] * ekf.U[(EKF_X_BAX  + b) * N + j];
            }
            W[(EKF_X_VN + a) * N + j] = s;
        }
        for (int a = 0; a < 3; a++) {
            real32_T s = ekf.U[(EKF_X_DTHX + a) * N + j];
            for (int b = 0; b < 3; b++) {
                s += B_AA[a * 3 + b] * ekf.U[(EKF_X_DTHX + b) * N + j];
                s += B_AG[a * 3 + b] * ekf.U[(EKF_X_BGX  + b) * N + j];
            }
            W[(EKF_X_DTHX + a) * N + j] = s;
        }
    }
    memcpy(&W[EKF_X_BGX * N], &ekf.U[EKF_X_BGX * N],
           (size_t)(N - EKF_X_BGX) * N * sizeof(real32_T));

    /* V = I  (augmenting identity for the diagonal process noise Q_d)   */
    real32_T* V = s_V;
    for (int k = 0; k < N * N; k++) V[k] = 0.0f;
    for (int k = 0; k < N; k++)     V[k * N + k] = 1.0f;

    /* Snapshot D before the MWGS overwrites it row by row.              */
    real32_T D_old[N];
    for (int k = 0; k < N; k++) D_old[k] = ekf.D[k];

    /* Q_d: discrete-time process noise variance per state (diagonal).   */
    real32_T sg2  = INS_PARAM.EKF_GYR_NOISE    * INS_PARAM.EKF_GYR_NOISE    * dt;
    real32_T sa2  = INS_PARAM.EKF_ACC_NOISE    * INS_PARAM.EKF_ACC_NOISE    * dt;
    real32_T sbg2 = INS_PARAM.EKF_BG_NOISE     * INS_PARAM.EKF_BG_NOISE     * dt;
    real32_T sba2 = INS_PARAM.EKF_BA_NOISE     * INS_PARAM.EKF_BA_NOISE     * dt;
    real32_T sb2  = INS_PARAM.EKF_BARO_B_NOISE * INS_PARAM.EKF_BARO_B_NOISE * dt;
    real32_T st2  = INS_PARAM.EKF_TERR_NOISE   * INS_PARAM.EKF_TERR_NOISE   * dt;
    real32_T Q_d[N];
    for (int k = 0; k < N; k++) Q_d[k] = 0.0f;
    for (int k = 0; k < 3; k++) {
        Q_d[EKF_X_VN   + k] = sa2;
        Q_d[EKF_X_DTHX + k] = sg2;
        Q_d[EKF_X_BGX  + k] = sbg2;
        Q_d[EKF_X_BAX  + k] = sba2;
    }
    Q_d[EKF_X_BARO_B] = sb2;
    Q_d[EKF_X_TERR]   = st2;

    /* Thornton Modified Weighted Gram-Schmidt over (W | V) with weights *
     * (D_old | Q_d).  Produces U_new (upper unit triangular) and D_new  *
     * such that  U_new * diag(D_new) * U_new^T = F*P*F^T + G*Q*G^T.     *
     * Reference: Thornton 1976, Bierman 1977 ch.7.                      */
    for (int j = N - 1; j >= 0; j--) {
        real32_T sigma = 0.0f;
        for (int k = 0; k < N; k++) {
            sigma += D_old[k] * W[j * N + k] * W[j * N + k];
            sigma += Q_d[k]   * V[j * N + k] * V[j * N + k];
        }
        ekf.D[j] = sigma;
        ekf.U[j * N + j] = 1.0f;

        if (sigma > 1.0e-30f) {
            real32_T inv_sigma = 1.0f / sigma;
            for (int i = 0; i < j; i++) {
                real32_T num = 0.0f;
                for (int k = 0; k < N; k++) {
                    num += D_old[k] * W[i * N + k] * W[j * N + k];
                    num += Q_d[k]   * V[i * N + k] * V[j * N + k];
                }
                real32_T u_ij = num * inv_sigma;
                ekf.U[i * N + j] = u_ij;
                for (int k = 0; k < N; k++) {
                    W[i * N + k] -= u_ij * W[j * N + k];
                    V[i * N + k] -= u_ij * V[j * N + k];
                }
            }
        } else {
            /* Degenerate sigma; drop column to keep U well-defined.     */
            for (int i = 0; i < j; i++) ekf.U[i * N + j] = 0.0f;
        }
    }
    /* Strictly lower triangle of U is unused but kept at zero for clean *
     * dumps and the ekf_P_diag helper.                                  */
    for (int i = 1; i < N; i++) {
        for (int j = 0; j < i; j++) ekf.U[i * N + j] = 0.0f;
    }

    ekf_clamp_diag(1.0e-9f);
    ekf_clamp_floor();
}

/* ------------------------------------------------------------------ */
/*  Scalar Bierman measurement update                                  */
/*                                                                     */
/*  Operates directly on the UDU' factorisation:                       */
/*    1.  a = U^T * H^T          (column, N)                           */
/*    2.  b = diag(D) * a        (column, N)                           */
/*    3.  alpha_j running sum    (innovation variance S at the end)    */
/*    4.  Forward sweep over j updating D[j] and column j of U so that */
/*        P_post = U_new * diag(D_new) * U_new^T = (I - K H) * P_prior.*/
/*  Reference: Bierman 1977, "Factorization Methods for Discrete       */
/*  Sequential Estimation"; Grewal & Andrews, "Kalman Filtering:       */
/*  Theory and Practice" (4th ed.) Algorithm 6.14.                     */
/*                                                                     */
/*  Numerical robustness: the factor form keeps D >= 0 by construction,*/
/*  removing the need for the ad-hoc clamp_floor scaffolding that the  */
/*  conventional formulation relied on (the table is still applied at  */
/*  the end as a final safety net but it should never trip in practice)*/
int ekf_update_scalar(const real32_T H[N],
                      real32_T innov,
                      real32_T R,
                      real32_T gate,
                      real32_T dx_out[N])
{
    /* a = U^T * H^T:   a[i] = H[i] + sum_{k<i} U[k,i] * H[k]            */
    real32_T a[N];
    for (int i = 0; i < N; i++) {
        real32_T s = H[i];        /* U[i,i] = 1 */
        for (int k = 0; k < i; k++) s += ekf.U[k * N + i] * H[k];
        a[i] = s;
    }
    /* b = D * a (elementwise)                                           */
    real32_T b[N];
    for (int i = 0; i < N; i++) b[i] = ekf.D[i] * a[i];

    /* Innovation variance S = R + a^T * b  (precomputed for gating).    */
    real32_T S = R;
    for (int i = 0; i < N; i++) S += a[i] * b[i];

    if (S <= 0.0f) {
        if (s_innov_cb) s_innov_cb(s_innov_tag, innov, R, S, 0, 0U);
        return 0;
    }

    /* Adaptive gating (Path A): widen the gate if this tag has been      *
     * gated out consecutively many times, to break filter starvation.    *
     * An accepted update zeroes the counter on the way out.              */
    int      tag_idx = s_current_tag_id;
    real32_T eff_gate = gate;
    if (tag_idx >= 0) eff_gate = adaptive_gate(gate, s_fail_count[tag_idx]);

    int accepted = 1;
    if (eff_gate > 0.0f && innov * innov > eff_gate * eff_gate * S) accepted = 0;
    if (s_innov_cb) {
        extern INS_U_T INS_U;
        s_innov_cb(s_innov_tag, innov, R, S, accepted, INS_U.IMU.timestamp);
    }
    if (!accepted) {
        if (tag_idx >= 0) {
            s_fail_count[tag_idx]++;
            if (s_fail_count[tag_idx] > EKF_FAIL_HARD_THRESH) {
                s_fault_mask |= (1u << tag_idx);
            }
        }
        return 0;
    }
    if (tag_idx >= 0) s_fail_count[tag_idx] = 0;

    /* Forward sweep: maintain alpha_run as the running prefix sum       *
     *   alpha_run^{(j)} = R + sum_{k<=j} a[k]*b[k],                     *
     * and K_accum[i] as the accumulated U*b "numerator" of the gain.    */
    real32_T K_accum[N];
    real32_T alpha_run = R;
    real32_T alpha_old;

    alpha_old   = alpha_run;
    alpha_run  += a[0] * b[0];
    ekf.D[0]    = ekf.D[0] * alpha_old / alpha_run;
    K_accum[0]  = b[0];

    for (int j = 1; j < N; j++) {
        alpha_old   = alpha_run;
        alpha_run  += a[j] * b[j];
        ekf.D[j]    = ekf.D[j] * alpha_old / alpha_run;
        real32_T gamma_j = -a[j] / alpha_old;

        for (int i = 0; i < j; i++) {
            real32_T u_old   = ekf.U[i * N + j];
            ekf.U[i * N + j] = u_old + gamma_j * K_accum[i];
            K_accum[i]      += u_old * b[j];
        }
        K_accum[j] = b[j];
    }

    ekf_clamp_diag(1.0e-9f);

    if (dx_out != NULL) {
        real32_T inv_S = 1.0f / S;
        for (int i = 0; i < N; i++) dx_out[i] = K_accum[i] * innov * inv_S;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Iterated EKF scalar update (IES-EKF)                               */
/*                                                                     */
/*  Fixed-point iteration as documented in ekf_core.h:                 */
/*    delta_x = 0                                                      */
/*    loop:                                                            */
/*      (H, h_pred) = cb(ctx, delta_x)                                 */
/*      K           = P*H' / (H*P*H' + R)                              */
/*      step        = K * ((z - h_pred) - H * delta_x)                 */
/*      delta_x    += step                                             */
/*      if |step|^2 < tol^2: break                                     */
/*                                                                     */
/*  At convergence we redo HP / S / K with the final H once more so    */
/*  that the Joseph covariance update is consistent with the final     */
/*  linearisation point.  Gate is checked against the initial          */
/*  innovation (the pre-iteration normalised residual) - rejecting     */
/*  outliers earlier than the iteration converges to them.             */
/* ------------------------------------------------------------------ */
int ekf_update_scalar_iterated(ekf_obs_cb_t cb,
                               void*        ctx,
                               real32_T     z,
                               real32_T     R,
                               real32_T     gate,
                               real32_T     dx_out[N],
                               int          max_iter,
                               real32_T     tol)
{
    if (cb == NULL || max_iter <= 0) return 0;

    real32_T delta_x[N] = { 0.0f };
    real32_T H[N];
    real32_T a[N], b[N], K_gain[N];
    real32_T h_pred = 0.0f;
    real32_T S      = 0.0f;
    int      gated  = 0;

    for (int iter = 0; iter < max_iter; iter++) {
        cb(ctx, delta_x, H, &h_pred);
        real32_T residual = z - h_pred;

        /* a = U^T H, b = D a, S = R + a^T b   (using current U, D)      */
        for (int i = 0; i < N; i++) {
            real32_T s = H[i];
            for (int k = 0; k < i; k++) s += ekf.U[k * N + i] * H[k];
            a[i] = s;
        }
        for (int i = 0; i < N; i++) b[i] = ekf.D[i] * a[i];
        S = R;
        for (int i = 0; i < N; i++) S += a[i] * b[i];

        if (S <= 0.0f) {
            if (s_innov_cb) s_innov_cb(s_innov_tag, residual, R, S, 0, 0U);
            return 0;
        }

        if (iter == 0) {
            int      tag_idx = s_current_tag_id;
            real32_T eff_gate = gate;
            if (tag_idx >= 0) eff_gate = adaptive_gate(gate, s_fail_count[tag_idx]);
            if (eff_gate > 0.0f && residual * residual > eff_gate * eff_gate * S) gated = 1;
            if (s_innov_cb) {
                extern INS_U_T INS_U;
                s_innov_cb(s_innov_tag, residual, R, S, !gated, INS_U.IMU.timestamp);
            }
            if (gated) {
                if (tag_idx >= 0) {
                    s_fail_count[tag_idx]++;
                    if (s_fail_count[tag_idx] > EKF_FAIL_HARD_THRESH) {
                        s_fault_mask |= (1u << tag_idx);
                    }
                }
                return 0;
            }
            if (tag_idx >= 0) s_fail_count[tag_idx] = 0;
        }

        /* Kalman gain:  K = U * b / S   (note U upper unit triangular)  */
        real32_T inv_S = 1.0f / S;
        for (int i = 0; i < N; i++) {
            real32_T s = b[i];                    /* U[i,i] = 1 */
            for (int j = i + 1; j < N; j++) s += ekf.U[i * N + j] * b[j];
            K_gain[i] = s * inv_S;
        }

        /* Gauss-Newton residual:  y = (z - h_pred) - H * delta_x        */
        real32_T y = residual;
        for (int i = 0; i < N; i++) y -= H[i] * delta_x[i];

        real32_T step_sq = 0.0f;
        for (int i = 0; i < N; i++) {
            real32_T step_i = K_gain[i] * y;
            delta_x[i] += step_i;
            step_sq += step_i * step_i;
        }
        if (step_sq < tol * tol) break;
    }

    /* At convergence: do the Bierman covariance update with the final  *
     * (a, b, S).  We reuse the running-sum formulation from             *
     * ekf_update_scalar so behaviour is bit-identical when max_iter=1   *
     * and H is constant.                                                */
    real32_T K_accum[N];
    real32_T alpha_run = R;
    real32_T alpha_old;

    alpha_old   = alpha_run;
    alpha_run  += a[0] * b[0];
    ekf.D[0]    = ekf.D[0] * alpha_old / alpha_run;
    K_accum[0]  = b[0];

    for (int j = 1; j < N; j++) {
        alpha_old   = alpha_run;
        alpha_run  += a[j] * b[j];
        ekf.D[j]    = ekf.D[j] * alpha_old / alpha_run;
        real32_T gamma_j = -a[j] / alpha_old;
        for (int i = 0; i < j; i++) {
            real32_T u_old   = ekf.U[i * N + j];
            ekf.U[i * N + j] = u_old + gamma_j * K_accum[i];
            K_accum[i]      += u_old * b[j];
        }
        K_accum[j] = b[j];
    }
    /* K_accum was needed inside the loop to pair with the U updates;    *
     * the gain itself has already been folded into delta_x via the      *
     * iteration above, so we discard K_accum here.                      */

    ekf_clamp_diag(1.0e-9f);

    if (dx_out != NULL) {
        for (int i = 0; i < N; i++) dx_out[i] = delta_x[i];
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

/* ------------------------------------------------------------------ */
/*  Path A: filter-health recovery                                     */
/*                                                                     */
/*  Re-seed the attitude quaternion from the latest IMU accel (tilt)  */
/*  and, optionally, the latest mag sample (heading) - same formulas  */
/*  ekf_mag_align_initial uses, but applied at runtime to break out   */
/*  of a starved filter rather than during initial alignment.         */
/*                                                                     */
/*  Pos / vel / gyro & accel biases / augmented states are kept; only */
/*  the attitude block of the nominal state is overwritten and the    */
/*  attitude / gyro-bias diagonals of the covariance are inflated     */
/*  back to their initial-alignment values so subsequent observations */
/*  can pull the filter the rest of the way.                          */
/*                                                                     */
/*  Returns 1 if the reset was applied, 0 if the accel magnitude is   */
/*  too far from g for tilt to be trusted (i.e. vehicle in dynamic    */
/*  motion - postpone the reset until the next quiet window).         */
/* ------------------------------------------------------------------ */
int ekf_attitude_reset_from_accel(int use_mag)
{
    real32_T ax = INS_U.IMU.acc_x - ekf.b_a[0];
    real32_T ay = INS_U.IMU.acc_y - ekf.b_a[1];
    real32_T az = INS_U.IMU.acc_z - ekf.b_a[2];
    real32_T an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 5.0f || fabsf(an - 9.80665f) > 2.0f) {
        /* Too far from 1 g to trust accel as gravity */
        return 0;
    }

    real32_T phi   = atan2f(-ay, -az);
    real32_T theta = atan2f(ax, sqrtf(ay * ay + az * az));
    real32_T psi   = 0.0f;
    if (use_mag && INS_U.MAG.timestamp != 0U) {
        real32_T mx = INS_U.MAG.mag_x;
        real32_T my = INS_U.MAG.mag_y;
        real32_T mz = INS_U.MAG.mag_z;
        real32_T cphi = cosf(phi),   sphi = sinf(phi);
        real32_T cthe = cosf(theta), sthe = sinf(theta);
        real32_T mag_n =  mx * cthe + my * sthe * sphi + mz * sthe * cphi;
        real32_T mag_e =  my * cphi - mz * sphi;
        psi = atan2f(-mag_e, mag_n);
    } else {
        /* Keep the existing heading: extract psi from current quaternion */
        real32_T phi_cur, theta_cur, psi_cur;
        quat_to_euler(ekf.q, &phi_cur, &theta_cur, &psi_cur);
        psi = psi_cur;
    }
    quat_from_euler(ekf.q, phi, theta, psi);

    /* Also discard the gyro / accel bias estimates - the chaos that led *
     * to the starvation almost certainly contaminated them.  After the  *
     * reset the filter re-estimates both from scratch via subsequent    *
     * gravity / mag updates.                                            */
    ekf.b_g[0] = ekf.b_g[1] = ekf.b_g[2] = 0.0f;
    ekf.b_a[0] = ekf.b_a[1] = ekf.b_a[2] = 0.0f;

    /* Inflate attitude / gyro-bias / accel-bias diagonals so subsequent *
     * observations can move them, and clear any UDU off-diagonals       *
     * coupling those rows to other states (the old correlations are no  *
     * longer meaningful after a discontinuous state jump).              */
    real32_T av = INS_PARAM.EKF_P0_ATT * INS_PARAM.EKF_P0_ATT;
    real32_T bg = INS_PARAM.EKF_P0_BG  * INS_PARAM.EKF_P0_BG;
    real32_T ba = INS_PARAM.EKF_P0_BA  * INS_PARAM.EKF_P0_BA;
    int cols[9] = { EKF_X_DTHX, EKF_X_DTHY, EKF_X_DTHZ,
                    EKF_X_BGX,  EKF_X_BGY,  EKF_X_BGZ,
                    EKF_X_BAX,  EKF_X_BAY,  EKF_X_BAZ };
    for (int k = 0; k < 3; k++) {
        ekf.D[EKF_X_DTHX + k] = av;
        ekf.D[EKF_X_BGX  + k] = bg;
        ekf.D[EKF_X_BAX  + k] = ba;
    }
    /* Clear U[i, j] for j in ATT, BG or BA block, i < j (off-diag)      */
    for (int c = 0; c < 9; c++) {
        int j = cols[c];
        for (int i = 0; i < j; i++) ekf.U[i * N + j] = 0.0f;
    }

    /* Reset the previous-step IMU snapshot so coning/sculling does not  *
     * carry stale samples across the discontinuity.                     */
    for (int k = 0; k < 3; k++) {
        ekf.omega_prev[k] = 0.0f;
        ekf.acc_prev[k]   = 0.0f;
    }

    /* Clear fault flags for the gravity / mag tags so the post-reset    *
     * filter starts on a clean slate.                                   */
    for (int k = 0; k < EKF_INNOV_TAG_COUNT; k++) s_fail_count[k] = 0;
    s_fault_mask = 0u;

    return 1;
}

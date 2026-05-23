/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_core.h
 *
 * Generic EKF helpers shared by every measurement front-end:
 *
 *   ekf_predict          - strapdown integration of the nominal state and
 *                          covariance propagation (F P F' + Q).
 *
 *   ekf_update_scalar    - one-row Kalman update.  Multi-dimensional
 *                          measurements with diagonal R are applied as a
 *                          sequence of scalar updates.
 *
 *   ekf_inject_error     - fold an estimated error-state vector dx into the
 *                          nominal state and zero the small-angle attitude
 *                          error component.
 *
 *   ekf_symmetrize       - average upper / lower triangle of P.
 *
 *   ekf_clamp_diag       - clamp diagonal of P to a strictly positive value.
 */

#ifndef EKF_CORE_H__
#define EKF_CORE_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

void ekf_predict(const real32_T omega_meas[3],
                 const real32_T acc_meas[3],
                 real32_T dt);

/* Apply a scalar Kalman update.
 *
 *   H        : 1 x EKF_NSTATES Jacobian row.
 *   innov    : z_meas - z_pred.
 *   R        : measurement variance (sigma^2).
 *   gate     : innovation gate in sigma; <=0 disables gating.
 *   dx_out   : out, EKF_NSTATES error injection vector (caller passes to
 *              ekf_inject_error).  May be NULL to discard.
 *
 * Returns 1 if the update was accepted, 0 if rejected by the gate or if the
 * innovation covariance was non-positive.
 */
int ekf_update_scalar(const real32_T H[EKF_NSTATES],
                      real32_T innov,
                      real32_T R,
                      real32_T gate,
                      real32_T dx_out[EKF_NSTATES]);

/* ------------------------------------------------------------------ */
/*  Iterated EKF (IES-EKF) scalar update                               */
/*                                                                     */
/*  Useful when H depends strongly on the state (gravity tilt obs,     */
/*  optflow velocity, ...); re-linearising at the post-update iterate  */
/*  gives a better MAP estimate than single-pass EKF.                  */
/*                                                                     */
/*  The caller supplies a callback that, given the cumulative error    */
/*  iterate delta_x, returns the predicted measurement h(x_bar +       */
/*  delta_x) and its Jacobian H = d h / d delta_x.  Iteration is the   */
/*  Bell-Cathey 1993 / Sola 2017 Sec 6.5 fixed point:                  */
/*                                                                     */
/*      delta_x[0] = 0                                                 */
/*      loop:                                                          */
/*          (H, h_pred) = cb(ctx, delta_x)                             */
/*          K           = P*H' / (H*P*H' + R)                          */
/*          step        = K * ((z - h_pred) - H * delta_x)             */
/*          delta_x    += step                                         */
/*          if |step| < tol: break                                     */
/*                                                                     */
/*  Covariance is updated only at convergence using the final H, in    */
/*  the same Joseph form as ekf_update_scalar.                         */
/*                                                                     */
/*  For constant-H measurements, the loop terminates after one         */
/*  iteration with results bit-identical to ekf_update_scalar - no     */
/*  penalty for using this primitive uniformly.                        */
/*                                                                     */
/*  Sensible defaults: max_iter = 3, tol = 1e-4.                       */
/* ------------------------------------------------------------------ */
typedef void (*ekf_obs_cb_t)(void*          ctx,
                             const real32_T delta_x[EKF_NSTATES],
                             real32_T       H_out[EKF_NSTATES],
                             real32_T*      h_out);

int ekf_update_scalar_iterated(ekf_obs_cb_t cb,
                               void*        ctx,
                               real32_T     z,
                               real32_T     R,
                               real32_T     gate,
                               real32_T     dx_out[EKF_NSTATES],
                               int          max_iter,
                               real32_T     tol);

void ekf_inject_error(const real32_T dx[EKF_NSTATES]);

void ekf_symmetrize(void);
void ekf_clamp_diag(real32_T floor_value);

/* Apply per-state-type lower bounds on the covariance diagonal.  Stops
 * the EKF from collapsing into over-confident corners (the typical
 * failure mode when several sensors fight over a tightly correlated
 * sub-system). */
void ekf_clamp_floor(void);

/* ------------------------------------------------------------------ */
/*  Innovation instrumentation hook                                    */
/*                                                                     */
/*  Called once per scalar measurement update with the post-gating     */
/*  outcome.  The pointer is NULL by default so the firmware build     */
/*  pays nothing; offline tools (utils/ekf_replay) install a callback  */
/*  to log every innovation for diagnostic post-processing.            */
/*                                                                     */
/*  Tag is a free-form short string (e.g. "mag", "gps_pos_x") set      */
/*  by each fusion module via ekf_set_innov_tag() before its updates.  */
/* ------------------------------------------------------------------ */
typedef void (*ekf_innov_cb_t)(const char* tag,
                               real32_T innov,
                               real32_T R,
                               real32_T S,
                               int      accepted,
                               uint32_T timestamp);

void ekf_set_innov_cb(ekf_innov_cb_t cb);
void ekf_set_innov_tag(const char* tag);

/* Convert between the free-form tag strings used inside the EKF and a
 * compact integer id used by the firmware mlog bus.  -1 / "?" are
 * returned for unknown tags. */
int         ekf_innov_tag_to_id(const char* tag);
const char* ekf_innov_tag_from_id(int id);
int         ekf_innov_tag_count(void);

/* ------------------------------------------------------------------ */
/*  Filter-health + recovery (Path A)                                  */
/*                                                                     */
/*  Counters that track how many consecutive times each measurement    */
/*  tag has been gated out.  Used to drive the adaptive-gate widening  */
/*  in ekf_update_scalar (so the filter cannot starve forever) and to  */
/*  trigger ekf_attitude_reset_from_accel when the EKF is genuinely    */
/*  divergent (the failure mode seen in log/20260519/ekf_test8).       */
/* ------------------------------------------------------------------ */
int      ekf_innov_fail_count(int tag_id);
uint32_T ekf_fault_mask(void);           /* bit i set per tag i in fault */
void     ekf_fault_clear(int tag_id);

/* Re-seed the attitude quaternion from the latest accel (tilt) and
 * optionally the latest mag sample (heading).  Pos / vel / biases are
 * untouched; the attitude block of the UDU' covariance is re-inflated
 * to EKF_P0_ATT^2 so the next observation can move it again.  Returns
 * 1 on success, 0 if accel magnitude is too far from g to trust.     */
int      ekf_attitude_reset_from_accel(int use_mag);

#ifdef __cplusplus
}
#endif

#endif /* EKF_CORE_H__ */

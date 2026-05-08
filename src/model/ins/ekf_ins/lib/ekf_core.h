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

void ekf_inject_error(const real32_T dx[EKF_NSTATES]);

void ekf_symmetrize(void);
void ekf_clamp_diag(real32_T floor_value);

/* Apply per-state-type lower bounds on the covariance diagonal.  Stops
 * the EKF from collapsing into over-confident corners (the typical
 * failure mode when several sensors fight over a tightly correlated
 * sub-system). */
void ekf_clamp_floor(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_CORE_H__ */

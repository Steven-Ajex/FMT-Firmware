/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_extpos.h
 *
 * External pose front-end (e.g. visual odometry or motion-capture).
 * Reads the External_Pos_Bus published by other modules.  The
 * field_valid bit-mask selects which observations are fused:
 *
 *     bit 0  : x, y       (planar position)
 *     bit 1  : z          (vertical position)
 *     bit 2  : phi, theta (roll, pitch)
 *     bit 3  : psi        (yaw)
 *
 * EKF_EXTPOS_PSI_MODE further constrains how the external yaw is used:
 *
 *     0  do not fuse external psi
 *     1  fuse psi as-is
 *     2  fuse psi + EKF_EXTPOS_PSI offset
 *     3  ignore external psi entirely (alias of 0, kept for cf_ins
 *        compatibility)
 */

#ifndef EKF_EXTPOS_H__
#define EKF_EXTPOS_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXTPOS_BIT_XY     (1U << 0)
#define EXTPOS_BIT_Z      (1U << 1)
#define EXTPOS_BIT_RP     (1U << 2)
#define EXTPOS_BIT_PSI    (1U << 3)

int ekf_update_extpos(void);
int ekf_update_extatt(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_EXTPOS_H__ */

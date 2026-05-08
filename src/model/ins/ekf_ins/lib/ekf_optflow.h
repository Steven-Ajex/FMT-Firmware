/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_optflow.h
 *
 * Optical-flow front-end.  The sensor delivers gyro-compensated and
 * range-scaled body-frame horizontal velocity (vx, vy in m/s).  We
 * fuse it as a 2-axis observation of  R^T * v_NED  restricted to the
 * body x/y components.
 *
 * Quality below a hard-coded threshold is rejected so dropouts do
 * not pull the state.
 */

#ifndef EKF_OPTFLOW_H__
#define EKF_OPTFLOW_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

int ekf_update_optflow(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_OPTFLOW_H__ */

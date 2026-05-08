/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_baro.h
 *
 * Pressure-altimeter front-end:
 *
 *   ekf_baro_height_from_pressure  ISA standard atmosphere conversion.
 *
 *   ekf_update_baro                Scalar height update.  Observes the
 *                                  combination -p_NED[2] + alt_0 + baro_b.
 *                                  Fused only when the active height
 *                                  source (EKF_HGT_MODE) is BARO.
 */

#ifndef EKF_BARO_H__
#define EKF_BARO_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

real32_T ekf_baro_height_from_pressure(real32_T pressure_pa);
int      ekf_update_baro(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_BARO_H__ */

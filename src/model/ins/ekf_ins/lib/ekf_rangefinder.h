/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_rangefinder.h
 *
 * Downward-looking range finder front-end.  The sensor measures the
 * distance from the body z-axis to the surface; with the small-tilt
 * approximation that distance equals the height-above-ground (AGL),
 * which the EKF expresses as terr_d - p_NED[2].
 *
 * The update simultaneously feeds back into both the vehicle altitude
 * (when EKF_HGT_MODE selects RF) and the terrain state, so it doubles
 * as a terrain estimator.  Tilt > ~30 deg or distance outside the
 * sensor's valid range is rejected.
 */

#ifndef EKF_RANGEFINDER_H__
#define EKF_RANGEFINDER_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

int ekf_update_rangefinder(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_RANGEFINDER_H__ */

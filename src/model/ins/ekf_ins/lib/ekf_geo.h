/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ekf_geo.h
 *
 * WGS84 lat/lon/alt <-> local NED conversion using the equirectangular
 * approximation (Meridian / Prime-vertical radii of curvature).  Origin
 * is held inside ekf_t (double precision) to avoid losing accuracy at
 * non-zero latitudes; relative NED coordinates are kept in float.
 */

#ifndef EKF_GEO_H__
#define EKF_GEO_H__

#include "INS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_PI_D     3.14159265358979323846
#define EKF_DEG2RAD  (EKF_PI_D / 180.0)
#define EKF_RAD2DEG  (180.0 / EKF_PI_D)

void ekf_geo_set_origin(real_T lat_rad, real_T lon_rad, real_T alt_m);

/* convert geodetic to local NED, relative to the stored origin */
void ekf_geo_lla_to_ned(real_T lat_rad, real_T lon_rad, real_T alt_m,
                        real32_T ned[3]);

/* convert local NED back to geodetic */
void ekf_geo_ned_to_lla(const real32_T ned[3],
                        real_T* lat_rad, real_T* lon_rad, real_T* alt_m);

#ifdef __cplusplus
}
#endif

#endif /* EKF_GEO_H__ */

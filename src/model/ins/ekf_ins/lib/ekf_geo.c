/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "ekf_geo.h"
#include "ekf_state.h"
#include <math.h>

/* WGS84 constants */
#define WGS84_A   6378137.0           /* semi-major axis [m]            */
#define WGS84_E2  6.69437999014e-3    /* first eccentricity squared     */

void ekf_geo_set_origin(real_T lat_rad, real_T lon_rad, real_T alt_m)
{
    real_T s     = sin(lat_rad);
    real_T denom = sqrt(1.0 - WGS84_E2 * s * s);
    real_T R_N   = WGS84_A / denom;                               /* prime vertical */
    real_T R_M   = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom); /* meridian */

    ekf.lat0_rad   = lat_rad;
    ekf.lon0_rad   = lon_rad;
    ekf.alt0_m     = alt_m;
    ekf.dx_dlat    = R_M;
    ekf.dy_dlon    = R_N * cos(lat_rad);
    ekf.origin_set = 1;
}

void ekf_geo_lla_to_ned(real_T lat_rad, real_T lon_rad, real_T alt_m,
                        real32_T ned[3])
{
    ned[0] = (real32_T)((lat_rad - ekf.lat0_rad) * ekf.dx_dlat);
    ned[1] = (real32_T)((lon_rad - ekf.lon0_rad) * ekf.dy_dlon);
    ned[2] = (real32_T)(ekf.alt0_m - alt_m);
}

void ekf_geo_ned_to_lla(const real32_T ned[3],
                        real_T* lat_rad, real_T* lon_rad, real_T* alt_m)
{
    real_T inv_dx = (ekf.dx_dlat > 1.0) ? 1.0 / ekf.dx_dlat : 0.0;
    real_T inv_dy = (ekf.dy_dlon > 1.0) ? 1.0 / ekf.dy_dlon : 0.0;
    *lat_rad = ekf.lat0_rad + (real_T)ned[0] * inv_dx;
    *lon_rad = ekf.lon0_rad + (real_T)ned[1] * inv_dy;
    *alt_m   = ekf.alt0_m   - (real_T)ned[2];
}

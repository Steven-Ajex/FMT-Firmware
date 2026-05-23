/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#ifndef INS_H__
#define INS_H__

#include "rtwtypes.h"
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Sensor / output bus types                                          */
/*  Layout matches cf_ins so external code (FMS / control / mlog)      */
/*  can be reused unchanged.                                           */
/* ------------------------------------------------------------------ */

#ifndef DEFINED_TYPEDEF_FOR_IMU_Bus_
#define DEFINED_TYPEDEF_FOR_IMU_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T gyr_x;
    real32_T gyr_y;
    real32_T gyr_z;
    real32_T acc_x;
    real32_T acc_y;
    real32_T acc_z;
} IMU_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAG_Bus_
#define DEFINED_TYPEDEF_FOR_MAG_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T mag_x;
    real32_T mag_y;
    real32_T mag_z;
} MAG_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_Barometer_Bus_
#define DEFINED_TYPEDEF_FOR_Barometer_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T pressure;
    real32_T temperature;
} Barometer_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_
#define DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_
typedef struct {
    uint32_T timestamp;
    uint32_T iTOW;
    uint16_T year;
    uint8_T  month;
    uint8_T  day;
    uint8_T  hour;
    uint8_T  min;
    uint8_T  sec;
    uint8_T  valid;
    uint32_T tAcc;
    int32_T  nano;
    uint8_T  fixType;
    uint8_T  flags;
    uint8_T  reserved1;
    uint8_T  numSV;
    int32_T  lon;
    int32_T  lat;
    int32_T  height;
    int32_T  hMSL;
    uint32_T hAcc;
    uint32_T vAcc;
    int32_T  velN;
    int32_T  velE;
    int32_T  velD;
    int32_T  gSpeed;
    int32_T  heading;
    uint32_T sAcc;
    uint32_T headingAcc;
    uint16_T pDOP;
    uint16_T reserved2;
} GPS_uBlox_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_Rangefinder_Bus_
#define DEFINED_TYPEDEF_FOR_Rangefinder_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T distance;
} Rangefinder_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_
#define DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T vx;
    real32_T vy;
    uint8_T  quality;
    uint8_T  reserved1;
    uint16_T reserved2;
} Optical_Flow_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_AirSpeed_Bus_
#define DEFINED_TYPEDEF_FOR_AirSpeed_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T diff_pressure;
    real32_T temperature;
} AirSpeed_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_External_Pos_Bus_
#define DEFINED_TYPEDEF_FOR_External_Pos_Bus_
typedef struct {
    uint32_T timestamp;
    uint32_T field_valid;
    real32_T x;
    real32_T y;
    real32_T z;
    real32_T phi;
    real32_T theta;
    real32_T psi;
} External_Pos_Bus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_INS_Out_Bus_
#define DEFINED_TYPEDEF_FOR_INS_Out_Bus_
typedef struct {
    uint32_T timestamp;
    real32_T phi;
    real32_T theta;
    real32_T psi;
    real32_T quat[4];
    real32_T p;
    real32_T q;
    real32_T r;
    real32_T ax;
    real32_T ay;
    real32_T az;
    real32_T vn;
    real32_T ve;
    real32_T vd;
    real32_T airspeed;
    real_T   lat;
    real_T   lon;
    real_T   alt;
    real_T   lat_0;
    real_T   lon_0;
    real_T   alt_0;
    real_T   dx_dlat;
    real_T   dy_dlon;
    real32_T x_R;
    real32_T y_R;
    real32_T h_R;
    real32_T h_AGL;
    uint32_T flag;
    uint32_T status;
} INS_Out_Bus;
#endif

/* ------------------------------------------------------------------ */
/*  Top-level input / output containers                                */
/* ------------------------------------------------------------------ */

typedef struct {
    IMU_Bus          IMU;
    MAG_Bus          MAG;
    Barometer_Bus    Barometer;
    GPS_uBlox_Bus    GPS_uBlox;
    Rangefinder_Bus  Rangefinder;
    Optical_Flow_Bus Optical_Flow;
    AirSpeed_Bus     AirSpeed;
    External_Pos_Bus External_Pos;
} INS_U_T;

typedef struct {
    INS_Out_Bus INS_Out;
} INS_Y_T;

/* ------------------------------------------------------------------ */
/*  EKF state index map                                                */
/*                                                                     */
/*  Error-state vector layout (15 core + 2 augmented = 17).            */
/*  Adding more states (e.g. wind, mag bias) only requires extending   */
/*  this enum; matrix sizes are derived from EKF_NSTATES.              */
/* ------------------------------------------------------------------ */
typedef enum {
    EKF_X_PN = 0,   /* position North     [m]           */
    EKF_X_PE,       /* position East      [m]           */
    EKF_X_PD,       /* position Down      [m]           */
    EKF_X_VN,       /* velocity North     [m/s]         */
    EKF_X_VE,       /* velocity East      [m/s]         */
    EKF_X_VD,       /* velocity Down      [m/s]         */
    EKF_X_DTHX,     /* attitude error x   [rad]         */
    EKF_X_DTHY,     /* attitude error y   [rad]         */
    EKF_X_DTHZ,     /* attitude error z   [rad]         */
    EKF_X_BGX,      /* gyro bias x        [rad/s]       */
    EKF_X_BGY,      /* gyro bias y        [rad/s]       */
    EKF_X_BGZ,      /* gyro bias z        [rad/s]       */
    EKF_X_BAX,      /* accel bias x       [m/s^2]       */
    EKF_X_BAY,      /* accel bias y       [m/s^2]       */
    EKF_X_BAZ,      /* accel bias z       [m/s^2]       */
    EKF_X_BARO_B,   /* baro height bias   [m]           */
    EKF_X_TERR,     /* terrain height (NED Down) [m]    */
    EKF_NSTATES
} ekf_state_idx_t;

/* Convenience block sizes (must match the enum order above) */
#define EKF_BLK_POS  EKF_X_PN
#define EKF_BLK_VEL  EKF_X_VN
#define EKF_BLK_ATT  EKF_X_DTHX
#define EKF_BLK_BG   EKF_X_BGX
#define EKF_BLK_BA   EKF_X_BAX

/* ------------------------------------------------------------------ */
/*  Parameter set                                                      */
/*  Continuous-time noise densities; covariance is integrated by dt.   */
/* ------------------------------------------------------------------ */
typedef struct {
    /* process noise (continuous) */
    real32_T EKF_GYR_NOISE;     /* [rad/s/sqrt(Hz)]   */
    real32_T EKF_ACC_NOISE;     /* [m/s^2/sqrt(Hz)]   */
    real32_T EKF_BG_NOISE;      /* gyro bias RW       */
    real32_T EKF_BA_NOISE;      /* accel bias RW      */
    real32_T EKF_BARO_B_NOISE;  /* baro bias RW       */
    real32_T EKF_TERR_NOISE;    /* terrain RW         */

    /* initial covariance (1-sigma) */
    real32_T EKF_P0_POS;
    real32_T EKF_P0_VEL;
    real32_T EKF_P0_ATT;
    real32_T EKF_P0_BG;
    real32_T EKF_P0_BA;
    real32_T EKF_P0_BARO;
    real32_T EKF_P0_TERR;

    /* measurement noise (1-sigma) */
    real32_T EKF_GPS_POS_NSE;
    real32_T EKF_GPS_VEL_NSE;
    real32_T EKF_GPS_ALT_NSE;
    real32_T EKF_BARO_NSE;
    real32_T EKF_MAG_NSE;
    real32_T EKF_MAG_DECL;          /* magnetic declination, rad (E +) */
    real32_T EKF_RF_NSE;
    real32_T EKF_OPF_NSE;
    real32_T EKF_EXT_POS_NSE;
    real32_T EKF_EXT_ATT_NSE;

    /* innovation gate (sigma) */
    real32_T EKF_GPS_GATE;
    real32_T EKF_MAG_GATE;
    real32_T EKF_BARO_GATE;
    real32_T EKF_RF_GATE;
    real32_T EKF_OPF_GATE;

    /* control */
    uint32_T EKF_AID_MASK;          /* bit0 GPS  bit1 MAG  bit2 BARO
                                       bit3 RF   bit4 OPF  bit5 EXT_POS
                                       bit6 EXT_ATT                 */
    uint8_T  EKF_HGT_MODE;          /* 0=BARO 1=GPS 2=RF 3=EXT       */
    uint8_T  EKF_EXTPOS_PSI_MODE;
    real32_T EKF_EXTPOS_PSI;

    /* delays [ms] */
    uint32_T EKF_GPS_DELAY;
    uint32_T EKF_BARO_DELAY;
    uint32_T EKF_MAG_DELAY;
    uint32_T EKF_RF_DELAY;
    uint32_T EKF_OPF_DELAY;
    uint32_T EKF_EXT_DELAY;

    /* GPS lever arm in body frame [m] */
    real32_T EKF_GPS_X_OFFSET;
    real32_T EKF_GPS_Y_OFFSET;
    real32_T EKF_GPS_Z_OFFSET;
} INS_PARAM_T;

/* AID_MASK bit layout */
#define EKF_AID_GPS      (1U << 0)
#define EKF_AID_MAG      (1U << 1)
#define EKF_AID_BARO     (1U << 2)
#define EKF_AID_RF       (1U << 3)
#define EKF_AID_OPF      (1U << 4)
#define EKF_AID_EXT_POS  (1U << 5)
#define EKF_AID_EXT_ATT  (1U << 6)

/* HGT_MODE values */
#define EKF_HGT_SRC_BARO 0U
#define EKF_HGT_SRC_GPS  1U
#define EKF_HGT_SRC_RF   2U
#define EKF_HGT_SRC_EXT  3U

/* ------------------------------------------------------------------ */
/*  Module export / model info                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_T period;            /* step period [ms]   */
    int8_T   model_info[20];    /* free-form ASCII id */
} INS_EXPORT_T;

/* Globals (defined in INS.c) */
extern INS_U_T      INS_U;
extern INS_Y_T      INS_Y;
extern INS_PARAM_T  INS_PARAM;
extern INS_EXPORT_T INS_EXPORT;

/* Entry points */
void INS_init(void);
void INS_step(void);

#ifdef __cplusplus
}
#endif

#endif /* INS_H__ */

/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

/*
 * ins_interface.c
 *
 * Sensor / publish-subscribe / parameter / mlog plumbing for the ekf_ins
 * variant.  Fills INS_U from MCN topics, runs INS_step(), publishes the
 * result to ins_output and records every input bus to mlog.
 *
 * Layout matches the existing cf_ins / px4_ecl interfaces so downstream
 * code (FMS, control, mavproxy, log parsers) does not need to change.
 */

#include <INS.h>
#include <firmament.h>

#include "module/log/mlog.h"
#include "module/param/param.h"
#include "module/sensor/sensor_hub.h"

#include "ekf_core.h"
#include "ekf_state.h"

#ifdef BIT
    #undef BIT
#endif
#define BIT(u, n) (u & (1 << n))

/* INS input bus */
MCN_DECLARE(sensor_imu0);
MCN_DECLARE(sensor_mag0);
MCN_DECLARE(sensor_baro);
MCN_DECLARE(sensor_gps);
MCN_DECLARE(sensor_rangefinder);
MCN_DECLARE(sensor_optflow);
MCN_DECLARE(sensor_airspeed);

/* External Position */
MCN_DEFINE(external_pos, sizeof(External_Pos_Bus));

/* INS output bus */
MCN_DEFINE(ins_output, sizeof(INS_Out_Bus));

/* ------------------------------------------------------------------ */
/*  Parameters                                                         */
/* ------------------------------------------------------------------ */
static param_t __param_list[] = {
    /* process noise (continuous) */
    PARAM_FLOAT(EKF_GYR_NOISE,    1.5e-2f, false),
    PARAM_FLOAT(EKF_ACC_NOISE,    3.5e-1f, false),
    PARAM_FLOAT(EKF_BG_NOISE,     1.0e-4f, false),
    PARAM_FLOAT(EKF_BA_NOISE,     1.0e-3f, false),
    PARAM_FLOAT(EKF_BARO_B_NOISE, 1.0e-2f, false),
    PARAM_FLOAT(EKF_TERR_NOISE,   5.0e-2f, false),
    /* initial 1-sigma covariance */
    PARAM_FLOAT(EKF_P0_POS,  10.0f, false),
    PARAM_FLOAT(EKF_P0_VEL,   1.0f, false),
    PARAM_FLOAT(EKF_P0_ATT,  0.35f, false),
    PARAM_FLOAT(EKF_P0_BG,   0.05f, false),
    PARAM_FLOAT(EKF_P0_BA,   0.5f,  false),
    PARAM_FLOAT(EKF_P0_BARO, 5.0f,  false),
    PARAM_FLOAT(EKF_P0_TERR, 5.0f,  false),
    /* measurement 1-sigma noise */
    PARAM_FLOAT(EKF_GPS_POS_NSE, 0.5f,  false),
    PARAM_FLOAT(EKF_GPS_VEL_NSE, 0.3f,  false),
    PARAM_FLOAT(EKF_GPS_ALT_NSE, 1.5f,  false),
    PARAM_FLOAT(EKF_BARO_NSE,    2.0f,  false),
    PARAM_FLOAT(EKF_MAG_NSE,     0.05f, false),
    PARAM_FLOAT(EKF_RF_NSE,      0.1f,  false),
    PARAM_FLOAT(EKF_OPF_NSE,     0.2f,  false),
    PARAM_FLOAT(EKF_EXT_POS_NSE, 0.05f, false),
    PARAM_FLOAT(EKF_EXT_ATT_NSE, 0.05f, false),
    /* innovation gating (sigma) */
    PARAM_FLOAT(EKF_GPS_GATE,  5.0f, false),
    PARAM_FLOAT(EKF_MAG_GATE,  5.0f, false),
    PARAM_FLOAT(EKF_BARO_GATE, 5.0f, false),
    PARAM_FLOAT(EKF_RF_GATE,   5.0f, false),
    PARAM_FLOAT(EKF_OPF_GATE,  5.0f, false),
    /* control */
    PARAM_UINT32(EKF_AID_MASK, 0x07U, false),  /* GPS + MAG + BARO        */
    PARAM_UINT8(EKF_HGT_MODE,  0U,    false),
    PARAM_UINT8(EKF_EXTPOS_PSI_MODE, 3U, false),
    PARAM_FLOAT(EKF_EXTPOS_PSI, 0.0f, false),
    /* delays [ms] */
    PARAM_UINT32(EKF_GPS_DELAY,  100U, false),
    PARAM_UINT32(EKF_BARO_DELAY, 10U,  false),
    PARAM_UINT32(EKF_MAG_DELAY,  0U,   false),
    PARAM_UINT32(EKF_RF_DELAY,   10U,  false),
    PARAM_UINT32(EKF_OPF_DELAY,  10U,  false),
    PARAM_UINT32(EKF_EXT_DELAY,  20U,  false),
    /* GPS lever arm in body frame [m] */
    PARAM_FLOAT(EKF_GPS_X_OFFSET, 0.0f, false),
    PARAM_FLOAT(EKF_GPS_Y_OFFSET, 0.0f, false),
    PARAM_FLOAT(EKF_GPS_Z_OFFSET, 0.0f, false),
};
PARAM_GROUP_DEFINE(INS, __param_list);

/* ------------------------------------------------------------------ */
/*  mlog bus definitions (kept identical to cf_ins for log tools)      */
/* ------------------------------------------------------------------ */
static mlog_elem_t IMU_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(gyr_x, MLOG_FLOAT),
    MLOG_ELEMENT(gyr_y, MLOG_FLOAT),
    MLOG_ELEMENT(gyr_z, MLOG_FLOAT),
    MLOG_ELEMENT(acc_x, MLOG_FLOAT),
    MLOG_ELEMENT(acc_y, MLOG_FLOAT),
    MLOG_ELEMENT(acc_z, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(IMU, IMU_Elems);

static mlog_elem_t MAG_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(mag_x, MLOG_FLOAT),
    MLOG_ELEMENT(mag_y, MLOG_FLOAT),
    MLOG_ELEMENT(mag_z, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(MAG, MAG_Elems);

static mlog_elem_t Barometer_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(pressure, MLOG_FLOAT),
    MLOG_ELEMENT(temperature, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(Barometer, Barometer_Elems);

static mlog_elem_t GPS_uBlox_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(iTOW, MLOG_UINT32),
    MLOG_ELEMENT(year, MLOG_UINT16),
    MLOG_ELEMENT(month, MLOG_UINT8),
    MLOG_ELEMENT(day, MLOG_UINT8),
    MLOG_ELEMENT(hour, MLOG_UINT8),
    MLOG_ELEMENT(min, MLOG_UINT8),
    MLOG_ELEMENT(sec, MLOG_UINT8),
    MLOG_ELEMENT(valid, MLOG_UINT8),
    MLOG_ELEMENT(tAcc, MLOG_UINT32),
    MLOG_ELEMENT(nano, MLOG_INT32),
    MLOG_ELEMENT(fixType, MLOG_UINT8),
    MLOG_ELEMENT(flags, MLOG_UINT8),
    MLOG_ELEMENT(reserved1, MLOG_UINT8),
    MLOG_ELEMENT(numSV, MLOG_UINT8),
    MLOG_ELEMENT(lon, MLOG_INT32),
    MLOG_ELEMENT(lat, MLOG_INT32),
    MLOG_ELEMENT(height, MLOG_INT32),
    MLOG_ELEMENT(hMSL, MLOG_INT32),
    MLOG_ELEMENT(hAcc, MLOG_UINT32),
    MLOG_ELEMENT(vAcc, MLOG_UINT32),
    MLOG_ELEMENT(velN, MLOG_INT32),
    MLOG_ELEMENT(velE, MLOG_INT32),
    MLOG_ELEMENT(velD, MLOG_INT32),
    MLOG_ELEMENT(gSpeed, MLOG_INT32),
    MLOG_ELEMENT(heading, MLOG_INT32),
    MLOG_ELEMENT(sAcc, MLOG_UINT32),
    MLOG_ELEMENT(headingAcc, MLOG_UINT32),
    MLOG_ELEMENT(pDOP, MLOG_UINT16),
    MLOG_ELEMENT(reserved2, MLOG_UINT16),
};
MLOG_BUS_DEFINE(GPS_uBlox, GPS_uBlox_Elems);

mlog_elem_t Rangefinder_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(distance, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(Rangefinder, Rangefinder_Elems);

mlog_elem_t Optflow_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(vx, MLOG_FLOAT),
    MLOG_ELEMENT(vy, MLOG_FLOAT),
    MLOG_ELEMENT(quality, MLOG_UINT8),
    MLOG_ELEMENT(reserved1, MLOG_UINT8),
    MLOG_ELEMENT(reserved2, MLOG_UINT16),
};
MLOG_BUS_DEFINE(OpticalFlow, Optflow_Elems);

mlog_elem_t Airspeed_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(diff_pressure, MLOG_FLOAT),
    MLOG_ELEMENT(temperature, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(AirSpeed, Airspeed_Elems);

static mlog_elem_t External_Pos_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(field_valid, MLOG_UINT32),
    MLOG_ELEMENT(x, MLOG_FLOAT),
    MLOG_ELEMENT(y, MLOG_FLOAT),
    MLOG_ELEMENT(z, MLOG_FLOAT),
    MLOG_ELEMENT(phi, MLOG_FLOAT),
    MLOG_ELEMENT(theta, MLOG_FLOAT),
    MLOG_ELEMENT(psi, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(External_Pos, External_Pos_Elems);

/* ------------------------------------------------------------------ */
/*  INS_Innov - one row per scalar measurement update                   */
/*                                                                     */
/*  Logged for offline diagnostics.  tag_id maps to a fixed table       */
/*  defined in ekf_core.c (use ekf_innov_tag_from_id() in tools).       */
/*  grav_* updates fire every IMU step; they are throttled by the cb    */
/*  to keep the SD card volume manageable.                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t timestamp;
    int8_t   tag_id;
    uint8_t  accepted;
    uint16_t reserved;
    float    innov;
    float    R;
    float    S;
    float    nis;
} INS_Innov_Bus;

static mlog_elem_t INS_Innov_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(tag_id,    MLOG_INT8),
    MLOG_ELEMENT(accepted,  MLOG_UINT8),
    MLOG_ELEMENT(reserved,  MLOG_UINT16),
    MLOG_ELEMENT(innov,     MLOG_FLOAT),
    MLOG_ELEMENT(R,         MLOG_FLOAT),
    MLOG_ELEMENT(S,         MLOG_FLOAT),
    MLOG_ELEMENT(nis,       MLOG_FLOAT),
};
MLOG_BUS_DEFINE(INS_Innov, INS_Innov_Elems);

/* ------------------------------------------------------------------ */
/*  INS_State - throttled snapshot of EKF internal state                */
/*                                                                     */
/*  Logged at the same 10 Hz cadence as INS_Out.  Captures the bias    */
/*  estimates and the per-block sigma so a drift / divergence is       */
/*  immediately visible from the recorded log.                         */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t timestamp;
    float    bg_x;     float bg_y;     float bg_z;
    float    ba_x;     float ba_y;     float ba_z;
    float    baro_b;
    float    terr_d;
    float    sigma_pos_n; float sigma_pos_e; float sigma_pos_d;
    float    sigma_vel_n; float sigma_vel_e; float sigma_vel_d;
    float    sigma_att_x; float sigma_att_y; float sigma_att_z;
} INS_State_Bus;

static mlog_elem_t INS_State_Elems[] = {
    MLOG_ELEMENT(timestamp,   MLOG_UINT32),
    MLOG_ELEMENT(bg_x,        MLOG_FLOAT),
    MLOG_ELEMENT(bg_y,        MLOG_FLOAT),
    MLOG_ELEMENT(bg_z,        MLOG_FLOAT),
    MLOG_ELEMENT(ba_x,        MLOG_FLOAT),
    MLOG_ELEMENT(ba_y,        MLOG_FLOAT),
    MLOG_ELEMENT(ba_z,        MLOG_FLOAT),
    MLOG_ELEMENT(baro_b,      MLOG_FLOAT),
    MLOG_ELEMENT(terr_d,      MLOG_FLOAT),
    MLOG_ELEMENT(sigma_pos_n, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_pos_e, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_pos_d, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_vel_n, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_vel_e, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_vel_d, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_att_x, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_att_y, MLOG_FLOAT),
    MLOG_ELEMENT(sigma_att_z, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(INS_State, INS_State_Elems);

mlog_elem_t INS_Out_Elems[] = {
    MLOG_ELEMENT(timestamp, MLOG_UINT32),
    MLOG_ELEMENT(phi, MLOG_FLOAT),
    MLOG_ELEMENT(theta, MLOG_FLOAT),
    MLOG_ELEMENT(psi, MLOG_FLOAT),
    MLOG_ELEMENT_VEC(quat, MLOG_FLOAT, 4),
    MLOG_ELEMENT(p, MLOG_FLOAT),
    MLOG_ELEMENT(q, MLOG_FLOAT),
    MLOG_ELEMENT(r, MLOG_FLOAT),
    MLOG_ELEMENT(ax, MLOG_FLOAT),
    MLOG_ELEMENT(ay, MLOG_FLOAT),
    MLOG_ELEMENT(az, MLOG_FLOAT),
    MLOG_ELEMENT(vn, MLOG_FLOAT),
    MLOG_ELEMENT(ve, MLOG_FLOAT),
    MLOG_ELEMENT(vd, MLOG_FLOAT),
    MLOG_ELEMENT(airspeed, MLOG_FLOAT),
    MLOG_ELEMENT(lat, MLOG_DOUBLE),
    MLOG_ELEMENT(lon, MLOG_DOUBLE),
    MLOG_ELEMENT(alt, MLOG_DOUBLE),
    MLOG_ELEMENT(lat_0, MLOG_DOUBLE),
    MLOG_ELEMENT(lon_0, MLOG_DOUBLE),
    MLOG_ELEMENT(alt_0, MLOG_DOUBLE),
    MLOG_ELEMENT(dx_dlat, MLOG_DOUBLE),
    MLOG_ELEMENT(dy_dlon, MLOG_DOUBLE),
    MLOG_ELEMENT(x_R, MLOG_FLOAT),
    MLOG_ELEMENT(y_R, MLOG_FLOAT),
    MLOG_ELEMENT(h_R, MLOG_FLOAT),
    MLOG_ELEMENT(h_AGL, MLOG_FLOAT),
    MLOG_ELEMENT(flag, MLOG_UINT32),
    MLOG_ELEMENT(status, MLOG_UINT32),
};
MLOG_BUS_DEFINE(INS_Out, INS_Out_Elems);

/* ------------------------------------------------------------------ */
/*  Local handler                                                      */
/* ------------------------------------------------------------------ */
static struct INS_Handler {
    McnNode_t imu_sub_node_t;
    McnNode_t mag_sub_node_t;
    McnNode_t baro_sub_node_t;
    McnNode_t gps_sub_node_t;
    McnNode_t rf_sub_node_t;
    McnNode_t optflow_sub_node_t;
    McnNode_t airspeed_sub_node_t;
    McnNode_t ext_pos_sub_node_t;

    imu_data_t       imu_report;
    mag_data_t       mag_report;
    baro_data_t      baro_report;
    gps_data_t       gps_report;
    rf_data_t        rf_report;
    optflow_data_t   optflow_report;
    airspeed_data_t  airspeed_report;
    External_Pos_Bus ext_pos_report;
} ins_handle;

static uint8_t imu_data_updated;
static uint8_t mag_data_updated;
static uint8_t baro_data_updated;
static uint8_t gps_data_updated;
static uint8_t rf_data_updated;
static uint8_t optflow_data_updated;
static uint8_t airspeed_data_updated;
static uint8_t ext_pos_data_updated;

static int IMU_ID;
static int MAG_ID;
static int Barometer_ID;
static int GPS_ID;
static int Rangefinder_ID;
static int OpticalFlow_ID;
static int AirSpeed_ID;
static int ExtPos_ID;
static int INS_Out_ID;
static int INS_Innov_ID;
static int INS_State_ID;

fmt_model_info_t ins_model_info;

/* ------------------------------------------------------------------ */
/*  Echo callbacks (mavconsole)                                        */
/* ------------------------------------------------------------------ */
static int ins_output_echo(void* param)
{
    INS_Out_Bus ins_out;
    mcn_copy_from_hub((McnHub*)param, &ins_out);

    printf("timestamp:%u\n", ins_out.timestamp);
    printf("att: %.2f %.2f %.2f\n",
           RAD2DEG(ins_out.phi), RAD2DEG(ins_out.theta), RAD2DEG(ins_out.psi));
    printf("rate: %.2f %.2f %.2f\n", ins_out.p, ins_out.q, ins_out.r);
    printf("accel: %.2f %.2f %.2f\n", ins_out.ax, ins_out.ay, ins_out.az);
    printf("vel: %.2f %.2f %.2f airspeed:%.2f\n",
           ins_out.vn, ins_out.ve, ins_out.vd, ins_out.airspeed);
    printf("xyh: %.2f %.2f %.2f, h_AGL: %.2f\n",
           ins_out.x_R, ins_out.y_R, ins_out.h_R, ins_out.h_AGL);
    printf("LLA: %lf %lf %f LLA0: %lf %lf %f\n",
           ins_out.lat, ins_out.lon, ins_out.alt,
           ins_out.lat_0, ins_out.lon_0, ins_out.alt_0);
    printf("flag: 0x%08x  status: 0x%08x\n", ins_out.flag, ins_out.status);
    printf("------------------------------------------\n");
    return 0;
}

static int external_pos_echo(void* param)
{
    External_Pos_Bus ext_att_pos;
    mcn_copy_from_hub((McnHub*)param, &ext_att_pos);

    printf("timestamp:%u\n", ext_att_pos.timestamp);
    printf("xyz: %.2f %.2f %.2f\n", ext_att_pos.x, ext_att_pos.y, ext_att_pos.z);
    printf("att: %.2f %.2f %.2f\n", ext_att_pos.phi, ext_att_pos.theta, ext_att_pos.psi);
    printf("valid xy:%d z:%d phi,theta:%d psi:%d\n",
           (ext_att_pos.field_valid & 0x01),
           (ext_att_pos.field_valid & 0x02) > 0,
           (ext_att_pos.field_valid & 0x04) > 0,
           (ext_att_pos.field_valid & 0x08) > 0);
    printf("------------------------------------------\n");
    return 0;
}

static void mlog_start_cb(void)
{
    /* on log start, force first record of every input bus */
    imu_data_updated = mag_data_updated = baro_data_updated = 1;
    gps_data_updated = rf_data_updated = optflow_data_updated = 1;
    airspeed_data_updated = ext_pos_data_updated = 1;
}

/* ------------------------------------------------------------------ */
/*  Innovation callback (firmware path)                                */
/*                                                                     */
/*  Called by the EKF after every scalar measurement update.  We push  */
/*  one INS_Innov_Bus row to mlog.  grav_* fires on every IMU step     */
/*  (~500 Hz x 2 axes) which would dominate the log volume; throttle   */
/*  it to ~10 Hz instead.                                              */
/* ------------------------------------------------------------------ */
static void firmware_innov_cb(const char* tag, real32_T innov, real32_T R,
                              real32_T S, int accepted, uint32_T ts)
{
    static uint32_T grav_skip = 0;
    if (tag != NULL && tag[0] == 'g' && tag[1] == 'r' && tag[2] == 'a' && tag[3] == 'v') {
        if ((grav_skip++ % 50U) != 0U) return;     /* 500 Hz / 50 = 10 Hz */
    }

    INS_Innov_Bus row;
    row.timestamp = ts;
    row.tag_id    = (int8_t)ekf_innov_tag_to_id(tag);
    row.accepted  = (uint8_t)(accepted ? 1 : 0);
    row.reserved  = 0;
    row.innov     = innov;
    row.R         = R;
    row.S         = S;
    row.nis       = (S > 0.0f) ? (innov * innov / S) : 0.0f;
    mlog_push_msg((uint8_t*)&row, INS_Innov_ID, sizeof(row));
}

static void publish_ins_state(uint32_t ts)
{
    INS_State_Bus row;
    row.timestamp   = ts;
    row.bg_x        = ekf.b_g[0];
    row.bg_y        = ekf.b_g[1];
    row.bg_z        = ekf.b_g[2];
    row.ba_x        = ekf.b_a[0];
    row.ba_y        = ekf.b_a[1];
    row.ba_z        = ekf.b_a[2];
    row.baro_b      = ekf.baro_b;
    row.terr_d      = ekf.terr_d;
    int N = EKF_NSTATES;
    row.sigma_pos_n = sqrtf(ekf.P[(EKF_X_PN  ) * N + EKF_X_PN  ]);
    row.sigma_pos_e = sqrtf(ekf.P[(EKF_X_PE  ) * N + EKF_X_PE  ]);
    row.sigma_pos_d = sqrtf(ekf.P[(EKF_X_PD  ) * N + EKF_X_PD  ]);
    row.sigma_vel_n = sqrtf(ekf.P[(EKF_X_VN  ) * N + EKF_X_VN  ]);
    row.sigma_vel_e = sqrtf(ekf.P[(EKF_X_VE  ) * N + EKF_X_VE  ]);
    row.sigma_vel_d = sqrtf(ekf.P[(EKF_X_VD  ) * N + EKF_X_VD  ]);
    row.sigma_att_x = sqrtf(ekf.P[(EKF_X_DTHX) * N + EKF_X_DTHX]);
    row.sigma_att_y = sqrtf(ekf.P[(EKF_X_DTHY) * N + EKF_X_DTHY]);
    row.sigma_att_z = sqrtf(ekf.P[(EKF_X_DTHZ) * N + EKF_X_DTHZ]);
    mlog_push_msg((uint8_t*)&row, INS_State_ID, sizeof(row));
}

/* ------------------------------------------------------------------ */
/*  Parameter binding                                                  */
/* ------------------------------------------------------------------ */
static void init_parameter(void)
{
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GYR_NOISE),    &INS_PARAM.EKF_GYR_NOISE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_ACC_NOISE),    &INS_PARAM.EKF_ACC_NOISE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BG_NOISE),     &INS_PARAM.EKF_BG_NOISE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BA_NOISE),     &INS_PARAM.EKF_BA_NOISE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BARO_B_NOISE), &INS_PARAM.EKF_BARO_B_NOISE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_TERR_NOISE),   &INS_PARAM.EKF_TERR_NOISE));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_POS),  &INS_PARAM.EKF_P0_POS));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_VEL),  &INS_PARAM.EKF_P0_VEL));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_ATT),  &INS_PARAM.EKF_P0_ATT));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_BG),   &INS_PARAM.EKF_P0_BG));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_BA),   &INS_PARAM.EKF_P0_BA));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_BARO), &INS_PARAM.EKF_P0_BARO));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_P0_TERR), &INS_PARAM.EKF_P0_TERR));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_POS_NSE), &INS_PARAM.EKF_GPS_POS_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_VEL_NSE), &INS_PARAM.EKF_GPS_VEL_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_ALT_NSE), &INS_PARAM.EKF_GPS_ALT_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BARO_NSE),    &INS_PARAM.EKF_BARO_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_MAG_NSE),     &INS_PARAM.EKF_MAG_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_RF_NSE),      &INS_PARAM.EKF_RF_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_OPF_NSE),     &INS_PARAM.EKF_OPF_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_EXT_POS_NSE), &INS_PARAM.EKF_EXT_POS_NSE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_EXT_ATT_NSE), &INS_PARAM.EKF_EXT_ATT_NSE));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_GATE),  &INS_PARAM.EKF_GPS_GATE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_MAG_GATE),  &INS_PARAM.EKF_MAG_GATE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BARO_GATE), &INS_PARAM.EKF_BARO_GATE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_RF_GATE),   &INS_PARAM.EKF_RF_GATE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_OPF_GATE),  &INS_PARAM.EKF_OPF_GATE));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_AID_MASK),       &INS_PARAM.EKF_AID_MASK));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_HGT_MODE),       &INS_PARAM.EKF_HGT_MODE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_EXTPOS_PSI_MODE),&INS_PARAM.EKF_EXTPOS_PSI_MODE));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_EXTPOS_PSI),     &INS_PARAM.EKF_EXTPOS_PSI));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_DELAY),  &INS_PARAM.EKF_GPS_DELAY));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_BARO_DELAY), &INS_PARAM.EKF_BARO_DELAY));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_MAG_DELAY),  &INS_PARAM.EKF_MAG_DELAY));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_RF_DELAY),   &INS_PARAM.EKF_RF_DELAY));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_OPF_DELAY),  &INS_PARAM.EKF_OPF_DELAY));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_EXT_DELAY),  &INS_PARAM.EKF_EXT_DELAY));

    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_X_OFFSET), &INS_PARAM.EKF_GPS_X_OFFSET));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_Y_OFFSET), &INS_PARAM.EKF_GPS_Y_OFFSET));
    FMT_CHECK(param_link_variable(PARAM_GET(INS, EKF_GPS_Z_OFFSET), &INS_PARAM.EKF_GPS_Z_OFFSET));
}

/* ------------------------------------------------------------------ */
/*  Step / init                                                        */
/* ------------------------------------------------------------------ */
void ins_interface_step(uint32_t timestamp)
{
    if (mcn_poll(ins_handle.imu_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_imu0), ins_handle.imu_sub_node_t, &ins_handle.imu_report);

        INS_U.IMU.gyr_x = ins_handle.imu_report.gyr_B_radDs[0];
        INS_U.IMU.gyr_y = ins_handle.imu_report.gyr_B_radDs[1];
        INS_U.IMU.gyr_z = ins_handle.imu_report.gyr_B_radDs[2];
        INS_U.IMU.acc_x = ins_handle.imu_report.acc_B_mDs2[0];
        INS_U.IMU.acc_y = ins_handle.imu_report.acc_B_mDs2[1];
        INS_U.IMU.acc_z = ins_handle.imu_report.acc_B_mDs2[2];
        INS_U.IMU.timestamp = timestamp;
        imu_data_updated = 1;
    }

    if (mcn_poll(ins_handle.mag_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_mag0), ins_handle.mag_sub_node_t, &ins_handle.mag_report);

        INS_U.MAG.mag_x = ins_handle.mag_report.mag_B_gauss[0];
        INS_U.MAG.mag_y = ins_handle.mag_report.mag_B_gauss[1];
        INS_U.MAG.mag_z = ins_handle.mag_report.mag_B_gauss[2];
        INS_U.MAG.timestamp = timestamp;
        mag_data_updated = 1;
    }

    if (mcn_poll(ins_handle.baro_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_baro), ins_handle.baro_sub_node_t, &ins_handle.baro_report);

        INS_U.Barometer.pressure    = (float)ins_handle.baro_report.pressure_pa;
        INS_U.Barometer.temperature = ins_handle.baro_report.temperature_deg;
        INS_U.Barometer.timestamp   = timestamp;
        baro_data_updated = 1;
    }

    if (mcn_poll(ins_handle.gps_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_gps), ins_handle.gps_sub_node_t, &ins_handle.gps_report);

        INS_U.GPS_uBlox.fixType    = ins_handle.gps_report.fixType;
        INS_U.GPS_uBlox.lat        = ins_handle.gps_report.lat;
        INS_U.GPS_uBlox.lon        = ins_handle.gps_report.lon;
        INS_U.GPS_uBlox.height     = ins_handle.gps_report.height;
        INS_U.GPS_uBlox.velN       = (int32_t)(ins_handle.gps_report.velN * 1e3);
        INS_U.GPS_uBlox.velE       = (int32_t)(ins_handle.gps_report.velE * 1e3);
        INS_U.GPS_uBlox.velD       = (int32_t)(ins_handle.gps_report.velD * 1e3);
        INS_U.GPS_uBlox.heading    = (int32_t)(ins_handle.gps_report.heading * 1e3);
        INS_U.GPS_uBlox.hAcc       = (uint32_t)(ins_handle.gps_report.hAcc * 1e3);
        INS_U.GPS_uBlox.vAcc       = (uint32_t)(ins_handle.gps_report.vAcc * 1e3);
        INS_U.GPS_uBlox.sAcc       = (uint32_t)(ins_handle.gps_report.sAcc * 1e3);
        INS_U.GPS_uBlox.headingAcc = (uint32_t)(ins_handle.gps_report.headingAcc * 1e3);
        INS_U.GPS_uBlox.numSV      = ins_handle.gps_report.numSV;
        INS_U.GPS_uBlox.timestamp  = timestamp;
        gps_data_updated = 1;
    }

    if (mcn_poll(ins_handle.rf_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_rangefinder), ins_handle.rf_sub_node_t, &ins_handle.rf_report);

        INS_U.Rangefinder.distance  = ins_handle.rf_report.distance_m;
        INS_U.Rangefinder.timestamp = timestamp;
        rf_data_updated = 1;
    }

    if (mcn_poll(ins_handle.optflow_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_optflow), ins_handle.optflow_sub_node_t, &ins_handle.optflow_report);

        INS_U.Optical_Flow.vx        = ins_handle.optflow_report.vx_mPs;
        INS_U.Optical_Flow.vy        = ins_handle.optflow_report.vy_mPs;
        INS_U.Optical_Flow.quality   = ins_handle.optflow_report.quality;
        INS_U.Optical_Flow.timestamp = timestamp;
        optflow_data_updated = 1;
    }

    if (mcn_poll(ins_handle.airspeed_sub_node_t)) {
        mcn_copy(MCN_HUB(sensor_airspeed), ins_handle.airspeed_sub_node_t, &ins_handle.airspeed_report);

        INS_U.AirSpeed.diff_pressure = ins_handle.airspeed_report.diff_pressure_pa;
        INS_U.AirSpeed.temperature   = ins_handle.airspeed_report.temperature_deg;
        INS_U.AirSpeed.timestamp     = timestamp;
        airspeed_data_updated = 1;
    }

    if (mcn_poll(ins_handle.ext_pos_sub_node_t)) {
        mcn_copy(MCN_HUB(external_pos), ins_handle.ext_pos_sub_node_t, &ins_handle.ext_pos_report);

        INS_U.External_Pos.timestamp   = timestamp;
        INS_U.External_Pos.field_valid = ins_handle.ext_pos_report.field_valid;
        INS_U.External_Pos.x           = ins_handle.ext_pos_report.x;
        INS_U.External_Pos.y           = ins_handle.ext_pos_report.y;
        INS_U.External_Pos.z           = ins_handle.ext_pos_report.z;
        INS_U.External_Pos.phi         = ins_handle.ext_pos_report.phi;
        INS_U.External_Pos.theta       = ins_handle.ext_pos_report.theta;
        INS_U.External_Pos.psi         = ins_handle.ext_pos_report.psi;
        ext_pos_data_updated = 1;
    }

    /* run EKF */
    INS_step();

    /* publish */
    mcn_publish(MCN_HUB(ins_output), &INS_Y.INS_Out);

    /* mlog input buses */
    if (imu_data_updated)      { imu_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.IMU,          IMU_ID,         sizeof(INS_U.IMU)); }
    if (mag_data_updated)      { mag_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.MAG,          MAG_ID,         sizeof(INS_U.MAG)); }
    if (baro_data_updated)     { baro_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.Barometer,    Barometer_ID,   sizeof(INS_U.Barometer)); }
    if (gps_data_updated)      { gps_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.GPS_uBlox,    GPS_ID,         sizeof(INS_U.GPS_uBlox)); }
    if (rf_data_updated)       { rf_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.Rangefinder,  Rangefinder_ID, sizeof(INS_U.Rangefinder)); }
    if (optflow_data_updated)  { optflow_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.Optical_Flow, OpticalFlow_ID, sizeof(INS_U.Optical_Flow)); }
    if (airspeed_data_updated) { airspeed_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.AirSpeed,     AirSpeed_ID,    sizeof(INS_U.AirSpeed)); }
    if (ext_pos_data_updated)  { ext_pos_data_updated = 0;
        mlog_push_msg((uint8_t*)&INS_U.External_Pos, ExtPos_ID,      sizeof(INS_U.External_Pos)); }

    /* throttle INS_Out + INS_State logging to ~10 Hz */
    DEFINE_TIMETAG(ins_output, 100);
    if (check_timetag(TIMETAG(ins_output))) {
        mlog_push_msg((uint8_t*)&INS_Y.INS_Out, INS_Out_ID, sizeof(INS_Y.INS_Out));
        publish_ins_state(timestamp);
    }
}

void ins_interface_init(void)
{
    ins_model_info.period = INS_EXPORT.period;
    ins_model_info.info   = (char*)INS_EXPORT.model_info;

    mcn_advertise(MCN_HUB(ins_output),   ins_output_echo);
    mcn_advertise(MCN_HUB(external_pos), external_pos_echo);

    ins_handle.imu_sub_node_t      = mcn_subscribe(MCN_HUB(sensor_imu0),         NULL);
    ins_handle.mag_sub_node_t      = mcn_subscribe(MCN_HUB(sensor_mag0),         NULL);
    ins_handle.baro_sub_node_t     = mcn_subscribe(MCN_HUB(sensor_baro),         NULL);
    ins_handle.gps_sub_node_t      = mcn_subscribe(MCN_HUB(sensor_gps),          NULL);
    ins_handle.rf_sub_node_t       = mcn_subscribe(MCN_HUB(sensor_rangefinder),  NULL);
    ins_handle.optflow_sub_node_t  = mcn_subscribe(MCN_HUB(sensor_optflow),      NULL);
    ins_handle.airspeed_sub_node_t = mcn_subscribe(MCN_HUB(sensor_airspeed),     NULL);
    ins_handle.ext_pos_sub_node_t  = mcn_subscribe(MCN_HUB(external_pos),        NULL);

    IMU_ID         = mlog_get_bus_id("IMU");
    MAG_ID         = mlog_get_bus_id("MAG");
    Barometer_ID   = mlog_get_bus_id("Barometer");
    GPS_ID         = mlog_get_bus_id("GPS_uBlox");
    Rangefinder_ID = mlog_get_bus_id("Rangefinder");
    OpticalFlow_ID = mlog_get_bus_id("OpticalFlow");
    AirSpeed_ID    = mlog_get_bus_id("AirSpeed");
    ExtPos_ID      = mlog_get_bus_id("External_Pos");
    INS_Out_ID     = mlog_get_bus_id("INS_Out");
    INS_Innov_ID   = mlog_get_bus_id("INS_Innov");
    INS_State_ID   = mlog_get_bus_id("INS_State");
    FMT_ASSERT(IMU_ID         >= 0);
    FMT_ASSERT(MAG_ID         >= 0);
    FMT_ASSERT(Barometer_ID   >= 0);
    FMT_ASSERT(GPS_ID         >= 0);
    FMT_ASSERT(Rangefinder_ID >= 0);
    FMT_ASSERT(OpticalFlow_ID >= 0);
    FMT_ASSERT(AirSpeed_ID    >= 0);
    FMT_ASSERT(ExtPos_ID      >= 0);
    FMT_ASSERT(INS_Out_ID     >= 0);
    FMT_ASSERT(INS_Innov_ID   >= 0);
    FMT_ASSERT(INS_State_ID   >= 0);

    mlog_register_callback(MLOG_CB_START, mlog_start_cb);

    /* Hook the innovation logger.  Has to come before INS_init runs the
     * first alignment so we capture even the start-up updates. */
    ekf_set_innov_cb(firmware_innov_cb);

    INS_init();
    init_parameter();
}

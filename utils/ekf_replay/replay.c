/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 *  ekf_replay -- offline driver for the ekf_ins library.
 *
 *  Reads one CSV per input bus (the format that parse_mlog.py emits),
 *  merges them on the timestamp column, and feeds them into INS_U /
 *  INS_step exactly the way the firmware's ins_interface_step does.
 *  Each call to INS_step emits a row of INS_Out_Bus to the output CSV.
 *
 *  Usage:
 *     ekf_replay --imu  IMU.csv  --mag  MAG.csv
 *                --baro Barometer.csv  --gps GPS_uBlox.csv
 *                --rf   Rangefinder.csv  --opf OpticalFlow.csv
 *                --ext  External_Pos.csv  --out INS_Out.csv
 *
 *  --selftest runs an in-process synthetic scenario and exits non-zero
 *  if the EKF fails to converge or produces NaN.
 *****************************************************************************/

#define _POSIX_C_SOURCE 200809L  /* strdup */

#include "INS.h"
#include "ekf_state.h"
#include "ekf_health.h"
#include "ekf_extpos.h"
#include "ekf_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/*  CSV reader                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    char**   header;            /* column names (lower-cased)         */
    int      n_cols;
    /* values for current row, one double per column */
    double*  row;
    FILE*    fp;
    int      eof;
    long     line_no;
    char*    line_buf;
    size_t   line_buf_cap;
} csv_reader_t;

static char* lc(char* s) {
    for (char* p = s; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
    return s;
}

static char* strip(char* s) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    char* end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
        *--end = '\0';
    }
    return s;
}

static int csv_read_line(csv_reader_t* r) {
    if (r->fp == NULL || r->eof) return 0;
    if (r->line_buf == NULL) {
        r->line_buf_cap = 4096;
        r->line_buf = malloc(r->line_buf_cap);
    }
    if (fgets(r->line_buf, (int)r->line_buf_cap, r->fp) == NULL) {
        r->eof = 1;
        return 0;
    }
    r->line_no++;
    return 1;
}

static int csv_open(csv_reader_t* r, const char* path) {
    memset(r, 0, sizeof(*r));
    r->fp = fopen(path, "r");
    if (r->fp == NULL) {
        fprintf(stderr, "ekf_replay: cannot open %s\n", path);
        return -1;
    }
    if (!csv_read_line(r)) {
        fprintf(stderr, "ekf_replay: %s is empty\n", path);
        return -1;
    }
    /* count columns */
    char* line = strip(r->line_buf);
    int n = 1;
    for (char* p = line; *p; p++) if (*p == ',') n++;
    r->n_cols = n;
    r->header = calloc((size_t)n, sizeof(char*));
    r->row    = calloc((size_t)n, sizeof(double));
    /* split header */
    int i = 0;
    char* p = line;
    while (*p && i < n) {
        char* q = strchr(p, ',');
        if (q) *q = '\0';
        r->header[i++] = strdup(lc(strip(p)));
        if (q) p = q + 1; else break;
    }
    return 0;
}

static int csv_col_index(const csv_reader_t* r, const char* name) {
    for (int i = 0; i < r->n_cols; i++) {
        if (strcmp(r->header[i], name) == 0) return i;
    }
    return -1;
}

/* Fetch the next row.  Returns 1 if a row was read, 0 on EOF. */
static int csv_next(csv_reader_t* r) {
    if (r->fp == NULL) return 0;
    if (!csv_read_line(r)) return 0;
    char* line = strip(r->line_buf);
    char* p = line;
    int   i = 0;
    while (*p && i < r->n_cols) {
        char* q = strchr(p, ',');
        if (q) *q = '\0';
        r->row[i] = (*p) ? strtod(p, NULL) : 0.0;
        i++;
        if (q) p = q + 1; else break;
    }
    while (i < r->n_cols) r->row[i++] = 0.0;
    return 1;
}

static void csv_close(csv_reader_t* r) {
    if (r->fp) fclose(r->fp);
    if (r->line_buf) free(r->line_buf);
    if (r->header) {
        for (int i = 0; i < r->n_cols; i++) free(r->header[i]);
        free(r->header);
    }
    free(r->row);
    memset(r, 0, sizeof(*r));
}

/* ------------------------------------------------------------------ */
/*  Bus drivers - copy one CSV row into INS_U.<bus>                    */
/* ------------------------------------------------------------------ */
typedef struct {
    csv_reader_t r;
    int          ts_col;
    uint32_t     ts_now;        /* timestamp of the row currently held */
    int          has_pending;
    /* list of (column index, target field setter) pairs precomputed
     * once on open() */
} bus_t;

#define DECLARE_BUS(NAME) \
    static bus_t b_##NAME = {0};

DECLARE_BUS(imu)
DECLARE_BUS(mag)
DECLARE_BUS(baro)
DECLARE_BUS(gps)
DECLARE_BUS(rf)
DECLARE_BUS(opf)
DECLARE_BUS(ext)

static int bus_open(bus_t* b, const char* path) {
    if (path == NULL) return 0;
    if (csv_open(&b->r, path) != 0) return -1;
    b->ts_col = csv_col_index(&b->r, "timestamp");
    if (b->ts_col < 0) {
        fprintf(stderr, "ekf_replay: %s has no 'timestamp' column\n", path);
        return -1;
    }
    b->has_pending = csv_next(&b->r);
    if (b->has_pending) b->ts_now = (uint32_t)b->r.row[b->ts_col];
    return 0;
}

#define COPY_FIELD(BUS, FIELD, COLNAME) do {                                  \
    int _i = csv_col_index(&BUS.r, COLNAME);                                  \
    if (_i >= 0) INS_U.FIELD = (__typeof__(INS_U.FIELD))BUS.r.row[_i];        \
} while (0)

static void bus_apply_imu(void) {
    if (!b_imu.has_pending) return;
    INS_U.IMU.timestamp = b_imu.ts_now;
    COPY_FIELD(b_imu, IMU.gyr_x, "gyr_x");
    COPY_FIELD(b_imu, IMU.gyr_y, "gyr_y");
    COPY_FIELD(b_imu, IMU.gyr_z, "gyr_z");
    COPY_FIELD(b_imu, IMU.acc_x, "acc_x");
    COPY_FIELD(b_imu, IMU.acc_y, "acc_y");
    COPY_FIELD(b_imu, IMU.acc_z, "acc_z");
}
static void bus_apply_mag(void) {
    if (!b_mag.has_pending) return;
    INS_U.MAG.timestamp = b_mag.ts_now;
    COPY_FIELD(b_mag, MAG.mag_x, "mag_x");
    COPY_FIELD(b_mag, MAG.mag_y, "mag_y");
    COPY_FIELD(b_mag, MAG.mag_z, "mag_z");
}
static void bus_apply_baro(void) {
    if (!b_baro.has_pending) return;
    INS_U.Barometer.timestamp = b_baro.ts_now;
    COPY_FIELD(b_baro, Barometer.pressure,    "pressure");
    COPY_FIELD(b_baro, Barometer.temperature, "temperature");
}
static void bus_apply_gps(void) {
    if (!b_gps.has_pending) return;
    INS_U.GPS_uBlox.timestamp = b_gps.ts_now;
    COPY_FIELD(b_gps, GPS_uBlox.fixType, "fixtype");
    COPY_FIELD(b_gps, GPS_uBlox.lat,     "lat");
    COPY_FIELD(b_gps, GPS_uBlox.lon,     "lon");
    COPY_FIELD(b_gps, GPS_uBlox.height,  "height");
    COPY_FIELD(b_gps, GPS_uBlox.velN,    "veln");
    COPY_FIELD(b_gps, GPS_uBlox.velE,    "vele");
    COPY_FIELD(b_gps, GPS_uBlox.velD,    "veld");
    COPY_FIELD(b_gps, GPS_uBlox.numSV,   "numsv");
    COPY_FIELD(b_gps, GPS_uBlox.hAcc,    "hacc");
    COPY_FIELD(b_gps, GPS_uBlox.vAcc,    "vacc");
    COPY_FIELD(b_gps, GPS_uBlox.sAcc,    "sacc");
}
static void bus_apply_rf(void) {
    if (!b_rf.has_pending) return;
    INS_U.Rangefinder.timestamp = b_rf.ts_now;
    COPY_FIELD(b_rf, Rangefinder.distance, "distance");
}
static void bus_apply_opf(void) {
    if (!b_opf.has_pending) return;
    INS_U.Optical_Flow.timestamp = b_opf.ts_now;
    COPY_FIELD(b_opf, Optical_Flow.vx,      "vx");
    COPY_FIELD(b_opf, Optical_Flow.vy,      "vy");
    COPY_FIELD(b_opf, Optical_Flow.quality, "quality");
}
static void bus_apply_ext(void) {
    if (!b_ext.has_pending) return;
    INS_U.External_Pos.timestamp = b_ext.ts_now;
    COPY_FIELD(b_ext, External_Pos.field_valid, "field_valid");
    COPY_FIELD(b_ext, External_Pos.x,           "x");
    COPY_FIELD(b_ext, External_Pos.y,           "y");
    COPY_FIELD(b_ext, External_Pos.z,           "z");
    COPY_FIELD(b_ext, External_Pos.phi,         "phi");
    COPY_FIELD(b_ext, External_Pos.theta,       "theta");
    COPY_FIELD(b_ext, External_Pos.psi,         "psi");
}

static void bus_advance(bus_t* b) {
    b->has_pending = csv_next(&b->r);
    if (b->has_pending) b->ts_now = (uint32_t)b->r.row[b->ts_col];
}

/* ------------------------------------------------------------------ */
/*  Innovation logger                                                  */
/*                                                                     */
/*  Writes one row per scalar measurement update.  NIS = innov^2 / S   */
/*  is precomputed for convenience: under correct R the channel-wise   */
/*  NIS distribution should be chi-square(1) (mean 1, 95th pct ~3.84). */
/* ------------------------------------------------------------------ */
static FILE* innov_fp = NULL;

static void innov_cb(const char* tag, real32_T innov, real32_T R,
                     real32_T S, int accepted, uint32_T ts)
{
    if (innov_fp == NULL) return;
    real32_T nis = (S > 0.0f) ? (innov * innov / S) : 0.0f;
    fprintf(innov_fp, "%u,%s,%.6f,%.6f,%.6f,%.6f,%d\n",
            ts, tag, innov, R, S, nis, accepted);
}

/* ------------------------------------------------------------------ */
/*  Output writer                                                      */
/* ------------------------------------------------------------------ */
static FILE* out_fp = NULL;

static void out_open(const char* path) {
    out_fp = fopen(path, "w");
    if (out_fp == NULL) {
        fprintf(stderr, "ekf_replay: cannot open %s for write\n", path);
        exit(1);
    }
    fprintf(out_fp,
            "timestamp,phi,theta,psi,quat0,quat1,quat2,quat3,"
            "p,q,r,ax,ay,az,vn,ve,vd,airspeed,"
            "lat,lon,alt,lat_0,lon_0,alt_0,dx_dlat,dy_dlon,"
            "x_R,y_R,h_R,h_AGL,flag,status\n");
}

static void out_emit(void) {
    INS_Out_Bus* y = &INS_Y.INS_Out;
    fprintf(out_fp,
            "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.10f,%.10f,%.4f,%.10f,%.10f,%.4f,%.4f,%.4f,"
            "%.6f,%.6f,%.6f,%.6f,%u,%u\n",
            y->timestamp, y->phi, y->theta, y->psi,
            y->quat[0], y->quat[1], y->quat[2], y->quat[3],
            y->p, y->q, y->r, y->ax, y->ay, y->az,
            y->vn, y->ve, y->vd, y->airspeed,
            y->lat, y->lon, y->alt, y->lat_0, y->lon_0, y->alt_0,
            y->dx_dlat, y->dy_dlon,
            y->x_R, y->y_R, y->h_R, y->h_AGL, y->flag, y->status);
}

static void out_close(void) {
    if (out_fp) fclose(out_fp);
    out_fp = NULL;
}

/* ------------------------------------------------------------------ */
/*  Replay loop                                                        */
/*                                                                     */
/*  At each step pick the bus with the smallest pending timestamp,     */
/*  copy its row into INS_U, and:                                      */
/*    - if it is the IMU bus, also call INS_step and emit an output    */
/*      row (every IMU sample triggers a model step in the firmware).  */
/*  Continues until every bus is exhausted.                            */
/* ------------------------------------------------------------------ */
static void replay_run(void) {
    INS_init();

    while (b_imu.has_pending || b_mag.has_pending || b_baro.has_pending ||
           b_gps.has_pending  || b_rf.has_pending  || b_opf.has_pending  ||
           b_ext.has_pending) {

        bus_t* candidates[] = {
            &b_imu, &b_mag, &b_baro, &b_gps, &b_rf, &b_opf, &b_ext };
        bus_t* next = NULL;
        for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); i++) {
            if (!candidates[i]->has_pending) continue;
            if (next == NULL || candidates[i]->ts_now < next->ts_now) {
                next = candidates[i];
            }
        }
        if (next == NULL) break;

        if      (next == &b_imu)  bus_apply_imu();
        else if (next == &b_mag)  bus_apply_mag();
        else if (next == &b_baro) bus_apply_baro();
        else if (next == &b_gps)  bus_apply_gps();
        else if (next == &b_rf)   bus_apply_rf();
        else if (next == &b_opf)  bus_apply_opf();
        else                      bus_apply_ext();

        if (next == &b_imu) {
            INS_step();
            out_emit();
        }
        bus_advance(next);
    }
}

/* ------------------------------------------------------------------ */
/*  Selftest -- internal synthetic scenario, no inputs                 */
/* ------------------------------------------------------------------ */
static int selftest(void) {
    INS_init();
    INS_PARAM.EKF_AID_MASK = 0x7Fu;

    INS_U.IMU.acc_x = 0; INS_U.IMU.acc_y = 0; INS_U.IMU.acc_z = -9.80665f;
    INS_U.MAG.mag_x = 1; INS_U.MAG.mag_y = 0; INS_U.MAG.mag_z = 0;
    INS_U.GPS_uBlox.fixType = 3;
    INS_U.GPS_uBlox.lat = (int32_t)(30.0 * 1e7);
    INS_U.GPS_uBlox.lon = (int32_t)(120.0 * 1e7);
    INS_U.GPS_uBlox.height = (int32_t)(100.0 * 1000);
    INS_U.Barometer.pressure = 100129.4f;

    for (int i = 0; i < 1000; i++) {
        uint32_t now = (uint32_t)(i * 2);
        INS_U.IMU.timestamp = now;
        if ((i %  5) == 0) INS_U.MAG.timestamp = now;
        if ((i % 50) == 0) INS_U.GPS_uBlox.timestamp = now;
        if ((i % 10) == 0) INS_U.Barometer.timestamp = now;
        INS_step();
    }

    if (!isfinite(INS_Y.INS_Out.phi)) { fprintf(stderr, "FAIL nan\n"); return 1; }
    if (fabsf(INS_Y.INS_Out.phi)   > 0.05f
     || fabsf(INS_Y.INS_Out.theta) > 0.05f) { fprintf(stderr, "FAIL tilt drift\n"); return 2; }

    printf("selftest PASS  euler=(%.4f, %.4f, %.4f)  lla=(%.7f, %.7f, %.3f)\n",
           INS_Y.INS_Out.phi, INS_Y.INS_Out.theta, INS_Y.INS_Out.psi,
           INS_Y.INS_Out.lat * 180.0/M_PI, INS_Y.INS_Out.lon * 180.0/M_PI,
           INS_Y.INS_Out.alt);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Argument parsing                                                   */
/* ------------------------------------------------------------------ */
static const char* opt_imu      = NULL;
static const char* opt_mag      = NULL;
static const char* opt_baro     = NULL;
static const char* opt_gps      = NULL;
static const char* opt_rf       = NULL;
static const char* opt_opf      = NULL;
static const char* opt_ext      = NULL;
static const char* opt_out      = "INS_Out_replay.csv";
static const char* opt_innov    = NULL;
static int         opt_selftest = 0;

static void usage(const char* argv0) {
    fprintf(stderr,
        "usage: %s [--selftest]\n"
        "       %s --imu IMU.csv [--mag MAG.csv]\n"
        "         [--baro Barometer.csv] [--gps GPS_uBlox.csv]\n"
        "         [--rf Rangefinder.csv] [--opf OpticalFlow.csv]\n"
        "         [--ext External_Pos.csv]\n"
        "         [--out INS_Out_replay.csv]\n"
        "         [--innov-csv innov.csv]\n",
        argv0, argv0);
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        if      (strcmp(a, "--selftest") == 0) opt_selftest = 1;
        else if (strcmp(a, "--imu")  == 0 && i+1 < argc) opt_imu  = argv[++i];
        else if (strcmp(a, "--mag")  == 0 && i+1 < argc) opt_mag  = argv[++i];
        else if (strcmp(a, "--baro") == 0 && i+1 < argc) opt_baro = argv[++i];
        else if (strcmp(a, "--gps")  == 0 && i+1 < argc) opt_gps  = argv[++i];
        else if (strcmp(a, "--rf")   == 0 && i+1 < argc) opt_rf   = argv[++i];
        else if (strcmp(a, "--opf")  == 0 && i+1 < argc) opt_opf  = argv[++i];
        else if (strcmp(a, "--ext")  == 0 && i+1 < argc) opt_ext  = argv[++i];
        else if (strcmp(a, "--out")  == 0 && i+1 < argc) opt_out  = argv[++i];
        else if (strcmp(a, "--innov-csv") == 0 && i+1 < argc) opt_innov = argv[++i];
        else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(argv[0]); return 0;
        } else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(argv[0]); return 1;
        }
    }

    if (opt_selftest) return selftest();

    if (opt_imu == NULL) {
        fprintf(stderr, "ekf_replay: --imu is required\n");
        usage(argv[0]); return 1;
    }
    if (bus_open(&b_imu,  opt_imu)  != 0) return 1;
    if (opt_mag  && bus_open(&b_mag,  opt_mag ) != 0) return 1;
    if (opt_baro && bus_open(&b_baro, opt_baro) != 0) return 1;
    if (opt_gps  && bus_open(&b_gps,  opt_gps ) != 0) return 1;
    if (opt_rf   && bus_open(&b_rf,   opt_rf  ) != 0) return 1;
    if (opt_opf  && bus_open(&b_opf,  opt_opf ) != 0) return 1;
    if (opt_ext  && bus_open(&b_ext,  opt_ext ) != 0) return 1;

    if (opt_innov != NULL) {
        innov_fp = fopen(opt_innov, "w");
        if (innov_fp == NULL) {
            fprintf(stderr, "ekf_replay: cannot open %s for write\n", opt_innov);
            return 1;
        }
        fprintf(innov_fp, "timestamp,tag,innov,R,S,nis,accepted\n");
        ekf_set_innov_cb(innov_cb);
    }

    out_open(opt_out);
    replay_run();
    out_close();
    if (innov_fp) { fclose(innov_fp); innov_fp = NULL; }

    csv_close(&b_imu.r);
    csv_close(&b_mag.r);
    csv_close(&b_baro.r);
    csv_close(&b_gps.r);
    csv_close(&b_rf.r);
    csv_close(&b_opf.r);
    csv_close(&b_ext.r);

    fprintf(stderr, "ekf_replay: wrote %s\n", opt_out);
    return 0;
}

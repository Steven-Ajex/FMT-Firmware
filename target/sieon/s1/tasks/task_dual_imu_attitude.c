/******************************************************************************
 * Dual-IMU attitude estimator for folding arms (hinge about body X).
 * - Estimates hinge angle using acc/mag alignment and gyro difference.
 * - Transforms both IMUs into body frame and fuses gravity/mag observations.
 * Not enabled by default; set DUAL_IMU_ATT_ENABLE to 1 and export task to use.
 *****************************************************************************/

#include <firmament.h>

#include <math.h>
#include <string.h>

#include "module/ipc/uMCN.h"
#include "module/sensor/sensor_hub.h"
#include "module/system/systime.h"
#include "module/task_manager/task_manager.h"
#include "task_dual_imu_attitude.h"

/* Enable to build/run this task */
#ifndef DUAL_IMU_ATT_ENABLE
    #define DUAL_IMU_ATT_ENABLE 1
#endif
/* Mount parameters (body frame, meters) */
#ifndef DUAL_IMU_LEFT_POS
    #define DUAL_IMU_LEFT_POS   { -0.15f, -0.2f, 0.0f }
#endif
#ifndef DUAL_IMU_RIGHT_POS
    #define DUAL_IMU_RIGHT_POS  { -0.15f, 0.2f, 0.0f }
#endif

/* Fixed mount rotations (IMU to arm frame). Identity if aligned. */
#ifndef DUAL_IMU_LEFT_R_MOUNT
    #define DUAL_IMU_LEFT_R_MOUNT   { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f }
#endif
#ifndef DUAL_IMU_RIGHT_R_MOUNT
    #define DUAL_IMU_RIGHT_R_MOUNT  { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f }
#endif

/* Filter gains */
#define DUAL_IMU_ANGLE_KP         0.4f   /* complementary gain for obs */
#define DUAL_IMU_ANGLE_KI         0.0f
#define DUAL_IMU_ACC_TRUST_MIN_G  0.8f   /* gate acc observation */
#define DUAL_IMU_ACC_TRUST_MAX_G  1.2f

/* Timing/health thresholds */
#ifndef DUAL_IMU_DT_DEFAULT_S
    #define DUAL_IMU_DT_DEFAULT_S 0.001f
#endif
#ifndef DUAL_IMU_DT_MIN_S
    #define DUAL_IMU_DT_MIN_S     0.0002f
#endif
#ifndef DUAL_IMU_DT_MAX_S
    #define DUAL_IMU_DT_MAX_S     0.02f
#endif
#ifndef DUAL_IMU_MAX_TIME_SKEW_MS
    #define DUAL_IMU_MAX_TIME_SKEW_MS 5
#endif
#ifndef DUAL_IMU_MAX_IMU_STALE_MS
    #define DUAL_IMU_MAX_IMU_STALE_MS 10
#endif
#ifndef DUAL_IMU_MAX_MAG_STALE_MS
    #define DUAL_IMU_MAX_MAG_STALE_MS 50
#endif
#ifndef DUAL_IMU_MAG_NORM_RATIO_MAX
    #define DUAL_IMU_MAG_NORM_RATIO_MAX 1.6f
#endif
#ifndef DUAL_IMU_GYR_MAX_RAD_S
    #define DUAL_IMU_GYR_MAX_RAD_S 35.0f
#endif

/* Simple 2-state (theta, bias) Kalman tuning */
#define DUAL_IMU_PROC_VAR_RATE    (0.02f * 0.02f)   /* (rad/s)^2 process on hinge rate */
#define DUAL_IMU_PROC_VAR_BIAS    (1e-6f)           /* bias random walk */
#define DUAL_IMU_MEAS_VAR_ACC     (0.05f * 0.05f)   /* rad^2 from acc/mag obs */
#define DUAL_IMU_MEAS_VAR_MAG     (0.10f * 0.10f)   /* rad^2 */

/* Angular acceleration low-pass (for tangential term) */
#define DUAL_IMU_ALPHA_LPF        0.2f

/* MCN topics: local IMU and remote IMU from CAN bridge */
MCN_DECLARE(sensor_imu0);
MCN_DECLARE(sensor_mag0);
MCN_DECLARE(bridge_imu);
MCN_DECLARE(bridge_mag);

typedef struct {
    float theta;      /* hinge angle */
    float theta_bias;
    float P[4];       /* 2x2 covariance: [P00 P01; P10 P11] */
} hinge_state_t;

typedef struct {
    imu_data_t imu_local;
    imu_data_t imu_remote;
    mag_data_t mag_local;
    mag_data_t mag_remote;
    hinge_state_t hinge;
    float prev_gyr_b_L[3];
    float prev_gyr_b_R[3];
    float alpha_b_L[3];
    float alpha_b_R[3];
} dual_imu_state_t;

static dual_imu_state_t g_state;
static dual_imu_att_output_t g_att_out;
#if DUAL_IMU_ATT_STATUS_ENABLE
static dual_imu_att_status_t g_att_status;
#endif

MCN_DEFINE(dual_imu_att, sizeof(dual_imu_att_output_t));
#if DUAL_IMU_ATT_STATUS_ENABLE
MCN_DEFINE(dual_imu_att_status, sizeof(dual_imu_att_status_t));
#endif

static int echo_dual_imu_att(void* param)
{
    dual_imu_att_output_t data;

    if (mcn_copy_from_hub((McnHub*)param, &data) != FMT_EOK) {
        return -1;
    }

    console_printf("ts:%u theta:%.4f accL:[%.3f %.3f %.3f] accR:[%.3f %.3f %.3f] accCG:[%.3f %.3f %.3f] gyrL:[%.3f %.3f %.3f] gyrR:[%.3f %.3f %.3f] gyr:[%.3f %.3f %.3f] magL:[%.3f %.3f %.3f] magR:[%.3f %.3f %.3f] magCG:[%.3f %.3f %.3f]\n",
                   data.timestamp_ms,
                   data.hinge_theta,
                   data.acc_b_L[0], data.acc_b_L[1], data.acc_b_L[2],
                   data.acc_b_R[0], data.acc_b_R[1], data.acc_b_R[2],
                   data.acc_b_cg[0], data.acc_b_cg[1], data.acc_b_cg[2],
                   data.gyr_b_L[0], data.gyr_b_L[1], data.gyr_b_L[2],
                   data.gyr_b_R[0], data.gyr_b_R[1], data.gyr_b_R[2],
                   data.gyr_b[0], data.gyr_b[1], data.gyr_b[2],
                   data.mag_b_L[0], data.mag_b_L[1], data.mag_b_L[2],
                   data.mag_b_R[0], data.mag_b_R[1], data.mag_b_R[2],
                   data.mag_b_cg[0], data.mag_b_cg[1], data.mag_b_cg[2]);
    return 0;
}

static void vec3_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec3_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float vec3_norm(const float a[3])
{
    return sqrtf(vec3_dot(a, a));
}

static void vec3_normalize(float a[3])
{
    float n = vec3_norm(a);
    if (n > 1e-6f) {
        a[0] /= n;
        a[1] /= n;
        a[2] /= n;
    }
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static int32_t abs_i32(int32_t v)
{
    return v < 0 ? -v : v;
}

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return (int16_t)v;
}

static uint16_t clamp_u16(uint32_t v)
{
    return v > 65535u ? 65535u : (uint16_t)v;
}

static bool vec3_isfinite(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void mat3_mul_vec(const float m[9], const float v[3], float out[3])
{
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

static void rot_x_vec(float c, float s, const float v[3], float out[3])
{
    out[0] = v[0];
    out[1] = c * v[1] - s * v[2];
    out[2] = s * v[1] + c * v[2];
}

static void rot_x_mul_mount(float c, float s, const float R_mount[9], float R_out[9])
{
    R_out[0] = R_mount[0];
    R_out[1] = R_mount[1];
    R_out[2] = R_mount[2];

    R_out[3] = c * R_mount[3] - s * R_mount[6];
    R_out[4] = c * R_mount[4] - s * R_mount[7];
    R_out[5] = c * R_mount[5] - s * R_mount[8];

    R_out[6] = s * R_mount[3] + c * R_mount[6];
    R_out[7] = s * R_mount[4] + c * R_mount[7];
    R_out[8] = s * R_mount[5] + c * R_mount[8];
}

static float hinge_obs_from_vecs(const float hinge[3], const float vL[3], const float vR[3])
{
    float vLp[3] = { vL[0] - vec3_dot(vL, hinge) * hinge[0],
                     vL[1] - vec3_dot(vL, hinge) * hinge[1],
                     vL[2] - vec3_dot(vL, hinge) * hinge[2] };
    float vRp[3] = { vR[0] - vec3_dot(vR, hinge) * hinge[0],
                     vR[1] - vec3_dot(vR, hinge) * hinge[1],
                     vR[2] - vec3_dot(vR, hinge) * hinge[2] };
    float nL = vec3_norm(vLp);
    float nR = vec3_norm(vRp);
    if (nL < 1e-6f || nR < 1e-6f) {
        return 0.0f;
    }
    float cosd = clampf(vec3_dot(vLp, vRp) / (nL * nR), -1.0f, 1.0f);
    float crossp[3];
    vec3_cross(vLp, vRp, crossp);
    float sind = vec3_dot(hinge, crossp) / (nL * nR);
    /* Relative rotation is 2*theta because each arm rotates ±theta about hinge */
    return 0.5f * atan2f(sind, cosd);
}

static void hinge_update_kf(float dt,
                            const imu_data_t* imuL,
                            const imu_data_t* imuR,
                            const mag_data_t* magL,
                            const mag_data_t* magR,
                            hinge_state_t* h,
                            uint32_t* obs_flags,
                            float* acc_norm_L,
                            float* acc_norm_R,
                            float* mag_norm_L,
                            float* mag_norm_R)
{
    if (obs_flags) {
        *obs_flags = 0;
    }
    if (acc_norm_L) {
        *acc_norm_L = 0.0f;
    }
    if (acc_norm_R) {
        *acc_norm_R = 0.0f;
    }
    if (mag_norm_L) {
        *mag_norm_L = 0.0f;
    }
    if (mag_norm_R) {
        *mag_norm_R = 0.0f;
    }

    /* State: [theta, bias]^T */
    float wL = imuL->gyr_B_radDs[0];
    float wR = imuR->gyr_B_radDs[0];
    float theta_rate_meas = 0.5f * (wR - wL) - h->theta_bias;
    float theta_pred = h->theta + dt * theta_rate_meas;
    float bias_pred = h->theta_bias;

    /* P prediction: F = [[1, -dt],[0,1]]; G = [dt;0] */
    float P00 = h->P[0]; float P01 = h->P[1];
    float P10 = h->P[2]; float P11 = h->P[3];
    float F00 = 1.0f, F01 = -dt, F10 = 0.0f, F11 = 1.0f;
    float P00p = F00 * P00 + F01 * P10;
    float P01p = F00 * P01 + F01 * P11;
    float P10p = F10 * P00 + F11 * P10;
    float P11p = F10 * P01 + F11 * P11;
    float Q00 = dt * dt * DUAL_IMU_PROC_VAR_RATE;
    float Q11 = dt * DUAL_IMU_PROC_VAR_BIAS;
    P00 = P00p * F00 + P01p * F01 + Q00;
    P01 = P00p * F10 + P01p * F11;
    P10 = P10p * F00 + P11p * F01;
    P11 = P10p * F10 + P11p * F11 + Q11;

    /* Observation */
    float z = theta_pred;
    bool have_obs = false;
    float R_meas = DUAL_IMU_MEAS_VAR_ACC;
    float accL[3] = { imuL->acc_B_mDs2[0], imuL->acc_B_mDs2[1], imuL->acc_B_mDs2[2] };
    float accR[3] = { imuR->acc_B_mDs2[0], imuR->acc_B_mDs2[1], imuR->acc_B_mDs2[2] };
    float accnL = vec3_norm(accL);
    float accnR = vec3_norm(accR);
    if (acc_norm_L) {
        *acc_norm_L = accnL;
    }
    if (acc_norm_R) {
        *acc_norm_R = accnR;
    }

    const float hinge_axis[3] = { 1.0f, 0.0f, 0.0f };
    bool acc_valid = (accnL > (DUAL_IMU_ACC_TRUST_MIN_G * 9.8f)) && (accnL < (DUAL_IMU_ACC_TRUST_MAX_G * 9.8f))
                     && (accnR > (DUAL_IMU_ACC_TRUST_MIN_G * 9.8f)) && (accnR < (DUAL_IMU_ACC_TRUST_MAX_G * 9.8f));
    if (acc_valid) {
        float gL[3] = { accL[0] / accnL, accL[1] / accnL, accL[2] / accnL };
        float gR[3] = { accR[0] / accnR, accR[1] / accnR, accR[2] / accnR };
        z = hinge_obs_from_vecs(hinge_axis, gL, gR);
        R_meas = DUAL_IMU_MEAS_VAR_ACC;
        have_obs = true;
        if (obs_flags) {
            *obs_flags |= DUAL_IMU_STATUS_ACC_TRUST;
        }
    } else if (magL && magR) {
        float mL[3] = { magL->mag_B_gauss[0], magL->mag_B_gauss[1], magL->mag_B_gauss[2] };
        float mR[3] = { magR->mag_B_gauss[0], magR->mag_B_gauss[1], magR->mag_B_gauss[2] };
        float magnL = vec3_norm(mL);
        float magnR = vec3_norm(mR);
        if (mag_norm_L) {
            *mag_norm_L = magnL;
        }
        if (mag_norm_R) {
            *mag_norm_R = magnR;
        }
        if (magnL > 1e-6f && magnR > 1e-6f) {
            float ratio = magnL / magnR;
            if (ratio < 1.0f) {
                ratio = 1.0f / ratio;
            }
            if (ratio <= DUAL_IMU_MAG_NORM_RATIO_MAX) {
                vec3_normalize(mL);
                vec3_normalize(mR);
                z = hinge_obs_from_vecs(hinge_axis, mL, mR);
                R_meas = DUAL_IMU_MEAS_VAR_MAG;
                have_obs = true;
                if (obs_flags) {
                    *obs_flags |= DUAL_IMU_STATUS_MAG_TRUST;
                }
            }
        }
    }

    /* If no observation, keep prediction */
    if (have_obs) {
        float S = P00 + R_meas;
        float K0 = P00 / S;
        float K1 = P10 / S;
        float err = z - theta_pred;
        theta_pred += K0 * err;
        bias_pred  += K1 * err;

        float P00n = (1.0f - K0) * P00;
        float P01n = (1.0f - K0) * P01;
        float P10n = -K1 * P00 + P10;
        float P11n = -K1 * P01 + P11;
        P00 = P00n; P01 = P01n; P10 = P10n; P11 = P11n;
    }

    h->theta = theta_pred;
    h->theta_bias = bias_pred;
    h->P[0] = P00; h->P[1] = P01; h->P[2] = P10; h->P[3] = P11;
}

static void att_compute_body_vectors(float dt,
                                     float theta,
                                     const imu_data_t* imuL,
                                     const imu_data_t* imuR,
                                     const mag_data_t* magL,
                                     const mag_data_t* magR,
                                     dual_imu_state_t* state,
                                     dual_imu_att_output_t* out)
{
    const float pos_mount_L[3] = DUAL_IMU_LEFT_POS;
    const float pos_mount_R[3] = DUAL_IMU_RIGHT_POS;
    float c = cosf(theta);
    float s = sinf(theta);
    const float R_mount_L[9] = DUAL_IMU_LEFT_R_MOUNT;
    const float R_mount_R[9] = DUAL_IMU_RIGHT_R_MOUNT;

    float R_bi_L[9];
    float R_bi_R[9];
    float pos_b_L[3];
    float pos_b_R[3];
    rot_x_mul_mount(c, s, R_mount_L, R_bi_L);
    rot_x_mul_mount(c, -s, R_mount_R, R_bi_R);
    rot_x_vec(c, s, pos_mount_L, pos_b_L);
    rot_x_vec(c, -s, pos_mount_R, pos_b_R);

    float acc_b_L[3];
    float acc_b_R[3];
    float gyr_b_L[3];
    float gyr_b_R[3];
    float mag_b_L[3];
    float mag_b_R[3];
    mat3_mul_vec(R_bi_L, imuL->acc_B_mDs2, acc_b_L);
    mat3_mul_vec(R_bi_R, imuR->acc_B_mDs2, acc_b_R);
    mat3_mul_vec(R_bi_L, imuL->gyr_B_radDs, gyr_b_L);
    mat3_mul_vec(R_bi_R, imuR->gyr_B_radDs, gyr_b_R);
    mat3_mul_vec(R_bi_L, magL->mag_B_gauss, mag_b_L);
    mat3_mul_vec(R_bi_R, magR->mag_B_gauss, mag_b_R);
    float gyr_b_avg[3] = {
        0.5f * (gyr_b_L[0] + gyr_b_R[0]),
        0.5f * (gyr_b_L[1] + gyr_b_R[1]),
        0.5f * (gyr_b_L[2] + gyr_b_R[2])
    };

    /* Angular acceleration (per arm) for tangential term */
    if (dt > 1e-4f) {
        float alpha_L_inst[3] = {
            (gyr_b_L[0] - state->prev_gyr_b_L[0]) / dt,
            (gyr_b_L[1] - state->prev_gyr_b_L[1]) / dt,
            (gyr_b_L[2] - state->prev_gyr_b_L[2]) / dt
        };
        float alpha_R_inst[3] = {
            (gyr_b_R[0] - state->prev_gyr_b_R[0]) / dt,
            (gyr_b_R[1] - state->prev_gyr_b_R[1]) / dt,
            (gyr_b_R[2] - state->prev_gyr_b_R[2]) / dt
        };
        for (int i = 0; i < 3; i++) {
            state->alpha_b_L[i] += DUAL_IMU_ALPHA_LPF * (alpha_L_inst[i] - state->alpha_b_L[i]);
            state->alpha_b_R[i] += DUAL_IMU_ALPHA_LPF * (alpha_R_inst[i] - state->alpha_b_R[i]);
            state->prev_gyr_b_L[i] = gyr_b_L[i];
            state->prev_gyr_b_R[i] = gyr_b_R[i];
        }
    }

    /* Remove tangential (alpha x r) and centripetal (omega x (omega x r)) terms, per arm */
    float acc_cg_L[3];
    float acc_cg_R[3];
    float omega_cross_r[3];
    float cross_tmp[3];
    vec3_cross(state->alpha_b_L, pos_b_L, cross_tmp);
    vec3_cross(gyr_b_L, pos_b_L, omega_cross_r);
    vec3_cross(gyr_b_L, omega_cross_r, omega_cross_r);
    acc_cg_L[0] = acc_b_L[0] - cross_tmp[0] - omega_cross_r[0];
    acc_cg_L[1] = acc_b_L[1] - cross_tmp[1] - omega_cross_r[1];
    acc_cg_L[2] = acc_b_L[2] - cross_tmp[2] - omega_cross_r[2];

    vec3_cross(state->alpha_b_R, pos_b_R, cross_tmp);
    vec3_cross(gyr_b_R, pos_b_R, omega_cross_r);
    vec3_cross(gyr_b_R, omega_cross_r, omega_cross_r);
    acc_cg_R[0] = acc_b_R[0] - cross_tmp[0] - omega_cross_r[0];
    acc_cg_R[1] = acc_b_R[1] - cross_tmp[1] - omega_cross_r[1];
    acc_cg_R[2] = acc_b_R[2] - cross_tmp[2] - omega_cross_r[2];

    float acc_b_cg[3] = {
        0.5f * (acc_cg_L[0] + acc_cg_R[0]),
        0.5f * (acc_cg_L[1] + acc_cg_R[1]),
        0.5f * (acc_cg_L[2] + acc_cg_R[2])
    };
    float mag_b_cg[3] = {
        0.5f * (mag_b_L[0] + mag_b_R[0]),
        0.5f * (mag_b_L[1] + mag_b_R[1]),
        0.5f * (mag_b_L[2] + mag_b_R[2])
    };
    if (out) {
        out->timestamp_ms = imuL->timestamp_ms;
        out->hinge_theta = theta;
        memcpy(out->acc_b_L, acc_b_L, sizeof(acc_b_L));
        memcpy(out->acc_b_R, acc_b_R, sizeof(acc_b_R));
        memcpy(out->mag_b_L, mag_b_L, sizeof(mag_b_L));
        memcpy(out->mag_b_R, mag_b_R, sizeof(mag_b_R));
        memcpy(out->gyr_b_L, gyr_b_L, sizeof(gyr_b_L));
        memcpy(out->gyr_b_R, gyr_b_R, sizeof(gyr_b_R));
        memcpy(out->gyr_b, gyr_b_avg, sizeof(gyr_b_avg));
        memcpy(out->acc_b_cg, acc_b_cg, sizeof(acc_b_cg));
        memcpy(out->mag_b_cg, mag_b_cg, sizeof(mag_b_cg));
    }
}

static fmt_err_t dual_imu_att_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.hinge.theta = 0.0f;
    g_state.hinge.theta_bias = 0.0f;
    g_state.hinge.P[0] = 1e-3f;
    g_state.hinge.P[3] = 1e-3f;
    memset(&g_att_out, 0, sizeof(g_att_out));
#if DUAL_IMU_ATT_STATUS_ENABLE
    memset(&g_att_status, 0, sizeof(g_att_status));
#endif

    mcn_advertise(MCN_HUB(dual_imu_att), echo_dual_imu_att);
#if DUAL_IMU_ATT_STATUS_ENABLE
    mcn_advertise(MCN_HUB(dual_imu_att_status), NULL);
#endif
    return FMT_EOK;
}

static void dual_imu_att_entry(void* parameter)
{
    DEFINE_TIMETAG(loop_tt, 1); /* target 1000 Hz */
    uint32_t last_ts_ms = 0;

    while (1) {
        if (check_timetag(TIMETAG(loop_tt))) {
            uint32_t flags = 0;
            uint32_t obs_flags = 0;
            float acc_norm_L = 0.0f;
            float acc_norm_R = 0.0f;
            float mag_norm_L = 0.0f;
            float mag_norm_R = 0.0f;

            (void)mcn_copy_from_hub(MCN_HUB(sensor_imu0), &g_state.imu_local);
            (void)mcn_copy_from_hub(MCN_HUB(bridge_imu), &g_state.imu_remote);
            (void)mcn_copy_from_hub(MCN_HUB(sensor_mag0), &g_state.mag_local);
            (void)mcn_copy_from_hub(MCN_HUB(bridge_mag), &g_state.mag_remote);

            /* derive dt from local imu timestamp to match actual data rate */
            uint32_t cur_ts_ms = g_state.imu_local.timestamp_ms;
            uint32_t dt_ms = 0;
            if (last_ts_ms != 0 && cur_ts_ms > last_ts_ms) {
                dt_ms = cur_ts_ms - last_ts_ms;
            }
            float dt = dt_ms > 0 ? ((float)dt_ms / 1000.0f) : DUAL_IMU_DT_DEFAULT_S;
            if (last_ts_ms != 0) {
                if (dt_ms == 0 || dt_ms > DUAL_IMU_MAX_IMU_STALE_MS) {
                    flags |= DUAL_IMU_STATUS_IMU_LOCAL_STALE;
                }
                if (dt < DUAL_IMU_DT_MIN_S || dt > DUAL_IMU_DT_MAX_S) {
                    flags |= DUAL_IMU_STATUS_IMU_LOCAL_STALE;
                    dt = DUAL_IMU_DT_DEFAULT_S;
                }
            }
            last_ts_ms = cur_ts_ms;

            int32_t skew_ms = (int32_t)g_state.imu_local.timestamp_ms - (int32_t)g_state.imu_remote.timestamp_ms;
            if (abs_i32(skew_ms) > DUAL_IMU_MAX_TIME_SKEW_MS) {
                flags |= DUAL_IMU_STATUS_TIME_SKEW;
            }

            uint32_t remote_age_ms = cur_ts_ms >= g_state.imu_remote.timestamp_ms
                                         ? (cur_ts_ms - g_state.imu_remote.timestamp_ms)
                                         : 0;
            if (remote_age_ms > DUAL_IMU_MAX_IMU_STALE_MS) {
                flags |= DUAL_IMU_STATUS_IMU_REMOTE_STALE;
            }

            uint32_t mag_local_age_ms = cur_ts_ms >= g_state.mag_local.timestamp_ms
                                            ? (cur_ts_ms - g_state.mag_local.timestamp_ms)
                                            : 0;
            uint32_t mag_remote_age_ms = cur_ts_ms >= g_state.mag_remote.timestamp_ms
                                             ? (cur_ts_ms - g_state.mag_remote.timestamp_ms)
                                             : 0;
            if (mag_local_age_ms > DUAL_IMU_MAX_MAG_STALE_MS) {
                flags |= DUAL_IMU_STATUS_MAG_LOCAL_STALE;
            }
            if (mag_remote_age_ms > DUAL_IMU_MAX_MAG_STALE_MS) {
                flags |= DUAL_IMU_STATUS_MAG_REMOTE_STALE;
            }
            bool mag_valid = (flags & (DUAL_IMU_STATUS_MAG_LOCAL_STALE | DUAL_IMU_STATUS_MAG_REMOTE_STALE)) == 0;

            if (!vec3_isfinite(g_state.imu_local.gyr_B_radDs)
                || !vec3_isfinite(g_state.imu_local.acc_B_mDs2)
                || !vec3_isfinite(g_state.imu_remote.gyr_B_radDs)
                || !vec3_isfinite(g_state.imu_remote.acc_B_mDs2)
                || !vec3_isfinite(g_state.mag_local.mag_B_gauss)
                || !vec3_isfinite(g_state.mag_remote.mag_B_gauss)) {
                flags |= DUAL_IMU_STATUS_NAN_DETECTED;
            }

            for (int i = 0; i < 3; i++) {
                if (fabsf(g_state.imu_local.gyr_B_radDs[i]) > DUAL_IMU_GYR_MAX_RAD_S
                    || fabsf(g_state.imu_remote.gyr_B_radDs[i]) > DUAL_IMU_GYR_MAX_RAD_S) {
                    flags |= DUAL_IMU_STATUS_GYR_SAT;
                    break;
                }
            }

            if ((flags & DUAL_IMU_STATUS_NAN_DETECTED) == 0) {
                hinge_update_kf(dt,
                                &g_state.imu_local,
                                &g_state.imu_remote,
                                mag_valid ? &g_state.mag_local : NULL,
                                mag_valid ? &g_state.mag_remote : NULL,
                                &g_state.hinge,
                                &obs_flags,
                                &acc_norm_L,
                                &acc_norm_R,
                                &mag_norm_L,
                                &mag_norm_R);
                att_compute_body_vectors(dt,
                                         g_state.hinge.theta,
                                         &g_state.imu_local,
                                         &g_state.imu_remote,
                                         &g_state.mag_local,
                                         &g_state.mag_remote,
                                         &g_state,
                                         &g_att_out);
            }

            flags |= obs_flags;
            if ((flags & (DUAL_IMU_STATUS_TIME_SKEW
                          | DUAL_IMU_STATUS_IMU_LOCAL_STALE
                          | DUAL_IMU_STATUS_IMU_REMOTE_STALE
                          | DUAL_IMU_STATUS_MAG_LOCAL_STALE
                          | DUAL_IMU_STATUS_MAG_REMOTE_STALE
                          | DUAL_IMU_STATUS_NAN_DETECTED
                          | DUAL_IMU_STATUS_GYR_SAT)) == 0) {
                flags |= DUAL_IMU_STATUS_VALID;
            }

            float gyr_diff_x = 0.0f;
            if ((flags & DUAL_IMU_STATUS_NAN_DETECTED) == 0) {
                gyr_diff_x = g_state.imu_remote.gyr_B_radDs[0] - g_state.imu_local.gyr_B_radDs[0];
            }

#if DUAL_IMU_ATT_STATUS_ENABLE
            g_att_status.timestamp_ms = cur_ts_ms;
            g_att_status.flags = flags;
            g_att_status.time_skew_ms = clamp_i16(skew_ms);
            g_att_status.dt_ms = clamp_u16(dt_ms);
            g_att_status.hinge_theta = g_state.hinge.theta;
            g_att_status.hinge_bias = g_state.hinge.theta_bias;
            g_att_status.acc_norm_L = acc_norm_L;
            g_att_status.acc_norm_R = acc_norm_R;
            g_att_status.mag_norm_L = mag_norm_L;
            g_att_status.mag_norm_R = mag_norm_R;
            g_att_status.gyr_diff_x = gyr_diff_x;
            (void)mcn_publish(MCN_HUB(dual_imu_att_status), &g_att_status);
#endif

            (void)mcn_publish(MCN_HUB(dual_imu_att), &g_att_out);
        }
        sys_msleep(1);
    }
}

#if DUAL_IMU_ATT_ENABLE
TASK_EXPORT __fmt_task_desc = {
    .name = "dual_imu_att",
    .init = dual_imu_att_init,
    .entry = dual_imu_att_entry,
    .priority = 15,
    .auto_start = true,
    .stack_size = 2048,
    .param = NULL,
    .dependency = NULL
};
#endif

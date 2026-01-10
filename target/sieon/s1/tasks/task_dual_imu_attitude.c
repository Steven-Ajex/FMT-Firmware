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

MCN_DEFINE(dual_imu_att, sizeof(dual_imu_att_output_t));

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

static void mat3_mul_vec(const float m[9], const float v[3], float out[3])
{
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

static void rot_x(float angle, float R[9])
{
    float c = cosf(angle);
    float s = sinf(angle);
    R[0] = 1.0f; R[1] = 0.0f; R[2] = 0.0f;
    R[3] = 0.0f; R[4] = c;    R[5] = -s;
    R[6] = 0.0f; R[7] = s;    R[8] = c;
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

static void hinge_update_kf(float dt, const imu_data_t* imuL, const imu_data_t* imuR, const mag_data_t* magL, const mag_data_t* magR, hinge_state_t* h)
{
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
    if ((accnL > (DUAL_IMU_ACC_TRUST_MIN_G * 9.8f)) && (accnL < (DUAL_IMU_ACC_TRUST_MAX_G * 9.8f))
        && (accnR > (DUAL_IMU_ACC_TRUST_MIN_G * 9.8f)) && (accnR < (DUAL_IMU_ACC_TRUST_MAX_G * 9.8f))) {
        float gL[3] = { accL[0] / accnL, accL[1] / accnL, accL[2] / accnL };
        float gR[3] = { accR[0] / accnR, accR[1] / accnR, accR[2] / accnR };
        const float hinge_axis[3] = { 1.0f, 0.0f, 0.0f };
        z = hinge_obs_from_vecs(hinge_axis, gL, gR);
        R_meas = DUAL_IMU_MEAS_VAR_ACC;
        have_obs = true;
    } else if (magL && magR) {
        float mL[3] = { magL->mag_B_gauss[0], magL->mag_B_gauss[1], magL->mag_B_gauss[2] };
        float mR[3] = { magR->mag_B_gauss[0], magR->mag_B_gauss[1], magR->mag_B_gauss[2] };
        vec3_normalize(mL);
        vec3_normalize(mR);
        const float hinge_axis[3] = { 1.0f, 0.0f, 0.0f };
        z = hinge_obs_from_vecs(hinge_axis, mL, mR);
        R_meas = DUAL_IMU_MEAS_VAR_MAG;
        have_obs = true;
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
    /* Build arm rotations */
    float R_arm_L[9];
    float R_arm_R[9];
    rot_x(theta, R_arm_L);
    rot_x(-theta, R_arm_R);

    const float R_mount_L[9] = DUAL_IMU_LEFT_R_MOUNT;
    const float R_mount_R[9] = DUAL_IMU_RIGHT_R_MOUNT;

    /* Compose R_bi = R_arm * R_mount */
    float R_bi_L[9];
    float R_bi_R[9];
    float pos_b_L[3];
    float pos_b_R[3];
    /* Manual mat mul (R_arm * R_mount) */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            R_bi_L[3 * r + c] = R_arm_L[3 * r + 0] * R_mount_L[0 + c]
                                + R_arm_L[3 * r + 1] * R_mount_L[3 + c]
                                + R_arm_L[3 * r + 2] * R_mount_L[6 + c];
            R_bi_R[3 * r + c] = R_arm_R[3 * r + 0] * R_mount_R[0 + c]
                                + R_arm_R[3 * r + 1] * R_mount_R[3 + c]
                                + R_arm_R[3 * r + 2] * R_mount_R[6 + c];
        }
    }
    mat3_mul_vec(R_arm_L, pos_mount_L, pos_b_L);
    mat3_mul_vec(R_arm_R, pos_mount_R, pos_b_R);

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

    mcn_advertise(MCN_HUB(dual_imu_att), echo_dual_imu_att);
    return FMT_EOK;
}

static void dual_imu_att_entry(void* parameter)
{
    DEFINE_TIMETAG(loop_tt, 1); /* target 1000 Hz */
    uint32_t last_ts_ms = 0;

    while (1) {
        if (check_timetag(TIMETAG(loop_tt))) {
            (void)mcn_copy_from_hub(MCN_HUB(sensor_imu0), &g_state.imu_local);
            (void)mcn_copy_from_hub(MCN_HUB(bridge_imu), &g_state.imu_remote);
            (void)mcn_copy_from_hub(MCN_HUB(sensor_mag0), &g_state.mag_local);
            (void)mcn_copy_from_hub(MCN_HUB(bridge_mag), &g_state.mag_remote);

            /* derive dt from local imu timestamp to match actual data rate */
            uint32_t cur_ts_ms = g_state.imu_local.timestamp_ms;
            float dt = 0.001f;
            if (last_ts_ms != 0 && cur_ts_ms > last_ts_ms) {
                dt = (float)(cur_ts_ms - last_ts_ms) / 1000.0f;
            }
            last_ts_ms = cur_ts_ms;

            hinge_update_kf(dt, &g_state.imu_local, &g_state.imu_remote, &g_state.mag_local, &g_state.mag_remote, &g_state.hinge);
            att_compute_body_vectors(dt,
                                     g_state.hinge.theta,
                                     &g_state.imu_local,
                                     &g_state.imu_remote,
                                     &g_state.mag_local,
                                     &g_state.mag_remote,
                                     &g_state,
                                     &g_att_out);

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

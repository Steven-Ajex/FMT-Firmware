#ifndef TASK_DUAL_IMU_ATTITUDE_H__
#define TASK_DUAL_IMU_ATTITUDE_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DUAL_IMU_ATT_STATUS_ENABLE
#define DUAL_IMU_ATT_STATUS_ENABLE 1
#endif

#ifndef DUAL_IMU_ATT_LOG_DIAG
#define DUAL_IMU_ATT_LOG_DIAG 1
#endif

#ifndef DUAL_IMU_STATUS_VALID
#define DUAL_IMU_STATUS_VALID           (1u << 0)
#define DUAL_IMU_STATUS_TIME_SKEW       (1u << 1)
#define DUAL_IMU_STATUS_IMU_LOCAL_STALE (1u << 2)
#define DUAL_IMU_STATUS_IMU_REMOTE_STALE (1u << 3)
#define DUAL_IMU_STATUS_MAG_LOCAL_STALE (1u << 4)
#define DUAL_IMU_STATUS_MAG_REMOTE_STALE (1u << 5)
#define DUAL_IMU_STATUS_ACC_TRUST       (1u << 6)
#define DUAL_IMU_STATUS_MAG_TRUST       (1u << 7)
#define DUAL_IMU_STATUS_NAN_DETECTED    (1u << 8)
#define DUAL_IMU_STATUS_GYR_SAT         (1u << 9)
#endif

typedef struct {
    uint32_t timestamp_ms;
    float hinge_theta;
    float acc_b_L[3];
    float acc_b_R[3];
    float mag_b_L[3];
    float mag_b_R[3];
    float gyr_b_L[3];
    float gyr_b_R[3];
    float gyr_b[3];
    float acc_b_cg[3];
    float mag_b_cg[3];
} dual_imu_att_output_t;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t flags;
    int16_t time_skew_ms;
    uint16_t dt_ms;
    float hinge_theta;
    float hinge_bias;
    float acc_norm_L;
    float acc_norm_R;
    float mag_norm_L;
    float mag_norm_R;
    float gyr_diff_x;
} dual_imu_att_status_t;

#ifdef __cplusplus
}
#endif

#endif /* TASK_DUAL_IMU_ATTITUDE_H__ */

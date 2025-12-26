#ifndef TASK_DUAL_IMU_ATTITUDE_H__
#define TASK_DUAL_IMU_ATTITUDE_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
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

#ifdef __cplusplus
}
#endif

#endif /* TASK_DUAL_IMU_ATTITUDE_H__ */

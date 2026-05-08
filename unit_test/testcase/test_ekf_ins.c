/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Black-box unit tests for the active ins variant (ekf_ins, cf_ins,
 * px4_ecl ...).  Only touches the public INS_U / INS_Y / INS_init /
 * INS_step interface so the same test runs against every variant.
 *****************************************************************************/

#include <firmament.h>
#include <math.h>
#include <utest.h>
#include <INS.h>

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */
static void feed_static_imu(uint32_t now_ms)
{
    INS_U.IMU.timestamp = now_ms;
    INS_U.IMU.gyr_x = 0.0f;
    INS_U.IMU.gyr_y = 0.0f;
    INS_U.IMU.gyr_z = 0.0f;
    INS_U.IMU.acc_x = 0.0f;
    INS_U.IMU.acc_y = 0.0f;
    INS_U.IMU.acc_z = -9.80665f;
}

static void feed_mag_north(uint32_t now_ms)
{
    INS_U.MAG.timestamp = now_ms;
    INS_U.MAG.mag_x = 1.0f;
    INS_U.MAG.mag_y = 0.0f;
    INS_U.MAG.mag_z = 0.0f;
}

/* ------------------------------------------------------------------ */
/*  Tests                                                              */
/* ------------------------------------------------------------------ */
static void test_ins_init_does_not_crash(void)
{
    INS_init();
    uassert_true(isfinite(INS_Y.INS_Out.quat[0]));
    uassert_true(isfinite(INS_Y.INS_Out.phi));
    uassert_true(isfinite(INS_Y.INS_Out.theta));
    uassert_true(isfinite(INS_Y.INS_Out.psi));
}

static void test_static_attitude_converges(void)
{
    INS_init();
    feed_mag_north(1);
    for (uint32_t i = 0; i < 200; i++) {
        feed_static_imu(2 + i * 2);
        INS_U.MAG.timestamp = 2 + i * 2;
        INS_step();
    }

    /* tilt should be within 1 deg, yaw within 5 deg of 0 */
    uassert_true(fabsf(INS_Y.INS_Out.phi)   < 0.0175f);
    uassert_true(fabsf(INS_Y.INS_Out.theta) < 0.0175f);
    uassert_true(fabsf(INS_Y.INS_Out.psi)   < 0.0873f);

    /* quaternion stays unit norm */
    real32_T qn = sqrtf(INS_Y.INS_Out.quat[0] * INS_Y.INS_Out.quat[0]
                      + INS_Y.INS_Out.quat[1] * INS_Y.INS_Out.quat[1]
                      + INS_Y.INS_Out.quat[2] * INS_Y.INS_Out.quat[2]
                      + INS_Y.INS_Out.quat[3] * INS_Y.INS_Out.quat[3]);
    uassert_true(fabsf(qn - 1.0f) < 1e-3f);
}

static void test_outputs_are_finite_after_many_steps(void)
{
    INS_init();
    feed_mag_north(1);
    for (uint32_t i = 0; i < 5000; i++) {
        feed_static_imu(2 + i * 2);
        INS_U.MAG.timestamp = 2 + i * 2;
        INS_step();
    }
    uassert_true(isfinite(INS_Y.INS_Out.phi));
    uassert_true(isfinite(INS_Y.INS_Out.theta));
    uassert_true(isfinite(INS_Y.INS_Out.psi));
    uassert_true(isfinite(INS_Y.INS_Out.vn));
    uassert_true(isfinite(INS_Y.INS_Out.ve));
    uassert_true(isfinite(INS_Y.INS_Out.vd));
    uassert_true(isfinite(INS_Y.INS_Out.x_R));
    uassert_true(isfinite(INS_Y.INS_Out.y_R));
    uassert_true(isfinite(INS_Y.INS_Out.h_R));
}

static void test_imu_passthrough(void)
{
    INS_init();
    feed_mag_north(1);
    /* run a few warm-up steps so init_done becomes true */
    for (uint32_t i = 0; i < 5; i++) {
        feed_static_imu(2 + i * 2);
        INS_U.MAG.timestamp = 2 + i * 2;
        INS_step();
    }
    /* now feed a non-trivial gyro / accel and check the post-bias output */
    INS_U.IMU.timestamp = 100;
    INS_U.IMU.gyr_x = 0.1f;  INS_U.IMU.gyr_y = 0.2f;  INS_U.IMU.gyr_z = -0.3f;
    INS_U.IMU.acc_x = 0.5f;  INS_U.IMU.acc_y = -0.5f; INS_U.IMU.acc_z = -9.0f;
    INS_step();
    /* p, q, r are output as omega_meas - b_g; biases are tiny so output
     * should be very close to the raw gyro value. */
    uassert_true(fabsf(INS_Y.INS_Out.p - 0.1f)  < 0.05f);
    uassert_true(fabsf(INS_Y.INS_Out.q - 0.2f)  < 0.05f);
    uassert_true(fabsf(INS_Y.INS_Out.r + 0.3f)  < 0.05f);
}

/* ------------------------------------------------------------------ */
/*  utest entry points                                                 */
/* ------------------------------------------------------------------ */
static rt_err_t testcase_init(void)    { return RT_EOK; }
static rt_err_t testcase_cleanup(void) { return RT_EOK; }

static void testcase(void)
{
    UTEST_UNIT_RUN(test_ins_init_does_not_crash);
    UTEST_UNIT_RUN(test_static_attitude_converges);
    UTEST_UNIT_RUN(test_outputs_are_finite_after_many_steps);
    UTEST_UNIT_RUN(test_imu_passthrough);
}
UTEST_TC_EXPORT(testcase, "utest.ins.ekf_ins", testcase_init, testcase_cleanup, 30);

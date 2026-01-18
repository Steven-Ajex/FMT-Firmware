/******************************************************************************
 * Log bridge_imu, bridge_mag and dual_imu_att without blocking producers.
 *****************************************************************************/

#include <firmament.h>

#include "module/ipc/uMCN.h"
#include "module/log/mlog.h"
#include "module/sensor/sensor_hub.h"
#include "module/system/systime.h"
#include "task_dual_imu_attitude.h"
#include "module/task_manager/task_manager.h"

#ifndef DUAL_IMU_STATUS_LOG_PERIOD_MS
#define DUAL_IMU_STATUS_LOG_PERIOD_MS 20
#endif

MCN_DECLARE(bridge_imu);
MCN_DECLARE(bridge_mag);
MCN_DECLARE(dual_imu_att);
#if DUAL_IMU_ATT_STATUS_ENABLE
MCN_DECLARE(dual_imu_att_status);
#endif

static mlog_elem_t BridgeIMU_Elems[] = {
    MLOG_ELEMENT(timestamp_ms, MLOG_UINT32),
    MLOG_ELEMENT_VEC(gyr_B_radDs, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_B_mDs2, MLOG_FLOAT, 3),
};
static mlog_elem_t BridgeMAG_Elems[] = {
    MLOG_ELEMENT(timestamp_ms, MLOG_UINT32),
    MLOG_ELEMENT_VEC(mag_B_gauss, MLOG_FLOAT, 3),
};
static mlog_elem_t DualIMUAtt_Elems[] = {
    MLOG_ELEMENT(timestamp_ms, MLOG_UINT32),
    MLOG_ELEMENT(hinge_theta, MLOG_FLOAT),
    MLOG_ELEMENT_VEC(acc_b_L, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_b_R, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(mag_b_L, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(mag_b_R, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyr_b_L, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyr_b_R, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(gyr_b, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_b_cg, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(mag_b_cg, MLOG_FLOAT, 3),
};

MLOG_BUS_DEFINE(bridge_imu, BridgeIMU_Elems);
MLOG_BUS_DEFINE(bridge_mag, BridgeMAG_Elems);
MLOG_BUS_DEFINE(dual_imu_att, DualIMUAtt_Elems);

#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
static mlog_elem_t DualIMUAttStatus_Elems[] = {
    MLOG_ELEMENT(timestamp_ms, MLOG_UINT32),
    MLOG_ELEMENT(flags, MLOG_UINT32),
    MLOG_ELEMENT(time_skew_ms, MLOG_INT16),
    MLOG_ELEMENT(dt_ms, MLOG_UINT16),
    MLOG_ELEMENT(hinge_theta, MLOG_FLOAT),
    MLOG_ELEMENT(hinge_bias, MLOG_FLOAT),
    MLOG_ELEMENT(acc_norm_L, MLOG_FLOAT),
    MLOG_ELEMENT(acc_norm_R, MLOG_FLOAT),
    MLOG_ELEMENT(mag_norm_L, MLOG_FLOAT),
    MLOG_ELEMENT(mag_norm_R, MLOG_FLOAT),
    MLOG_ELEMENT(gyr_diff_x, MLOG_FLOAT),
};
MLOG_BUS_DEFINE(dual_imu_att_status, DualIMUAttStatus_Elems);
#endif

static int bridge_imu_id = -1;
static int bridge_mag_id = -1;
static int dual_imu_att_id = -1;
#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
static int dual_imu_att_status_id = -1;
#endif

static McnNode_t bridge_imu_node = NULL;
static McnNode_t bridge_mag_node = NULL;
static McnNode_t dual_imu_att_node = NULL;
#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
static McnNode_t dual_imu_att_status_node = NULL;
#endif

static void log_bridge_imu(void)
{
    imu_data_t data;

    if (bridge_imu_id < 0 || bridge_imu_node == NULL) {
        return;
    }

    while (mcn_poll(bridge_imu_node)) {
        if (mcn_copy(MCN_HUB(bridge_imu), bridge_imu_node, &data) == FMT_EOK) {
            (void)mlog_push_msg((uint8_t*)&data, (uint8_t)bridge_imu_id, sizeof(data));
        }
    }
}

static void log_bridge_mag(void)
{
    mag_data_t data;

    if (bridge_mag_id < 0 || bridge_mag_node == NULL) {
        return;
    }

    while (mcn_poll(bridge_mag_node)) {
        if (mcn_copy(MCN_HUB(bridge_mag), bridge_mag_node, &data) == FMT_EOK) {
            (void)mlog_push_msg((uint8_t*)&data, (uint8_t)bridge_mag_id, sizeof(data));
        }
    }
}

static void log_dual_imu_att(void)
{
    dual_imu_att_output_t data;

    if (dual_imu_att_id < 0 || dual_imu_att_node == NULL) {
        return;
    }

    while (mcn_poll(dual_imu_att_node)) {
        if (mcn_copy(MCN_HUB(dual_imu_att), dual_imu_att_node, &data) == FMT_EOK) {
            (void)mlog_push_msg((uint8_t*)&data, (uint8_t)dual_imu_att_id, sizeof(data));
        }
    }
}

#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
static void log_dual_imu_att_status(void)
{
    static dual_imu_att_status_t data;
    static uint8_t have_data = 0;
    DEFINE_TIMETAG(status_log_tt, DUAL_IMU_STATUS_LOG_PERIOD_MS);

    if (dual_imu_att_status_id < 0 || dual_imu_att_status_node == NULL) {
        return;
    }

    while (mcn_poll(dual_imu_att_status_node)) {
        if (mcn_copy(MCN_HUB(dual_imu_att_status), dual_imu_att_status_node, &data) == FMT_EOK) {
            have_data = 1;
        }
    }

    if (have_data && check_timetag(TIMETAG(status_log_tt))) {
        (void)mlog_push_msg((uint8_t*)&data, (uint8_t)dual_imu_att_status_id, sizeof(data));
    }
}
#endif

fmt_err_t task_bridge_mlog_init(void)
{
    bridge_imu_id = mlog_get_bus_id("bridge_imu");
    bridge_mag_id = mlog_get_bus_id("bridge_mag");
    dual_imu_att_id = mlog_get_bus_id("dual_imu_att");
#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
    dual_imu_att_status_id = mlog_get_bus_id("dual_imu_att_status");
#endif

    bridge_imu_node = mcn_subscribe(MCN_HUB(bridge_imu), NULL);
    bridge_mag_node = mcn_subscribe(MCN_HUB(bridge_mag), NULL);
    dual_imu_att_node = mcn_subscribe(MCN_HUB(dual_imu_att), NULL);
#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
    dual_imu_att_status_node = mcn_subscribe(MCN_HUB(dual_imu_att_status), NULL);
#endif

    return FMT_EOK;
}

void task_bridge_mlog_entry(void* parameter)
{
    /* Lower priority logging loop to avoid blocking producers */
    while (1) {
        log_bridge_imu();
        log_bridge_mag();
        log_dual_imu_att();
#if DUAL_IMU_ATT_STATUS_ENABLE && DUAL_IMU_ATT_LOG_DIAG
        log_dual_imu_att_status();
#endif
        sys_msleep(1);
    }
}

TASK_EXPORT __fmt_task_desc = {
    .name = "bridge_mlog",
    .init = task_bridge_mlog_init,
    .entry = task_bridge_mlog_entry,
    .priority = 20,
    .auto_start = true,
    .stack_size = 1024,
    .param = NULL,
    .dependency = (char*[]) { "logger", NULL }
};

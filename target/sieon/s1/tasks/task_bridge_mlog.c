/******************************************************************************
 * Log bridge_imu, bridge_mag and dual_imu_att without blocking producers.
 *****************************************************************************/

#include <firmament.h>

#include "module/ipc/uMCN.h"
#include "module/log/mlog.h"
#include "module/sensor/sensor_hub.h"
#include "task_dual_imu_attitude.h"
#include "module/task_manager/task_manager.h"

MCN_DECLARE(bridge_imu);
MCN_DECLARE(bridge_mag);
MCN_DECLARE(dual_imu_att);

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
    MLOG_ELEMENT_VEC(gyr_b, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(acc_b_cg, MLOG_FLOAT, 3),
    MLOG_ELEMENT_VEC(mag_b_cg, MLOG_FLOAT, 3),
};

MLOG_BUS_DEFINE(bridge_imu, BridgeIMU_Elems);
MLOG_BUS_DEFINE(bridge_mag, BridgeMAG_Elems);
MLOG_BUS_DEFINE(dual_imu_att, DualIMUAtt_Elems);

static int bridge_imu_id = -1;
static int bridge_mag_id = -1;
static int dual_imu_att_id = -1;

static McnNode_t bridge_imu_node = NULL;
static McnNode_t bridge_mag_node = NULL;
static McnNode_t dual_imu_att_node = NULL;

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

fmt_err_t task_bridge_mlog_init(void)
{
    bridge_imu_id = mlog_get_bus_id("bridge_imu");
    bridge_mag_id = mlog_get_bus_id("bridge_mag");
    dual_imu_att_id = mlog_get_bus_id("dual_imu_att");

    bridge_imu_node = mcn_subscribe(MCN_HUB(bridge_imu), NULL);
    bridge_mag_node = mcn_subscribe(MCN_HUB(bridge_mag), NULL);
    dual_imu_att_node = mcn_subscribe(MCN_HUB(dual_imu_att), NULL);

    return FMT_EOK;
}

void task_bridge_mlog_entry(void* parameter)
{
    /* Lower priority logging loop to avoid blocking producers */
    while (1) {
        log_bridge_imu();
        log_bridge_mag();
        log_dual_imu_att();
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

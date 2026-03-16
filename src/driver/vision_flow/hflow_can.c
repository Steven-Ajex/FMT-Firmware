/******************************************************************************
 * Copyright 2024 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "driver/vision_flow/hflow_can.h"

#include <string.h>

#include "canard.h"
#include "com.hex.equipment.flow.Measurement.h"
#include "hal/can/can.h"
#define LOG_TAG "hflow_can"
#define LOG_LVL LOG_LVL_INFO

#include "module/log/mlog.h"
#include "module/log/ulog.h"
#include "module/sensor/sensor_hub.h"
#define HFLOW_POOL_SIZE      2048
#define EVENT_HFLOW_UPDATE   (1 << 0)

MCN_DECLARE(sensor_optflow);

typedef struct {
    uint32_t timestamp_ms;
    float    integration_interval_s;
    float    rate_gyro_integral_x;
    float    rate_gyro_integral_y;
    float    flow_integral_x;
    float    flow_integral_y;
    float    flow_rate_x;
    float    flow_rate_y;
    uint8_t  quality;
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  reserved3;
} hflow_log_t;

static mlog_elem_t HFlow_Elems[] = {
    MLOG_ELEMENT(timestamp_ms, MLOG_UINT32),
    MLOG_ELEMENT(integration_interval_s, MLOG_FLOAT),
    MLOG_ELEMENT(rate_gyro_integral_x, MLOG_FLOAT),
    MLOG_ELEMENT(rate_gyro_integral_y, MLOG_FLOAT),
    MLOG_ELEMENT(flow_integral_x, MLOG_FLOAT),
    MLOG_ELEMENT(flow_integral_y, MLOG_FLOAT),
    MLOG_ELEMENT(flow_rate_x, MLOG_FLOAT),
    MLOG_ELEMENT(flow_rate_y, MLOG_FLOAT),
    MLOG_ELEMENT(quality, MLOG_UINT8),
    MLOG_ELEMENT(reserved1, MLOG_UINT8),
    MLOG_ELEMENT(reserved2, MLOG_UINT8),
    MLOG_ELEMENT(reserved3, MLOG_UINT8),
};
MLOG_BUS_DEFINE(HFlow, HFlow_Elems);

static rt_device_t  can_dev;
static rt_thread_t  thread;
static struct rt_event event;
static int          hflow_bus_id = -1;

static CanardInstance g_hflow_canard;
static uint8_t g_hflow_pool[HFLOW_POOL_SIZE];

static optflow_data_t g_optflow;
static hflow_log_t g_hflow_log;

static void canmsg_to_canard(const can_msg* msg, CanardCANFrame* out_frame)
{
    uint32_t id = 0;

    if (msg->id_type == CAN_ID_EXTENDED) {
        id |= CANARD_CAN_FRAME_EFF;
        id |= (msg->ext_id & CANARD_CAN_EXT_ID_MASK);
    } else {
        id |= (msg->std_id & CANARD_CAN_STD_ID_MASK);
    }

    if (msg->frame_type == CAN_FRAME_REMOTE) {
        id |= CANARD_CAN_FRAME_RTR;
    }

    out_frame->id = id;
    out_frame->data_len = (msg->data_len > CANARD_CAN_FRAME_MAX_DATA_LEN)
                              ? CANARD_CAN_FRAME_MAX_DATA_LEN
                              : (uint8_t)msg->data_len;
    out_frame->iface_id = 0;
    memset(out_frame->data, 0, sizeof(out_frame->data));
    memcpy(out_frame->data, msg->data, out_frame->data_len);
}

static void handle_flow_measurement(const struct com_hex_equipment_flow_Measurement* msg)
{
    const uint32_t time_now = systime_now_ms();
    float integration = msg->integration_interval;

    if (integration <= 1e-6f) {
        integration = 1e-6f;
    }

    g_optflow.timestamp_ms = time_now;
    g_optflow.vx_mPs = msg->flow_integral[0] / integration;
    g_optflow.vy_mPs = msg->flow_integral[1] / integration;
    g_optflow.quality = msg->quality;

    g_hflow_log.timestamp_ms = time_now;
    g_hflow_log.integration_interval_s = msg->integration_interval;
    g_hflow_log.rate_gyro_integral_x = msg->rate_gyro_integral[0];
    g_hflow_log.rate_gyro_integral_y = msg->rate_gyro_integral[1];
    g_hflow_log.flow_integral_x = msg->flow_integral[0];
    g_hflow_log.flow_integral_y = msg->flow_integral[1];
    g_hflow_log.flow_rate_x = g_optflow.vx_mPs;
    g_hflow_log.flow_rate_y = g_optflow.vy_mPs;
    g_hflow_log.quality = msg->quality;
    g_hflow_log.reserved1 = 0;
    g_hflow_log.reserved2 = 0;
    g_hflow_log.reserved3 = 0;

    mcn_publish(MCN_HUB(sensor_optflow), &g_optflow);

    if (hflow_bus_id >= 0) {
        (void)mlog_push_msg((uint8_t*)&g_hflow_log, hflow_bus_id, sizeof(g_hflow_log));
    }
}

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (transfer->data_type_id == COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_ID) {
        struct com_hex_equipment_flow_Measurement flow_msg;
        com_hex_equipment_flow_Measurement_decode(transfer, &flow_msg);
        handle_flow_measurement(&flow_msg);
    }

    canardReleaseRxTransferPayload(ins, transfer);
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)ins;
    (void)transfer_type;
    (void)source_node_id;

    if (data_type_id == COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_ID) {
        *out_data_type_signature = COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_SIGNATURE;
        return true;
    }

    return false;
}

static rt_err_t rx_ind_cb(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    (void)size;
    return rt_event_send(&event, EVENT_HFLOW_UPDATE);
}

static void thread_entry(void* args)
{
    rt_err_t res;
    rt_uint32_t recv_set = 0;
    rt_uint32_t wait_set = EVENT_HFLOW_UPDATE;
    can_msg rx_msg;
    CanardCANFrame frame;
    uint32_t tick = 0;

    while (1) {
        res = rt_event_recv(&event, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 10, &recv_set);

        if ((res == RT_EOK && (recv_set & EVENT_HFLOW_UPDATE)) || res == -RT_ETIMEOUT) {
            while (rt_device_read(can_dev, 0, &rx_msg, sizeof(rx_msg)) > 0) {
                canmsg_to_canard(&rx_msg, &frame);
                (void)canardHandleRxFrame(&g_hflow_canard, &frame, systime_now_us());
            }
        }

        if (++tick >= 1000) {
            tick = 0;
            canardCleanupStaleTransfers(&g_hflow_canard, systime_now_us());
        }
    }
}

rt_err_t drv_hflow_can_init(const char* can_dev_name)
{
    const char* name = (can_dev_name != NULL) ? can_dev_name : "can1";

    can_dev = rt_device_find(name);
    if (can_dev == NULL && strcmp(name, "can1") == 0) {
        can_dev = rt_device_find("fdcan1");
    }

    if (can_dev == NULL) {
        ulog_w(LOG_TAG, "can device not found: %s", name);
        return FMT_EEMPTY;
    }

    if (rt_device_open(can_dev, RT_DEVICE_OFLAG_RDONLY | RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        ulog_e(LOG_TAG, "open can device failed: %s", rt_device_get_name(can_dev));
        return FMT_ERROR;
    }

    RT_CHECK(rt_event_init(&event, "hflow", RT_IPC_FLAG_FIFO));
    RT_CHECK(rt_device_set_rx_indicate(can_dev, rx_ind_cb));

    canardInit(&g_hflow_canard,
               g_hflow_pool,
               sizeof(g_hflow_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    hflow_bus_id = mlog_get_bus_id("HFlow");
    if (hflow_bus_id < 0) {
        ulog_w(LOG_TAG, "mlog bus HFlow not found");
    }

    thread = rt_thread_create("hflow",
                              thread_entry,
                              RT_NULL,
                              2048,
                              8,
                              1);
    RT_ASSERT(thread != NULL);
    RT_CHECK(rt_thread_startup(thread));

    ulog_i(LOG_TAG, "H-Flow CAN started on %s", rt_device_get_name(can_dev));

    return RT_EOK;
}

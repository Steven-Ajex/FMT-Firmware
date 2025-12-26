/******************************************************************************
 * Simple CAN sensor bridge: publish local IMU/MAG via CAN and republish the
 * remote copy into MCN topics `bridge_imu` / `bridge_mag`.
 *****************************************************************************/
#include <firmament.h>

#include <limits.h>
#include <string.h>

#include "hal/can/can.h"
#include "module/ipc/uMCN.h"
#include "module/sensor/sensor_hub.h"
#include "module/system/systime.h"
#include "module/task_manager/task_manager.h"

#define BRIDGE_CAN_DEVICE      "can2"
#define BRIDGE_ID_BASE         0x120
#define BRIDGE_ID(seg)         (BRIDGE_ID_BASE + (seg))
#define BRIDGE_TX_PERIOD_MS    1 /* 500 Hz, source IMU is 1 kHz */

#define BRIDGE_GYR_SCALE       1000.0f /* rad/s -> mdps */
#define BRIDGE_ACC_SCALE       1000.0f /* m/s^2 -> mg */
#define BRIDGE_MAG_SCALE       1000.0f /* gauss -> mGauss */

/* role control: override at build time if needed
 * e.g. add to CPPDEFINES: BRIDGE_ENABLE_TX=1, BRIDGE_ENABLE_RX=0 on TX-only node
 */
#ifndef BRIDGE_ENABLE_TX
    #define BRIDGE_ENABLE_TX 1
#endif
#ifndef BRIDGE_ENABLE_RX
    #define BRIDGE_ENABLE_RX 0
#endif

/* MCN topics we consume */
MCN_DECLARE(sensor_imu0);
MCN_DECLARE(sensor_mag0);

/* MCN topics we produce on the receiver */
MCN_DEFINE(bridge_imu, sizeof(imu_data_t));
MCN_DEFINE(bridge_mag, sizeof(mag_data_t));

static int echo_bridge_imu(void* param)
{
    imu_data_t imu;

    if (mcn_copy_from_hub((McnHub*)param, &imu) != FMT_EOK) {
        return -1;
    }

    console_printf("ts:%u gyr:%.4f %.4f %.4f acc:%.4f %.4f %.4f\n",
                   imu.timestamp_ms,
                   imu.gyr_B_radDs[0],
                   imu.gyr_B_radDs[1],
                   imu.gyr_B_radDs[2],
                   imu.acc_B_mDs2[0],
                   imu.acc_B_mDs2[1],
                   imu.acc_B_mDs2[2]);
    return 0;
}

static int echo_bridge_mag(void* param)
{
    mag_data_t mag;

    if (mcn_copy_from_hub((McnHub*)param, &mag) != FMT_EOK) {
        return -1;
    }

    console_printf("ts:%u mag:%.4f %.4f %.4f\n",
                   mag.timestamp_ms,
                   mag.mag_B_gauss[0],
                   mag.mag_B_gauss[1],
                   mag.mag_B_gauss[2]);
    return 0;
}

typedef struct {
    uint8_t seq;
    uint8_t mask;
    uint16_t ts_low;
    uint16_t ts_high;
    int16_t gyr[3];
    int16_t acc[3];
    int16_t mag[3];
    uint16_t status;
} bridge_accum_t;

static rt_device_t can_dev;
static uint8_t tx_seq;
static bridge_accum_t rx_accum;

static void write_i16(uint8_t* dst, int16_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t read_i16(const uint8_t* src)
{
    return (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static int16_t float_to_i16(float v, float scale)
{
    int32_t tmp = (int32_t)(v * scale);

    if (tmp > INT16_MAX) {
        tmp = INT16_MAX;
    } else if (tmp < INT16_MIN) {
        tmp = INT16_MIN;
    }

    return (int16_t)tmp;
}

static void send_segment(can_msg* msg,
                         uint8_t seg,
                         uint8_t seq,
                         int16_t v0,
                         int16_t v1,
                         int16_t v2)
{
    memset(msg, 0, sizeof(*msg));
    msg->std_id = BRIDGE_ID(seg);
    msg->ext_id = msg->std_id;
    msg->id_type = CAN_ID_STANDARD;
    msg->frame_type = CAN_FRAME_DATA;
    msg->data_len = 8;

    msg->data[0] = seq;
    msg->data[1] = seg;
    write_i16(&msg->data[2], v0);
    write_i16(&msg->data[4], v1);
    write_i16(&msg->data[6], v2);
}

static void bridge_tx_once(void)
{
    imu_data_t imu = { 0 };
    mag_data_t mag = { 0 };
    fmt_err_t imu_res = mcn_copy_from_hub(MCN_HUB(sensor_imu0), &imu);
    fmt_err_t mag_res = mcn_copy_from_hub(MCN_HUB(sensor_mag0), &mag);

    if (imu_res != FMT_EOK && mag_res != FMT_EOK) {
        return;
    }

    uint16_t ts_low = (uint16_t)(imu.timestamp_ms & 0xFFFF);
    uint16_t ts_high = (uint16_t)(imu.timestamp_ms >> 16);
    uint16_t status = 0;

    if (imu_res == FMT_EOK) {
        status |= 0x01;
    }
    if (mag_res == FMT_EOK) {
        status |= 0x02;
    }

    can_msg frames[4];
    send_segment(&frames[0],
                 0,
                 tx_seq,
                 (int16_t)ts_low,
                 float_to_i16(imu.gyr_B_radDs[0], BRIDGE_GYR_SCALE),
                 float_to_i16(imu.gyr_B_radDs[1], BRIDGE_GYR_SCALE));

    send_segment(&frames[1],
                 1,
                 tx_seq,
                 float_to_i16(imu.gyr_B_radDs[2], BRIDGE_GYR_SCALE),
                 float_to_i16(imu.acc_B_mDs2[0], BRIDGE_ACC_SCALE),
                 float_to_i16(imu.acc_B_mDs2[1], BRIDGE_ACC_SCALE));

    send_segment(&frames[2],
                 2,
                 tx_seq,
                 float_to_i16(imu.acc_B_mDs2[2], BRIDGE_ACC_SCALE),
                 float_to_i16(mag.mag_B_gauss[0], BRIDGE_MAG_SCALE),
                 float_to_i16(mag.mag_B_gauss[1], BRIDGE_MAG_SCALE));

    send_segment(&frames[3],
                 3,
                 tx_seq,
                 float_to_i16(mag.mag_B_gauss[2], BRIDGE_MAG_SCALE),
                 (int16_t)ts_high,
                 (int16_t)status);

    /* short timeout (ticks) to avoid blocking vehicle loop */
    (void)rt_device_write(can_dev, RT_WAITING_NO, frames, sizeof(frames) / sizeof(frames[0]));

    tx_seq++;
}

static void bridge_rx_reset(uint8_t seq)
{
    memset(&rx_accum, 0, sizeof(rx_accum));
    rx_accum.seq = seq;
}

static void bridge_publish_if_ready(void)
{
    if (rx_accum.mask != 0x0F) {
        return;
    }

    imu_data_t imu = { 0 };
    mag_data_t mag = { 0 };

    uint32_t timestamp = ((uint32_t)rx_accum.ts_high << 16) | rx_accum.ts_low;

    imu.timestamp_ms = timestamp;
    mag.timestamp_ms = timestamp;

    imu.gyr_B_radDs[0] = rx_accum.gyr[0] / BRIDGE_GYR_SCALE;
    imu.gyr_B_radDs[1] = rx_accum.gyr[1] / BRIDGE_GYR_SCALE;
    imu.gyr_B_radDs[2] = rx_accum.gyr[2] / BRIDGE_GYR_SCALE;
    imu.acc_B_mDs2[0] = rx_accum.acc[0] / BRIDGE_ACC_SCALE;
    imu.acc_B_mDs2[1] = rx_accum.acc[1] / BRIDGE_ACC_SCALE;
    imu.acc_B_mDs2[2] = rx_accum.acc[2] / BRIDGE_ACC_SCALE;

    mag.mag_B_gauss[0] = rx_accum.mag[0] / BRIDGE_MAG_SCALE;
    mag.mag_B_gauss[1] = rx_accum.mag[1] / BRIDGE_MAG_SCALE;
    mag.mag_B_gauss[2] = rx_accum.mag[2] / BRIDGE_MAG_SCALE;

    mcn_publish(MCN_HUB(bridge_imu), &imu);
    mcn_publish(MCN_HUB(bridge_mag), &mag);

    rx_accum.mask = 0;
}

static void bridge_rx_handle(const can_msg* msg)
{
    if (msg->id_type != CAN_ID_STANDARD) {
        return;
    }

    if (msg->std_id < BRIDGE_ID_BASE || msg->std_id > BRIDGE_ID(3)) {
        return;
    }

    uint8_t seg = msg->data[1];
    uint8_t seq = msg->data[0];

    if (seg > 3) {
        return;
    }

    /* start new assembly if seq changed */
    if (seq != rx_accum.seq || rx_accum.mask == 0) {
        bridge_rx_reset(seq);
    }

    switch (seg) {
    case 0:
        rx_accum.ts_low = (uint16_t)read_i16(&msg->data[2]);
        rx_accum.gyr[0] = read_i16(&msg->data[4]);
        rx_accum.gyr[1] = read_i16(&msg->data[6]);
        break;
    case 1:
        rx_accum.gyr[2] = read_i16(&msg->data[2]);
        rx_accum.acc[0] = read_i16(&msg->data[4]);
        rx_accum.acc[1] = read_i16(&msg->data[6]);
        break;
    case 2:
        rx_accum.acc[2] = read_i16(&msg->data[2]);
        rx_accum.mag[0] = read_i16(&msg->data[4]);
        rx_accum.mag[1] = read_i16(&msg->data[6]);
        break;
    case 3:
        rx_accum.mag[2] = read_i16(&msg->data[2]);
        rx_accum.ts_high = (uint16_t)read_i16(&msg->data[4]);
        rx_accum.status = (uint16_t)read_i16(&msg->data[6]);
        break;
    default:
        return;
    }

    rx_accum.mask |= (1u << seg);
    bridge_publish_if_ready();
}

static fmt_err_t bridge_can_init(void)
{
    can_dev = rt_device_find(BRIDGE_CAN_DEVICE);
    if (can_dev == RT_NULL) {
        return FMT_ERROR;
    }

    rt_uint16_t open_flag = RT_DEVICE_OFLAG_RDWR;
#if BRIDGE_ENABLE_RX
    open_flag |= RT_DEVICE_FLAG_INT_RX;
#endif

    if (rt_device_open(can_dev, open_flag) != RT_EOK) {
        return FMT_ERROR;
    }

#if BRIDGE_ENABLE_RX
    /* Limit RX load to our IDs */
    struct can_filter filter = {
        .filter_type = CAN_FILTER_TYPE_RANGE,
        .filter_id1 = BRIDGE_ID_BASE,
        .filter_id2 = BRIDGE_ID(3),
    };
    (void)rt_device_control(can_dev, CAN_SET_RX_FILTER, &filter);
#endif

    return FMT_EOK;
}

static void task_can_bridge_entry(void* parameter)
{
#if BRIDGE_ENABLE_TX
    DEFINE_TIMETAG(tx_tt, BRIDGE_TX_PERIOD_MS);
#endif

    bridge_rx_reset(0);

    while (1) {
        can_msg msg;
#if BRIDGE_ENABLE_RX
        /* drain rx fifo without blocking */
        while (rt_device_read(can_dev, 0, &msg, 1) > 0) {
            bridge_rx_handle(&msg);
        }
#endif

#if BRIDGE_ENABLE_TX
        if (check_timetag(TIMETAG(tx_tt))) {
            bridge_tx_once();
        }
#endif

        sys_msleep(1);
    }
}

fmt_err_t task_can_bridge_init(void)
{
    if (bridge_can_init() != FMT_EOK) {
        return FMT_ERROR;
    }

#if BRIDGE_ENABLE_RX
    /* advertise bridge topics so subscribers can attach */
    mcn_advertise(MCN_HUB(bridge_imu), echo_bridge_imu);
    mcn_advertise(MCN_HUB(bridge_mag), echo_bridge_mag);
#endif

    return FMT_EOK;
}

TASK_EXPORT __fmt_task_desc = {
    .name = "can_bridge",
    .init = task_can_bridge_init,
    .entry = task_can_bridge_entry,
    .priority = 8,
    .auto_start = true,
    .stack_size = 2048,
    .param = NULL,
    .dependency = NULL
};

/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
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

/*
 * Optional dedicated sensor sampling task.
 *
 * When FMT_USING_SENSOR_TASK is defined, sensor sampling (the blocking SPI/I2C
 * reads inside sensor_collect()) runs in this task instead of inline in the
 * vehicle control loop. The task runs at a priority above the vehicle task so
 * a fresh sample set is published before the control loop consumes it via
 * uMCN, and bus-access jitter is no longer injected directly into the control
 * loop timing.
 *
 * To enable:
 *   1. define FMT_USING_SENSOR_TASK (e.g. in the target fmtconfig.h)
 *   2. add 'sensor/*.c' to the target's TASKS list (config/task.py)
 *
 * When the macro is not defined this file compiles to nothing and the vehicle
 * task keeps calling sensor_collect() inline (default behaviour).
 */

#include <firmament.h>

#ifdef FMT_USING_SENSOR_TASK

#include "module/sensor/sensor_hub.h"
#include "module/task_manager/task_manager.h"

#define EVENT_SENSOR_UPDATE (1 << 0)

/* Sampling period (ms). The control loop runs at 1 ms, so sampling at 1 ms
 * keeps the consumed data at most one tick old. */
#ifndef FMT_SENSOR_TASK_PERIOD_MS
#define FMT_SENSOR_TASK_PERIOD_MS 1
#endif

static struct rt_timer timer_sensor;
static struct rt_event event_sensor;

static void timer_sensor_update(void* parameter)
{
    rt_event_send(&event_sensor, EVENT_SENSOR_UPDATE);
}

static void task_sensor_entry(void* parameter)
{
    rt_uint32_t recv_set = 0;

    while (1) {
        if (rt_event_recv(&event_sensor, EVENT_SENSOR_UPDATE,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recv_set)
            == RT_EOK) {
            if (recv_set & EVENT_SENSOR_UPDATE) {
                sensor_collect();
            }
        }
    }
}

static fmt_err_t task_sensor_init(void)
{
    if (rt_event_init(&event_sensor, "sensor", RT_IPC_FLAG_FIFO) != RT_EOK) {
        return FMT_ERROR;
    }

    rt_timer_init(&timer_sensor, "sensor", timer_sensor_update, RT_NULL,
        FMT_SENSOR_TASK_PERIOD_MS, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    if (rt_timer_start(&timer_sensor) != RT_EOK) {
        return FMT_ERROR;
    }

    return FMT_EOK;
}

TASK_EXPORT __fmt_task_desc = {
    .name = "sensor",
    .init = task_sensor_init,
    .entry = task_sensor_entry,
    .priority = SENSOR_THREAD_PRIORITY,
    .auto_start = true,
    .stack_size = 2048,
    .param = NULL,
    .dependency = NULL
};

#endif /* FMT_USING_SENSOR_TASK */

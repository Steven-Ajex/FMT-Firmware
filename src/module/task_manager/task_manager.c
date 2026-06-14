/******************************************************************************
 * Copyright 2020-2021 The Firmament-Autopilot Authors. All Rights Reserved.
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
#include <string.h>

#include "module/task_manager/task_manager.h"

#define MAX_INIT_TIME (5000)

static fmt_task_desc_t task_table;
static uint32_t task_num;
static uint8_t* task_status;
static rt_thread_t* task_tid;

/**
 * @brief Get the task num
 *
 * @return uint32_t Number of task
 */
uint32_t get_task_num(void)
{
    return task_num;
}

/**
 * @brief Get the task table object
 *
 * @return fmt_task_desc_t Task table object
 */
fmt_task_desc_t get_task_table(void)
{
    return task_table;
}

/**
 * @brief Get the task status object
 *
 * @param name Task name
 * @return uint8_t Task status
 */
uint8_t get_task_status(const char* name)
{
    for (uint32_t i = 0; i < task_num; i++) {
        if (strcmp(name, task_table[i].name) == 0) {
            return task_status[i];
        }
    }

    return 0;
}

/**
 * @brief Get the task id
 *
 * @param name Task name
 * @return int32_t Task id, -1 for invalid
 */
int32_t get_task_id(const char* name)
{
    for (uint32_t i = 0; i < task_num; i++) {
        if (strcmp(name, task_table[i].name) == 0) {
            return i;
        }
    }

    return -1;
}

/**
 * @brief Initialize tasks
 *
 */
void task_manager_init(void)
{
    extern const int __fmt_task_start;
    extern const int __fmt_task_end;

    uint32_t init_done = 0;
    uint32_t time_start_init = systime_now_ms();

    task_table = (fmt_task_desc_t)&__fmt_task_start;
    task_num = (fmt_task_desc_t)&__fmt_task_end - task_table;

    if (task_num == 0) {
        /* No task defined */
        return;
    }

    task_status = (uint8_t*)rt_malloc(task_num);
    RT_ASSERT(task_status != NULL);
    memset(task_status, TASK_IDLE, task_num);

    task_tid = (rt_thread_t*)rt_malloc(task_num * sizeof(rt_thread_t));
    RT_ASSERT(task_tid != NULL);
    memset(task_tid, 0, task_num * sizeof(rt_thread_t));

    for (uint32_t i = 0; i < task_num; i++) {
        RT_ASSERT(task_table[i].name != NULL);
        RT_ASSERT(task_table[i].init != NULL);
        RT_ASSERT(task_table[i].entry != NULL);
    }

    /* Resolve each task's dependency names to task indices once, so the init
     * loop below does not repeat string comparisons on every pass. dep_idx[i]
     * is a (-1)-terminated list of depended task indices (an entry of -1 from
     * get_task_id() means the dependency name is unknown and is ignored, as in
     * the original behaviour). */
    int32_t** dep_idx = (int32_t**)rt_malloc(task_num * sizeof(int32_t*));
    RT_ASSERT(dep_idx != NULL);
    for (uint32_t i = 0; i < task_num; i++) {
        uint32_t ndep = 0;
        while (task_table[i].dependency != NULL && task_table[i].dependency[ndep] != NULL) {
            ndep++;
        }
        dep_idx[i] = (int32_t*)rt_malloc((ndep + 1) * sizeof(int32_t));
        RT_ASSERT(dep_idx[i] != NULL);
        for (uint32_t n = 0; n < ndep; n++) {
            dep_idx[i][n] = get_task_id(task_table[i].dependency[n]);
        }
        dep_idx[i][ndep] = -1;
    }

    /* wait all task has been initialized or init timeout */
    while (init_done < task_num && (systime_now_ms() - time_start_init) < MAX_INIT_TIME) {
        for (uint32_t i = 0; i < task_num; i++) {
            if (task_status[i] != TASK_IDLE) {
                continue;
            }

            uint8_t depend_met = 1;

            /* check task dependency */
            for (uint32_t n = 0; dep_idx[i][n] != -1; n++) {
                int32_t k = dep_idx[i][n];
                if (k < 0) {
                    continue;
                }
                if (task_status[k] == TASK_FAIL) {
                    /* if the depended task failed, then the related task fails
                     * as well; count it once and stop checking */
                    depend_met = 0;
                    task_status[i] = TASK_FAIL;
                    init_done++;
                    break;
                } else if (task_status[k] == TASK_IDLE) {
                    /* wait the depended task to be initialized, but keep
                     * scanning in case a later dependency has already failed */
                    depend_met = 0;
                }
            }

            /* if task is not initialized yet, call the init function */
            if (depend_met && task_status[i] == TASK_IDLE) {
                if (task_table[i].init() == FMT_EOK) {
                    /* create thread for task */
                    task_tid[i] = rt_thread_create(task_table[i].name,
                                                   task_table[i].entry,
                                                   task_table[i].param,
                                                   task_table[i].stack_size,
                                                   task_table[i].priority,
                                                   1);
                    if (task_tid[i] != NULL) {
                        task_status[i] = TASK_READY;
                    }
                } else {
                    task_status[i] = TASK_FAIL;
                }
                /* increase init task count */
                init_done++;
            }
        }
    }

    for (uint32_t i = 0; i < task_num; i++) {
        rt_free(dep_idx[i]);
    }
    rt_free(dep_idx);
}

/**
 * @brief Start all tasks with auto_start = true
 *
 */
void task_manager_start(void)
{
    for (uint32_t i = 0; i < task_num; i++) {
        if (task_status[i] == TASK_READY && task_table[i].auto_start == true) {
            RT_CHECK(rt_thread_startup(task_tid[i]));
            task_status[i] = TASK_RUNNING;
        }
    }
}

/**
 * @brief Start a given task
 * 
 * @param task_id task id
 * @return fmt_err_t FMT_EOK for success
 */
fmt_err_t start_task(uint32_t task_id)
{
    if (task_id >= task_num) {
        return FMT_EINVAL;
    }

    if (task_status[task_id] != TASK_READY) {
        /* task is not in ready state, so we can't start it */
        return FMT_ENOSYS;
    }

    if (rt_thread_startup(task_tid[task_id]) != RT_EOK) {
        return FMT_ERROR;
    }

    task_status[task_id] = TASK_RUNNING;
    return FMT_EOK;
}

/**
 * @brief Suspend a given task
 * 
 * @param task_id Task id
 * @param timeout Wait up to timeout in ms
 * @return fmt_err_t 
 */
fmt_err_t suspend_task(uint32_t task_id, uint32_t timeout)
{
    uint32_t start = systime_now_ms();

    if (task_id >= task_num) {
        return FMT_EINVAL;
    }

    if (task_status[task_id] != TASK_RUNNING) {
        /* task is not running */
        return FMT_ENOSYS;
    }

    if (task_tid[task_id] != NULL) {
        while (systime_now_ms() - start < timeout) {
            if (rt_thread_suspend(task_tid[task_id]) == RT_EOK) {
                task_status[task_id] = TASK_SUSPEND;
                return FMT_EOK;
            }
        }
    }

    return FMT_ERROR;
}

/**
 * @brief Resume a given task
 * 
 * @param task_id task id
 * @return fmt_err_t FMT_EOK for success
 */
fmt_err_t resume_task(uint32_t task_id)
{
    if (task_id >= task_num) {
        return FMT_EINVAL;
    }

    if (task_status[task_id] != TASK_SUSPEND) {
        /* task is not suspended */
        return FMT_ENOSYS;
    }

    if (task_tid[task_id] != NULL) {
        if (rt_thread_resume(task_tid[task_id]) == RT_EOK) {
            task_status[task_id] = TASK_RUNNING;
            return FMT_EOK;
        }
    }

    return FMT_ERROR;
}

/**
 * @brief Kill a given task
 * 
 * @param task_id task id
 * @return fmt_err_t FMT_EOK for success
 */
fmt_err_t kill_task(uint32_t task_id)
{
    if (task_id >= task_num) {
        return FMT_EINVAL;
    }

    if (task_tid[task_id] != NULL) {
        if (task_status[task_id] == TASK_RUNNING) {
            if (rt_thread_delete(task_tid[task_id]) == RT_EOK) {
                task_status[task_id] = TASK_ZOMBIE;
            }
        } else if (task_status[task_id] == TASK_READY) {
            task_status[task_id] = TASK_ZOMBIE;
        } else {
            return FMT_ENOSYS;
        }
        return FMT_EOK;
    }

    return FMT_ERROR;
}

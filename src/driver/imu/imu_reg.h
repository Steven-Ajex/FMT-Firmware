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
#ifndef IMU_REG_H__
#define IMU_REG_H__

#include <firmament.h>

#include "hal/spi/spi.h"

/*
 * Shared SPI register-access helpers for IMU drivers.
 *
 * These were duplicated verbatim across several drivers (icm20600, icm20689,
 * icm42688p, bmi055, ...). Drivers with a non-standard read path (e.g. the
 * BMI088 accelerometer, which returns a dummy byte before the data) keep their
 * own __write_checked_reg()/__modify_reg() and should not include this header.
 */

#ifndef BIT
#define BIT(_idx) (1 << _idx)
#endif

#define REG_VAL(_setbits, _clearbits) \
    (reg_val_t) { .setbits = (_setbits), .clearbits = (_clearbits) }

typedef struct {
    uint8_t setbits;
    uint8_t clearbits;
} reg_val_t;

/* Write a register then read it back to verify the value was accepted. */
rt_inline rt_err_t __write_checked_reg(rt_device_t spi_device, rt_uint8_t reg, rt_uint8_t val)
{
    rt_uint8_t r_val;

    RT_TRY(spi_write_reg8(spi_device, reg, val));
    RT_TRY(spi_read_reg8(spi_device, reg, &r_val));

    return (r_val == val) ? RT_EOK : RT_ERROR;
}

/* Read-modify-write a register according to a (setbits, clearbits) mask. */
rt_inline rt_err_t __modify_reg(rt_device_t spi_device, rt_uint8_t reg, reg_val_t reg_val)
{
    uint8_t value;

    RT_TRY(spi_read_reg8(spi_device, reg, &value));

    value &= ~reg_val.clearbits;
    value |= reg_val.setbits;

    RT_TRY(__write_checked_reg(spi_device, reg, value));

    return RT_EOK;
}

#endif /* IMU_REG_H__ */

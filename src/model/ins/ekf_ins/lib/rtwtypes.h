/******************************************************************************
 * Copyright 2025 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#ifndef RTWTYPES_H
#define RTWTYPES_H

#include <stdint.h>

#if (!defined(__cplusplus))
#  ifndef false
#   define false                       (0U)
#  endif
#  ifndef true
#   define true                        (1U)
#  endif
#endif

typedef int8_t   int8_T;
typedef uint8_t  uint8_T;
typedef int16_t  int16_T;
typedef uint16_t uint16_T;
typedef int32_t  int32_T;
typedef uint32_t uint32_T;
typedef float    real32_T;
typedef double   real64_T;
typedef double   real_T;
typedef double   time_T;
typedef uint8_t  boolean_T;
typedef int      int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef char     char_T;
typedef unsigned char uchar_T;
typedef char_T   byte_T;

#define MAX_int8_T                     ((int8_T)(127))
#define MIN_int8_T                     ((int8_T)(-128))
#define MAX_uint8_T                    ((uint8_T)(255U))
#define MAX_int16_T                    ((int16_T)(32767))
#define MIN_int16_T                    ((int16_T)(-32768))
#define MAX_uint16_T                   ((uint16_T)(65535U))
#define MAX_int32_T                    ((int32_T)(2147483647))
#define MIN_int32_T                    ((int32_T)(-2147483647-1))
#define MAX_uint32_T                   ((uint32_T)(0xFFFFFFFFU))

typedef void * pointer_T;

#endif

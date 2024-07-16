/*
 * File: Controller_data.c
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 1.993
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Thu Jul 11 15:39:34 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Controller.h"
#include "Controller_private.h"

/* Invariant block signals (default storage) */
const ConstB_Controller_T Controller_ConstB = {
  { 0.0F, 0.0F, 1.0F },                /* '<S52>/Vector Concatenate3' */
  0.0F,                                /* '<S55>/Constant' */
  19.1986F,                            /* '<S40>/Gain' */

  { 0.0F, 0.0F, 0.0F },                /* '<S28>/Constant' */
  0.00250000018F,                      /* '<S53>/Square' */
  0.14709F,                            /* '<S53>/Multiply' */
  -58.836F,                            /* '<S53>/Gain4' */
  0.00250000018F,                      /* '<S67>/Square' */
  0.196120009F,                        /* '<S67>/Multiply' */
  -78.448F                             /* '<S67>/Gain4' */
};

/* Constant parameters (default storage) */
const ConstP_Controller_T Controller_ConstP = {
  /* Computed Parameter: Effective_Matrix_Value
   * Referenced by: '<S7>/Effective_Matrix'
   */
  { 1.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F, 0.0F, 1.0F, -1.0F, -1.0F, -1.0F, -1.0F
  }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

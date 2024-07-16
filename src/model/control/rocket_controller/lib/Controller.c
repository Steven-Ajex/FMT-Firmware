/*
 * File: Controller.c
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

const Control_Out_Bus Controller_rtZControl_Out_Bus = {
  0U,                                  /* timestamp */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  /* actuator_cmd */
} ;                                    /* Control_Out_Bus ground */

/* Exported block parameters */
struct_3zLBZv1DafkAI5Ov6R3JHD CONTROL_PARAM = {
  1.6F,
  0.2F,
  0.05F,
  0.5F,
  0.12F,
  0.0F,
  -1.0F,
  1.0F,
  -1.0F,
  1.0F,
  -0.25F,
  0.25F,
  -0.1F,
  0.1F,
  7.0F,
  7.0F,
  0.26F,
  0.5F,
  0.5F,
  0.115F,
  0.08F,
  0.08F,
  0.13F,
  0.002F,
  0.002F,
  0.002F,
  -0.1F,
  0.1F,
  -0.1F,
  0.1F,
  1.57079601F,
  3.14159298F,
  0.75F,
  90.0F,
  0.0F,
  40.0F,
  -20.0F,
  1.0F,
  1.0F,
  1.0F,
  1.0F
} ;                                    /* Variable: CONTROL_PARAM
                                        * Referenced by:
                                        *   '<S7>/Bias1'
                                        *   '<S7>/Bias2'
                                        *   '<S7>/Bias3'
                                        *   '<S7>/Bias4'
                                        *   '<S7>/Gain1'
                                        *   '<S7>/Gain2'
                                        *   '<S7>/Gain3'
                                        *   '<S7>/Gain4'
                                        *   '<S10>/hover_throttle'
                                        *   '<S45>/Saturation'
                                        *   '<S14>/Bias'
                                        *   '<S14>/Bias1'
                                        *   '<S14>/Bias2'
                                        *   '<S14>/Bias3'
                                        *   '<S15>/Bias'
                                        *   '<S15>/Bias1'
                                        *   '<S15>/Bias2'
                                        *   '<S15>/Bias3'
                                        *   '<S21>/Saturation'
                                        *   '<S21>/Saturation1'
                                        *   '<S54>/kd'
                                        *   '<S54>/Saturation'
                                        *   '<S55>/ki'
                                        *   '<S55>/Discrete-Time Integrator'
                                        *   '<S56>/kp'
                                        *   '<S68>/kd'
                                        *   '<S68>/Saturation'
                                        *   '<S69>/Constant'
                                        *   '<S69>/ki'
                                        *   '<S69>/Discrete-Time Integrator'
                                        *   '<S70>/kp'
                                        *   '<S38>/Constant1'
                                        *   '<S38>/Constant2'
                                        *   '<S27>/gain1'
                                        *   '<S27>/gain2'
                                        *   '<S27>/gain3'
                                        *   '<S27>/Saturation'
                                        *   '<S28>/gain1'
                                        *   '<S28>/gain2'
                                        *   '<S28>/gain3'
                                        *   '<S28>/Discrete-Time Integrator'
                                        *   '<S29>/gain1'
                                        *   '<S29>/gain2'
                                        *   '<S29>/gain3'
                                        */

struct_AtpMUMXHQ4EpFICCggTZNH CONTROL_EXPORT = {
  2U,

  { 82, 111, 99, 107, 101, 116, 32, 67, 111, 110, 116, 114, 111, 108, 108, 101,
    114, 32, 118, 48, 46, 48, 46, 49, 0 }
} ;                                    /* Variable: CONTROL_EXPORT
                                        * Referenced by: '<S3>/Constant'
                                        */

/* Block states (default storage) */
DW_Controller_T Controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Controller_T Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Controller_T Controller_Y;

/* Real-time model */
RT_MODEL_Controller_T Controller_M_;
RT_MODEL_Controller_T *const Controller_M = &Controller_M_;

/* Model step function */
void Controller_step(void)
{
  real32_T rtb_Add3_c;
  real32_T rtb_VectorConcatenate[9];
  real32_T rtb_Subtract3_i;
  real32_T rtb_DiscreteTimeIntegrator1_j;
  real32_T rtb_DiscreteTimeIntegrator_h;
  real32_T rtb_Gain_f;
  real32_T rtb_Gain_hb0;
  uint16_T rtb_throttle_cmd;
  real32_T rtb_Bias_m[4];
  uint16_T rtb_Saturation_f[5];
  int32_T i;
  real32_T rtb_VectorConcatenate_0[3];
  real32_T rtb_Subtract3_idx_0;
  real32_T rtb_Subtract3_idx_1;
  real32_T rtb_a_n_idx_0;
  real32_T rtb_a_n_idx_1;
  real32_T rtb_DiscreteTimeIntegrator1_b_i;
  real32_T rtb_Saturation_idx_1;
  real32_T rtb_rate_error_B_radPs_idx_0;
  real32_T rtb_rate_error_B_radPs_idx_1;
  real32_T rtb_rate_error_B_radPs_idx_2;
  real32_T rtb_Saturation_d_idx_0;
  real32_T rtb_Saturation_d_idx_1;
  real32_T rtb_Saturation_d_idx_2;
  real32_T rtb_uv_error_C_mPs_idx_0;
  real32_T rtb_uv_error_C_mPs_idx_1;
  real32_T rtb_Gain_idx_0;
  real32_T rtb_Gain_idx_1;

  /* Trigonometry: '<S52>/Trigonometric Function1' incorporates:
   *  Gain: '<S51>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S52>/Trigonometric Function3'
   */
  rtb_uv_error_C_mPs_idx_0 = arm_cos_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[0] = rtb_uv_error_C_mPs_idx_0;

  /* Trigonometry: '<S52>/Trigonometric Function' incorporates:
   *  Gain: '<S51>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S52>/Trigonometric Function2'
   */
  rtb_uv_error_C_mPs_idx_1 = arm_sin_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[1] = rtb_uv_error_C_mPs_idx_1;

  /* SignalConversion: '<S52>/ConcatBufferAtVector Concatenate1In3' incorporates:
   *  Constant: '<S52>/Constant3'
   */
  rtb_VectorConcatenate[2] = 0.0F;

  /* Gain: '<S52>/Gain' */
  rtb_VectorConcatenate[3] = -rtb_uv_error_C_mPs_idx_1;

  /* Trigonometry: '<S52>/Trigonometric Function3' */
  rtb_VectorConcatenate[4] = rtb_uv_error_C_mPs_idx_0;

  /* SignalConversion: '<S52>/ConcatBufferAtVector Concatenate2In3' incorporates:
   *  Constant: '<S52>/Constant4'
   */
  rtb_VectorConcatenate[5] = 0.0F;

  /* SignalConversion: '<S52>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_VectorConcatenate[6] = Controller_ConstB.VectorConcatenate3[0];
  rtb_VectorConcatenate[7] = Controller_ConstB.VectorConcatenate3[1];
  rtb_VectorConcatenate[8] = Controller_ConstB.VectorConcatenate3[2];

  /* Product: '<S49>/Multiply' incorporates:
   *  Inport: '<Root>/INS_Out'
   *  SignalConversion: '<S49>/TmpSignal ConversionAtMultiplyInport2'
   */
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_0[i] = rtb_VectorConcatenate[i + 3] *
      Controller_U.INS_Out.ve + rtb_VectorConcatenate[i] *
      Controller_U.INS_Out.vn;
  }

  /* End of Product: '<S49>/Multiply' */

  /* Sum: '<S46>/Sum' incorporates:
   *  DiscreteIntegrator: '<S50>/Integrator1'
   */
  rtb_uv_error_C_mPs_idx_0 = Controller_DW.Integrator1_DSTATE[0] -
    rtb_VectorConcatenate_0[0];
  rtb_uv_error_C_mPs_idx_1 = Controller_DW.Integrator1_DSTATE[1] -
    rtb_VectorConcatenate_0[1];

  /* DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator_PrevRese != 0)) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[0] = Controller_ConstB.Constant;
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] >=
        CONTROL_PARAM.VEL_XY_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[0] =
        CONTROL_PARAM.VEL_XY_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] <=
          CONTROL_PARAM.VEL_XY_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE[0] =
          CONTROL_PARAM.VEL_XY_I_MIN;
      }
    }

    Controller_DW.DiscreteTimeIntegrator_DSTATE[1] = Controller_ConstB.Constant;
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] >=
        CONTROL_PARAM.VEL_XY_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[1] =
        CONTROL_PARAM.VEL_XY_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] <=
          CONTROL_PARAM.VEL_XY_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE[1] =
          CONTROL_PARAM.VEL_XY_I_MIN;
      }
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[0] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[0] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[1] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[1] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  /* DiscreteIntegrator: '<S57>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] = rtb_uv_error_C_mPs_idx_0;
    Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] = rtb_uv_error_C_mPs_idx_1;
  }

  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator1_PrevRes != 0)) {
    Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] = rtb_uv_error_C_mPs_idx_0;
    Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] = rtb_uv_error_C_mPs_idx_1;
  }

  /* Gain: '<S57>/Gain' incorporates:
   *  DiscreteIntegrator: '<S57>/Discrete-Time Integrator1'
   *  Sum: '<S57>/Sum5'
   */
  rtb_Gain_idx_0 = (rtb_uv_error_C_mPs_idx_0 -
                    Controller_DW.DiscreteTimeIntegrator1_DSTATE[0]) *
    62.831852F;
  rtb_Gain_idx_1 = (rtb_uv_error_C_mPs_idx_1 -
                    Controller_DW.DiscreteTimeIntegrator1_DSTATE[1]) *
    62.831852F;

  /* Switch: '<S57>/Switch' incorporates:
   *  Gain: '<S57>/Gain1'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.reset > 0) {
    rtb_a_n_idx_0 = 0.0F;
    rtb_a_n_idx_1 = 0.0F;
  } else {
    rtb_a_n_idx_0 = rtb_Gain_idx_0;
    rtb_a_n_idx_1 = rtb_Gain_idx_1;
  }

  /* End of Switch: '<S57>/Switch' */

  /* Product: '<S54>/Multiply' incorporates:
   *  Constant: '<S54>/kd'
   */
  rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D * rtb_a_n_idx_0;

  /* Saturate: '<S54>/Saturation' */
  if (rtb_DiscreteTimeIntegrator1_j > CONTROL_PARAM.VEL_XY_D_MAX) {
    rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D_MAX;
  } else {
    if (rtb_DiscreteTimeIntegrator1_j < CONTROL_PARAM.VEL_XY_D_MIN) {
      rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D_MIN;
    }
  }

  /* Product: '<S54>/Multiply' incorporates:
   *  Constant: '<S54>/kd'
   */
  rtb_a_n_idx_0 = rtb_DiscreteTimeIntegrator1_j;
  rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D * rtb_a_n_idx_1;

  /* Saturate: '<S54>/Saturation' */
  if (rtb_DiscreteTimeIntegrator1_j > CONTROL_PARAM.VEL_XY_D_MAX) {
    rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D_MAX;
  } else {
    if (rtb_DiscreteTimeIntegrator1_j < CONTROL_PARAM.VEL_XY_D_MIN) {
      rtb_DiscreteTimeIntegrator1_j = CONTROL_PARAM.VEL_XY_D_MIN;
    }
  }

  /* Switch: '<S48>/Switch' incorporates:
   *  Constant: '<S56>/kp'
   *  Constant: '<S58>/Constant'
   *  Constant: '<S60>/Constant'
   *  Constant: '<S61>/Constant'
   *  DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
   *  Inport: '<Root>/FMS_Out'
   *  Product: '<S56>/Multiply'
   *  Product: '<S59>/Multiply2'
   *  Product: '<S59>/Multiply3'
   *  RelationalOperator: '<S58>/Compare'
   *  RelationalOperator: '<S60>/Compare'
   *  RelationalOperator: '<S61>/Compare'
   *  S-Function (sfix_bitop): '<S59>/cmd_ax valid'
   *  S-Function (sfix_bitop): '<S59>/cmd_ay valid'
   *  S-Function (sfix_bitop): '<S59>/cmd_u valid'
   *  S-Function (sfix_bitop): '<S59>/cmd_v valid'
   *  Sum: '<S47>/Add'
   *  Sum: '<S59>/Sum1'
   */
  if (Controller_U.FMS_Out.ctrl_mode == 6) {
    rtb_a_n_idx_0 = ((Controller_U.FMS_Out.cmd_mask & 64) > 0 ?
                     (CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_0 +
                      Controller_DW.DiscreteTimeIntegrator_DSTATE[0]) +
                     rtb_a_n_idx_0 : 0.0F) + ((Controller_U.FMS_Out.cmd_mask &
      512) > 0 ? Controller_U.FMS_Out.ax_cmd : 0.0F);
    rtb_a_n_idx_1 = ((Controller_U.FMS_Out.cmd_mask & 128) > 0 ?
                     (CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_1 +
                      Controller_DW.DiscreteTimeIntegrator_DSTATE[1]) +
                     rtb_DiscreteTimeIntegrator1_j : 0.0F) +
      ((Controller_U.FMS_Out.cmd_mask & 1024) > 0 ? Controller_U.FMS_Out.ay_cmd :
       0.0F);
  } else {
    rtb_a_n_idx_0 += CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_0 +
      Controller_DW.DiscreteTimeIntegrator_DSTATE[0];
    rtb_a_n_idx_1 = (CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_1 +
                     Controller_DW.DiscreteTimeIntegrator_DSTATE[1]) +
      rtb_DiscreteTimeIntegrator1_j;
  }

  /* End of Switch: '<S48>/Switch' */

  /* Trigonometry: '<S45>/Atan' incorporates:
   *  Constant: '<S45>/g'
   *  Gain: '<S45>/Gain1'
   *  Gain: '<S45>/gain'
   *  Product: '<S45>/Divide'
   */
  rtb_DiscreteTimeIntegrator1_j = atanf(1.1F * rtb_a_n_idx_1 / 9.8055F);
  rtb_a_n_idx_1 = atanf(1.1F * -rtb_a_n_idx_0 / 9.8055F);

  /* Switch: '<S36>/Switch' incorporates:
   *  Constant: '<S43>/Constant'
   *  Inport: '<Root>/FMS_Out'
   *  Logic: '<S36>/Logical Operator'
   *  RelationalOperator: '<S42>/Compare'
   *  RelationalOperator: '<S43>/Compare'
   *  Saturate: '<S45>/Saturation'
   *  Switch: '<S36>/Switch1'
   */
  if ((Controller_U.FMS_Out.ctrl_mode == 3) || (Controller_U.FMS_Out.ctrl_mode ==
       4)) {
    rtb_a_n_idx_0 = Controller_U.FMS_Out.phi_cmd;
    rtb_a_n_idx_1 = Controller_U.FMS_Out.theta_cmd;
  } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
    /* Switch: '<S44>/Switch' incorporates:
     *  S-Function (sfix_bitop): '<S44>/cmd_phi valid'
     *  S-Function (sfix_bitop): '<S44>/cmd_theta valid'
     *  Saturate: '<S45>/Saturation'
     *  Switch: '<S36>/Switch1'
     */
    if ((Controller_U.FMS_Out.cmd_mask & 8) > 0) {
      rtb_a_n_idx_0 = Controller_U.FMS_Out.phi_cmd;
    } else if (rtb_DiscreteTimeIntegrator1_j > CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S45>/Saturation' */
      rtb_a_n_idx_0 = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_DiscreteTimeIntegrator1_j < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S45>/Saturation' */
      rtb_a_n_idx_0 = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      rtb_a_n_idx_0 = rtb_DiscreteTimeIntegrator1_j;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 16) > 0) {
      rtb_a_n_idx_1 = Controller_U.FMS_Out.theta_cmd;
    } else if (rtb_a_n_idx_1 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Saturate: '<S45>/Saturation' */
      rtb_a_n_idx_1 = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      if (rtb_a_n_idx_1 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
        /* Saturate: '<S45>/Saturation' */
        rtb_a_n_idx_1 = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
      }
    }

    /* End of Switch: '<S44>/Switch' */
  } else {
    if (rtb_DiscreteTimeIntegrator1_j > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Saturate: '<S45>/Saturation' incorporates:
       *  Switch: '<S36>/Switch1'
       */
      rtb_a_n_idx_0 = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_DiscreteTimeIntegrator1_j < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S45>/Saturation' incorporates:
       *  Switch: '<S36>/Switch1'
       */
      rtb_a_n_idx_0 = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      /* Switch: '<S36>/Switch1' incorporates:
       *  Saturate: '<S45>/Saturation'
       */
      rtb_a_n_idx_0 = rtb_DiscreteTimeIntegrator1_j;
    }

    /* Saturate: '<S45>/Saturation' */
    if (rtb_a_n_idx_1 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Switch: '<S36>/Switch1' */
      rtb_a_n_idx_1 = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      if (rtb_a_n_idx_1 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
        /* Switch: '<S36>/Switch1' */
        rtb_a_n_idx_1 = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
      }
    }
  }

  /* End of Switch: '<S36>/Switch' */

  /* Sum: '<S37>/Sum' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_DiscreteTimeIntegrator1_j = rtb_a_n_idx_0 - Controller_U.INS_Out.phi;

  /* Product: '<S40>/Divide1' incorporates:
   *  Abs: '<S40>/Abs'
   *  Constant: '<S40>/const2'
   */
  rtb_DiscreteTimeIntegrator1_b_i = fabsf(rtb_DiscreteTimeIntegrator1_j) /
    0.002F;

  /* Product: '<S40>/Divide' incorporates:
   *  Constant: '<S38>/Constant1'
   *  Constant: '<S40>/const1'
   *  Math: '<S40>/Square'
   *  SignalConversion: '<S40>/TmpSignal ConversionAtSquareInport1'
   */
  rtb_Subtract3_i = 9.5993F / (CONTROL_PARAM.ROLL_P * CONTROL_PARAM.ROLL_P);

  /* Signum: '<S40>/Sign' */
  if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
    rtb_DiscreteTimeIntegrator_h = -1.0F;
  } else if (rtb_DiscreteTimeIntegrator1_j > 0.0F) {
    rtb_DiscreteTimeIntegrator_h = 1.0F;
  } else {
    rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator1_j;
  }

  /* Switch: '<S40>/Switch' incorporates:
   *  Constant: '<S38>/Constant1'
   *  Gain: '<S40>/Gain1'
   *  Gain: '<S40>/Gain2'
   *  Logic: '<S40>/Logical Operator'
   *  Product: '<S40>/Multiply'
   *  Product: '<S40>/Multiply1'
   *  Product: '<S40>/Multiply2'
   *  Product: '<S40>/Multiply3'
   *  RelationalOperator: '<S40>/Relational Operator'
   *  RelationalOperator: '<S40>/Relational Operator2'
   *  SignalConversion: '<S40>/TmpSignal ConversionAtSquareInport1'
   *  Sqrt: '<S40>/Sqrt'
   *  Sum: '<S40>/Subtract'
   */
  if ((rtb_DiscreteTimeIntegrator1_j <= rtb_Subtract3_i) &&
      (rtb_DiscreteTimeIntegrator1_j >= -rtb_Subtract3_i)) {
    rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator1_j *
      CONTROL_PARAM.ROLL_P;
  } else {
    rtb_DiscreteTimeIntegrator_h *= sqrtf((rtb_DiscreteTimeIntegrator_h *
      rtb_DiscreteTimeIntegrator1_j - 0.5F * rtb_Subtract3_i) *
      Controller_ConstB.Gain);
  }

  /* Gain: '<S40>/Gain3' */
  rtb_Subtract3_i = -rtb_DiscreteTimeIntegrator1_b_i;

  /* Switch: '<S41>/Switch' incorporates:
   *  Gain: '<S40>/Gain3'
   *  RelationalOperator: '<S41>/UpperRelop'
   */
  if (rtb_DiscreteTimeIntegrator_h >= -rtb_DiscreteTimeIntegrator1_b_i) {
    rtb_Subtract3_i = rtb_DiscreteTimeIntegrator_h;
  }

  /* Switch: '<S41>/Switch2' incorporates:
   *  RelationalOperator: '<S41>/LowerRelop1'
   */
  if (rtb_DiscreteTimeIntegrator_h <= rtb_DiscreteTimeIntegrator1_b_i) {
    rtb_DiscreteTimeIntegrator1_b_i = rtb_Subtract3_i;
  }

  /* Saturate: '<S21>/Saturation1' */
  if (rtb_DiscreteTimeIntegrator1_b_i > CONTROL_PARAM.P_Q_CMD_LIM) {
    rtb_DiscreteTimeIntegrator1_b_i = CONTROL_PARAM.P_Q_CMD_LIM;
  } else {
    if (rtb_DiscreteTimeIntegrator1_b_i < -CONTROL_PARAM.P_Q_CMD_LIM) {
      rtb_DiscreteTimeIntegrator1_b_i = -CONTROL_PARAM.P_Q_CMD_LIM;
    }
  }

  /* Product: '<S40>/Divide1' */
  rtb_a_n_idx_0 = rtb_DiscreteTimeIntegrator1_b_i;

  /* Sum: '<S37>/Sum' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_DiscreteTimeIntegrator1_j = rtb_a_n_idx_1 - Controller_U.INS_Out.theta;

  /* Product: '<S40>/Divide1' incorporates:
   *  Abs: '<S40>/Abs'
   *  Constant: '<S40>/const2'
   */
  rtb_DiscreteTimeIntegrator1_b_i = fabsf(rtb_DiscreteTimeIntegrator1_j) /
    0.002F;

  /* Product: '<S40>/Divide' incorporates:
   *  Constant: '<S38>/Constant2'
   *  Constant: '<S40>/const1'
   *  Math: '<S40>/Square'
   *  SignalConversion: '<S40>/TmpSignal ConversionAtSquareInport1'
   */
  rtb_Subtract3_i = 9.5993F / (CONTROL_PARAM.PITCH_P * CONTROL_PARAM.PITCH_P);

  /* Signum: '<S40>/Sign' */
  if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
    rtb_DiscreteTimeIntegrator_h = -1.0F;
  } else if (rtb_DiscreteTimeIntegrator1_j > 0.0F) {
    rtb_DiscreteTimeIntegrator_h = 1.0F;
  } else {
    rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator1_j;
  }

  /* Switch: '<S40>/Switch' incorporates:
   *  Constant: '<S38>/Constant2'
   *  Gain: '<S40>/Gain1'
   *  Gain: '<S40>/Gain2'
   *  Logic: '<S40>/Logical Operator'
   *  Product: '<S40>/Multiply'
   *  Product: '<S40>/Multiply1'
   *  Product: '<S40>/Multiply2'
   *  Product: '<S40>/Multiply3'
   *  RelationalOperator: '<S40>/Relational Operator'
   *  RelationalOperator: '<S40>/Relational Operator2'
   *  SignalConversion: '<S40>/TmpSignal ConversionAtSquareInport1'
   *  Sqrt: '<S40>/Sqrt'
   *  Sum: '<S40>/Subtract'
   */
  if ((rtb_DiscreteTimeIntegrator1_j <= rtb_Subtract3_i) &&
      (rtb_DiscreteTimeIntegrator1_j >= -rtb_Subtract3_i)) {
    rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator1_j *
      CONTROL_PARAM.PITCH_P;
  } else {
    rtb_DiscreteTimeIntegrator_h *= sqrtf((rtb_DiscreteTimeIntegrator_h *
      rtb_DiscreteTimeIntegrator1_j - 0.5F * rtb_Subtract3_i) *
      Controller_ConstB.Gain);
  }

  /* Gain: '<S40>/Gain3' */
  rtb_Subtract3_i = -rtb_DiscreteTimeIntegrator1_b_i;

  /* Switch: '<S41>/Switch' incorporates:
   *  Gain: '<S40>/Gain3'
   *  RelationalOperator: '<S41>/UpperRelop'
   */
  if (rtb_DiscreteTimeIntegrator_h >= -rtb_DiscreteTimeIntegrator1_b_i) {
    rtb_Subtract3_i = rtb_DiscreteTimeIntegrator_h;
  }

  /* Switch: '<S41>/Switch2' incorporates:
   *  RelationalOperator: '<S41>/LowerRelop1'
   */
  if (rtb_DiscreteTimeIntegrator_h <= rtb_DiscreteTimeIntegrator1_b_i) {
    rtb_DiscreteTimeIntegrator1_b_i = rtb_Subtract3_i;
  }

  /* Saturate: '<S21>/Saturation1' */
  if (rtb_DiscreteTimeIntegrator1_b_i > CONTROL_PARAM.P_Q_CMD_LIM) {
    rtb_DiscreteTimeIntegrator1_b_i = CONTROL_PARAM.P_Q_CMD_LIM;
  } else {
    if (rtb_DiscreteTimeIntegrator1_b_i < -CONTROL_PARAM.P_Q_CMD_LIM) {
      rtb_DiscreteTimeIntegrator1_b_i = -CONTROL_PARAM.P_Q_CMD_LIM;
    }
  }

  /* Saturate: '<S21>/Saturation' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.psi_rate_cmd > CONTROL_PARAM.R_CMD_LIM) {
    rtb_Subtract3_i = CONTROL_PARAM.R_CMD_LIM;
  } else if (Controller_U.FMS_Out.psi_rate_cmd < -CONTROL_PARAM.R_CMD_LIM) {
    rtb_Subtract3_i = -CONTROL_PARAM.R_CMD_LIM;
  } else {
    rtb_Subtract3_i = Controller_U.FMS_Out.psi_rate_cmd;
  }

  /* End of Saturate: '<S21>/Saturation' */

  /* Trigonometry: '<S31>/Sin' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_Add3_c = arm_sin_f32(Controller_U.INS_Out.phi);

  /* Trigonometry: '<S31>/Cos1' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_DiscreteTimeIntegrator1_j = arm_cos_f32(Controller_U.INS_Out.theta);

  /* Product: '<S31>/Multiply3' */
  rtb_a_n_idx_1 = rtb_Add3_c * rtb_DiscreteTimeIntegrator1_j * rtb_Subtract3_i;

  /* Trigonometry: '<S31>/Cos' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_DiscreteTimeIntegrator_h = arm_cos_f32(Controller_U.INS_Out.phi);

  /* Product: '<S31>/Multiply1' */
  rtb_rate_error_B_radPs_idx_0 = rtb_DiscreteTimeIntegrator_h *
    rtb_DiscreteTimeIntegrator1_b_i;

  /* Product: '<S31>/Multiply4' */
  rtb_Add3_c *= rtb_DiscreteTimeIntegrator1_b_i;

  /* Switch: '<S22>/Switch' incorporates:
   *  Constant: '<S32>/Constant'
   *  Constant: '<S33>/Constant'
   *  Inport: '<Root>/FMS_Out'
   *  RelationalOperator: '<S32>/Compare'
   *  RelationalOperator: '<S33>/Compare'
   *  Switch: '<S22>/Switch1'
   */
  if (Controller_U.FMS_Out.ctrl_mode == 2) {
    rtb_DiscreteTimeIntegrator1_b_i = Controller_U.FMS_Out.p_cmd;
    rtb_a_n_idx_1 = Controller_U.FMS_Out.q_cmd;
    rtb_DiscreteTimeIntegrator_h = Controller_U.FMS_Out.r_cmd;
  } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
    /* Switch: '<S34>/Switch' incorporates:
     *  Inport: '<Root>/INS_Out'
     *  Product: '<S31>/Multiply'
     *  Product: '<S31>/Multiply1'
     *  Product: '<S31>/Multiply5'
     *  S-Function (sfix_bitop): '<S34>/cmd_p valid'
     *  S-Function (sfix_bitop): '<S34>/cmd_q valid'
     *  S-Function (sfix_bitop): '<S34>/cmd_r valid'
     *  Sum: '<S31>/Add'
     *  Sum: '<S31>/Add1'
     *  Sum: '<S31>/Add2'
     *  Switch: '<S22>/Switch1'
     *  Trigonometry: '<S31>/Sin1'
     */
    if ((Controller_U.FMS_Out.cmd_mask & 1) > 0) {
      rtb_DiscreteTimeIntegrator1_b_i = Controller_U.FMS_Out.p_cmd;
    } else {
      rtb_DiscreteTimeIntegrator1_b_i = rtb_a_n_idx_0 - arm_sin_f32
        (Controller_U.INS_Out.theta) * rtb_Subtract3_i;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 2) > 0) {
      rtb_a_n_idx_1 = Controller_U.FMS_Out.q_cmd;
    } else {
      rtb_a_n_idx_1 += rtb_rate_error_B_radPs_idx_0;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 4) > 0) {
      rtb_DiscreteTimeIntegrator_h = Controller_U.FMS_Out.r_cmd;
    } else {
      rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator_h *
        rtb_DiscreteTimeIntegrator1_j * rtb_Subtract3_i - rtb_Add3_c;
    }

    /* End of Switch: '<S34>/Switch' */
  } else {
    /* Switch: '<S22>/Switch1' incorporates:
     *  Inport: '<Root>/INS_Out'
     *  Product: '<S31>/Multiply'
     *  Product: '<S31>/Multiply3'
     *  Product: '<S31>/Multiply5'
     *  Sum: '<S31>/Add'
     *  Sum: '<S31>/Add1'
     *  Sum: '<S31>/Add2'
     *  Trigonometry: '<S31>/Sin1'
     */
    rtb_DiscreteTimeIntegrator1_b_i = rtb_a_n_idx_0 - arm_sin_f32
      (Controller_U.INS_Out.theta) * rtb_Subtract3_i;
    rtb_a_n_idx_1 += rtb_rate_error_B_radPs_idx_0;
    rtb_DiscreteTimeIntegrator_h = rtb_DiscreteTimeIntegrator_h *
      rtb_DiscreteTimeIntegrator1_j * rtb_Subtract3_i - rtb_Add3_c;
  }

  /* End of Switch: '<S22>/Switch' */

  /* Sum: '<S23>/Sum' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_rate_error_B_radPs_idx_0 = rtb_DiscreteTimeIntegrator1_b_i -
    Controller_U.INS_Out.p;
  rtb_rate_error_B_radPs_idx_1 = rtb_a_n_idx_1 - Controller_U.INS_Out.q;
  rtb_rate_error_B_radPs_idx_2 = rtb_DiscreteTimeIntegrator_h -
    Controller_U.INS_Out.r;

  /* DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' */
  if (Controller_DW.DiscreteTimeIntegrator5_IC_LOAD != 0) {
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] =
      rtb_rate_error_B_radPs_idx_0;
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] =
      rtb_rate_error_B_radPs_idx_1;
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] =
      rtb_rate_error_B_radPs_idx_2;
  }

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator_PrevRe_g != 0)) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
      Controller_ConstB.Constant_n[0];
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] >=
        CONTROL_PARAM.RATE_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
        CONTROL_PARAM.RATE_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] <=
          CONTROL_PARAM.RATE_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
          CONTROL_PARAM.RATE_I_MIN;
      }
    }

    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
      Controller_ConstB.Constant_n[1];
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] >=
        CONTROL_PARAM.RATE_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
        CONTROL_PARAM.RATE_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] <=
          CONTROL_PARAM.RATE_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
          CONTROL_PARAM.RATE_I_MIN;
      }
    }

    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
      Controller_ConstB.Constant_n[2];
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] >=
        CONTROL_PARAM.RATE_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
        CONTROL_PARAM.RATE_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] <=
          CONTROL_PARAM.RATE_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
          CONTROL_PARAM.RATE_I_MIN;
      }
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  /* DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_DW.DiscreteTimeIntegrator1_IC_LO_l != 0) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[0];
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[1];
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[2];
  }

  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator1_PrevR_i != 0)) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[0];
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[1];
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2] =
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[2];
  }

  /* Gain: '<S30>/Gain' incorporates:
   *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
   *  DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
   *  Sum: '<S30>/Sum5'
   */
  rtb_DiscreteTimeIntegrator1_b_i =
    (Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] -
     Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0]) * 188.49556F;
  rtb_a_n_idx_1 = (Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] -
                   Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1]) *
    188.49556F;
  rtb_DiscreteTimeIntegrator_h = (Controller_DW.DiscreteTimeIntegrator5_DSTATE[2]
    - Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2]) * 188.49556F;

  /* Gain: '<S64>/Gain' incorporates:
   *  DiscreteIntegrator: '<S66>/Integrator1'
   *  Inport: '<Root>/INS_Out'
   *  Sum: '<S64>/Sum1'
   */
  rtb_Gain_f = -(Controller_DW.Integrator1_DSTATE_p - Controller_U.INS_Out.vd);

  /* DiscreteIntegrator: '<S69>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S69>/Constant'
   *  Inport: '<Root>/FMS_Out'
   */
  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator_PrevRe_m != 0)) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m >=
        CONTROL_PARAM.VEL_Z_I_MAX) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m <=
          CONTROL_PARAM.VEL_Z_I_MIN) {
        Controller_DW.DiscreteTimeIntegrator_DSTATE_m =
          CONTROL_PARAM.VEL_Z_I_MIN;
      }
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m >= CONTROL_PARAM.VEL_Z_I_MAX)
  {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m <=
        CONTROL_PARAM.VEL_Z_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;
    }
  }

  /* DiscreteIntegrator: '<S71>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_DW.DiscreteTimeIntegrator1_IC_LO_k != 0) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_h = rtb_Gain_f;
  }

  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator1_Prev_iy != 0)) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_h = rtb_Gain_f;
  }

  /* Gain: '<S71>/Gain' incorporates:
   *  DiscreteIntegrator: '<S71>/Discrete-Time Integrator1'
   *  Sum: '<S71>/Sum5'
   */
  rtb_Gain_hb0 = (rtb_Gain_f - Controller_DW.DiscreteTimeIntegrator1_DSTAT_h) *
    62.831852F;

  /* Switch: '<S71>/Switch' incorporates:
   *  Gain: '<S71>/Gain1'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.reset > 0) {
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_Gain_hb0;
  }

  /* End of Switch: '<S71>/Switch' */

  /* Product: '<S68>/Multiply' incorporates:
   *  Constant: '<S68>/kd'
   */
  rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D * rtb_Saturation_d_idx_2;

  /* Saturate: '<S68>/Saturation' */
  if (rtb_Subtract3_i > CONTROL_PARAM.VEL_Z_D_MAX) {
    rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D_MAX;
  } else {
    if (rtb_Subtract3_i < CONTROL_PARAM.VEL_Z_D_MIN) {
      rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D_MIN;
    }
  }

  /* End of Saturate: '<S68>/Saturation' */

  /* Sum: '<S65>/Add' incorporates:
   *  Constant: '<S70>/kp'
   *  DiscreteIntegrator: '<S69>/Discrete-Time Integrator'
   *  Product: '<S70>/Multiply'
   */
  rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.VEL_Z_P * rtb_Gain_f +
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m) + rtb_Subtract3_i;

  /* Product: '<S63>/Multiply' incorporates:
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S63>/Cos'
   *  Trigonometry: '<S63>/Cos1'
   */
  rtb_Subtract3_i = arm_cos_f32(Controller_U.INS_Out.phi) * arm_cos_f32
    (Controller_U.INS_Out.theta);

  /* Saturate: '<S63>/Saturation1' */
  if (rtb_Subtract3_i > 1.0F) {
    rtb_Add3_c = 1.0F;
  } else if (rtb_Subtract3_i < 0.5F) {
    rtb_Add3_c = 0.5F;
  } else {
    rtb_Add3_c = rtb_Subtract3_i;
  }

  /* End of Saturate: '<S63>/Saturation1' */

  /* Gain: '<S63>/Gain' */
  rtb_Subtract3_i *= 2.0F;

  /* Saturate: '<S63>/Saturation' */
  if (rtb_Subtract3_i > 1.0F) {
    rtb_Subtract3_i = 1.0F;
  } else {
    if (rtb_Subtract3_i < 0.0F) {
      rtb_Subtract3_i = 0.0F;
    }
  }

  /* End of Saturate: '<S63>/Saturation' */

  /* Product: '<S63>/Multiply1' incorporates:
   *  Constant: '<S63>/Constant'
   *  Product: '<S63>/Divide'
   */
  rtb_Subtract3_i *= 1.0F / rtb_Add3_c * rtb_DiscreteTimeIntegrator1_j;

  /* Outputs for Atomic SubSystem: '<S4>/Ducted_Rocket' */
  /* Saturate: '<S14>/Saturation' incorporates:
   *  Constant: '<S14>/Constant1'
   *  Reshape: '<S14>/Reshape'
   */
  rtb_Saturation_f[0] = 1000U;

  /* Saturate: '<S14>/Saturation1' incorporates:
   *  Bias: '<S14>/Bias'
   *  Constant: '<S14>/Constant2'
   */
  if (1500.0F + CONTROL_PARAM.SERVO1_BIAS > 2000.0F) {
    rtb_Saturation_d_idx_2 = 2000.0F;
  } else if (1500.0F + CONTROL_PARAM.SERVO1_BIAS < 1000.0F) {
    rtb_Saturation_d_idx_2 = 1000.0F;
  } else {
    rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO1_BIAS;
  }

  /* DataTypeConversion: '<S14>/Data Type Conversion' */
  rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

  /* Saturate: '<S14>/Saturation' */
  if (rtb_throttle_cmd < 1000) {
    rtb_Saturation_f[1] = 1000U;
  } else {
    rtb_Saturation_f[1] = rtb_throttle_cmd;
  }

  /* Saturate: '<S14>/Saturation1' incorporates:
   *  Bias: '<S14>/Bias1'
   *  Constant: '<S14>/Constant3'
   */
  if (1500.0F + CONTROL_PARAM.SERVO2_BIAS > 2000.0F) {
    rtb_Saturation_d_idx_2 = 2000.0F;
  } else if (1500.0F + CONTROL_PARAM.SERVO2_BIAS < 1000.0F) {
    rtb_Saturation_d_idx_2 = 1000.0F;
  } else {
    rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO2_BIAS;
  }

  /* DataTypeConversion: '<S14>/Data Type Conversion' */
  rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

  /* Saturate: '<S14>/Saturation' */
  if (rtb_throttle_cmd < 1000) {
    rtb_Saturation_f[2] = 1000U;
  } else {
    rtb_Saturation_f[2] = rtb_throttle_cmd;
  }

  /* Saturate: '<S14>/Saturation1' incorporates:
   *  Bias: '<S14>/Bias2'
   *  Constant: '<S14>/Constant4'
   */
  if (1500.0F + CONTROL_PARAM.SERVO3_BIAS > 2000.0F) {
    rtb_Saturation_d_idx_2 = 2000.0F;
  } else if (1500.0F + CONTROL_PARAM.SERVO3_BIAS < 1000.0F) {
    rtb_Saturation_d_idx_2 = 1000.0F;
  } else {
    rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO3_BIAS;
  }

  /* DataTypeConversion: '<S14>/Data Type Conversion' */
  rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

  /* Saturate: '<S14>/Saturation' */
  if (rtb_throttle_cmd < 1000) {
    rtb_Saturation_f[3] = 1000U;
  } else {
    rtb_Saturation_f[3] = rtb_throttle_cmd;
  }

  /* Saturate: '<S14>/Saturation1' incorporates:
   *  Bias: '<S14>/Bias3'
   *  Constant: '<S14>/Constant5'
   */
  if (1500.0F + CONTROL_PARAM.SERVO4_BIAS > 2000.0F) {
    rtb_Saturation_d_idx_2 = 2000.0F;
  } else if (1500.0F + CONTROL_PARAM.SERVO4_BIAS < 1000.0F) {
    rtb_Saturation_d_idx_2 = 1000.0F;
  } else {
    rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO4_BIAS;
  }

  /* DataTypeConversion: '<S14>/Data Type Conversion' */
  rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

  /* Saturate: '<S14>/Saturation' */
  if (rtb_throttle_cmd < 1000) {
    rtb_Saturation_f[4] = 1000U;
  } else {
    rtb_Saturation_f[4] = rtb_throttle_cmd;
  }

  /* MultiPortSwitch: '<S9>/Multiport Switch' incorporates:
   *  Constant: '<S15>/Constant1'
   *  Inport: '<Root>/FMS_Out'
   *  Reshape: '<S15>/Reshape'
   *  Saturate: '<S15>/Saturation'
   *  Saturate: '<S7>/Saturation4'
   */
  switch (Controller_U.FMS_Out.status) {
   case 1:
    break;

   case 2:
    rtb_Saturation_f[0] = 1500U;

    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S15>/Bias'
     *  Constant: '<S15>/Constant1'
     *  Constant: '<S15>/Constant2'
     *  Reshape: '<S15>/Reshape'
     *  Saturate: '<S15>/Saturation'
     */
    if (1500.0F + CONTROL_PARAM.SERVO1_BIAS > 2000.0F) {
      rtb_Saturation_d_idx_2 = 2000.0F;
    } else if (1500.0F + CONTROL_PARAM.SERVO1_BIAS < 1000.0F) {
      rtb_Saturation_d_idx_2 = 1000.0F;
    } else {
      rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO1_BIAS;
    }

    /* DataTypeConversion: '<S15>/Data Type Conversion' */
    rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

    /* Saturate: '<S15>/Saturation' */
    if (rtb_throttle_cmd < 1000) {
      rtb_Saturation_f[1] = 1000U;
    } else {
      rtb_Saturation_f[1] = rtb_throttle_cmd;
    }

    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S15>/Bias1'
     *  Constant: '<S15>/Constant3'
     */
    if (1500.0F + CONTROL_PARAM.SERVO2_BIAS > 2000.0F) {
      rtb_Saturation_d_idx_2 = 2000.0F;
    } else if (1500.0F + CONTROL_PARAM.SERVO2_BIAS < 1000.0F) {
      rtb_Saturation_d_idx_2 = 1000.0F;
    } else {
      rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO2_BIAS;
    }

    /* DataTypeConversion: '<S15>/Data Type Conversion' */
    rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

    /* Saturate: '<S15>/Saturation' */
    if (rtb_throttle_cmd < 1000) {
      rtb_Saturation_f[2] = 1000U;
    } else {
      rtb_Saturation_f[2] = rtb_throttle_cmd;
    }

    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S15>/Bias2'
     *  Constant: '<S15>/Constant4'
     */
    if (1500.0F + CONTROL_PARAM.SERVO3_BIAS > 2000.0F) {
      rtb_Saturation_d_idx_2 = 2000.0F;
    } else if (1500.0F + CONTROL_PARAM.SERVO3_BIAS < 1000.0F) {
      rtb_Saturation_d_idx_2 = 1000.0F;
    } else {
      rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO3_BIAS;
    }

    /* DataTypeConversion: '<S15>/Data Type Conversion' */
    rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

    /* Saturate: '<S15>/Saturation' */
    if (rtb_throttle_cmd < 1000) {
      rtb_Saturation_f[3] = 1000U;
    } else {
      rtb_Saturation_f[3] = rtb_throttle_cmd;
    }

    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S15>/Bias3'
     *  Constant: '<S15>/Constant5'
     */
    if (1500.0F + CONTROL_PARAM.SERVO4_BIAS > 2000.0F) {
      rtb_Saturation_d_idx_2 = 2000.0F;
    } else if (1500.0F + CONTROL_PARAM.SERVO4_BIAS < 1000.0F) {
      rtb_Saturation_d_idx_2 = 1000.0F;
    } else {
      rtb_Saturation_d_idx_2 = 1500.0F + CONTROL_PARAM.SERVO4_BIAS;
    }

    /* DataTypeConversion: '<S15>/Data Type Conversion' */
    rtb_throttle_cmd = (uint16_T)fmodf(floorf(rtb_Saturation_d_idx_2), 65536.0F);

    /* Saturate: '<S15>/Saturation' */
    if (rtb_throttle_cmd < 1000) {
      rtb_Saturation_f[4] = 1000U;
    } else {
      rtb_Saturation_f[4] = rtb_throttle_cmd;
    }
    break;

   case 3:
    /* Switch: '<S8>/Switch' incorporates:
     *  Constant: '<S12>/Constant'
     *  Logic: '<S8>/Logical Operator'
     *  RelationalOperator: '<S11>/Compare'
     *  RelationalOperator: '<S12>/Compare'
     *  Saturate: '<S62>/Saturation'
     *  Switch: '<S8>/Switch1'
     */
    if ((Controller_U.FMS_Out.ctrl_mode == 1) || (Controller_U.FMS_Out.ctrl_mode
         == 2) || (Controller_U.FMS_Out.ctrl_mode == 3)) {
      rtb_throttle_cmd = Controller_U.FMS_Out.throttle_cmd;
    } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
      /* Switch: '<S13>/Switch' incorporates:
       *  Constant: '<S10>/Constant1'
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       *  Gain: '<S10>/Gain1'
       *  S-Function (sfix_bitop): '<S13>/cmd_throttle valid'
       *  Saturate: '<S62>/Saturation'
       *  Sum: '<S10>/Sum1'
       *  Switch: '<S8>/Switch1'
       */
      if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
        rtb_throttle_cmd = Controller_U.FMS_Out.throttle_cmd;
      } else {
        if (rtb_Subtract3_i > 0.35F) {
          /* Saturate: '<S62>/Saturation' */
          rtb_Subtract3_i = 0.35F;
        } else {
          if (rtb_Subtract3_i < -0.35F) {
            /* Saturate: '<S62>/Saturation' */
            rtb_Subtract3_i = -0.35F;
          }
        }

        /* Sum: '<S10>/Sum' incorporates:
         *  Constant: '<S10>/hover_throttle'
         *  Saturate: '<S62>/Saturation'
         */
        rtb_DiscreteTimeIntegrator1_j = rtb_Subtract3_i +
          CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S10>/Saturation' */
        if (rtb_DiscreteTimeIntegrator1_j > 1.0F) {
          rtb_DiscreteTimeIntegrator1_j = 1.0F;
        } else {
          if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
            rtb_DiscreteTimeIntegrator1_j = 0.0F;
          }
        }

        rtb_throttle_cmd = (uint16_T)((uint32_T)fmodf(floorf(1000.0F *
          rtb_DiscreteTimeIntegrator1_j), 4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S13>/Switch' */
    } else {
      if (rtb_Subtract3_i > 0.35F) {
        /* Saturate: '<S62>/Saturation' */
        rtb_Subtract3_i = 0.35F;
      } else {
        if (rtb_Subtract3_i < -0.35F) {
          /* Saturate: '<S62>/Saturation' */
          rtb_Subtract3_i = -0.35F;
        }
      }

      /* Sum: '<S10>/Sum' incorporates:
       *  Constant: '<S10>/hover_throttle'
       *  Saturate: '<S62>/Saturation'
       */
      rtb_DiscreteTimeIntegrator1_j = rtb_Subtract3_i + CONTROL_PARAM.HOVER_THRO;

      /* Saturate: '<S10>/Saturation' */
      if (rtb_DiscreteTimeIntegrator1_j > 1.0F) {
        rtb_DiscreteTimeIntegrator1_j = 1.0F;
      } else {
        if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
          rtb_DiscreteTimeIntegrator1_j = 0.0F;
        }
      }

      /* Switch: '<S8>/Switch1' incorporates:
       *  Constant: '<S10>/Constant1'
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       *  Gain: '<S10>/Gain1'
       *  Sum: '<S10>/Sum1'
       */
      rtb_throttle_cmd = (uint16_T)((uint32_T)fmodf(floorf(1000.0F *
        rtb_DiscreteTimeIntegrator1_j), 4.2949673E+9F) + 1000U);
    }

    /* End of Switch: '<S8>/Switch' */

    /* Switch: '<S30>/Switch' incorporates:
     *  Gain: '<S30>/Gain1'
     */
    if (Controller_U.FMS_Out.reset > 0) {
      rtb_Saturation_d_idx_0 = 0.0F;
      rtb_Saturation_d_idx_1 = 0.0F;
      rtb_Saturation_d_idx_2 = 0.0F;
    } else {
      rtb_Saturation_d_idx_0 = rtb_DiscreteTimeIntegrator1_b_i;
      rtb_Saturation_d_idx_1 = rtb_a_n_idx_1;
      rtb_Saturation_d_idx_2 = rtb_DiscreteTimeIntegrator_h;
    }

    /* End of Switch: '<S30>/Switch' */

    /* Product: '<S27>/Multiply' incorporates:
     *  Constant: '<S27>/gain1'
     */
    rtb_a_n_idx_0 = CONTROL_PARAM.ROLL_RATE_D * rtb_Saturation_d_idx_0;

    /* Saturate: '<S27>/Saturation' */
    if (rtb_a_n_idx_0 > CONTROL_PARAM.RATE_D_MAX) {
      rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MAX;
    } else {
      if (rtb_a_n_idx_0 < CONTROL_PARAM.RATE_D_MIN) {
        rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MIN;
      }
    }

    /* Saturate: '<S7>/Saturation1' incorporates:
     *  Constant: '<S29>/gain1'
     *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S28>/Discrete-Time Integrator'
     *  Product: '<S29>/Multiply'
     *  Sum: '<S24>/Add'
     */
    rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.ROLL_RATE_P *
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] +
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0]) + rtb_a_n_idx_0;
    if (rtb_DiscreteTimeIntegrator1_j > 1.0F) {
      rtb_DiscreteTimeIntegrator1_j = 1.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator1_j < -1.0F) {
        rtb_DiscreteTimeIntegrator1_j = -1.0F;
      }
    }

    /* Product: '<S27>/Multiply' incorporates:
     *  Constant: '<S27>/gain2'
     */
    rtb_a_n_idx_0 = CONTROL_PARAM.PITCH_RATE_D * rtb_Saturation_d_idx_1;

    /* Saturate: '<S27>/Saturation' */
    if (rtb_a_n_idx_0 > CONTROL_PARAM.RATE_D_MAX) {
      rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MAX;
    } else {
      if (rtb_a_n_idx_0 < CONTROL_PARAM.RATE_D_MIN) {
        rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MIN;
      }
    }

    /* Saturate: '<S7>/Saturation1' incorporates:
     *  Constant: '<S29>/gain2'
     *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S28>/Discrete-Time Integrator'
     *  Product: '<S29>/Multiply'
     *  Sum: '<S24>/Add'
     */
    rtb_Add3_c = (CONTROL_PARAM.PITCH_RATE_P *
                  Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] +
                  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1]) +
      rtb_a_n_idx_0;
    if (rtb_Add3_c > 1.0F) {
      rtb_Add3_c = 1.0F;
    } else {
      if (rtb_Add3_c < -1.0F) {
        rtb_Add3_c = -1.0F;
      }
    }

    /* Product: '<S27>/Multiply' incorporates:
     *  Constant: '<S27>/gain3'
     */
    rtb_a_n_idx_0 = CONTROL_PARAM.YAW_RATE_D * rtb_Saturation_d_idx_2;

    /* Saturate: '<S27>/Saturation' */
    if (rtb_a_n_idx_0 > CONTROL_PARAM.RATE_D_MAX) {
      rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MAX;
    } else {
      if (rtb_a_n_idx_0 < CONTROL_PARAM.RATE_D_MIN) {
        rtb_a_n_idx_0 = CONTROL_PARAM.RATE_D_MIN;
      }
    }

    /* Saturate: '<S7>/Saturation1' incorporates:
     *  Constant: '<S29>/gain3'
     *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S28>/Discrete-Time Integrator'
     *  Product: '<S29>/Multiply'
     *  Sum: '<S24>/Add'
     */
    rtb_a_n_idx_0 += CONTROL_PARAM.YAW_RATE_P *
      Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] +
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2];
    if (rtb_a_n_idx_0 > 1.0F) {
      rtb_a_n_idx_0 = 1.0F;
    } else {
      if (rtb_a_n_idx_0 < -1.0F) {
        rtb_a_n_idx_0 = -1.0F;
      }
    }

    for (i = 0; i < 4; i++) {
      rtb_Saturation_d_idx_2 = Controller_ConstP.Effective_Matrix_Value[i + 8] *
        rtb_a_n_idx_0 + (Controller_ConstP.Effective_Matrix_Value[i + 4] *
                         rtb_Add3_c + Controller_ConstP.Effective_Matrix_Value[i]
                         * rtb_DiscreteTimeIntegrator1_j);

      /* Saturate: '<S7>/Saturation3' incorporates:
       *  Constant: '<S7>/Effective_Matrix'
       *  Product: '<S7>/Multiply'
       */
      if (rtb_Saturation_d_idx_2 > 1.0F) {
        rtb_Saturation_d_idx_2 = 1.0F;
      } else {
        if (rtb_Saturation_d_idx_2 < -1.0F) {
          rtb_Saturation_d_idx_2 = -1.0F;
        }
      }

      /* Gain: '<S7>/Gain' incorporates:
       *  Saturate: '<S7>/Saturation3'
       */
      rtb_Bias_m[i] = 500.0F * rtb_Saturation_d_idx_2;
    }

    /* Saturate: '<S7>/Saturation' incorporates:
     *  Constant: '<S7>/Effective_Matrix'
     *  Product: '<S7>/Multiply'
     */
    if (rtb_throttle_cmd > 2000) {
      rtb_Saturation_f[0] = 2000U;
    } else if (rtb_throttle_cmd < 1100) {
      rtb_Saturation_f[0] = 1100U;
    } else {
      rtb_Saturation_f[0] = rtb_throttle_cmd;
    }

    /* End of Saturate: '<S7>/Saturation' */

    /* Bias: '<S7>/Bias' incorporates:
     *  Bias: '<S7>/Bias1'
     *  Gain: '<S7>/Gain1'
     */
    rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.SERVO1_SCALE * rtb_Bias_m[0]
      + CONTROL_PARAM.SERVO1_BIAS) + 1500.0F;

    /* Saturate: '<S7>/Saturation4' */
    if (rtb_DiscreteTimeIntegrator1_j > 1900.0F) {
      rtb_DiscreteTimeIntegrator1_j = 1900.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator1_j < 1100.0F) {
        rtb_DiscreteTimeIntegrator1_j = 1100.0F;
      }
    }

    rtb_Saturation_f[1] = (uint16_T)fmodf(floorf(rtb_DiscreteTimeIntegrator1_j),
      65536.0F);

    /* Bias: '<S7>/Bias' incorporates:
     *  Bias: '<S7>/Bias2'
     *  Gain: '<S7>/Gain2'
     *  Saturate: '<S7>/Saturation4'
     */
    rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.SERVO2_SCALE * rtb_Bias_m[1]
      + CONTROL_PARAM.SERVO2_BIAS) + 1500.0F;

    /* Saturate: '<S7>/Saturation4' */
    if (rtb_DiscreteTimeIntegrator1_j > 1900.0F) {
      rtb_DiscreteTimeIntegrator1_j = 1900.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator1_j < 1100.0F) {
        rtb_DiscreteTimeIntegrator1_j = 1100.0F;
      }
    }

    rtb_Saturation_f[2] = (uint16_T)fmodf(floorf(rtb_DiscreteTimeIntegrator1_j),
      65536.0F);

    /* Bias: '<S7>/Bias' incorporates:
     *  Bias: '<S7>/Bias3'
     *  Gain: '<S7>/Gain3'
     *  Saturate: '<S7>/Saturation4'
     */
    rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.SERVO3_SCALE * rtb_Bias_m[2]
      + CONTROL_PARAM.SERVO3_BIAS) + 1500.0F;

    /* Saturate: '<S7>/Saturation4' */
    if (rtb_DiscreteTimeIntegrator1_j > 1900.0F) {
      rtb_DiscreteTimeIntegrator1_j = 1900.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator1_j < 1100.0F) {
        rtb_DiscreteTimeIntegrator1_j = 1100.0F;
      }
    }

    rtb_Saturation_f[3] = (uint16_T)fmodf(floorf(rtb_DiscreteTimeIntegrator1_j),
      65536.0F);

    /* Bias: '<S7>/Bias' incorporates:
     *  Bias: '<S7>/Bias4'
     *  Gain: '<S7>/Gain4'
     *  Saturate: '<S7>/Saturation4'
     */
    rtb_DiscreteTimeIntegrator1_j = (CONTROL_PARAM.SERVO4_SCALE * rtb_Bias_m[3]
      + CONTROL_PARAM.SERVO4_BIAS) + 1500.0F;

    /* Saturate: '<S7>/Saturation4' */
    if (rtb_DiscreteTimeIntegrator1_j > 1900.0F) {
      rtb_DiscreteTimeIntegrator1_j = 1900.0F;
    } else {
      if (rtb_DiscreteTimeIntegrator1_j < 1100.0F) {
        rtb_DiscreteTimeIntegrator1_j = 1100.0F;
      }
    }

    rtb_Saturation_f[4] = (uint16_T)fmodf(floorf(rtb_DiscreteTimeIntegrator1_j),
      65536.0F);
    break;
  }

  /* End of MultiPortSwitch: '<S9>/Multiport Switch' */
  /* End of Outputs for SubSystem: '<S4>/Ducted_Rocket' */

  /* Product: '<S28>/Multiply' incorporates:
   *  Constant: '<S28>/gain1'
   *  Constant: '<S28>/gain2'
   *  Constant: '<S28>/gain3'
   *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator5'
   */
  rtb_Saturation_d_idx_0 = CONTROL_PARAM.ROLL_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[0];
  rtb_Saturation_d_idx_1 = CONTROL_PARAM.PITCH_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[1];
  rtb_Saturation_d_idx_2 = CONTROL_PARAM.YAW_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[2];

  /* Product: '<S53>/Multiply1' incorporates:
   *  Constant: '<S53>/const1'
   *  DiscreteIntegrator: '<S50>/Integrator'
   */
  rtb_a_n_idx_0 = Controller_DW.Integrator_DSTATE[0] * 0.05F;
  rtb_Saturation_idx_1 = Controller_DW.Integrator_DSTATE[1] * 0.05F;

  /* Sum: '<S53>/Add' incorporates:
   *  DiscreteIntegrator: '<S50>/Integrator1'
   *  Inport: '<Root>/FMS_Out'
   *  Sum: '<S50>/Subtract'
   */
  rtb_Subtract3_idx_0 = (Controller_DW.Integrator1_DSTATE[0] -
    Controller_U.FMS_Out.u_cmd) + rtb_a_n_idx_0;
  rtb_Subtract3_idx_1 = (Controller_DW.Integrator1_DSTATE[1] -
    Controller_U.FMS_Out.v_cmd) + rtb_Saturation_idx_1;

  /* Signum: '<S53>/Sign' */
  if (rtb_Subtract3_idx_0 < 0.0F) {
    rtb_DiscreteTimeIntegrator1_j = -1.0F;
  } else if (rtb_Subtract3_idx_0 > 0.0F) {
    rtb_DiscreteTimeIntegrator1_j = 1.0F;
  } else {
    rtb_DiscreteTimeIntegrator1_j = rtb_Subtract3_idx_0;
  }

  /* Sum: '<S53>/Add2' incorporates:
   *  Abs: '<S53>/Abs'
   *  Gain: '<S53>/Gain'
   *  Gain: '<S53>/Gain1'
   *  Product: '<S53>/Multiply2'
   *  Product: '<S53>/Multiply3'
   *  Signum: '<S53>/Sign'
   *  Sqrt: '<S53>/Sqrt'
   *  Sum: '<S53>/Add1'
   *  Sum: '<S53>/Subtract'
   */
  rtb_DiscreteTimeIntegrator1_j = (sqrtf((8.0F * fabsf(rtb_Subtract3_idx_0) +
    Controller_ConstB.d) * Controller_ConstB.d) - Controller_ConstB.d) * 0.5F *
    rtb_DiscreteTimeIntegrator1_j + rtb_a_n_idx_0;

  /* Sum: '<S53>/Add3' incorporates:
   *  Signum: '<S53>/Sign'
   */
  rtb_Add3_c = rtb_Subtract3_idx_0 + Controller_ConstB.d;

  /* Sum: '<S53>/Subtract1' incorporates:
   *  Signum: '<S53>/Sign'
   */
  rtb_Subtract3_i = rtb_Subtract3_idx_0 - Controller_ConstB.d;

  /* Signum: '<S53>/Sign1' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign2' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Sum: '<S53>/Add2' incorporates:
   *  Gain: '<S53>/Gain2'
   *  Product: '<S53>/Multiply4'
   *  Signum: '<S53>/Sign'
   *  Sum: '<S53>/Add4'
   *  Sum: '<S53>/Add5'
   *  Sum: '<S53>/Subtract2'
   */
  rtb_a_n_idx_0 = ((rtb_Subtract3_idx_0 - rtb_DiscreteTimeIntegrator1_j) +
                   rtb_a_n_idx_0) * ((rtb_Add3_c - rtb_Subtract3_i) * 0.5F) +
    rtb_DiscreteTimeIntegrator1_j;

  /* Signum: '<S53>/Sign' */
  if (rtb_Subtract3_idx_1 < 0.0F) {
    rtb_DiscreteTimeIntegrator1_j = -1.0F;
  } else if (rtb_Subtract3_idx_1 > 0.0F) {
    rtb_DiscreteTimeIntegrator1_j = 1.0F;
  } else {
    rtb_DiscreteTimeIntegrator1_j = rtb_Subtract3_idx_1;
  }

  /* Sum: '<S53>/Add2' incorporates:
   *  Abs: '<S53>/Abs'
   *  Gain: '<S53>/Gain'
   *  Gain: '<S53>/Gain1'
   *  Product: '<S53>/Multiply2'
   *  Product: '<S53>/Multiply3'
   *  Signum: '<S53>/Sign'
   *  Sqrt: '<S53>/Sqrt'
   *  Sum: '<S53>/Add1'
   *  Sum: '<S53>/Subtract'
   */
  rtb_DiscreteTimeIntegrator1_j = (sqrtf((8.0F * fabsf(rtb_Subtract3_idx_1) +
    Controller_ConstB.d) * Controller_ConstB.d) - Controller_ConstB.d) * 0.5F *
    rtb_DiscreteTimeIntegrator1_j + rtb_Saturation_idx_1;

  /* Sum: '<S53>/Add3' incorporates:
   *  Signum: '<S53>/Sign'
   */
  rtb_Add3_c = rtb_Subtract3_idx_1 + Controller_ConstB.d;

  /* Sum: '<S53>/Subtract1' incorporates:
   *  Signum: '<S53>/Sign'
   */
  rtb_Subtract3_i = rtb_Subtract3_idx_1 - Controller_ConstB.d;

  /* Signum: '<S53>/Sign1' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign2' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Sum: '<S53>/Add5' incorporates:
   *  Gain: '<S53>/Gain2'
   *  Product: '<S53>/Multiply4'
   *  Signum: '<S53>/Sign'
   *  Sum: '<S53>/Add2'
   *  Sum: '<S53>/Add4'
   *  Sum: '<S53>/Subtract2'
   */
  rtb_DiscreteTimeIntegrator1_j += ((rtb_Subtract3_idx_1 -
    rtb_DiscreteTimeIntegrator1_j) + rtb_Saturation_idx_1) * ((rtb_Add3_c -
    rtb_Subtract3_i) * 0.5F);

  /* Outport: '<Root>/Control_Out' incorporates:
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   *  Reshape: '<S9>/Reshape'
   */
  Controller_Y.Control_Out.timestamp =
    Controller_DW.DiscreteTimeIntegrator_DSTATE_n;
  for (i = 0; i < 5; i++) {
    /* Outputs for Atomic SubSystem: '<S4>/Ducted_Rocket' */
    Controller_Y.Control_Out.actuator_cmd[i] = rtb_Saturation_f[i];

    /* End of Outputs for SubSystem: '<S4>/Ducted_Rocket' */
  }

  for (i = 0; i < 11; i++) {
    Controller_Y.Control_Out.actuator_cmd[i + 5] = 0U;
  }

  /* End of Outport: '<Root>/Control_Out' */

  /* Product: '<S67>/Multiply1' incorporates:
   *  Constant: '<S67>/const1'
   *  DiscreteIntegrator: '<S66>/Integrator'
   */
  rtb_Add3_c = Controller_DW.Integrator_DSTATE_p * 0.05F;

  /* Sum: '<S67>/Add' incorporates:
   *  DiscreteIntegrator: '<S66>/Integrator1'
   *  Inport: '<Root>/FMS_Out'
   *  Sum: '<S66>/Subtract'
   */
  rtb_Subtract3_i = (Controller_DW.Integrator1_DSTATE_p -
                     Controller_U.FMS_Out.w_cmd) + rtb_Add3_c;

  /* Signum: '<S67>/Sign' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Saturation_idx_1 = -1.0F;
  } else if (rtb_Subtract3_i > 0.0F) {
    rtb_Saturation_idx_1 = 1.0F;
  } else {
    rtb_Saturation_idx_1 = rtb_Subtract3_i;
  }

  /* End of Signum: '<S67>/Sign' */

  /* Sum: '<S67>/Add2' incorporates:
   *  Abs: '<S67>/Abs'
   *  Gain: '<S67>/Gain'
   *  Gain: '<S67>/Gain1'
   *  Product: '<S67>/Multiply2'
   *  Product: '<S67>/Multiply3'
   *  Sqrt: '<S67>/Sqrt'
   *  Sum: '<S67>/Add1'
   *  Sum: '<S67>/Subtract'
   */
  rtb_Saturation_idx_1 = (sqrtf((8.0F * fabsf(rtb_Subtract3_i) +
    Controller_ConstB.d_n) * Controller_ConstB.d_n) - Controller_ConstB.d_n) *
    0.5F * rtb_Saturation_idx_1 + rtb_Add3_c;

  /* Sum: '<S67>/Add4' */
  rtb_Subtract3_idx_1 = (rtb_Subtract3_i - rtb_Saturation_idx_1) + rtb_Add3_c;

  /* Sum: '<S67>/Add3' */
  rtb_Add3_c = rtb_Subtract3_i + Controller_ConstB.d_n;

  /* Sum: '<S67>/Subtract1' */
  rtb_Subtract3_i -= Controller_ConstB.d_n;

  /* Signum: '<S67>/Sign1' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* End of Signum: '<S67>/Sign1' */

  /* Signum: '<S67>/Sign2' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* End of Signum: '<S67>/Sign2' */

  /* Sum: '<S67>/Add5' incorporates:
   *  Gain: '<S67>/Gain2'
   *  Product: '<S67>/Multiply4'
   *  Sum: '<S67>/Subtract2'
   */
  rtb_Saturation_idx_1 += (rtb_Add3_c - rtb_Subtract3_i) * 0.5F *
    rtb_Subtract3_idx_1;

  /* Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;

  /* Update for DiscreteIntegrator: '<S50>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S50>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE[0] += 0.002F *
    Controller_DW.Integrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S55>/ki'
   *  Product: '<S55>/Multiply'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE[0] += CONTROL_PARAM.VEL_XY_I *
    rtb_uv_error_C_mPs_idx_0 * 0.002F;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[0] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[0] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  /* Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] += 0.002F * rtb_Gain_idx_0;

  /* Update for DiscreteIntegrator: '<S50>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S50>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE[1] += 0.002F *
    Controller_DW.Integrator_DSTATE[1];

  /* Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S55>/ki'
   *  Product: '<S55>/Multiply'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE[1] += CONTROL_PARAM.VEL_XY_I *
    rtb_uv_error_C_mPs_idx_1 * 0.002F;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[1] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[1] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  /* Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] += 0.002F * rtb_Gain_idx_1;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' */
  Controller_DW.DiscreteTimeIntegrator5_IC_LOAD = 0U;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator_PrevRe_g = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_l = 0U;

  /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S26>/Gain'
   *  Sum: '<S26>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] +=
    (rtb_rate_error_B_radPs_idx_0 -
     Controller_DW.DiscreteTimeIntegrator5_DSTATE[0]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] += 0.002F *
    rtb_Saturation_d_idx_0;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  /* Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0] += 0.002F *
    rtb_DiscreteTimeIntegrator1_b_i;

  /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S26>/Gain'
   *  Sum: '<S26>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] +=
    (rtb_rate_error_B_radPs_idx_1 -
     Controller_DW.DiscreteTimeIntegrator5_DSTATE[1]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] += 0.002F *
    rtb_Saturation_d_idx_1;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  /* Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1] += 0.002F * rtb_a_n_idx_1;

  /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S26>/Gain'
   *  Sum: '<S26>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] +=
    (rtb_rate_error_B_radPs_idx_2 -
     Controller_DW.DiscreteTimeIntegrator5_DSTATE[2]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] += 0.002F *
    rtb_Saturation_d_idx_2;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  /* Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2] += 0.002F *
    rtb_DiscreteTimeIntegrator_h;
  Controller_DW.DiscreteTimeIntegrator1_PrevR_i = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S66>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S66>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE_p += 0.002F *
    Controller_DW.Integrator_DSTATE_p;

  /* Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S69>/ki'
   *  Inport: '<Root>/FMS_Out'
   *  Product: '<S69>/Multiply'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_m += CONTROL_PARAM.VEL_Z_I *
    rtb_Gain_f * 0.002F;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m >= CONTROL_PARAM.VEL_Z_I_MAX)
  {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m <=
        CONTROL_PARAM.VEL_Z_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRe_m = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* End of Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_k = 0U;
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_h += 0.002F * rtb_Gain_hb0;
  Controller_DW.DiscreteTimeIntegrator1_Prev_iy = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Signum: '<S53>/Sign3' incorporates:
   *  Product: '<S53>/Divide'
   *  Sum: '<S53>/Add6'
   */
  rtb_Add3_c = rtb_a_n_idx_0 + Controller_ConstB.d;

  /* Signum: '<S53>/Sign4' incorporates:
   *  Product: '<S53>/Divide'
   *  Sum: '<S53>/Subtract3'
   */
  rtb_uv_error_C_mPs_idx_0 = rtb_a_n_idx_0 - Controller_ConstB.d;

  /* Signum: '<S53>/Sign5' incorporates:
   *  Product: '<S53>/Divide'
   */
  if (rtb_a_n_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;
  } else if (rtb_a_n_idx_0 > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_a_n_idx_0;
  }

  /* Signum: '<S53>/Sign3' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign6' incorporates:
   *  Product: '<S53>/Divide'
   */
  if (rtb_a_n_idx_0 < 0.0F) {
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_a_n_idx_0 > 0.0F) {
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_Gain_idx_0 = rtb_a_n_idx_0;
  }

  /* Update for DiscreteIntegrator: '<S50>/Integrator' incorporates:
   *  Constant: '<S53>/const'
   *  Gain: '<S53>/Gain3'
   *  Product: '<S53>/Divide'
   *  Product: '<S53>/Multiply5'
   *  Product: '<S53>/Multiply6'
   *  Sum: '<S53>/Subtract4'
   *  Sum: '<S53>/Subtract5'
   *  Sum: '<S53>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE[0] += ((rtb_a_n_idx_0 / Controller_ConstB.d -
    rtb_uv_error_C_mPs_idx_1) * Controller_ConstB.Gain4 * ((rtb_Add3_c -
    rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_Gain_idx_0 * 58.836F) * 0.002F;

  /* Signum: '<S53>/Sign3' incorporates:
   *  Sum: '<S53>/Add6'
   */
  rtb_Add3_c = rtb_DiscreteTimeIntegrator1_j + Controller_ConstB.d;

  /* Signum: '<S53>/Sign4' incorporates:
   *  Sum: '<S53>/Subtract3'
   */
  rtb_uv_error_C_mPs_idx_0 = rtb_DiscreteTimeIntegrator1_j - Controller_ConstB.d;

  /* Signum: '<S53>/Sign5' */
  if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;
  } else if (rtb_DiscreteTimeIntegrator1_j > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_DiscreteTimeIntegrator1_j;
  }

  /* Signum: '<S53>/Sign3' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S53>/Sign6' */
  if (rtb_DiscreteTimeIntegrator1_j < 0.0F) {
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_DiscreteTimeIntegrator1_j > 0.0F) {
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_Gain_idx_0 = rtb_DiscreteTimeIntegrator1_j;
  }

  /* Update for DiscreteIntegrator: '<S50>/Integrator' incorporates:
   *  Constant: '<S53>/const'
   *  Gain: '<S53>/Gain3'
   *  Product: '<S53>/Divide'
   *  Product: '<S53>/Multiply5'
   *  Product: '<S53>/Multiply6'
   *  Sum: '<S53>/Subtract4'
   *  Sum: '<S53>/Subtract5'
   *  Sum: '<S53>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE[1] += ((rtb_DiscreteTimeIntegrator1_j /
    Controller_ConstB.d - rtb_uv_error_C_mPs_idx_1) * Controller_ConstB.Gain4 *
    ((rtb_Add3_c - rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_Gain_idx_0 * 58.836F)
    * 0.002F;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_n += CONTROL_EXPORT.period;

  /* Signum: '<S67>/Sign6' incorporates:
   *  Signum: '<S67>/Sign5'
   */
  if (rtb_Saturation_idx_1 < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;

    /* Signum: '<S67>/Sign5' */
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_Saturation_idx_1 > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;

    /* Signum: '<S67>/Sign5' */
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_Saturation_idx_1;

    /* Signum: '<S67>/Sign5' */
    rtb_Gain_idx_0 = rtb_Saturation_idx_1;
  }

  /* End of Signum: '<S67>/Sign6' */

  /* Sum: '<S67>/Add6' */
  rtb_Add3_c = rtb_Saturation_idx_1 + Controller_ConstB.d_n;

  /* Sum: '<S67>/Subtract3' */
  rtb_uv_error_C_mPs_idx_0 = rtb_Saturation_idx_1 - Controller_ConstB.d_n;

  /* Signum: '<S67>/Sign3' */
  if (rtb_Add3_c < 0.0F) {
    rtb_Add3_c = -1.0F;
  } else {
    if (rtb_Add3_c > 0.0F) {
      rtb_Add3_c = 1.0F;
    }
  }

  /* End of Signum: '<S67>/Sign3' */

  /* Signum: '<S67>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* End of Signum: '<S67>/Sign4' */

  /* Update for DiscreteIntegrator: '<S66>/Integrator' incorporates:
   *  Constant: '<S67>/const'
   *  Gain: '<S67>/Gain3'
   *  Product: '<S67>/Divide'
   *  Product: '<S67>/Multiply5'
   *  Product: '<S67>/Multiply6'
   *  Sum: '<S67>/Subtract4'
   *  Sum: '<S67>/Subtract5'
   *  Sum: '<S67>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE_p += ((rtb_Saturation_idx_1 /
    Controller_ConstB.d_n - rtb_Gain_idx_0) * Controller_ConstB.Gain4_k *
    ((rtb_Add3_c - rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_uv_error_C_mPs_idx_1 *
    78.448F) * 0.002F;
}

/* Model initialize function */
void Controller_init(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Controller_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&Controller_DW, 0,
                sizeof(DW_Controller_T));

  /* external inputs */
  (void)memset(&Controller_U, 0, sizeof(ExtU_Controller_T));

  /* external outputs */
  Controller_Y.Control_Out = Controller_rtZControl_Out_Bus;

  /* Start for Constant: '<S69>/Constant' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;

  /* InitializeConditions for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE[0] = Controller_ConstB.Constant;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[0] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[0] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[0] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_DSTATE[1] = Controller_ConstB.Constant;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] >=
      CONTROL_PARAM.VEL_XY_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE[1] = CONTROL_PARAM.VEL_XY_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE[1] <=
        CONTROL_PARAM.VEL_XY_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE[1] =
        CONTROL_PARAM.VEL_XY_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* End of InitializeConditions for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S26>/Discrete-Time Integrator5' */
  Controller_DW.DiscreteTimeIntegrator5_IC_LOAD = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
    Controller_ConstB.Constant_n[0];
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
    Controller_ConstB.Constant_n[1];
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
    Controller_ConstB.Constant_n[2];
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] >=
      CONTROL_PARAM.RATE_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] = CONTROL_PARAM.RATE_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] <=
        CONTROL_PARAM.RATE_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] =
        CONTROL_PARAM.RATE_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRe_g = 0;

  /* End of InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_l = 1U;
  Controller_DW.DiscreteTimeIntegrator1_PrevR_i = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m >= CONTROL_PARAM.VEL_Z_I_MAX)
  {
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE_m <=
        CONTROL_PARAM.VEL_Z_I_MIN) {
      Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRe_m = 0;

  /* End of InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S71>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_k = 1U;
  Controller_DW.DiscreteTimeIntegrator1_Prev_iy = 0;
}

/* Model terminate function */
void Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

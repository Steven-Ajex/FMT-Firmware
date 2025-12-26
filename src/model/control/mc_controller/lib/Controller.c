/*
 * File: Controller.c
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 1.1081
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Fri Dec 26 22:51:00 2025
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

/* Exported block signals */
real32_T att_cmd_B_rad[2];             /* '<S84>/Switch' */
real32_T att_est_B_rad[2];             /* '<S87>/Signal Copy' */
real32_T rate_cmd_B_radPs[3];          /* '<S68>/Switch' */
real32_T rate_est_B_radPs[3];          /* '<S72>/Signal Copy' */
real32_T w_cmd_C_mPs;                  /* '<S116>/Integrator1' */
real32_T w_est_C_mPs;                  /* '<S113>/Signal Copy1' */

/* Exported block parameters */
struct_oqmoiGpNYV6eEvDoym9FPF CONTROL_PARAM = {
  1.4F,
  0.2F,
  0.2F,
  0.6F,
  0.1F,
  0.0F,
  -1.0F,
  1.0F,
  -1.0F,
  1.0F,
  -0.15F,
  0.15F,
  -0.1F,
  0.1F,
  5.0F,
  5.0F,
  0.52359879F,
  0.1F,
  0.1F,
  0.15F,
  0.1F,
  0.1F,
  0.2F,
  0.003F,
  0.003F,
  0.001F,
  0.0F,
  0.0F,
  -0.1F,
  0.1F,
  -0.1F,
  0.1F,
  1.57079637F,
  3.14159274F,
  0.5F
} ;                                    /* Variable: CONTROL_PARAM
                                        * Referenced by:
                                        *   '<S14>/hover_throttle'
                                        *   '<S17>/hover_throttle'
                                        *   '<S23>/hover_throttle'
                                        *   '<S29>/hover_throttle'
                                        *   '<S35>/hover_throttle'
                                        *   '<S41>/hover_throttle'
                                        *   '<S47>/hover_throttle'
                                        *   '<S53>/hover_throttle'
                                        *   '<S93>/Saturation'
                                        *   '<S67>/Saturation'
                                        *   '<S67>/Saturation1'
                                        *   '<S103>/kd'
                                        *   '<S103>/Saturation'
                                        *   '<S104>/ki'
                                        *   '<S104>/Discrete-Time Integrator'
                                        *   '<S105>/kp'
                                        *   '<S118>/kd'
                                        *   '<S118>/Saturation'
                                        *   '<S119>/Constant'
                                        *   '<S119>/ki'
                                        *   '<S119>/Discrete-Time Integrator'
                                        *   '<S120>/kp'
                                        *   '<S70>/Gain'
                                        *   '<S70>/Gain1'
                                        *   '<S86>/Constant1'
                                        *   '<S86>/Constant2'
                                        *   '<S75>/gain1'
                                        *   '<S75>/gain2'
                                        *   '<S75>/gain3'
                                        *   '<S75>/Saturation'
                                        *   '<S76>/gain1'
                                        *   '<S76>/gain2'
                                        *   '<S76>/gain3'
                                        *   '<S76>/Discrete-Time Integrator'
                                        *   '<S77>/gain1'
                                        *   '<S77>/gain2'
                                        *   '<S77>/gain3'
                                        */

struct_ny3PY9hontv4J5WqwlFzJB CONTROL_EXPORT = {
  2U,

  { 77, 67, 32, 67, 111, 110, 116, 114, 111, 108, 108, 101, 114, 32, 118, 49, 46,
    48, 46, 48, 0 }
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

#if AIRFRAME == 1

/* Forward declaration for local functions */
#if AIRFRAME == 1

static real32_T Controller_sum(const real32_T x_e[4]);

#endif
#endif

/* Function for MATLAB Function: '<S14>/MATLAB Function1' */
#if AIRFRAME == 1
#if AIRFRAME == 1

static real32_T Controller_sum(const real32_T x_e[4])
{
  return ((x_e[0] + x_e[1]) + x_e[2]) + x_e[3];
}

#endif
#endif

/* Model step function */
void Controller_step(void)
{
  /* local block i/o variables */
  real32_T rtb_Sum[3];
  real32_T rtb_Saturation_i;
  uint16_T rtb_VariantMergeForOutportactua[16];
  boolean_T rtb_Compare_g;
  boolean_T rtb_Compare_eg;
  real32_T rtb_a;
  real32_T rtb_Subtract1_k;
  real32_T rtb_VectorConcatenate[9];
  real32_T rtb_Multiply_j[3];
  real32_T rtb_Subtract3_i;
  real32_T rtb_DiscreteTimeIntegrator1_j;
  real32_T rtb_Gain_am;
  int32_T i;
  real32_T rtb_Saturation_idx_0;
  real32_T rtb_att_error_B_rad_idx_1;
  real32_T rtb_att_error_B_rad_idx_0;
  real32_T rtb_Saturation1_idx_0;
  real32_T rtb_Add_g_idx_0;
  real32_T rtb_Add_g_idx_1;
  real32_T rtb_Saturation_d_idx_1;
  real32_T rtb_Add_g_idx_2;
  real32_T rtb_Saturation_d_idx_2;
  real32_T rtb_uv_error_C_mPs_idx_0;
  real32_T rtb_uv_error_C_mPs_idx_1;
  real32_T rtb_Gain_idx_0;
  real32_T rtb_Gain_idx_1;
  real32_T rtb_rate_error_B_radPs_idx_0;
  real32_T rtb_rate_error_B_radPs_idx_2;
  real32_T rtb_Gain_c_idx_1;
  real32_T rtb_Gain_c_idx_2;

  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   *  Inport: '<Root>/FMS_Out'
   */
  rtb_Compare_eg = (Controller_U.FMS_Out.mode >= 5);

  /* Trigonometry: '<S101>/Trigonometric Function1' incorporates:
   *  Gain: '<S100>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S101>/Trigonometric Function3'
   */
  rtb_uv_error_C_mPs_idx_0 = arm_cos_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[0] = rtb_uv_error_C_mPs_idx_0;

  /* Trigonometry: '<S101>/Trigonometric Function' incorporates:
   *  Gain: '<S100>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S101>/Trigonometric Function2'
   */
  rtb_uv_error_C_mPs_idx_1 = arm_sin_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[1] = rtb_uv_error_C_mPs_idx_1;

  /* SignalConversion: '<S101>/ConcatBufferAtVector Concatenate1In3' incorporates:
   *  Constant: '<S101>/Constant3'
   */
  rtb_VectorConcatenate[2] = 0.0F;

  /* Gain: '<S101>/Gain' */
  rtb_VectorConcatenate[3] = -rtb_uv_error_C_mPs_idx_1;

  /* Trigonometry: '<S101>/Trigonometric Function3' */
  rtb_VectorConcatenate[4] = rtb_uv_error_C_mPs_idx_0;

  /* SignalConversion: '<S101>/ConcatBufferAtVector Concatenate2In3' incorporates:
   *  Constant: '<S101>/Constant4'
   */
  rtb_VectorConcatenate[5] = 0.0F;

  /* SignalConversion: '<S101>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_VectorConcatenate[6] = Controller_ConstB.VectorConcatenate3[0];
  rtb_VectorConcatenate[7] = Controller_ConstB.VectorConcatenate3[1];
  rtb_VectorConcatenate[8] = Controller_ConstB.VectorConcatenate3[2];

  /* Product: '<S97>/Multiply' incorporates:
   *  Inport: '<Root>/INS_Out'
   *  SignalConversion: '<S97>/TmpSignal ConversionAtMultiplyInport2'
   */
  for (i = 0; i < 3; i++) {
    rtb_Multiply_j[i] = rtb_VectorConcatenate[i + 3] * Controller_U.INS_Out.ve +
      rtb_VectorConcatenate[i] * Controller_U.INS_Out.vn;
  }

  /* End of Product: '<S97>/Multiply' */

  /* Product: '<S94>/Multiply' incorporates:
   *  DataTypeConversion: '<S94>/Data Type Conversion'
   *  DiscreteIntegrator: '<S99>/Integrator1'
   *  Sum: '<S94>/Sum'
   */
  rtb_uv_error_C_mPs_idx_0 = (Controller_DW.Integrator1_DSTATE[0] -
    rtb_Multiply_j[0]) * (real32_T)rtb_Compare_eg;
  rtb_uv_error_C_mPs_idx_1 = (Controller_DW.Integrator1_DSTATE[1] -
    rtb_Multiply_j[1]) * (real32_T)rtb_Compare_eg;

  /* DiscreteIntegrator: '<S104>/Discrete-Time Integrator' incorporates:
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

  /* DiscreteIntegrator: '<S106>/Discrete-Time Integrator1' incorporates:
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

  /* Gain: '<S106>/Gain' incorporates:
   *  DiscreteIntegrator: '<S106>/Discrete-Time Integrator1'
   *  Sum: '<S106>/Sum5'
   */
  rtb_Gain_idx_0 = (rtb_uv_error_C_mPs_idx_0 -
                    Controller_DW.DiscreteTimeIntegrator1_DSTATE[0]) *
    62.831852F;
  rtb_Gain_idx_1 = (rtb_uv_error_C_mPs_idx_1 -
                    Controller_DW.DiscreteTimeIntegrator1_DSTATE[1]) *
    62.831852F;

  /* Switch: '<S106>/Switch' incorporates:
   *  Gain: '<S106>/Gain1'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.reset > 0) {
    rtb_att_error_B_rad_idx_0 = 0.0F;
    rtb_att_error_B_rad_idx_1 = 0.0F;
  } else {
    rtb_att_error_B_rad_idx_0 = rtb_Gain_idx_0;
    rtb_att_error_B_rad_idx_1 = rtb_Gain_idx_1;
  }

  /* End of Switch: '<S106>/Switch' */

  /* Product: '<S103>/Multiply' incorporates:
   *  Constant: '<S103>/kd'
   */
  rtb_a = CONTROL_PARAM.VEL_XY_D * rtb_att_error_B_rad_idx_0;

  /* Saturate: '<S103>/Saturation' */
  if (rtb_a > CONTROL_PARAM.VEL_XY_D_MAX) {
    rtb_a = CONTROL_PARAM.VEL_XY_D_MAX;
  } else {
    if (rtb_a < CONTROL_PARAM.VEL_XY_D_MIN) {
      rtb_a = CONTROL_PARAM.VEL_XY_D_MIN;
    }
  }

  /* Product: '<S103>/Multiply' incorporates:
   *  Constant: '<S103>/kd'
   */
  rtb_att_error_B_rad_idx_0 = rtb_a;
  rtb_a = CONTROL_PARAM.VEL_XY_D * rtb_att_error_B_rad_idx_1;

  /* Saturate: '<S103>/Saturation' */
  if (rtb_a > CONTROL_PARAM.VEL_XY_D_MAX) {
    rtb_a = CONTROL_PARAM.VEL_XY_D_MAX;
  } else {
    if (rtb_a < CONTROL_PARAM.VEL_XY_D_MIN) {
      rtb_a = CONTROL_PARAM.VEL_XY_D_MIN;
    }
  }

  /* Switch: '<S96>/Switch' incorporates:
   *  Constant: '<S105>/kp'
   *  Constant: '<S107>/Constant'
   *  Constant: '<S109>/Constant'
   *  Constant: '<S110>/Constant'
   *  DiscreteIntegrator: '<S104>/Discrete-Time Integrator'
   *  Inport: '<Root>/FMS_Out'
   *  Product: '<S105>/Multiply'
   *  Product: '<S108>/Multiply2'
   *  Product: '<S108>/Multiply3'
   *  RelationalOperator: '<S107>/Compare'
   *  RelationalOperator: '<S109>/Compare'
   *  RelationalOperator: '<S110>/Compare'
   *  S-Function (sfix_bitop): '<S108>/cmd_ax valid'
   *  S-Function (sfix_bitop): '<S108>/cmd_ay valid'
   *  S-Function (sfix_bitop): '<S108>/cmd_u valid'
   *  S-Function (sfix_bitop): '<S108>/cmd_v valid'
   *  Sum: '<S108>/Sum1'
   *  Sum: '<S95>/Add'
   */
  if (Controller_U.FMS_Out.ctrl_mode == 6) {
    rtb_att_error_B_rad_idx_0 = ((Controller_U.FMS_Out.cmd_mask & 64) > 0 ?
      (CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_0 +
       Controller_DW.DiscreteTimeIntegrator_DSTATE[0]) +
      rtb_att_error_B_rad_idx_0 : 0.0F) + ((Controller_U.FMS_Out.cmd_mask & 512)
      > 0 ? Controller_U.FMS_Out.ax_cmd : 0.0F);
    rtb_att_error_B_rad_idx_1 = ((Controller_U.FMS_Out.cmd_mask & 128) > 0 ?
      (CONTROL_PARAM.VEL_XY_P * rtb_uv_error_C_mPs_idx_1 +
       Controller_DW.DiscreteTimeIntegrator_DSTATE[1]) + rtb_a : 0.0F) +
      ((Controller_U.FMS_Out.cmd_mask & 1024) > 0 ? Controller_U.FMS_Out.ay_cmd :
       0.0F);
  } else {
    rtb_att_error_B_rad_idx_0 += CONTROL_PARAM.VEL_XY_P *
      rtb_uv_error_C_mPs_idx_0 + Controller_DW.DiscreteTimeIntegrator_DSTATE[0];
    rtb_att_error_B_rad_idx_1 = (CONTROL_PARAM.VEL_XY_P *
      rtb_uv_error_C_mPs_idx_1 + Controller_DW.DiscreteTimeIntegrator_DSTATE[1])
      + rtb_a;
  }

  /* End of Switch: '<S96>/Switch' */

  /* Trigonometry: '<S93>/Atan' incorporates:
   *  Constant: '<S93>/g'
   *  Gain: '<S93>/Gain1'
   *  Gain: '<S93>/gain'
   *  Product: '<S93>/Divide'
   */
  rtb_rate_error_B_radPs_idx_0 = atanf(1.1F * rtb_att_error_B_rad_idx_1 /
    9.8055F);
  rtb_Saturation1_idx_0 = atanf(1.1F * -rtb_att_error_B_rad_idx_0 / 9.8055F);

  /* Switch: '<S84>/Switch' incorporates:
   *  Constant: '<S91>/Constant'
   *  Inport: '<Root>/FMS_Out'
   *  Logic: '<S84>/Logical Operator'
   *  RelationalOperator: '<S90>/Compare'
   *  RelationalOperator: '<S91>/Compare'
   *  Saturate: '<S93>/Saturation'
   *  Switch: '<S84>/Switch1'
   */
  if ((Controller_U.FMS_Out.ctrl_mode == 3) || (Controller_U.FMS_Out.ctrl_mode ==
       4)) {
    att_cmd_B_rad[0] = Controller_U.FMS_Out.phi_cmd;
    att_cmd_B_rad[1] = Controller_U.FMS_Out.theta_cmd;
  } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
    /* Switch: '<S92>/Switch' incorporates:
     *  S-Function (sfix_bitop): '<S92>/cmd_phi valid'
     *  S-Function (sfix_bitop): '<S92>/cmd_theta valid'
     *  Saturate: '<S93>/Saturation'
     *  Switch: '<S84>/Switch1'
     */
    if ((Controller_U.FMS_Out.cmd_mask & 8) > 0) {
      att_cmd_B_rad[0] = Controller_U.FMS_Out.phi_cmd;
    } else if (rtb_rate_error_B_radPs_idx_0 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S93>/Saturation' */
      att_cmd_B_rad[0] = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_rate_error_B_radPs_idx_0 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S93>/Saturation' */
      att_cmd_B_rad[0] = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      att_cmd_B_rad[0] = rtb_rate_error_B_radPs_idx_0;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 16) > 0) {
      att_cmd_B_rad[1] = Controller_U.FMS_Out.theta_cmd;
    } else if (rtb_Saturation1_idx_0 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Saturate: '<S93>/Saturation' */
      att_cmd_B_rad[1] = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_Saturation1_idx_0 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Saturate: '<S93>/Saturation' */
      att_cmd_B_rad[1] = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      att_cmd_B_rad[1] = rtb_Saturation1_idx_0;
    }

    /* End of Switch: '<S92>/Switch' */
  } else {
    if (rtb_rate_error_B_radPs_idx_0 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Saturate: '<S93>/Saturation' incorporates:
       *  Switch: '<S84>/Switch1'
       */
      att_cmd_B_rad[0] = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_rate_error_B_radPs_idx_0 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM)
    {
      /* Saturate: '<S93>/Saturation' incorporates:
       *  Switch: '<S84>/Switch1'
       */
      att_cmd_B_rad[0] = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      /* Switch: '<S84>/Switch1' incorporates:
       *  Saturate: '<S93>/Saturation'
       */
      att_cmd_B_rad[0] = rtb_rate_error_B_radPs_idx_0;
    }

    /* Saturate: '<S93>/Saturation' */
    if (rtb_Saturation1_idx_0 > CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Switch: '<S84>/Switch1' */
      att_cmd_B_rad[1] = CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else if (rtb_Saturation1_idx_0 < -CONTROL_PARAM.ROLL_PITCH_CMD_LIM) {
      /* Switch: '<S84>/Switch1' */
      att_cmd_B_rad[1] = -CONTROL_PARAM.ROLL_PITCH_CMD_LIM;
    } else {
      /* Switch: '<S84>/Switch1' */
      att_cmd_B_rad[1] = rtb_Saturation1_idx_0;
    }
  }

  /* End of Switch: '<S84>/Switch' */

  /* SignalConversion: '<S87>/Signal Copy' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  att_est_B_rad[0] = Controller_U.INS_Out.phi;
  att_est_B_rad[1] = Controller_U.INS_Out.theta;

  /* Sum: '<S85>/Sum' */
  rtb_a = att_cmd_B_rad[0] - att_est_B_rad[0];

  /* Product: '<S88>/Divide1' incorporates:
   *  Abs: '<S88>/Abs'
   *  Constant: '<S88>/const2'
   */
  rtb_rate_error_B_radPs_idx_0 = fabsf(rtb_a) / 0.002F;

  /* Product: '<S88>/Divide' incorporates:
   *  Constant: '<S86>/Constant1'
   *  Constant: '<S88>/const1'
   *  Math: '<S88>/Square'
   *  SignalConversion: '<S88>/TmpSignal ConversionAtSquareInport1'
   */
  rtb_rate_error_B_radPs_idx_2 = 9.5993F / (CONTROL_PARAM.ROLL_P *
    CONTROL_PARAM.ROLL_P);

  /* Signum: '<S88>/Sign' */
  if (rtb_a < 0.0F) {
    rtb_Saturation_d_idx_2 = -1.0F;
  } else if (rtb_a > 0.0F) {
    rtb_Saturation_d_idx_2 = 1.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_a;
  }

  /* Switch: '<S88>/Switch' incorporates:
   *  Constant: '<S86>/Constant1'
   *  Gain: '<S88>/Gain1'
   *  Gain: '<S88>/Gain2'
   *  Logic: '<S88>/Logical Operator'
   *  Product: '<S88>/Multiply'
   *  Product: '<S88>/Multiply1'
   *  Product: '<S88>/Multiply2'
   *  Product: '<S88>/Multiply3'
   *  RelationalOperator: '<S88>/Relational Operator'
   *  RelationalOperator: '<S88>/Relational Operator2'
   *  SignalConversion: '<S88>/TmpSignal ConversionAtSquareInport1'
   *  Sqrt: '<S88>/Sqrt'
   *  Sum: '<S88>/Subtract'
   */
  if ((rtb_a <= rtb_rate_error_B_radPs_idx_2) && (rtb_a >=
       -rtb_rate_error_B_radPs_idx_2)) {
    rtb_Saturation_d_idx_2 = rtb_a * CONTROL_PARAM.ROLL_P;
  } else {
    rtb_Saturation_d_idx_2 *= sqrtf((rtb_Saturation_d_idx_2 * rtb_a - 0.5F *
      rtb_rate_error_B_radPs_idx_2) * Controller_ConstB.Gain);
  }

  /* Gain: '<S88>/Gain3' */
  rtb_rate_error_B_radPs_idx_2 = -rtb_rate_error_B_radPs_idx_0;

  /* Switch: '<S89>/Switch' incorporates:
   *  Gain: '<S88>/Gain3'
   *  RelationalOperator: '<S89>/UpperRelop'
   */
  if (rtb_Saturation_d_idx_2 >= -rtb_rate_error_B_radPs_idx_0) {
    rtb_rate_error_B_radPs_idx_2 = rtb_Saturation_d_idx_2;
  }

  /* Switch: '<S89>/Switch2' incorporates:
   *  RelationalOperator: '<S89>/LowerRelop1'
   */
  if (rtb_Saturation_d_idx_2 <= rtb_rate_error_B_radPs_idx_0) {
    rtb_rate_error_B_radPs_idx_0 = rtb_rate_error_B_radPs_idx_2;
  }

  /* Saturate: '<S67>/Saturation1' */
  if (rtb_rate_error_B_radPs_idx_0 > CONTROL_PARAM.P_Q_CMD_LIM) {
    rtb_rate_error_B_radPs_idx_0 = CONTROL_PARAM.P_Q_CMD_LIM;
  } else {
    if (rtb_rate_error_B_radPs_idx_0 < -CONTROL_PARAM.P_Q_CMD_LIM) {
      rtb_rate_error_B_radPs_idx_0 = -CONTROL_PARAM.P_Q_CMD_LIM;
    }
  }

  /* Product: '<S88>/Divide1' */
  rtb_Saturation1_idx_0 = rtb_rate_error_B_radPs_idx_0;

  /* Sum: '<S85>/Sum' */
  rtb_a = att_cmd_B_rad[1] - att_est_B_rad[1];

  /* Product: '<S88>/Divide1' incorporates:
   *  Abs: '<S88>/Abs'
   *  Constant: '<S88>/const2'
   */
  rtb_rate_error_B_radPs_idx_0 = fabsf(rtb_a) / 0.002F;

  /* Product: '<S88>/Divide' incorporates:
   *  Constant: '<S86>/Constant2'
   *  Constant: '<S88>/const1'
   *  Math: '<S88>/Square'
   *  SignalConversion: '<S88>/TmpSignal ConversionAtSquareInport1'
   */
  rtb_rate_error_B_radPs_idx_2 = 9.5993F / (CONTROL_PARAM.PITCH_P *
    CONTROL_PARAM.PITCH_P);

  /* Signum: '<S88>/Sign' */
  if (rtb_a < 0.0F) {
    rtb_Saturation_d_idx_2 = -1.0F;
  } else if (rtb_a > 0.0F) {
    rtb_Saturation_d_idx_2 = 1.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_a;
  }

  /* Switch: '<S88>/Switch' incorporates:
   *  Constant: '<S86>/Constant2'
   *  Gain: '<S88>/Gain1'
   *  Gain: '<S88>/Gain2'
   *  Logic: '<S88>/Logical Operator'
   *  Product: '<S88>/Multiply'
   *  Product: '<S88>/Multiply1'
   *  Product: '<S88>/Multiply2'
   *  Product: '<S88>/Multiply3'
   *  RelationalOperator: '<S88>/Relational Operator'
   *  RelationalOperator: '<S88>/Relational Operator2'
   *  SignalConversion: '<S88>/TmpSignal ConversionAtSquareInport1'
   *  Sqrt: '<S88>/Sqrt'
   *  Sum: '<S88>/Subtract'
   */
  if ((rtb_a <= rtb_rate_error_B_radPs_idx_2) && (rtb_a >=
       -rtb_rate_error_B_radPs_idx_2)) {
    rtb_Saturation_d_idx_2 = rtb_a * CONTROL_PARAM.PITCH_P;
  } else {
    rtb_Saturation_d_idx_2 *= sqrtf((rtb_Saturation_d_idx_2 * rtb_a - 0.5F *
      rtb_rate_error_B_radPs_idx_2) * Controller_ConstB.Gain);
  }

  /* Gain: '<S88>/Gain3' */
  rtb_rate_error_B_radPs_idx_2 = -rtb_rate_error_B_radPs_idx_0;

  /* Switch: '<S89>/Switch' incorporates:
   *  Gain: '<S88>/Gain3'
   *  RelationalOperator: '<S89>/UpperRelop'
   */
  if (rtb_Saturation_d_idx_2 >= -rtb_rate_error_B_radPs_idx_0) {
    rtb_rate_error_B_radPs_idx_2 = rtb_Saturation_d_idx_2;
  }

  /* Switch: '<S89>/Switch2' incorporates:
   *  RelationalOperator: '<S89>/LowerRelop1'
   */
  if (rtb_Saturation_d_idx_2 <= rtb_rate_error_B_radPs_idx_0) {
    rtb_rate_error_B_radPs_idx_0 = rtb_rate_error_B_radPs_idx_2;
  }

  /* Saturate: '<S67>/Saturation1' */
  if (rtb_rate_error_B_radPs_idx_0 > CONTROL_PARAM.P_Q_CMD_LIM) {
    rtb_rate_error_B_radPs_idx_0 = CONTROL_PARAM.P_Q_CMD_LIM;
  } else {
    if (rtb_rate_error_B_radPs_idx_0 < -CONTROL_PARAM.P_Q_CMD_LIM) {
      rtb_rate_error_B_radPs_idx_0 = -CONTROL_PARAM.P_Q_CMD_LIM;
    }
  }

  /* Saturate: '<S67>/Saturation' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.psi_rate_cmd > CONTROL_PARAM.R_CMD_LIM) {
    rtb_a = CONTROL_PARAM.R_CMD_LIM;
  } else if (Controller_U.FMS_Out.psi_rate_cmd < -CONTROL_PARAM.R_CMD_LIM) {
    rtb_a = -CONTROL_PARAM.R_CMD_LIM;
  } else {
    rtb_a = Controller_U.FMS_Out.psi_rate_cmd;
  }

  /* End of Saturate: '<S67>/Saturation' */

  /* Trigonometry: '<S79>/Sin' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_Subtract1_k = arm_sin_f32(Controller_U.INS_Out.phi);

  /* Trigonometry: '<S79>/Cos1' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_Subtract3_i = arm_cos_f32(Controller_U.INS_Out.theta);

  /* Product: '<S79>/Multiply3' */
  rtb_rate_error_B_radPs_idx_2 = rtb_Subtract1_k * rtb_Subtract3_i * rtb_a;

  /* Trigonometry: '<S79>/Cos' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rtb_DiscreteTimeIntegrator1_j = arm_cos_f32(Controller_U.INS_Out.phi);

  /* Product: '<S79>/Multiply1' */
  rtb_Gain_c_idx_1 = rtb_DiscreteTimeIntegrator1_j *
    rtb_rate_error_B_radPs_idx_0;

  /* Product: '<S79>/Multiply4' */
  rtb_Subtract1_k *= rtb_rate_error_B_radPs_idx_0;

  /* Switch: '<S68>/Switch' incorporates:
   *  Constant: '<S80>/Constant'
   *  Constant: '<S81>/Constant'
   *  Inport: '<Root>/FMS_Out'
   *  RelationalOperator: '<S80>/Compare'
   *  RelationalOperator: '<S81>/Compare'
   *  Switch: '<S68>/Switch1'
   */
  if (Controller_U.FMS_Out.ctrl_mode == 2) {
    rate_cmd_B_radPs[0] = Controller_U.FMS_Out.p_cmd;
    rate_cmd_B_radPs[1] = Controller_U.FMS_Out.q_cmd;
    rate_cmd_B_radPs[2] = Controller_U.FMS_Out.r_cmd;
  } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
    /* Switch: '<S82>/Switch' incorporates:
     *  Inport: '<Root>/INS_Out'
     *  Product: '<S79>/Multiply'
     *  Product: '<S79>/Multiply1'
     *  Product: '<S79>/Multiply5'
     *  S-Function (sfix_bitop): '<S82>/cmd_p valid'
     *  S-Function (sfix_bitop): '<S82>/cmd_q valid'
     *  S-Function (sfix_bitop): '<S82>/cmd_r valid'
     *  Sum: '<S79>/Add'
     *  Sum: '<S79>/Add1'
     *  Sum: '<S79>/Add2'
     *  Switch: '<S68>/Switch1'
     *  Trigonometry: '<S79>/Sin1'
     */
    if ((Controller_U.FMS_Out.cmd_mask & 1) > 0) {
      rate_cmd_B_radPs[0] = Controller_U.FMS_Out.p_cmd;
    } else {
      rate_cmd_B_radPs[0] = rtb_Saturation1_idx_0 - arm_sin_f32
        (Controller_U.INS_Out.theta) * rtb_a;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 2) > 0) {
      rate_cmd_B_radPs[1] = Controller_U.FMS_Out.q_cmd;
    } else {
      rate_cmd_B_radPs[1] = rtb_rate_error_B_radPs_idx_2 + rtb_Gain_c_idx_1;
    }

    if ((Controller_U.FMS_Out.cmd_mask & 4) > 0) {
      rate_cmd_B_radPs[2] = Controller_U.FMS_Out.r_cmd;
    } else {
      rate_cmd_B_radPs[2] = rtb_DiscreteTimeIntegrator1_j * rtb_Subtract3_i *
        rtb_a - rtb_Subtract1_k;
    }

    /* End of Switch: '<S82>/Switch' */
  } else {
    /* Switch: '<S68>/Switch1' incorporates:
     *  Inport: '<Root>/INS_Out'
     *  Product: '<S79>/Multiply'
     *  Product: '<S79>/Multiply3'
     *  Product: '<S79>/Multiply5'
     *  Sum: '<S79>/Add'
     *  Sum: '<S79>/Add1'
     *  Sum: '<S79>/Add2'
     *  Trigonometry: '<S79>/Sin1'
     */
    rate_cmd_B_radPs[0] = rtb_Saturation1_idx_0 - arm_sin_f32
      (Controller_U.INS_Out.theta) * rtb_a;
    rate_cmd_B_radPs[1] = rtb_Gain_c_idx_1 + rtb_rate_error_B_radPs_idx_2;
    rate_cmd_B_radPs[2] = rtb_DiscreteTimeIntegrator1_j * rtb_Subtract3_i *
      rtb_a - rtb_Subtract1_k;
  }

  /* End of Switch: '<S68>/Switch' */

  /* SignalConversion: '<S72>/Signal Copy' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  rate_est_B_radPs[0] = Controller_U.INS_Out.p;
  rate_est_B_radPs[1] = Controller_U.INS_Out.q;
  rate_est_B_radPs[2] = Controller_U.INS_Out.r;

  /* Sum: '<S69>/Sum' */
  rtb_rate_error_B_radPs_idx_0 = rate_cmd_B_radPs[0] - rate_est_B_radPs[0];
  rtb_Saturation1_idx_0 = rate_cmd_B_radPs[1] - rate_est_B_radPs[1];
  rtb_rate_error_B_radPs_idx_2 = rate_cmd_B_radPs[2] - rate_est_B_radPs[2];

  /* DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' */
  if (Controller_DW.DiscreteTimeIntegrator5_IC_LOAD != 0) {
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] =
      rtb_rate_error_B_radPs_idx_0;
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] = rtb_Saturation1_idx_0;
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] =
      rtb_rate_error_B_radPs_idx_2;
  }

  /* DiscreteIntegrator: '<S76>/Discrete-Time Integrator' incorporates:
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

  /* DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
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

  /* Gain: '<S78>/Gain' incorporates:
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
   *  DiscreteIntegrator: '<S78>/Discrete-Time Integrator1'
   *  Sum: '<S78>/Sum5'
   */
  rtb_DiscreteTimeIntegrator1_j = (Controller_DW.DiscreteTimeIntegrator5_DSTATE
    [0] - Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0]) * 188.49556F;
  rtb_Gain_c_idx_1 = (Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] -
                      Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1]) *
    188.49556F;
  rtb_Gain_c_idx_2 = (Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] -
                      Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2]) *
    188.49556F;

  /* Switch: '<S78>/Switch' incorporates:
   *  Gain: '<S78>/Gain1'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.reset > 0) {
    rtb_a = 0.0F;
    rtb_Saturation_d_idx_1 = 0.0F;
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_a = rtb_DiscreteTimeIntegrator1_j;
    rtb_Saturation_d_idx_1 = rtb_Gain_c_idx_1;
    rtb_Saturation_d_idx_2 = rtb_Gain_c_idx_2;
  }

  /* End of Switch: '<S78>/Switch' */

  /* Product: '<S75>/Multiply' incorporates:
   *  Constant: '<S75>/gain1'
   */
  rtb_Gain_am = CONTROL_PARAM.ROLL_RATE_D * rtb_a;

  /* Saturate: '<S75>/Saturation' */
  if (rtb_Gain_am > CONTROL_PARAM.RATE_D_MAX) {
    rtb_Gain_am = CONTROL_PARAM.RATE_D_MAX;
  } else {
    if (rtb_Gain_am < CONTROL_PARAM.RATE_D_MIN) {
      rtb_Gain_am = CONTROL_PARAM.RATE_D_MIN;
    }
  }

  /* Product: '<S75>/Multiply' incorporates:
   *  Constant: '<S75>/gain2'
   */
  rtb_a = rtb_Gain_am;
  rtb_Gain_am = CONTROL_PARAM.PITCH_RATE_D * rtb_Saturation_d_idx_1;

  /* Saturate: '<S75>/Saturation' */
  if (rtb_Gain_am > CONTROL_PARAM.RATE_D_MAX) {
    rtb_Gain_am = CONTROL_PARAM.RATE_D_MAX;
  } else {
    if (rtb_Gain_am < CONTROL_PARAM.RATE_D_MIN) {
      rtb_Gain_am = CONTROL_PARAM.RATE_D_MIN;
    }
  }

  /* Product: '<S75>/Multiply' incorporates:
   *  Constant: '<S75>/gain3'
   */
  rtb_Saturation_d_idx_1 = rtb_Gain_am;
  rtb_Gain_am = CONTROL_PARAM.YAW_RATE_D * rtb_Saturation_d_idx_2;

  /* Saturate: '<S75>/Saturation' */
  if (rtb_Gain_am > CONTROL_PARAM.RATE_D_MAX) {
    rtb_Gain_am = CONTROL_PARAM.RATE_D_MAX;
  } else {
    if (rtb_Gain_am < CONTROL_PARAM.RATE_D_MIN) {
      rtb_Gain_am = CONTROL_PARAM.RATE_D_MIN;
    }
  }

  /* DeadZone: '<S74>/Dead Zone' */
  if (rate_cmd_B_radPs[0] > 0.1F) {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[0] - 0.1F;
  } else if (rate_cmd_B_radPs[0] >= -0.1F) {
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[0] - -0.1F;
  }

  /* Sum: '<S66>/Sum' incorporates:
   *  Constant: '<S77>/gain1'
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
   *  DiscreteIntegrator: '<S76>/Discrete-Time Integrator'
   *  Gain: '<S70>/Gain'
   *  Gain: '<S74>/Gain'
   *  Product: '<S77>/Multiply'
   *  Sum: '<S71>/Add'
   */
  rtb_Sum[0] = ((CONTROL_PARAM.ROLL_RATE_P *
                 Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] +
                 Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0]) + rtb_a) +
    1.11111116F * rtb_Saturation_d_idx_2 * CONTROL_PARAM.ROLL_RATE_FF;

  /* DeadZone: '<S74>/Dead Zone' */
  if (rate_cmd_B_radPs[1] > 0.1F) {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[1] - 0.1F;
  } else if (rate_cmd_B_radPs[1] >= -0.1F) {
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[1] - -0.1F;
  }

  /* Sum: '<S66>/Sum' incorporates:
   *  Constant: '<S77>/gain2'
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
   *  DiscreteIntegrator: '<S76>/Discrete-Time Integrator'
   *  Gain: '<S70>/Gain'
   *  Gain: '<S74>/Gain'
   *  Product: '<S77>/Multiply'
   *  Sum: '<S71>/Add'
   */
  rtb_Sum[1] = ((CONTROL_PARAM.PITCH_RATE_P *
                 Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] +
                 Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1]) +
                rtb_Saturation_d_idx_1) + 1.11111116F * rtb_Saturation_d_idx_2 *
    CONTROL_PARAM.ROLL_RATE_FF;

  /* DeadZone: '<S74>/Dead Zone' */
  if (rate_cmd_B_radPs[2] > 0.1F) {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[2] - 0.1F;
  } else if (rate_cmd_B_radPs[2] >= -0.1F) {
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_Saturation_d_idx_2 = rate_cmd_B_radPs[2] - -0.1F;
  }

  /* Sum: '<S66>/Sum' incorporates:
   *  Constant: '<S77>/gain3'
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
   *  DiscreteIntegrator: '<S76>/Discrete-Time Integrator'
   *  Gain: '<S70>/Gain1'
   *  Gain: '<S74>/Gain'
   *  Product: '<S77>/Multiply'
   *  Sum: '<S71>/Add'
   */
  rtb_Sum[2] = ((CONTROL_PARAM.YAW_RATE_P *
                 Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] +
                 Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2]) + rtb_Gain_am)
    + 1.11111116F * rtb_Saturation_d_idx_2 * CONTROL_PARAM.PITCH_RATE_FF;

  /* RelationalOperator: '<S115>/Compare' incorporates:
   *  Constant: '<S115>/Constant'
   *  Inport: '<Root>/FMS_Out'
   */
  rtb_Compare_g = (Controller_U.FMS_Out.mode >= 4);

  /* DiscreteIntegrator: '<S116>/Integrator1' */
  w_cmd_C_mPs = Controller_DW.Integrator1_DSTATE_p;

  /* SignalConversion: '<S113>/Signal Copy1' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  w_est_C_mPs = Controller_U.INS_Out.vd;

  /* Product: '<S113>/Multiply' incorporates:
   *  DataTypeConversion: '<S113>/Data Type Conversion'
   *  Gain: '<S113>/Gain'
   *  Sum: '<S113>/Sum1'
   */
  rtb_Saturation_d_idx_1 = -(w_cmd_C_mPs - w_est_C_mPs) * (real32_T)
    rtb_Compare_g;

  /* DiscreteIntegrator: '<S119>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S119>/Constant'
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

  /* DiscreteIntegrator: '<S121>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_DW.DiscreteTimeIntegrator1_IC_LO_k != 0) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_h = rtb_Saturation_d_idx_1;
  }

  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator1_Prev_iy != 0)) {
    Controller_DW.DiscreteTimeIntegrator1_DSTAT_h = rtb_Saturation_d_idx_1;
  }

  /* Gain: '<S121>/Gain' incorporates:
   *  DiscreteIntegrator: '<S121>/Discrete-Time Integrator1'
   *  Sum: '<S121>/Sum5'
   */
  rtb_Gain_am = (rtb_Saturation_d_idx_1 -
                 Controller_DW.DiscreteTimeIntegrator1_DSTAT_h) * 62.831852F;

  /* Switch: '<S121>/Switch' incorporates:
   *  Gain: '<S121>/Gain1'
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_U.FMS_Out.reset > 0) {
    rtb_Saturation_d_idx_2 = 0.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_Gain_am;
  }

  /* End of Switch: '<S121>/Switch' */

  /* Product: '<S118>/Multiply' incorporates:
   *  Constant: '<S118>/kd'
   */
  rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D * rtb_Saturation_d_idx_2;

  /* Saturate: '<S118>/Saturation' */
  if (rtb_Subtract3_i > CONTROL_PARAM.VEL_Z_D_MAX) {
    rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D_MAX;
  } else {
    if (rtb_Subtract3_i < CONTROL_PARAM.VEL_Z_D_MIN) {
      rtb_Subtract3_i = CONTROL_PARAM.VEL_Z_D_MIN;
    }
  }

  /* End of Saturate: '<S118>/Saturation' */

  /* Sum: '<S114>/Add' incorporates:
   *  Constant: '<S120>/kp'
   *  DiscreteIntegrator: '<S119>/Discrete-Time Integrator'
   *  Product: '<S120>/Multiply'
   */
  rtb_Saturation_d_idx_2 = (CONTROL_PARAM.VEL_Z_P * rtb_Saturation_d_idx_1 +
    Controller_DW.DiscreteTimeIntegrator_DSTATE_m) + rtb_Subtract3_i;

  /* Product: '<S112>/Multiply' incorporates:
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S112>/Cos'
   *  Trigonometry: '<S112>/Cos1'
   */
  rtb_Subtract3_i = arm_cos_f32(Controller_U.INS_Out.phi) * arm_cos_f32
    (Controller_U.INS_Out.theta);

  /* Saturate: '<S112>/Saturation1' */
  if (rtb_Subtract3_i > 1.0F) {
    rtb_Subtract1_k = 1.0F;
  } else if (rtb_Subtract3_i < 0.5F) {
    rtb_Subtract1_k = 0.5F;
  } else {
    rtb_Subtract1_k = rtb_Subtract3_i;
  }

  /* End of Saturate: '<S112>/Saturation1' */

  /* Gain: '<S112>/Gain' */
  rtb_Subtract3_i *= 2.0F;

  /* Saturate: '<S112>/Saturation' */
  if (rtb_Subtract3_i > 1.0F) {
    rtb_Subtract3_i = 1.0F;
  } else {
    if (rtb_Subtract3_i < 0.0F) {
      rtb_Subtract3_i = 0.0F;
    }
  }

  /* End of Saturate: '<S112>/Saturation' */

  /* Product: '<S112>/Multiply1' incorporates:
   *  Constant: '<S112>/Constant'
   *  Product: '<S112>/Divide'
   */
  rtb_Subtract3_i *= 1.0F / rtb_Subtract1_k * rtb_Saturation_d_idx_2;

  /* Saturate: '<S111>/Saturation' */
  if (rtb_Subtract3_i > 0.35F) {
    rtb_Saturation_i = 0.35F;
  } else if (rtb_Subtract3_i < -0.35F) {
    rtb_Saturation_i = -0.35F;
  } else {
    rtb_Saturation_i = rtb_Subtract3_i;
  }

  /* End of Saturate: '<S111>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S2>/Control_Allocation' */
#if AIRFRAME == 6

  /* Output and update for atomic system: '<S4>/Coxial_Hexacopter_x' */
  {
    uint16_T rtb_throttle_cmd;
    uint16_T rtb_MultiportSwitch[12];
    int32_T i_b;
    real32_T tmp_b;
    real32_T tmp_k;
    real32_T tmp_i;
    real32_T u0_b;
    uint16_T u0_k;

    /* MultiPortSwitch: '<S16>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_b = 0; i_b < 12; i_b++) {
        rtb_MultiportSwitch[i_b] = 1000U;
      }
      break;

     case 2:
      for (i_b = 0; i_b < 12; i_b++) {
        rtb_MultiportSwitch[i_b] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S15>/Switch' incorporates:
       *  Constant: '<S19>/Constant'
       *  Logic: '<S15>/Logical Operator'
       *  RelationalOperator: '<S18>/Compare'
       *  RelationalOperator: '<S19>/Compare'
       *  Saturate: '<S7>/Saturation2'
       *  Switch: '<S15>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S20>/Switch' incorporates:
         *  Constant: '<S17>/Constant1'
         *  DataTypeConversion: '<S17>/Data Type Conversion'
         *  Gain: '<S17>/Gain1'
         *  S-Function (sfix_bitop): '<S20>/cmd_throttle valid'
         *  Saturate: '<S7>/Saturation2'
         *  Sum: '<S17>/Sum1'
         *  Switch: '<S15>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S7>/Saturation2' */
            u0_b = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S7>/Saturation2' */
            u0_b = -1.0F;
          } else {
            /* Saturate: '<S7>/Saturation2' */
            u0_b = rtb_Saturation_i;
          }

          /* Sum: '<S17>/Sum' incorporates:
           *  Constant: '<S17>/hover_throttle'
           *  Saturate: '<S7>/Saturation2'
           */
          u0_b += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S17>/Saturation' */
          if (u0_b > 1.0F) {
            u0_b = 1.0F;
          } else {
            if (u0_b < 0.0F) {
              u0_b = 0.0F;
            }
          }

          rtb_throttle_cmd = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_b),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S20>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S7>/Saturation2' */
          u0_b = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S7>/Saturation2' */
          u0_b = -1.0F;
        } else {
          /* Saturate: '<S7>/Saturation2' */
          u0_b = rtb_Saturation_i;
        }

        /* Sum: '<S17>/Sum' incorporates:
         *  Constant: '<S17>/hover_throttle'
         *  Saturate: '<S7>/Saturation2'
         */
        u0_b += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S17>/Saturation' */
        if (u0_b > 1.0F) {
          u0_b = 1.0F;
        } else {
          if (u0_b < 0.0F) {
            u0_b = 0.0F;
          }
        }

        /* Switch: '<S15>/Switch1' incorporates:
         *  Constant: '<S17>/Constant1'
         *  DataTypeConversion: '<S17>/Data Type Conversion'
         *  Gain: '<S17>/Gain1'
         *  Sum: '<S17>/Sum1'
         */
        rtb_throttle_cmd = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_b),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S15>/Switch' */

      /* Saturate: '<S7>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_b = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_b = -1.0F;
      } else {
        tmp_b = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_k = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_k = -1.0F;
      } else {
        tmp_k = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_i = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_i = -1.0F;
      } else {
        tmp_i = rtb_Sum[2];
      }

      /* End of Saturate: '<S7>/Saturation1' */

      /* Product: '<S7>/Multiply' */
      for (i_b = 0; i_b < 12; i_b++) {
        /* Saturate: '<S7>/Saturation' incorporates:
         *  Constant: '<S7>/Effective_Matrix'
         *  Gain: '<S7>/Gain'
         *  Sum: '<S7>/Add'
         */
        u0_b = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value[i_b + 24] *
                             tmp_i +
                             (Controller_ConstP.Effective_Matrix_Value[i_b + 12]
                              * tmp_k +
                              Controller_ConstP.Effective_Matrix_Value[i_b] *
                              tmp_b)) + (real32_T)rtb_throttle_cmd), 65536.0F);
        u0_k = (uint16_T)(u0_b < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_b : (int32_T)(uint16_T)u0_b);
        if (u0_k > 1900) {
          rtb_MultiportSwitch[i_b] = 1900U;
        } else if (u0_k < 1100) {
          rtb_MultiportSwitch[i_b] = 1100U;
        } else {
          rtb_MultiportSwitch[i_b] = u0_k;
        }

        /* End of Saturate: '<S7>/Saturation' */
      }

      /* End of Product: '<S7>/Multiply' */
      break;

     default:
      for (i_b = 0; i_b < 12; i_b++) {
        rtb_MultiportSwitch[i_b] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S16>/Multiport Switch' */

    /* Reshape: '<S16>/Reshape' */
    for (i_b = 0; i_b < 12; i_b++) {
      rtb_VariantMergeForOutportactua[i_b] = rtb_MultiportSwitch[i_b];
    }

    rtb_VariantMergeForOutportactua[12] = 0U;
    rtb_VariantMergeForOutportactua[13] = 0U;
    rtb_VariantMergeForOutportactua[14] = 0U;
    rtb_VariantMergeForOutportactua[15] = 0U;

    /* End of Reshape: '<S16>/Reshape' */
  }

#elif AIRFRAME == 3

  /* Output and update for atomic system: '<S4>/Coxial_Quadcopter_x' */
  {
    uint16_T rtb_throttle_cmd_b;
    uint16_T rtb_MultiportSwitch_i[8];
    int32_T i_n;
    real32_T tmp_n;
    real32_T tmp_d;
    real32_T tmp_a;
    real32_T u0_n;
    uint16_T u0_d;

    /* MultiPortSwitch: '<S22>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_n = 0; i_n < 8; i_n++) {
        rtb_MultiportSwitch_i[i_n] = 1000U;
      }
      break;

     case 2:
      for (i_n = 0; i_n < 8; i_n++) {
        rtb_MultiportSwitch_i[i_n] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S21>/Switch' incorporates:
       *  Constant: '<S25>/Constant'
       *  Logic: '<S21>/Logical Operator'
       *  RelationalOperator: '<S24>/Compare'
       *  RelationalOperator: '<S25>/Compare'
       *  Saturate: '<S8>/Saturation2'
       *  Switch: '<S21>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_b = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S26>/Switch' incorporates:
         *  Constant: '<S23>/Constant1'
         *  DataTypeConversion: '<S23>/Data Type Conversion'
         *  Gain: '<S23>/Gain1'
         *  S-Function (sfix_bitop): '<S26>/cmd_throttle valid'
         *  Saturate: '<S8>/Saturation2'
         *  Sum: '<S23>/Sum1'
         *  Switch: '<S21>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_b = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S8>/Saturation2' */
            u0_n = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S8>/Saturation2' */
            u0_n = -1.0F;
          } else {
            /* Saturate: '<S8>/Saturation2' */
            u0_n = rtb_Saturation_i;
          }

          /* Sum: '<S23>/Sum' incorporates:
           *  Constant: '<S23>/hover_throttle'
           *  Saturate: '<S8>/Saturation2'
           */
          u0_n += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S23>/Saturation' */
          if (u0_n > 1.0F) {
            u0_n = 1.0F;
          } else {
            if (u0_n < 0.0F) {
              u0_n = 0.0F;
            }
          }

          rtb_throttle_cmd_b = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_n),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S26>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S8>/Saturation2' */
          u0_n = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S8>/Saturation2' */
          u0_n = -1.0F;
        } else {
          /* Saturate: '<S8>/Saturation2' */
          u0_n = rtb_Saturation_i;
        }

        /* Sum: '<S23>/Sum' incorporates:
         *  Constant: '<S23>/hover_throttle'
         *  Saturate: '<S8>/Saturation2'
         */
        u0_n += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S23>/Saturation' */
        if (u0_n > 1.0F) {
          u0_n = 1.0F;
        } else {
          if (u0_n < 0.0F) {
            u0_n = 0.0F;
          }
        }

        /* Switch: '<S21>/Switch1' incorporates:
         *  Constant: '<S23>/Constant1'
         *  DataTypeConversion: '<S23>/Data Type Conversion'
         *  Gain: '<S23>/Gain1'
         *  Sum: '<S23>/Sum1'
         */
        rtb_throttle_cmd_b = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_n),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S21>/Switch' */

      /* Saturate: '<S8>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_n = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_n = -1.0F;
      } else {
        tmp_n = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_d = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_d = -1.0F;
      } else {
        tmp_d = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_a = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_a = -1.0F;
      } else {
        tmp_a = rtb_Sum[2];
      }

      /* End of Saturate: '<S8>/Saturation1' */

      /* Product: '<S8>/Multiply' */
      for (i_n = 0; i_n < 8; i_n++) {
        /* Saturate: '<S8>/Saturation' incorporates:
         *  Constant: '<S8>/Effective_Matrix'
         *  Gain: '<S8>/Gain'
         *  Sum: '<S8>/Add'
         */
        u0_n = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_j[i_n + 16]
                             * tmp_a +
                             (Controller_ConstP.Effective_Matrix_Value_j[i_n + 8]
                              * tmp_d +
                              Controller_ConstP.Effective_Matrix_Value_j[i_n] *
                              tmp_n)) + (real32_T)rtb_throttle_cmd_b), 65536.0F);
        u0_d = (uint16_T)(u0_n < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_n : (int32_T)(uint16_T)u0_n);
        if (u0_d > 1900) {
          rtb_MultiportSwitch_i[i_n] = 1900U;
        } else if (u0_d < 1100) {
          rtb_MultiportSwitch_i[i_n] = 1100U;
        } else {
          rtb_MultiportSwitch_i[i_n] = u0_d;
        }

        /* End of Saturate: '<S8>/Saturation' */
      }

      /* End of Product: '<S8>/Multiply' */
      break;

     default:
      for (i_n = 0; i_n < 8; i_n++) {
        rtb_MultiportSwitch_i[i_n] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S22>/Multiport Switch' */

    /* Reshape: '<S22>/Reshape' */
    for (i_n = 0; i_n < 8; i_n++) {
      rtb_VariantMergeForOutportactua[i_n] = rtb_MultiportSwitch_i[i_n];
      rtb_VariantMergeForOutportactua[i_n + 8] = 0U;
    }

    /* End of Reshape: '<S22>/Reshape' */
  }

#elif AIRFRAME == 5

  /* Output and update for atomic system: '<S4>/Hexacopter_+' */
  {
    uint16_T rtb_throttle_cmd_k;
    uint16_T rtb_MultiportSwitch_h[6];
    int32_T i_i;
    real32_T tmp_i;
    real32_T tmp_o;
    real32_T tmp_d;
    real32_T u0_i;
    uint16_T u0_o;

    /* MultiPortSwitch: '<S28>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_i = 0; i_i < 6; i_i++) {
        rtb_MultiportSwitch_h[i_i] = 1000U;
      }
      break;

     case 2:
      for (i_i = 0; i_i < 6; i_i++) {
        rtb_MultiportSwitch_h[i_i] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S27>/Switch' incorporates:
       *  Constant: '<S31>/Constant'
       *  Logic: '<S27>/Logical Operator'
       *  RelationalOperator: '<S30>/Compare'
       *  RelationalOperator: '<S31>/Compare'
       *  Saturate: '<S9>/Saturation2'
       *  Switch: '<S27>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_k = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S32>/Switch' incorporates:
         *  Constant: '<S29>/Constant1'
         *  DataTypeConversion: '<S29>/Data Type Conversion'
         *  Gain: '<S29>/Gain1'
         *  S-Function (sfix_bitop): '<S32>/cmd_throttle valid'
         *  Saturate: '<S9>/Saturation2'
         *  Sum: '<S29>/Sum1'
         *  Switch: '<S27>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_k = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S9>/Saturation2' */
            u0_i = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S9>/Saturation2' */
            u0_i = -1.0F;
          } else {
            /* Saturate: '<S9>/Saturation2' */
            u0_i = rtb_Saturation_i;
          }

          /* Sum: '<S29>/Sum' incorporates:
           *  Constant: '<S29>/hover_throttle'
           *  Saturate: '<S9>/Saturation2'
           */
          u0_i += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S29>/Saturation' */
          if (u0_i > 1.0F) {
            u0_i = 1.0F;
          } else {
            if (u0_i < 0.0F) {
              u0_i = 0.0F;
            }
          }

          rtb_throttle_cmd_k = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_i),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S32>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S9>/Saturation2' */
          u0_i = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S9>/Saturation2' */
          u0_i = -1.0F;
        } else {
          /* Saturate: '<S9>/Saturation2' */
          u0_i = rtb_Saturation_i;
        }

        /* Sum: '<S29>/Sum' incorporates:
         *  Constant: '<S29>/hover_throttle'
         *  Saturate: '<S9>/Saturation2'
         */
        u0_i += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S29>/Saturation' */
        if (u0_i > 1.0F) {
          u0_i = 1.0F;
        } else {
          if (u0_i < 0.0F) {
            u0_i = 0.0F;
          }
        }

        /* Switch: '<S27>/Switch1' incorporates:
         *  Constant: '<S29>/Constant1'
         *  DataTypeConversion: '<S29>/Data Type Conversion'
         *  Gain: '<S29>/Gain1'
         *  Sum: '<S29>/Sum1'
         */
        rtb_throttle_cmd_k = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_i),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S27>/Switch' */

      /* Saturate: '<S9>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_i = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_i = -1.0F;
      } else {
        tmp_i = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_o = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_o = -1.0F;
      } else {
        tmp_o = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_d = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_d = -1.0F;
      } else {
        tmp_d = rtb_Sum[2];
      }

      /* End of Saturate: '<S9>/Saturation1' */

      /* Product: '<S9>/Multiply' */
      for (i_i = 0; i_i < 6; i_i++) {
        /* Saturate: '<S9>/Saturation' incorporates:
         *  Constant: '<S9>/Effective_Matrix'
         *  Gain: '<S9>/Gain'
         *  Sum: '<S9>/Add'
         */
        u0_i = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_je[i_i +
                             12] * tmp_d +
                             (Controller_ConstP.Effective_Matrix_Value_je[i_i +
                              6] * tmp_o +
                              Controller_ConstP.Effective_Matrix_Value_je[i_i] *
                              tmp_i)) + (real32_T)rtb_throttle_cmd_k), 65536.0F);
        u0_o = (uint16_T)(u0_i < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_i : (int32_T)(uint16_T)u0_i);
        if (u0_o > 1900) {
          rtb_MultiportSwitch_h[i_i] = 1900U;
        } else if (u0_o < 1100) {
          rtb_MultiportSwitch_h[i_i] = 1100U;
        } else {
          rtb_MultiportSwitch_h[i_i] = u0_o;
        }

        /* End of Saturate: '<S9>/Saturation' */
      }

      /* End of Product: '<S9>/Multiply' */
      break;

     default:
      for (i_i = 0; i_i < 6; i_i++) {
        rtb_MultiportSwitch_h[i_i] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S28>/Multiport Switch' */

    /* Reshape: '<S28>/Reshape' */
    for (i_i = 0; i_i < 6; i_i++) {
      rtb_VariantMergeForOutportactua[i_i] = rtb_MultiportSwitch_h[i_i];
    }

    for (i_i = 0; i_i < 10; i_i++) {
      rtb_VariantMergeForOutportactua[i_i + 6] = 0U;
    }

    /* End of Reshape: '<S28>/Reshape' */
  }

#elif AIRFRAME == 4

  /* Output and update for atomic system: '<S4>/Hexacopter_x' */
  {
    uint16_T rtb_throttle_cmd_d;
    uint16_T rtb_MultiportSwitch_k[6];
    int32_T i_j;
    real32_T tmp_j;
    real32_T tmp_e;
    real32_T tmp_g;
    real32_T u0_j;
    uint16_T u0_e;

    /* MultiPortSwitch: '<S34>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_j = 0; i_j < 6; i_j++) {
        rtb_MultiportSwitch_k[i_j] = 1000U;
      }
      break;

     case 2:
      for (i_j = 0; i_j < 6; i_j++) {
        rtb_MultiportSwitch_k[i_j] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S33>/Switch' incorporates:
       *  Constant: '<S37>/Constant'
       *  Logic: '<S33>/Logical Operator'
       *  RelationalOperator: '<S36>/Compare'
       *  RelationalOperator: '<S37>/Compare'
       *  Saturate: '<S10>/Saturation2'
       *  Switch: '<S33>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_d = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S38>/Switch' incorporates:
         *  Constant: '<S35>/Constant1'
         *  DataTypeConversion: '<S35>/Data Type Conversion'
         *  Gain: '<S35>/Gain1'
         *  S-Function (sfix_bitop): '<S38>/cmd_throttle valid'
         *  Saturate: '<S10>/Saturation2'
         *  Sum: '<S35>/Sum1'
         *  Switch: '<S33>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_d = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S10>/Saturation2' */
            u0_j = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S10>/Saturation2' */
            u0_j = -1.0F;
          } else {
            /* Saturate: '<S10>/Saturation2' */
            u0_j = rtb_Saturation_i;
          }

          /* Sum: '<S35>/Sum' incorporates:
           *  Constant: '<S35>/hover_throttle'
           *  Saturate: '<S10>/Saturation2'
           */
          u0_j += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S35>/Saturation' */
          if (u0_j > 1.0F) {
            u0_j = 1.0F;
          } else {
            if (u0_j < 0.0F) {
              u0_j = 0.0F;
            }
          }

          rtb_throttle_cmd_d = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_j),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S38>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S10>/Saturation2' */
          u0_j = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S10>/Saturation2' */
          u0_j = -1.0F;
        } else {
          /* Saturate: '<S10>/Saturation2' */
          u0_j = rtb_Saturation_i;
        }

        /* Sum: '<S35>/Sum' incorporates:
         *  Constant: '<S35>/hover_throttle'
         *  Saturate: '<S10>/Saturation2'
         */
        u0_j += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S35>/Saturation' */
        if (u0_j > 1.0F) {
          u0_j = 1.0F;
        } else {
          if (u0_j < 0.0F) {
            u0_j = 0.0F;
          }
        }

        /* Switch: '<S33>/Switch1' incorporates:
         *  Constant: '<S35>/Constant1'
         *  DataTypeConversion: '<S35>/Data Type Conversion'
         *  Gain: '<S35>/Gain1'
         *  Sum: '<S35>/Sum1'
         */
        rtb_throttle_cmd_d = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_j),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S33>/Switch' */

      /* Saturate: '<S10>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_j = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_j = -1.0F;
      } else {
        tmp_j = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_e = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_e = -1.0F;
      } else {
        tmp_e = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_g = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_g = -1.0F;
      } else {
        tmp_g = rtb_Sum[2];
      }

      /* End of Saturate: '<S10>/Saturation1' */

      /* Product: '<S10>/Multiply' */
      for (i_j = 0; i_j < 6; i_j++) {
        /* Saturate: '<S10>/Saturation' incorporates:
         *  Constant: '<S10>/Effective_Matrix'
         *  Gain: '<S10>/Gain'
         *  Sum: '<S10>/Add'
         */
        u0_j = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_f[i_j + 12]
                             * tmp_g +
                             (Controller_ConstP.Effective_Matrix_Value_f[i_j + 6]
                              * tmp_e +
                              Controller_ConstP.Effective_Matrix_Value_f[i_j] *
                              tmp_j)) + (real32_T)rtb_throttle_cmd_d), 65536.0F);
        u0_e = (uint16_T)(u0_j < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_j : (int32_T)(uint16_T)u0_j);
        if (u0_e > 1900) {
          rtb_MultiportSwitch_k[i_j] = 1900U;
        } else if (u0_e < 1100) {
          rtb_MultiportSwitch_k[i_j] = 1100U;
        } else {
          rtb_MultiportSwitch_k[i_j] = u0_e;
        }

        /* End of Saturate: '<S10>/Saturation' */
      }

      /* End of Product: '<S10>/Multiply' */
      break;

     default:
      for (i_j = 0; i_j < 6; i_j++) {
        rtb_MultiportSwitch_k[i_j] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S34>/Multiport Switch' */

    /* Reshape: '<S34>/Reshape' */
    for (i_j = 0; i_j < 6; i_j++) {
      rtb_VariantMergeForOutportactua[i_j] = rtb_MultiportSwitch_k[i_j];
    }

    for (i_j = 0; i_j < 10; i_j++) {
      rtb_VariantMergeForOutportactua[i_j + 6] = 0U;
    }

    /* End of Reshape: '<S34>/Reshape' */
  }

#elif AIRFRAME == 8

  /* Output and update for atomic system: '<S4>/Octocopter_+' */
  {
    uint16_T rtb_throttle_cmd_m;
    uint16_T rtb_MultiportSwitch_d[8];
    int32_T i_i;
    real32_T tmp_i;
    real32_T tmp_d;
    real32_T tmp_j;
    real32_T u0_i;
    uint16_T u0_d;

    /* MultiPortSwitch: '<S40>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_i = 0; i_i < 8; i_i++) {
        rtb_MultiportSwitch_d[i_i] = 1000U;
      }
      break;

     case 2:
      for (i_i = 0; i_i < 8; i_i++) {
        rtb_MultiportSwitch_d[i_i] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S39>/Switch' incorporates:
       *  Constant: '<S43>/Constant'
       *  Logic: '<S39>/Logical Operator'
       *  RelationalOperator: '<S42>/Compare'
       *  RelationalOperator: '<S43>/Compare'
       *  Saturate: '<S11>/Saturation2'
       *  Switch: '<S39>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_m = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S44>/Switch' incorporates:
         *  Constant: '<S41>/Constant1'
         *  DataTypeConversion: '<S41>/Data Type Conversion'
         *  Gain: '<S41>/Gain1'
         *  S-Function (sfix_bitop): '<S44>/cmd_throttle valid'
         *  Saturate: '<S11>/Saturation2'
         *  Sum: '<S41>/Sum1'
         *  Switch: '<S39>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_m = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S11>/Saturation2' */
            u0_i = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S11>/Saturation2' */
            u0_i = -1.0F;
          } else {
            /* Saturate: '<S11>/Saturation2' */
            u0_i = rtb_Saturation_i;
          }

          /* Sum: '<S41>/Sum' incorporates:
           *  Constant: '<S41>/hover_throttle'
           *  Saturate: '<S11>/Saturation2'
           */
          u0_i += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S41>/Saturation' */
          if (u0_i > 1.0F) {
            u0_i = 1.0F;
          } else {
            if (u0_i < 0.0F) {
              u0_i = 0.0F;
            }
          }

          rtb_throttle_cmd_m = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_i),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S44>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S11>/Saturation2' */
          u0_i = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S11>/Saturation2' */
          u0_i = -1.0F;
        } else {
          /* Saturate: '<S11>/Saturation2' */
          u0_i = rtb_Saturation_i;
        }

        /* Sum: '<S41>/Sum' incorporates:
         *  Constant: '<S41>/hover_throttle'
         *  Saturate: '<S11>/Saturation2'
         */
        u0_i += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S41>/Saturation' */
        if (u0_i > 1.0F) {
          u0_i = 1.0F;
        } else {
          if (u0_i < 0.0F) {
            u0_i = 0.0F;
          }
        }

        /* Switch: '<S39>/Switch1' incorporates:
         *  Constant: '<S41>/Constant1'
         *  DataTypeConversion: '<S41>/Data Type Conversion'
         *  Gain: '<S41>/Gain1'
         *  Sum: '<S41>/Sum1'
         */
        rtb_throttle_cmd_m = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_i),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S39>/Switch' */

      /* Saturate: '<S11>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_i = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_i = -1.0F;
      } else {
        tmp_i = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_d = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_d = -1.0F;
      } else {
        tmp_d = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_j = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_j = -1.0F;
      } else {
        tmp_j = rtb_Sum[2];
      }

      /* End of Saturate: '<S11>/Saturation1' */

      /* Product: '<S11>/Multiply' */
      for (i_i = 0; i_i < 8; i_i++) {
        /* Saturate: '<S11>/Saturation' incorporates:
         *  Constant: '<S11>/Effective_Matrix'
         *  Gain: '<S11>/Gain'
         *  Sum: '<S11>/Add'
         */
        u0_i = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_m[i_i + 16]
                             * tmp_j +
                             (Controller_ConstP.Effective_Matrix_Value_m[i_i + 8]
                              * tmp_d +
                              Controller_ConstP.Effective_Matrix_Value_m[i_i] *
                              tmp_i)) + (real32_T)rtb_throttle_cmd_m), 65536.0F);
        u0_d = (uint16_T)(u0_i < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_i : (int32_T)(uint16_T)u0_i);
        if (u0_d > 1900) {
          rtb_MultiportSwitch_d[i_i] = 1900U;
        } else if (u0_d < 1100) {
          rtb_MultiportSwitch_d[i_i] = 1100U;
        } else {
          rtb_MultiportSwitch_d[i_i] = u0_d;
        }

        /* End of Saturate: '<S11>/Saturation' */
      }

      /* End of Product: '<S11>/Multiply' */
      break;

     default:
      for (i_i = 0; i_i < 8; i_i++) {
        rtb_MultiportSwitch_d[i_i] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S40>/Multiport Switch' */

    /* Reshape: '<S40>/Reshape' */
    for (i_i = 0; i_i < 8; i_i++) {
      rtb_VariantMergeForOutportactua[i_i] = rtb_MultiportSwitch_d[i_i];
      rtb_VariantMergeForOutportactua[i_i + 8] = 0U;
    }

    /* End of Reshape: '<S40>/Reshape' */
  }

#elif AIRFRAME == 7

  /* Output and update for atomic system: '<S4>/Octocopter_x' */
  {
    uint16_T rtb_throttle_cmd_n;
    uint16_T rtb_MultiportSwitch_hv[8];
    int32_T i_l;
    real32_T tmp_l;
    real32_T tmp_m;
    real32_T tmp_c;
    real32_T u0_l;
    uint16_T u0_m;

    /* MultiPortSwitch: '<S46>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      for (i_l = 0; i_l < 8; i_l++) {
        rtb_MultiportSwitch_hv[i_l] = 1000U;
      }
      break;

     case 2:
      for (i_l = 0; i_l < 8; i_l++) {
        rtb_MultiportSwitch_hv[i_l] = 1150U;
      }
      break;

     case 3:
      /* Switch: '<S45>/Switch' incorporates:
       *  Constant: '<S49>/Constant'
       *  Logic: '<S45>/Logical Operator'
       *  RelationalOperator: '<S48>/Compare'
       *  RelationalOperator: '<S49>/Compare'
       *  Saturate: '<S12>/Saturation2'
       *  Switch: '<S45>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_n = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S50>/Switch' incorporates:
         *  Constant: '<S47>/Constant1'
         *  DataTypeConversion: '<S47>/Data Type Conversion'
         *  Gain: '<S47>/Gain1'
         *  S-Function (sfix_bitop): '<S50>/cmd_throttle valid'
         *  Saturate: '<S12>/Saturation2'
         *  Sum: '<S47>/Sum1'
         *  Switch: '<S45>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_n = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S12>/Saturation2' */
            u0_l = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S12>/Saturation2' */
            u0_l = -1.0F;
          } else {
            /* Saturate: '<S12>/Saturation2' */
            u0_l = rtb_Saturation_i;
          }

          /* Sum: '<S47>/Sum' incorporates:
           *  Constant: '<S47>/hover_throttle'
           *  Saturate: '<S12>/Saturation2'
           */
          u0_l += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S47>/Saturation' */
          if (u0_l > 1.0F) {
            u0_l = 1.0F;
          } else {
            if (u0_l < 0.0F) {
              u0_l = 0.0F;
            }
          }

          rtb_throttle_cmd_n = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_l),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S50>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S12>/Saturation2' */
          u0_l = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S12>/Saturation2' */
          u0_l = -1.0F;
        } else {
          /* Saturate: '<S12>/Saturation2' */
          u0_l = rtb_Saturation_i;
        }

        /* Sum: '<S47>/Sum' incorporates:
         *  Constant: '<S47>/hover_throttle'
         *  Saturate: '<S12>/Saturation2'
         */
        u0_l += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S47>/Saturation' */
        if (u0_l > 1.0F) {
          u0_l = 1.0F;
        } else {
          if (u0_l < 0.0F) {
            u0_l = 0.0F;
          }
        }

        /* Switch: '<S45>/Switch1' incorporates:
         *  Constant: '<S47>/Constant1'
         *  DataTypeConversion: '<S47>/Data Type Conversion'
         *  Gain: '<S47>/Gain1'
         *  Sum: '<S47>/Sum1'
         */
        rtb_throttle_cmd_n = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_l),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S45>/Switch' */

      /* Saturate: '<S12>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_l = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_l = -1.0F;
      } else {
        tmp_l = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_m = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_m = -1.0F;
      } else {
        tmp_m = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_c = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_c = -1.0F;
      } else {
        tmp_c = rtb_Sum[2];
      }

      /* End of Saturate: '<S12>/Saturation1' */

      /* Product: '<S12>/Multiply' */
      for (i_l = 0; i_l < 8; i_l++) {
        /* Saturate: '<S12>/Saturation' incorporates:
         *  Constant: '<S12>/Effective_Matrix'
         *  Gain: '<S12>/Gain'
         *  Sum: '<S12>/Add'
         */
        u0_l = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_k[i_l + 16]
                             * tmp_c +
                             (Controller_ConstP.Effective_Matrix_Value_k[i_l + 8]
                              * tmp_m +
                              Controller_ConstP.Effective_Matrix_Value_k[i_l] *
                              tmp_l)) + (real32_T)rtb_throttle_cmd_n), 65536.0F);
        u0_m = (uint16_T)(u0_l < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_l : (int32_T)(uint16_T)u0_l);
        if (u0_m > 1900) {
          rtb_MultiportSwitch_hv[i_l] = 1900U;
        } else if (u0_m < 1100) {
          rtb_MultiportSwitch_hv[i_l] = 1100U;
        } else {
          rtb_MultiportSwitch_hv[i_l] = u0_m;
        }

        /* End of Saturate: '<S12>/Saturation' */
      }

      /* End of Product: '<S12>/Multiply' */
      break;

     default:
      for (i_l = 0; i_l < 8; i_l++) {
        rtb_MultiportSwitch_hv[i_l] = 1000U;
      }
      break;
    }

    /* End of MultiPortSwitch: '<S46>/Multiport Switch' */

    /* Reshape: '<S46>/Reshape' */
    for (i_l = 0; i_l < 8; i_l++) {
      rtb_VariantMergeForOutportactua[i_l] = rtb_MultiportSwitch_hv[i_l];
      rtb_VariantMergeForOutportactua[i_l + 8] = 0U;
    }

    /* End of Reshape: '<S46>/Reshape' */
  }

#elif AIRFRAME == 2

  /* Output and update for atomic system: '<S4>/Quadcopter_+' */
  {
    uint16_T rtb_throttle_cmd_o;
    uint16_T rtb_MultiportSwitch_o[4];
    int32_T i_m;
    real32_T tmp_m;
    real32_T tmp_n;
    real32_T tmp_al;
    real32_T u0_m;
    uint16_T u0_n;

    /* MultiPortSwitch: '<S52>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      rtb_MultiportSwitch_o[0] = 1000U;
      rtb_MultiportSwitch_o[1] = 1000U;
      rtb_MultiportSwitch_o[2] = 1000U;
      rtb_MultiportSwitch_o[3] = 1000U;
      break;

     case 2:
      rtb_MultiportSwitch_o[0] = 1150U;
      rtb_MultiportSwitch_o[1] = 1150U;
      rtb_MultiportSwitch_o[2] = 1150U;
      rtb_MultiportSwitch_o[3] = 1150U;
      break;

     case 3:
      /* Switch: '<S51>/Switch' incorporates:
       *  Constant: '<S55>/Constant'
       *  Logic: '<S51>/Logical Operator'
       *  RelationalOperator: '<S54>/Compare'
       *  RelationalOperator: '<S55>/Compare'
       *  Saturate: '<S13>/Saturation2'
       *  Switch: '<S51>/Switch1'
       */
      if ((Controller_U.FMS_Out.ctrl_mode == 1) ||
          (Controller_U.FMS_Out.ctrl_mode == 2) ||
          (Controller_U.FMS_Out.ctrl_mode == 3)) {
        rtb_throttle_cmd_o = Controller_U.FMS_Out.throttle_cmd;
      } else if (Controller_U.FMS_Out.ctrl_mode == 6) {
        /* Switch: '<S56>/Switch' incorporates:
         *  Constant: '<S53>/Constant1'
         *  DataTypeConversion: '<S53>/Data Type Conversion'
         *  Gain: '<S53>/Gain1'
         *  S-Function (sfix_bitop): '<S56>/cmd_throttle valid'
         *  Saturate: '<S13>/Saturation2'
         *  Sum: '<S53>/Sum1'
         *  Switch: '<S51>/Switch1'
         */
        if ((Controller_U.FMS_Out.cmd_mask & 4096) > 0) {
          rtb_throttle_cmd_o = Controller_U.FMS_Out.throttle_cmd;
        } else {
          if (rtb_Saturation_i > 1.0F) {
            /* Saturate: '<S13>/Saturation2' */
            u0_m = 1.0F;
          } else if (rtb_Saturation_i < -1.0F) {
            /* Saturate: '<S13>/Saturation2' */
            u0_m = -1.0F;
          } else {
            /* Saturate: '<S13>/Saturation2' */
            u0_m = rtb_Saturation_i;
          }

          /* Sum: '<S53>/Sum' incorporates:
           *  Constant: '<S53>/hover_throttle'
           *  Saturate: '<S13>/Saturation2'
           */
          u0_m += CONTROL_PARAM.HOVER_THRO;

          /* Saturate: '<S53>/Saturation' */
          if (u0_m > 1.0F) {
            u0_m = 1.0F;
          } else {
            if (u0_m < 0.0F) {
              u0_m = 0.0F;
            }
          }

          rtb_throttle_cmd_o = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_m),
            4.2949673E+9F) + 1000U);
        }

        /* End of Switch: '<S56>/Switch' */
      } else {
        if (rtb_Saturation_i > 1.0F) {
          /* Saturate: '<S13>/Saturation2' */
          u0_m = 1.0F;
        } else if (rtb_Saturation_i < -1.0F) {
          /* Saturate: '<S13>/Saturation2' */
          u0_m = -1.0F;
        } else {
          /* Saturate: '<S13>/Saturation2' */
          u0_m = rtb_Saturation_i;
        }

        /* Sum: '<S53>/Sum' incorporates:
         *  Constant: '<S53>/hover_throttle'
         *  Saturate: '<S13>/Saturation2'
         */
        u0_m += CONTROL_PARAM.HOVER_THRO;

        /* Saturate: '<S53>/Saturation' */
        if (u0_m > 1.0F) {
          u0_m = 1.0F;
        } else {
          if (u0_m < 0.0F) {
            u0_m = 0.0F;
          }
        }

        /* Switch: '<S51>/Switch1' incorporates:
         *  Constant: '<S53>/Constant1'
         *  DataTypeConversion: '<S53>/Data Type Conversion'
         *  Gain: '<S53>/Gain1'
         *  Sum: '<S53>/Sum1'
         */
        rtb_throttle_cmd_o = (uint16_T)((uint32_T)fmodf(floorf(1000.0F * u0_m),
          4.2949673E+9F) + 1000U);
      }

      /* End of Switch: '<S51>/Switch' */

      /* Saturate: '<S13>/Saturation1' */
      if (rtb_Sum[0] > 1.0F) {
        tmp_m = 1.0F;
      } else if (rtb_Sum[0] < -1.0F) {
        tmp_m = -1.0F;
      } else {
        tmp_m = rtb_Sum[0];
      }

      if (rtb_Sum[1] > 1.0F) {
        tmp_n = 1.0F;
      } else if (rtb_Sum[1] < -1.0F) {
        tmp_n = -1.0F;
      } else {
        tmp_n = rtb_Sum[1];
      }

      if (rtb_Sum[2] > 1.0F) {
        tmp_al = 1.0F;
      } else if (rtb_Sum[2] < -1.0F) {
        tmp_al = -1.0F;
      } else {
        tmp_al = rtb_Sum[2];
      }

      /* End of Saturate: '<S13>/Saturation1' */

      /* Product: '<S13>/Multiply' */
      for (i_m = 0; i_m < 4; i_m++) {
        /* Saturate: '<S13>/Saturation' incorporates:
         *  Constant: '<S13>/Effective_Matrix'
         *  Gain: '<S13>/Gain'
         *  Sum: '<S13>/Add'
         */
        u0_m = fmodf(floorf(1000.0F *
                            (Controller_ConstP.Effective_Matrix_Value_fr[i_m + 8]
                             * tmp_al +
                             (Controller_ConstP.Effective_Matrix_Value_fr[i_m +
                              4] * tmp_n +
                              Controller_ConstP.Effective_Matrix_Value_fr[i_m] *
                              tmp_m)) + (real32_T)rtb_throttle_cmd_o), 65536.0F);
        u0_n = (uint16_T)(u0_m < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                          -u0_m : (int32_T)(uint16_T)u0_m);
        if (u0_n > 1900) {
          rtb_MultiportSwitch_o[i_m] = 1900U;
        } else if (u0_n < 1100) {
          rtb_MultiportSwitch_o[i_m] = 1100U;
        } else {
          rtb_MultiportSwitch_o[i_m] = u0_n;
        }

        /* End of Saturate: '<S13>/Saturation' */
      }

      /* End of Product: '<S13>/Multiply' */
      break;

     default:
      rtb_MultiportSwitch_o[0] = 1000U;
      rtb_MultiportSwitch_o[1] = 1000U;
      rtb_MultiportSwitch_o[2] = 1000U;
      rtb_MultiportSwitch_o[3] = 1000U;
      break;
    }

    /* End of MultiPortSwitch: '<S52>/Multiport Switch' */

    /* Reshape: '<S52>/Reshape' */
    rtb_VariantMergeForOutportactua[0] = rtb_MultiportSwitch_o[0];
    rtb_VariantMergeForOutportactua[1] = rtb_MultiportSwitch_o[1];
    rtb_VariantMergeForOutportactua[2] = rtb_MultiportSwitch_o[2];
    rtb_VariantMergeForOutportactua[3] = rtb_MultiportSwitch_o[3];
    for (i_m = 0; i_m < 12; i_m++) {
      rtb_VariantMergeForOutportactua[i_m + 4] = 0U;
    }

    /* End of Reshape: '<S52>/Reshape' */
  }

#elif AIRFRAME == 1

  /* Output and update for atomic system: '<S4>/Quadcopter_x' */
  {
    real32_T r_e[12];
    real32_T B4_e[16];
    real32_T Braw_e[32];
    real32_T Mz_max_e;
    real32_T T_max_e;
    int32_T i_e;
    real32_T y_e[4];
    static const int8_T b_e[4] = { 1, 1, -1, -1 };

    real32_T rtb_Delay[8];
    real32_T rtb_TmpSignalConversionAtSFunct[4];
    real32_T rtb_B4x8[32];
    real32_T n_thrust_idx_1_e;
    real32_T n_thrust_idx_2_e;
    uint16_T u0_e;
    int32_T B4_tmp_e;
    int32_T rtb_B4x8_tmp_e;
    int32_T rtb_B4x8_tmp_o;
    int32_T rtb_B4x8_tmp_h;

    /* MATLAB Function: '<S14>/MATLAB Function1' incorporates:
     *  Constant: '<S14>/Constant'
     *  Constant: '<S14>/Constant1'
     *  Constant: '<S14>/Constant2'
     *  Constant: '<S14>/Constant3'
     *  Inport: '<Root>/INS_Out'
     */
    r_e[0] = 0.3226F;
    r_e[3] = -0.3226F;
    r_e[6] = 0.3226F;
    r_e[9] = -0.3226F;
    r_e[1] = 0.2904F;
    r_e[4] = -0.2904F;
    r_e[7] = -0.2904F;
    r_e[10] = 0.2904F;
    r_e[2] = 0.0769F;
    r_e[5] = 0.0769F;
    r_e[8] = 0.0769F;
    r_e[11] = 0.0769F;
    n_thrust_idx_1_e = arm_sin_f32(Controller_U.INS_Out.hinge_angle);
    n_thrust_idx_2_e = -arm_cos_f32(Controller_U.INS_Out.hinge_angle);
    for (i_e = 0; i_e < 4; i_e++) {
      B4_tmp_e = i_e << 2;
      B4_e[B4_tmp_e] = r_e[3 * i_e + 1] * n_thrust_idx_2_e - r_e[3 * i_e + 2] *
        n_thrust_idx_1_e;
      Mz_max_e = r_e[3 * i_e];
      B4_e[1 + B4_tmp_e] = 0.0F - Mz_max_e * n_thrust_idx_2_e;
      B4_e[2 + B4_tmp_e] = Mz_max_e * n_thrust_idx_1_e + (real32_T)b_e[i_e] *
        0.05F;
      B4_e[3 + B4_tmp_e] = -n_thrust_idx_2_e;
    }

    memset(&Braw_e[0], 0, sizeof(real32_T) << 5U);
    for (i_e = 0; i_e < 4; i_e++) {
      B4_tmp_e = i_e << 2;
      Braw_e[B4_tmp_e] = B4_e[B4_tmp_e];
      Braw_e[1 + B4_tmp_e] = B4_e[B4_tmp_e + 1];
      Braw_e[2 + B4_tmp_e] = B4_e[B4_tmp_e + 2];
      Braw_e[3 + B4_tmp_e] = B4_e[B4_tmp_e + 3];
      y_e[i_e] = fabsf(Braw_e[B4_tmp_e]);
    }

    n_thrust_idx_1_e = Controller_sum(y_e);
    y_e[0] = fabsf(Braw_e[1]);
    y_e[1] = fabsf(Braw_e[5]);
    y_e[2] = fabsf(Braw_e[9]);
    y_e[3] = fabsf(Braw_e[13]);
    n_thrust_idx_2_e = Controller_sum(y_e);
    y_e[0] = fabsf(Braw_e[2]);
    y_e[1] = fabsf(Braw_e[6]);
    y_e[2] = fabsf(Braw_e[10]);
    y_e[3] = fabsf(Braw_e[14]);
    Mz_max_e = Controller_sum(y_e);
    y_e[0] = Braw_e[3];
    y_e[1] = Braw_e[7];
    y_e[2] = Braw_e[11];
    y_e[3] = Braw_e[15];
    T_max_e = Controller_sum(y_e);
    if (n_thrust_idx_1_e < 1.0E-6F) {
      n_thrust_idx_1_e = 1.0F;
    }

    if (n_thrust_idx_2_e < 1.0E-6F) {
      n_thrust_idx_2_e = 1.0F;
    }

    if (Mz_max_e < 1.0E-6F) {
      Mz_max_e = 1.0F;
    }

    if (T_max_e < 1.0E-6F) {
      T_max_e = 1.0F;
    }

    memset(&B4_e[0], 0, sizeof(real32_T) << 4U);
    B4_e[0] = 1.0F / n_thrust_idx_1_e;
    B4_e[5] = 1.0F / n_thrust_idx_2_e;
    B4_e[10] = 1.0F / Mz_max_e;
    B4_e[15] = 1.0F / T_max_e;
    for (i_e = 0; i_e < 4; i_e++) {
      for (B4_tmp_e = 0; B4_tmp_e < 8; B4_tmp_e++) {
        rtb_B4x8_tmp_e = B4_tmp_e << 2;
        rtb_B4x8_tmp_o = i_e + rtb_B4x8_tmp_e;
        rtb_B4x8[rtb_B4x8_tmp_o] = 0.0F;
        rtb_B4x8_tmp_h = rtb_B4x8_tmp_e + i_e;
        rtb_B4x8[rtb_B4x8_tmp_o] = rtb_B4x8[rtb_B4x8_tmp_h] +
          Braw_e[rtb_B4x8_tmp_e] * B4_e[i_e];
        rtb_B4x8[rtb_B4x8_tmp_o] = Braw_e[rtb_B4x8_tmp_e + 1] * B4_e[i_e + 4] +
          rtb_B4x8[rtb_B4x8_tmp_h];
        rtb_B4x8[rtb_B4x8_tmp_o] = Braw_e[rtb_B4x8_tmp_e + 2] * B4_e[i_e + 8] +
          rtb_B4x8[rtb_B4x8_tmp_h];
        rtb_B4x8[rtb_B4x8_tmp_o] = Braw_e[rtb_B4x8_tmp_e + 3] * B4_e[i_e + 12] +
          rtb_B4x8[rtb_B4x8_tmp_h];
      }

      /* MATLAB Function: '<S14>/MATLAB Function2' incorporates:
       *  Constant: '<S14>/Constant7'
       */
      y_e[i_e] = (real32_T)Controller_ConstP.Constant7_Value[i_e];
    }

    /* End of MATLAB Function: '<S14>/MATLAB Function1' */

    /* MATLAB Function: '<S14>/MATLAB Function2' */
    memset(&B4_e[0], 0, sizeof(real32_T) << 4U);
    B4_e[0] = y_e[0];
    B4_e[5] = y_e[1];
    B4_e[10] = y_e[2];
    B4_e[15] = y_e[3];

    /* Delay: '<S14>/Delay' */
    for (i_e = 0; i_e < 8; i_e++) {
      rtb_Delay[i_e] = Controller_DW.Quadcopter_x.Delay_DSTATE[i_e];
    }

    /* Saturate: '<S14>/Saturation1' */
    if (rtb_Sum[0] > 1.0F) {
      /* SignalConversion: '<S57>/TmpSignal ConversionAt SFunction Inport2' incorporates:
       *  MATLAB Function: '<S14>/MATLAB Function'
       */
      rtb_TmpSignalConversionAtSFunct[0] = 1.0F;
    } else if (rtb_Sum[0] < -1.0F) {
      /* SignalConversion: '<S57>/TmpSignal ConversionAt SFunction Inport2' incorporates:
       *  MATLAB Function: '<S14>/MATLAB Function'
       */
      rtb_TmpSignalConversionAtSFunct[0] = -1.0F;
    } else {
      /* SignalConversion: '<S57>/TmpSignal ConversionAt SFunction Inport2' incorporates:
       *  MATLAB Function: '<S14>/MATLAB Function'
       */
      rtb_TmpSignalConversionAtSFunct[0] = rtb_Sum[0];
    }

    if (rtb_Sum[1] > 1.0F) {
      n_thrust_idx_1_e = 1.0F;
    } else if (rtb_Sum[1] < -1.0F) {
      n_thrust_idx_1_e = -1.0F;
    } else {
      n_thrust_idx_1_e = rtb_Sum[1];
    }

    if (rtb_Sum[2] > 1.0F) {
      n_thrust_idx_2_e = 1.0F;
    } else if (rtb_Sum[2] < -1.0F) {
      n_thrust_idx_2_e = -1.0F;
    } else {
      n_thrust_idx_2_e = rtb_Sum[2];
    }

    /* End of Saturate: '<S14>/Saturation1' */

    /* Saturate: '<S14>/Saturation2' */
    if (rtb_Saturation_i > 1.0F) {
      Mz_max_e = 1.0F;
    } else if (rtb_Saturation_i < -1.0F) {
      Mz_max_e = -1.0F;
    } else {
      Mz_max_e = rtb_Saturation_i;
    }

    /* End of Saturate: '<S14>/Saturation2' */

    /* SignalConversion: '<S57>/TmpSignal ConversionAt SFunction Inport2' incorporates:
     *  Constant: '<S14>/hover_throttle'
     *  MATLAB Function: '<S14>/MATLAB Function'
     *  Sum: '<S14>/Sum'
     */
    rtb_TmpSignalConversionAtSFunct[3] = Mz_max_e + CONTROL_PARAM.HOVER_THRO;

    /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
     *  Constant: '<S14>/Constant6'
     *  Delay: '<S14>/Delay'
     *  SignalConversion: '<S57>/TmpSignal ConversionAt SFunction Inport2'
     */
    for (rtb_B4x8_tmp_e = 0; rtb_B4x8_tmp_e < 8; rtb_B4x8_tmp_e++) {
      for (i_e = 0; i_e < 4; i_e++) {
        B4_tmp_e = rtb_B4x8_tmp_e << 2;
        Braw_e[i_e + B4_tmp_e] = ((rtb_B4x8[B4_tmp_e + 1] * B4_e[i_e + 4] +
          rtb_B4x8[B4_tmp_e] * B4_e[i_e]) + rtb_B4x8[B4_tmp_e + 2] * B4_e[i_e +
          8]) + rtb_B4x8[B4_tmp_e + 3] * B4_e[i_e + 12];
      }
    }

    Mz_max_e = 0.0F;
    for (i_e = 0; i_e < 4; i_e++) {
      y_e[i_e] = ((B4_e[i_e + 4] * n_thrust_idx_1_e + B4_e[i_e] *
                   rtb_TmpSignalConversionAtSFunct[0]) + B4_e[i_e + 8] *
                  n_thrust_idx_2_e) + B4_e[i_e + 12] *
        rtb_TmpSignalConversionAtSFunct[3];
      for (B4_tmp_e = 0; B4_tmp_e < 8; B4_tmp_e++) {
        T_max_e = Braw_e[(B4_tmp_e << 2) + i_e];
        Mz_max_e += T_max_e * T_max_e;
      }
    }

    n_thrust_idx_1_e = 1.0F / fmaxf(2.0F * Mz_max_e + 0.12F, 1.0E-6F);
    for (i_e = 0; i_e < 20; i_e++) {
      for (B4_tmp_e = 0; B4_tmp_e < 4; B4_tmp_e++) {
        n_thrust_idx_2_e = 0.0F;
        for (rtb_B4x8_tmp_e = 0; rtb_B4x8_tmp_e < 8; rtb_B4x8_tmp_e++) {
          n_thrust_idx_2_e += Braw_e[(rtb_B4x8_tmp_e << 2) + B4_tmp_e] *
            Controller_DW.Quadcopter_x.Delay_DSTATE[rtb_B4x8_tmp_e];
        }

        rtb_TmpSignalConversionAtSFunct[B4_tmp_e] = n_thrust_idx_2_e -
          y_e[B4_tmp_e];
      }

      for (B4_tmp_e = 0; B4_tmp_e < 8; B4_tmp_e++) {
        rtb_B4x8_tmp_e = B4_tmp_e << 2;
        n_thrust_idx_2_e = Controller_DW.Quadcopter_x.Delay_DSTATE[B4_tmp_e] -
          ((((Braw_e[rtb_B4x8_tmp_e + 1] * rtb_TmpSignalConversionAtSFunct[1] +
              Braw_e[rtb_B4x8_tmp_e] * rtb_TmpSignalConversionAtSFunct[0]) +
             Braw_e[rtb_B4x8_tmp_e + 2] * rtb_TmpSignalConversionAtSFunct[2]) +
            Braw_e[rtb_B4x8_tmp_e + 3] * rtb_TmpSignalConversionAtSFunct[3]) *
           2.0F + (Controller_DW.Quadcopter_x.Delay_DSTATE[B4_tmp_e] -
                   rtb_Delay[B4_tmp_e]) * 0.12F) * n_thrust_idx_1_e;
        if (n_thrust_idx_2_e < 0.0F) {
          n_thrust_idx_2_e = 0.0F;
        }

        if (n_thrust_idx_2_e > 1.0F) {
          n_thrust_idx_2_e = 1.0F;
        }

        Controller_DW.Quadcopter_x.Delay_DSTATE[B4_tmp_e] = n_thrust_idx_2_e;
      }
    }

    /* MultiPortSwitch: '<S61>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      /* Reshape: '<S61>/Reshape' */
      rtb_VariantMergeForOutportactua[0] = 1000U;
      rtb_VariantMergeForOutportactua[1] = 1000U;
      rtb_VariantMergeForOutportactua[2] = 1000U;
      rtb_VariantMergeForOutportactua[3] = 1000U;
      break;

     case 2:
      /* Reshape: '<S61>/Reshape' */
      rtb_VariantMergeForOutportactua[0] = 1150U;
      rtb_VariantMergeForOutportactua[1] = 1150U;
      rtb_VariantMergeForOutportactua[2] = 1150U;
      rtb_VariantMergeForOutportactua[3] = 1150U;
      break;

     case 3:
      /* Gain: '<S14>/Gain' incorporates:
       *  Delay: '<S14>/Delay'
       */
      Mz_max_e = fmodf(floorf(1000.0F * Controller_DW.Quadcopter_x.Delay_DSTATE
        [0]), 65536.0F);

      /* Sum: '<S14>/Add' incorporates:
       *  Constant: '<S14>/Constant10'
       *  Gain: '<S14>/Gain'
       */
      u0_e = (uint16_T)((Mz_max_e < 0.0F ? (int32_T)(uint16_T)-(int16_T)
                         (uint16_T)-Mz_max_e : (int32_T)(uint16_T)Mz_max_e) +
                        1000U);

      /* Saturate: '<S14>/Saturation' */
      if (u0_e > 1900) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 1900U;
      } else if (u0_e < 1100) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 1100U;
      } else {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = u0_e;
      }

      /* Gain: '<S14>/Gain' incorporates:
       *  Delay: '<S14>/Delay'
       */
      Mz_max_e = fmodf(floorf(1000.0F * Controller_DW.Quadcopter_x.Delay_DSTATE
        [1]), 65536.0F);

      /* Sum: '<S14>/Add' incorporates:
       *  Constant: '<S14>/Constant10'
       *  Gain: '<S14>/Gain'
       */
      u0_e = (uint16_T)((Mz_max_e < 0.0F ? (int32_T)(uint16_T)-(int16_T)
                         (uint16_T)-Mz_max_e : (int32_T)(uint16_T)Mz_max_e) +
                        1000U);

      /* Saturate: '<S14>/Saturation' */
      if (u0_e > 1900) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 1900U;
      } else if (u0_e < 1100) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 1100U;
      } else {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = u0_e;
      }

      /* Gain: '<S14>/Gain' incorporates:
       *  Delay: '<S14>/Delay'
       */
      Mz_max_e = fmodf(floorf(1000.0F * Controller_DW.Quadcopter_x.Delay_DSTATE
        [2]), 65536.0F);

      /* Sum: '<S14>/Add' incorporates:
       *  Constant: '<S14>/Constant10'
       *  Gain: '<S14>/Gain'
       */
      u0_e = (uint16_T)((Mz_max_e < 0.0F ? (int32_T)(uint16_T)-(int16_T)
                         (uint16_T)-Mz_max_e : (int32_T)(uint16_T)Mz_max_e) +
                        1000U);

      /* Saturate: '<S14>/Saturation' */
      if (u0_e > 1900) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[2] = 1900U;
      } else if (u0_e < 1100) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[2] = 1100U;
      } else {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[2] = u0_e;
      }

      /* Gain: '<S14>/Gain' incorporates:
       *  Delay: '<S14>/Delay'
       */
      Mz_max_e = fmodf(floorf(1000.0F * Controller_DW.Quadcopter_x.Delay_DSTATE
        [3]), 65536.0F);

      /* Sum: '<S14>/Add' incorporates:
       *  Constant: '<S14>/Constant10'
       *  Gain: '<S14>/Gain'
       */
      u0_e = (uint16_T)((Mz_max_e < 0.0F ? (int32_T)(uint16_T)-(int16_T)
                         (uint16_T)-Mz_max_e : (int32_T)(uint16_T)Mz_max_e) +
                        1000U);

      /* Saturate: '<S14>/Saturation' */
      if (u0_e > 1900) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[3] = 1900U;
      } else if (u0_e < 1100) {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[3] = 1100U;
      } else {
        /* Reshape: '<S61>/Reshape' */
        rtb_VariantMergeForOutportactua[3] = u0_e;
      }
      break;

     default:
      /* Reshape: '<S61>/Reshape' */
      rtb_VariantMergeForOutportactua[0] = 1000U;
      rtb_VariantMergeForOutportactua[1] = 1000U;
      rtb_VariantMergeForOutportactua[2] = 1000U;
      rtb_VariantMergeForOutportactua[3] = 1000U;
      break;
    }

    /* End of MultiPortSwitch: '<S61>/Multiport Switch' */

    /* Reshape: '<S61>/Reshape' */
    for (i_e = 0; i_e < 12; i_e++) {
      rtb_VariantMergeForOutportactua[i_e + 4] = 0U;
    }
  }

#endif

  /* End of Outputs for SubSystem: '<S2>/Control_Allocation' */

  /* Product: '<S76>/Multiply' incorporates:
   *  Constant: '<S76>/gain1'
   *  Constant: '<S76>/gain2'
   *  Constant: '<S76>/gain3'
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator5'
   */
  rtb_Add_g_idx_0 = CONTROL_PARAM.ROLL_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[0];
  rtb_Add_g_idx_1 = CONTROL_PARAM.PITCH_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[1];
  rtb_Add_g_idx_2 = CONTROL_PARAM.YAW_RATE_I *
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[2];

  /* Product: '<S102>/Multiply1' incorporates:
   *  Constant: '<S102>/const1'
   *  DiscreteIntegrator: '<S99>/Integrator'
   */
  rtb_Saturation_idx_0 = Controller_DW.Integrator_DSTATE[0] * 0.05F;
  rtb_Subtract1_k = Controller_DW.Integrator_DSTATE[1] * 0.05F;

  /* Switch: '<S94>/Switch' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (rtb_Compare_eg) {
    rtb_Saturation_d_idx_2 = Controller_U.FMS_Out.u_cmd;
  } else {
    rtb_Saturation_d_idx_2 = rtb_Multiply_j[0];
  }

  /* Sum: '<S102>/Add' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator1'
   *  Sum: '<S99>/Subtract'
   */
  rtb_att_error_B_rad_idx_0 = (Controller_DW.Integrator1_DSTATE[0] -
    rtb_Saturation_d_idx_2) + rtb_Saturation_idx_0;

  /* Switch: '<S94>/Switch' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (rtb_Compare_eg) {
    rtb_Saturation_d_idx_2 = Controller_U.FMS_Out.v_cmd;
  } else {
    rtb_Saturation_d_idx_2 = rtb_Multiply_j[1];
  }

  /* Sum: '<S102>/Add' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator1'
   *  Sum: '<S99>/Subtract'
   */
  rtb_att_error_B_rad_idx_1 = (Controller_DW.Integrator1_DSTATE[1] -
    rtb_Saturation_d_idx_2) + rtb_Subtract1_k;

  /* Signum: '<S102>/Sign' */
  if (rtb_att_error_B_rad_idx_0 < 0.0F) {
    rtb_Saturation_d_idx_2 = -1.0F;
  } else if (rtb_att_error_B_rad_idx_0 > 0.0F) {
    rtb_Saturation_d_idx_2 = 1.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_att_error_B_rad_idx_0;
  }

  /* Sum: '<S102>/Add2' incorporates:
   *  Abs: '<S102>/Abs'
   *  Gain: '<S102>/Gain'
   *  Gain: '<S102>/Gain1'
   *  Product: '<S102>/Multiply2'
   *  Product: '<S102>/Multiply3'
   *  Signum: '<S102>/Sign'
   *  Sqrt: '<S102>/Sqrt'
   *  Sum: '<S102>/Add1'
   *  Sum: '<S102>/Subtract'
   */
  rtb_Saturation_d_idx_2 = (sqrtf((8.0F * fabsf(rtb_att_error_B_rad_idx_0) +
    Controller_ConstB.d) * Controller_ConstB.d) - Controller_ConstB.d) * 0.5F *
    rtb_Saturation_d_idx_2 + rtb_Saturation_idx_0;

  /* Sum: '<S102>/Add3' incorporates:
   *  Signum: '<S102>/Sign'
   */
  rtb_Subtract3_i = rtb_att_error_B_rad_idx_0 + Controller_ConstB.d;

  /* Sum: '<S102>/Subtract1' incorporates:
   *  Signum: '<S102>/Sign'
   */
  rtb_a = rtb_att_error_B_rad_idx_0 - Controller_ConstB.d;

  /* Signum: '<S102>/Sign1' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign2' */
  if (rtb_a < 0.0F) {
    rtb_a = -1.0F;
  } else {
    if (rtb_a > 0.0F) {
      rtb_a = 1.0F;
    }
  }

  /* Sum: '<S102>/Add2' incorporates:
   *  Gain: '<S102>/Gain2'
   *  Product: '<S102>/Multiply4'
   *  Signum: '<S102>/Sign'
   *  Sum: '<S102>/Add4'
   *  Sum: '<S102>/Add5'
   *  Sum: '<S102>/Subtract2'
   */
  rtb_att_error_B_rad_idx_0 = ((rtb_att_error_B_rad_idx_0 -
    rtb_Saturation_d_idx_2) + rtb_Saturation_idx_0) * ((rtb_Subtract3_i - rtb_a)
    * 0.5F) + rtb_Saturation_d_idx_2;

  /* Signum: '<S102>/Sign' */
  if (rtb_att_error_B_rad_idx_1 < 0.0F) {
    rtb_Saturation_d_idx_2 = -1.0F;
  } else if (rtb_att_error_B_rad_idx_1 > 0.0F) {
    rtb_Saturation_d_idx_2 = 1.0F;
  } else {
    rtb_Saturation_d_idx_2 = rtb_att_error_B_rad_idx_1;
  }

  /* Sum: '<S102>/Add2' incorporates:
   *  Abs: '<S102>/Abs'
   *  Gain: '<S102>/Gain'
   *  Gain: '<S102>/Gain1'
   *  Product: '<S102>/Multiply2'
   *  Product: '<S102>/Multiply3'
   *  Signum: '<S102>/Sign'
   *  Sqrt: '<S102>/Sqrt'
   *  Sum: '<S102>/Add1'
   *  Sum: '<S102>/Subtract'
   */
  rtb_Saturation_d_idx_2 = (sqrtf((8.0F * fabsf(rtb_att_error_B_rad_idx_1) +
    Controller_ConstB.d) * Controller_ConstB.d) - Controller_ConstB.d) * 0.5F *
    rtb_Saturation_d_idx_2 + rtb_Subtract1_k;

  /* Sum: '<S102>/Add3' incorporates:
   *  Signum: '<S102>/Sign'
   */
  rtb_Subtract3_i = rtb_att_error_B_rad_idx_1 + Controller_ConstB.d;

  /* Sum: '<S102>/Subtract1' incorporates:
   *  Signum: '<S102>/Sign'
   */
  rtb_a = rtb_att_error_B_rad_idx_1 - Controller_ConstB.d;

  /* Signum: '<S102>/Sign1' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign2' */
  if (rtb_a < 0.0F) {
    rtb_a = -1.0F;
  } else {
    if (rtb_a > 0.0F) {
      rtb_a = 1.0F;
    }
  }

  /* Sum: '<S102>/Add5' incorporates:
   *  Gain: '<S102>/Gain2'
   *  Product: '<S102>/Multiply4'
   *  Signum: '<S102>/Sign'
   *  Sum: '<S102>/Add2'
   *  Sum: '<S102>/Add4'
   *  Sum: '<S102>/Subtract2'
   */
  rtb_Saturation_d_idx_2 += ((rtb_att_error_B_rad_idx_1 - rtb_Saturation_d_idx_2)
    + rtb_Subtract1_k) * ((rtb_Subtract3_i - rtb_a) * 0.5F);

  /* Outport: '<Root>/Control_Out' incorporates:
   *  BusAssignment: '<S1>/Bus Assignment'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   */
  Controller_Y.Control_Out.timestamp =
    Controller_DW.DiscreteTimeIntegrator_DSTATE_n;
  for (i = 0; i < 16; i++) {
    Controller_Y.Control_Out.actuator_cmd[i] = rtb_VariantMergeForOutportactua[i];
  }

  /* End of Outport: '<Root>/Control_Out' */

  /* Product: '<S117>/Multiply1' incorporates:
   *  Constant: '<S117>/const1'
   *  DiscreteIntegrator: '<S116>/Integrator'
   */
  rtb_Subtract3_i = Controller_DW.Integrator_DSTATE_p * 0.05F;

  /* Switch: '<S113>/Switch' incorporates:
   *  Inport: '<Root>/FMS_Out'
   *  Inport: '<Root>/INS_Out'
   */
  if (rtb_Compare_g) {
    rtb_a = Controller_U.FMS_Out.w_cmd;
  } else {
    rtb_a = Controller_U.INS_Out.vd;
  }

  /* End of Switch: '<S113>/Switch' */

  /* Sum: '<S117>/Add' incorporates:
   *  Sum: '<S116>/Subtract'
   */
  rtb_Subtract1_k = (w_cmd_C_mPs - rtb_a) + rtb_Subtract3_i;

  /* Signum: '<S117>/Sign' */
  if (rtb_Subtract1_k < 0.0F) {
    rtb_a = -1.0F;
  } else if (rtb_Subtract1_k > 0.0F) {
    rtb_a = 1.0F;
  } else {
    rtb_a = rtb_Subtract1_k;
  }

  /* End of Signum: '<S117>/Sign' */

  /* Sum: '<S117>/Add2' incorporates:
   *  Abs: '<S117>/Abs'
   *  Gain: '<S117>/Gain'
   *  Gain: '<S117>/Gain1'
   *  Product: '<S117>/Multiply2'
   *  Product: '<S117>/Multiply3'
   *  Sqrt: '<S117>/Sqrt'
   *  Sum: '<S117>/Add1'
   *  Sum: '<S117>/Subtract'
   */
  rtb_a = (sqrtf((8.0F * fabsf(rtb_Subtract1_k) + Controller_ConstB.d_n) *
                 Controller_ConstB.d_n) - Controller_ConstB.d_n) * 0.5F * rtb_a
    + rtb_Subtract3_i;

  /* Sum: '<S117>/Add4' */
  rtb_att_error_B_rad_idx_1 = (rtb_Subtract1_k - rtb_a) + rtb_Subtract3_i;

  /* Sum: '<S117>/Add3' */
  rtb_Subtract3_i = rtb_Subtract1_k + Controller_ConstB.d_n;

  /* Sum: '<S117>/Subtract1' */
  rtb_Subtract1_k -= Controller_ConstB.d_n;

  /* Signum: '<S117>/Sign1' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* End of Signum: '<S117>/Sign1' */

  /* Signum: '<S117>/Sign2' */
  if (rtb_Subtract1_k < 0.0F) {
    rtb_Subtract1_k = -1.0F;
  } else {
    if (rtb_Subtract1_k > 0.0F) {
      rtb_Subtract1_k = 1.0F;
    }
  }

  /* End of Signum: '<S117>/Sign2' */

  /* Sum: '<S117>/Add5' incorporates:
   *  Gain: '<S117>/Gain2'
   *  Product: '<S117>/Multiply4'
   *  Sum: '<S117>/Subtract2'
   */
  rtb_a += (rtb_Subtract3_i - rtb_Subtract1_k) * 0.5F *
    rtb_att_error_B_rad_idx_1;

  /* Update for DiscreteIntegrator: '<S104>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S106>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;

  /* Update for DiscreteIntegrator: '<S99>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE[0] += 0.002F *
    Controller_DW.Integrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S104>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S104>/ki'
   *  Product: '<S104>/Multiply'
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

  /* Update for DiscreteIntegrator: '<S106>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] += 0.002F * rtb_Gain_idx_0;

  /* Update for DiscreteIntegrator: '<S99>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE[1] += 0.002F *
    Controller_DW.Integrator_DSTATE[1];

  /* Update for DiscreteIntegrator: '<S104>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S104>/ki'
   *  Product: '<S104>/Multiply'
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

  /* Update for DiscreteIntegrator: '<S106>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] += 0.002F * rtb_Gain_idx_1;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' */
  Controller_DW.DiscreteTimeIntegrator5_IC_LOAD = 0U;

  /* Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator_PrevRe_g = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_l = 0U;

  /* Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S73>/Gain'
   *  Sum: '<S73>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[0] +=
    (rtb_rate_error_B_radPs_idx_0 -
     Controller_DW.DiscreteTimeIntegrator5_DSTATE[0]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[0] += 0.002F * rtb_Add_g_idx_0;
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

  /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[0] += 0.002F *
    rtb_DiscreteTimeIntegrator1_j;

  /* Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S73>/Gain'
   *  Sum: '<S73>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[1] += (rtb_Saturation1_idx_0 -
    Controller_DW.DiscreteTimeIntegrator5_DSTATE[1]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[1] += 0.002F * rtb_Add_g_idx_1;
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

  /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[1] += 0.002F * rtb_Gain_c_idx_1;

  /* Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' incorporates:
   *  Gain: '<S73>/Gain'
   *  Sum: '<S73>/Sum5'
   */
  Controller_DW.DiscreteTimeIntegrator5_DSTATE[2] +=
    (rtb_rate_error_B_radPs_idx_2 -
     Controller_DW.DiscreteTimeIntegrator5_DSTATE[2]) * 188.49556F * 0.002F;

  /* Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_o[2] += 0.002F * rtb_Add_g_idx_2;
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

  /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_o[2] += 0.002F * rtb_Gain_c_idx_2;
  Controller_DW.DiscreteTimeIntegrator1_PrevR_i = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S116>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S116>/Integrator'
   */
  Controller_DW.Integrator1_DSTATE_p += 0.002F *
    Controller_DW.Integrator_DSTATE_p;

  /* Update for DiscreteIntegrator: '<S119>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S119>/ki'
   *  Inport: '<Root>/FMS_Out'
   *  Product: '<S119>/Multiply'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_m += CONTROL_PARAM.VEL_Z_I *
    rtb_Saturation_d_idx_1 * 0.002F;
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

  /* End of Update for DiscreteIntegrator: '<S119>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S121>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_k = 0U;
  Controller_DW.DiscreteTimeIntegrator1_DSTAT_h += 0.002F * rtb_Gain_am;
  Controller_DW.DiscreteTimeIntegrator1_Prev_iy = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Signum: '<S102>/Sign3' incorporates:
   *  Product: '<S102>/Divide'
   *  Sum: '<S102>/Add6'
   */
  rtb_Subtract3_i = rtb_att_error_B_rad_idx_0 + Controller_ConstB.d;

  /* Signum: '<S102>/Sign4' incorporates:
   *  Product: '<S102>/Divide'
   *  Sum: '<S102>/Subtract3'
   */
  rtb_uv_error_C_mPs_idx_0 = rtb_att_error_B_rad_idx_0 - Controller_ConstB.d;

  /* Signum: '<S102>/Sign5' incorporates:
   *  Product: '<S102>/Divide'
   */
  if (rtb_att_error_B_rad_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;
  } else if (rtb_att_error_B_rad_idx_0 > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_att_error_B_rad_idx_0;
  }

  /* Signum: '<S102>/Sign3' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign6' incorporates:
   *  Product: '<S102>/Divide'
   */
  if (rtb_att_error_B_rad_idx_0 < 0.0F) {
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_att_error_B_rad_idx_0 > 0.0F) {
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_Gain_idx_0 = rtb_att_error_B_rad_idx_0;
  }

  /* Update for DiscreteIntegrator: '<S99>/Integrator' incorporates:
   *  Constant: '<S102>/const'
   *  Gain: '<S102>/Gain3'
   *  Product: '<S102>/Divide'
   *  Product: '<S102>/Multiply5'
   *  Product: '<S102>/Multiply6'
   *  Sum: '<S102>/Subtract4'
   *  Sum: '<S102>/Subtract5'
   *  Sum: '<S102>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE[0] += ((rtb_att_error_B_rad_idx_0 /
    Controller_ConstB.d - rtb_uv_error_C_mPs_idx_1) * Controller_ConstB.Gain4 *
    ((rtb_Subtract3_i - rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_Gain_idx_0 *
    58.836F) * 0.002F;

  /* Signum: '<S102>/Sign3' incorporates:
   *  Sum: '<S102>/Add6'
   */
  rtb_Subtract3_i = rtb_Saturation_d_idx_2 + Controller_ConstB.d;

  /* Signum: '<S102>/Sign4' incorporates:
   *  Sum: '<S102>/Subtract3'
   */
  rtb_uv_error_C_mPs_idx_0 = rtb_Saturation_d_idx_2 - Controller_ConstB.d;

  /* Signum: '<S102>/Sign5' */
  if (rtb_Saturation_d_idx_2 < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;
  } else if (rtb_Saturation_d_idx_2 > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_Saturation_d_idx_2;
  }

  /* Signum: '<S102>/Sign3' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S102>/Sign6' */
  if (rtb_Saturation_d_idx_2 < 0.0F) {
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_Saturation_d_idx_2 > 0.0F) {
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_Gain_idx_0 = rtb_Saturation_d_idx_2;
  }

  /* Update for DiscreteIntegrator: '<S99>/Integrator' incorporates:
   *  Constant: '<S102>/const'
   *  Gain: '<S102>/Gain3'
   *  Product: '<S102>/Divide'
   *  Product: '<S102>/Multiply5'
   *  Product: '<S102>/Multiply6'
   *  Sum: '<S102>/Subtract4'
   *  Sum: '<S102>/Subtract5'
   *  Sum: '<S102>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE[1] += ((rtb_Saturation_d_idx_2 /
    Controller_ConstB.d - rtb_uv_error_C_mPs_idx_1) * Controller_ConstB.Gain4 *
    ((rtb_Subtract3_i - rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_Gain_idx_0 *
    58.836F) * 0.002F;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_n += CONTROL_EXPORT.period;

  /* Signum: '<S117>/Sign6' incorporates:
   *  Signum: '<S117>/Sign5'
   */
  if (rtb_a < 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = -1.0F;

    /* Signum: '<S117>/Sign5' */
    rtb_Gain_idx_0 = -1.0F;
  } else if (rtb_a > 0.0F) {
    rtb_uv_error_C_mPs_idx_1 = 1.0F;

    /* Signum: '<S117>/Sign5' */
    rtb_Gain_idx_0 = 1.0F;
  } else {
    rtb_uv_error_C_mPs_idx_1 = rtb_a;

    /* Signum: '<S117>/Sign5' */
    rtb_Gain_idx_0 = rtb_a;
  }

  /* End of Signum: '<S117>/Sign6' */

  /* Sum: '<S117>/Add6' */
  rtb_Subtract3_i = rtb_a + Controller_ConstB.d_n;

  /* Sum: '<S117>/Subtract3' */
  rtb_uv_error_C_mPs_idx_0 = rtb_a - Controller_ConstB.d_n;

  /* Signum: '<S117>/Sign3' */
  if (rtb_Subtract3_i < 0.0F) {
    rtb_Subtract3_i = -1.0F;
  } else {
    if (rtb_Subtract3_i > 0.0F) {
      rtb_Subtract3_i = 1.0F;
    }
  }

  /* End of Signum: '<S117>/Sign3' */

  /* Signum: '<S117>/Sign4' */
  if (rtb_uv_error_C_mPs_idx_0 < 0.0F) {
    rtb_uv_error_C_mPs_idx_0 = -1.0F;
  } else {
    if (rtb_uv_error_C_mPs_idx_0 > 0.0F) {
      rtb_uv_error_C_mPs_idx_0 = 1.0F;
    }
  }

  /* End of Signum: '<S117>/Sign4' */

  /* Update for DiscreteIntegrator: '<S116>/Integrator' incorporates:
   *  Constant: '<S117>/const'
   *  Gain: '<S117>/Gain3'
   *  Product: '<S117>/Divide'
   *  Product: '<S117>/Multiply5'
   *  Product: '<S117>/Multiply6'
   *  Sum: '<S117>/Subtract4'
   *  Sum: '<S117>/Subtract5'
   *  Sum: '<S117>/Subtract6'
   */
  Controller_DW.Integrator_DSTATE_p += ((rtb_a / Controller_ConstB.d_n -
    rtb_Gain_idx_0) * Controller_ConstB.Gain4_k * ((rtb_Subtract3_i -
    rtb_uv_error_C_mPs_idx_0) * 0.5F) - rtb_uv_error_C_mPs_idx_1 * 78.448F) *
    0.002F;
}

/* Model initialize function */
void Controller_init(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Controller_M, (NULL));

  /* block I/O */

  /* exported global signals */
  att_cmd_B_rad[0] = 0.0F;
  att_cmd_B_rad[1] = 0.0F;
  att_est_B_rad[0] = 0.0F;
  att_est_B_rad[1] = 0.0F;
  rate_cmd_B_radPs[0] = 0.0F;
  rate_cmd_B_radPs[1] = 0.0F;
  rate_cmd_B_radPs[2] = 0.0F;
  rate_est_B_radPs[0] = 0.0F;
  rate_est_B_radPs[1] = 0.0F;
  rate_est_B_radPs[2] = 0.0F;
  w_cmd_C_mPs = 0.0F;
  w_est_C_mPs = 0.0F;

  /* states (dwork) */
  (void) memset((void *)&Controller_DW, 0,
                sizeof(DW_Controller_T));

  /* external inputs */
  (void)memset(&Controller_U, 0, sizeof(ExtU_Controller_T));

  /* external outputs */
  Controller_Y.Control_Out = Controller_rtZControl_Out_Bus;

  /* Start for Constant: '<S119>/Constant' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_m = CONTROL_PARAM.VEL_Z_I_MIN;

  /* InitializeConditions for DiscreteIntegrator: '<S104>/Discrete-Time Integrator' */
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

  /* End of InitializeConditions for DiscreteIntegrator: '<S104>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S106>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator5' */
  Controller_DW.DiscreteTimeIntegrator5_IC_LOAD = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' */
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

  /* End of InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LO_l = 1U;
  Controller_DW.DiscreteTimeIntegrator1_PrevR_i = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S119>/Discrete-Time Integrator' */
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

  /* End of InitializeConditions for DiscreteIntegrator: '<S119>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S121>/Discrete-Time Integrator1' */
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

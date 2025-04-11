/*
 * File: Controller.c
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 1.1191
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Thu Apr 10 16:46:35 2025
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
struct_pNbTadB0jgRVqt3p6wG0lB CONTROL_PARAM = {
  2.0F,
  2.0F,
  0.0F,
  0.1F,
  -0.1F,
  0.1F,
  -0.1F,
  3.0F,
  300.0F,
  1000U,
  200.0F,
  500U
} ;                                    /* Variable: CONTROL_PARAM
                                        * Referenced by:
                                        *   '<S5>/Gain'
                                        *   '<S7>/Bias'
                                        *   '<S7>/Bias1'
                                        *   '<S7>/Gain'
                                        *   '<S7>/Gain1'
                                        *   '<S8>/Bias'
                                        *   '<S8>/Bias1'
                                        *   '<S8>/Gain'
                                        *   '<S8>/Gain1'
                                        *   '<S23>/gain1'
                                        *   '<S23>/Saturation'
                                        *   '<S24>/gain1'
                                        *   '<S24>/Discrete-Time Integrator'
                                        *   '<S25>/gain1'
                                        *   '<S10>/Disarm'
                                        *   '<S10>/Disarm1'
                                        *   '<S11>/Standby'
                                        *   '<S11>/Standby1'
                                        *   '<S15>/Disarm'
                                        *   '<S15>/Disarm1'
                                        *   '<S16>/Standby'
                                        *   '<S16>/Standby1'
                                        */

struct_biZzOMrg0u3lxrb7POOubF CONTROL_EXPORT = {
  10U,

  { 66, 111, 97, 116, 32, 67, 111, 110, 116, 114, 111, 108, 108, 101, 114, 32,
    118, 49, 46, 48, 46, 48, 0 }
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
  /* local block i/o variables */
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_i;
  uint16_T rtb_VariantMergeForOutportactua[16];
  real32_T rtb_VectorConcatenate[9];
  real32_T rtb_u_err;
  real32_T rtb_Gain;
  int32_T i;
  real32_T rtb_VectorConcatenate_0[3];
  real32_T tmp;

  /* Trigonometry: '<S22>/Trigonometric Function1' incorporates:
   *  Gain: '<S21>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S22>/Trigonometric Function3'
   */
  rtb_u_err = arm_cos_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[0] = rtb_u_err;

  /* Trigonometry: '<S22>/Trigonometric Function' incorporates:
   *  Gain: '<S21>/Gain'
   *  Inport: '<Root>/INS_Out'
   *  Trigonometry: '<S22>/Trigonometric Function2'
   */
  rtb_Gain = arm_sin_f32(-Controller_U.INS_Out.psi);
  rtb_VectorConcatenate[1] = rtb_Gain;

  /* SignalConversion: '<S22>/ConcatBufferAtVector Concatenate1In3' incorporates:
   *  Constant: '<S22>/Constant3'
   */
  rtb_VectorConcatenate[2] = 0.0F;

  /* Gain: '<S22>/Gain' */
  rtb_VectorConcatenate[3] = -rtb_Gain;

  /* Trigonometry: '<S22>/Trigonometric Function3' */
  rtb_VectorConcatenate[4] = rtb_u_err;

  /* SignalConversion: '<S22>/ConcatBufferAtVector Concatenate2In3' incorporates:
   *  Constant: '<S22>/Constant4'
   */
  rtb_VectorConcatenate[5] = 0.0F;

  /* SignalConversion: '<S22>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_VectorConcatenate[6] = Controller_ConstB.VectorConcatenate3[0];
  rtb_VectorConcatenate[7] = Controller_ConstB.VectorConcatenate3[1];
  rtb_VectorConcatenate[8] = Controller_ConstB.VectorConcatenate3[2];

  /* Product: '<S20>/Multiply' incorporates:
   *  Inport: '<Root>/INS_Out'
   *  SignalConversion: '<S20>/TmpSignal ConversionAtMultiplyInport2'
   */
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_0[i] = rtb_VectorConcatenate[i + 3] *
      Controller_U.INS_Out.ve + rtb_VectorConcatenate[i] *
      Controller_U.INS_Out.vn;
  }

  /* End of Product: '<S20>/Multiply' */

  /* Sum: '<S17>/Sum' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  rtb_u_err = Controller_U.FMS_Out.u_cmd - rtb_VectorConcatenate_0[0];

  /* DiscreteIntegrator: '<S24>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator_PrevRese != 0)) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE = Controller_ConstB.Constant;
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE >= CONTROL_PARAM.VEL_I_MAX)
    {
      Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MAX;
    } else {
      if (Controller_DW.DiscreteTimeIntegrator_DSTATE <= CONTROL_PARAM.VEL_I_MIN)
      {
        Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MIN;
      }
    }
  }

  if (Controller_DW.DiscreteTimeIntegrator_DSTATE >= CONTROL_PARAM.VEL_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE <= CONTROL_PARAM.VEL_I_MIN)
    {
      Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MIN;
    }
  }

  /* DiscreteIntegrator: '<S26>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  if (Controller_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    Controller_DW.DiscreteTimeIntegrator1_DSTATE = rtb_u_err;
  }

  if ((Controller_U.FMS_Out.reset != 0) ||
      (Controller_DW.DiscreteTimeIntegrator1_PrevRes != 0)) {
    Controller_DW.DiscreteTimeIntegrator1_DSTATE = rtb_u_err;
  }

  /* Gain: '<S26>/Gain' incorporates:
   *  DiscreteIntegrator: '<S26>/Discrete-Time Integrator1'
   *  Sum: '<S26>/Sum5'
   */
  rtb_Gain = (rtb_u_err - Controller_DW.DiscreteTimeIntegrator1_DSTATE) *
    188.49556F;

  /* Switch: '<S19>/Switch' incorporates:
   *  Constant: '<S27>/Constant'
   *  Inport: '<Root>/FMS_Out'
   *  RelationalOperator: '<S27>/Compare'
   */
  if (Controller_U.FMS_Out.ctrl_mode >= 5) {
    /* Switch: '<S26>/Switch' incorporates:
     *  Gain: '<S26>/Gain1'
     */
    if (Controller_U.FMS_Out.reset > 0) {
      tmp = 0.0F;
    } else {
      tmp = rtb_Gain;
    }

    /* End of Switch: '<S26>/Switch' */

    /* Product: '<S23>/Multiply' incorporates:
     *  Constant: '<S23>/gain1'
     */
    rtb_Saturation = CONTROL_PARAM.VEL_D * tmp;

    /* Saturate: '<S23>/Saturation' */
    if (rtb_Saturation > CONTROL_PARAM.VEL_D_MAX) {
      rtb_Saturation = CONTROL_PARAM.VEL_D_MAX;
    } else {
      if (rtb_Saturation < CONTROL_PARAM.VEL_D_MIN) {
        rtb_Saturation = CONTROL_PARAM.VEL_D_MIN;
      }
    }

    /* End of Saturate: '<S23>/Saturation' */

    /* Sum: '<S18>/Add' incorporates:
     *  Constant: '<S25>/gain1'
     *  DiscreteIntegrator: '<S24>/Discrete-Time Integrator'
     *  Product: '<S25>/Multiply'
     */
    rtb_Saturation += CONTROL_PARAM.VEL_P * rtb_u_err +
      Controller_DW.DiscreteTimeIntegrator_DSTATE;

    /* Saturate: '<S18>/Saturation' */
    if (rtb_Saturation > 1.0F) {
      rtb_Saturation = 1.0F;
    } else {
      if (rtb_Saturation < -1.0F) {
        rtb_Saturation = -1.0F;
      }
    }

    /* End of Saturate: '<S18>/Saturation' */
  } else {
    rtb_Saturation = Controller_U.FMS_Out.u_cmd;
  }

  /* End of Switch: '<S19>/Switch' */

  /* Saturate: '<S6>/Saturation' */
  if (rtb_Saturation > 1.0F) {
    rtb_Saturation = 1.0F;
  } else {
    if (rtb_Saturation < -1.0F) {
      rtb_Saturation = -1.0F;
    }
  }

  /* End of Saturate: '<S6>/Saturation' */

  /* Gain: '<S5>/Gain' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  rtb_Saturation_i = CONTROL_PARAM.PSI_RATE_P *
    Controller_U.FMS_Out.psi_rate_cmd;

  /* Saturate: '<S5>/Saturation' */
  if (rtb_Saturation_i > 1.0F) {
    /* Gain: '<S5>/Gain' */
    rtb_Saturation_i = 1.0F;
  } else {
    if (rtb_Saturation_i < -1.0F) {
      /* Gain: '<S5>/Gain' */
      rtb_Saturation_i = -1.0F;
    }
  }

  /* End of Saturate: '<S5>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S2>/Control_Allocation' */
#if AIRFRAME == 1

  /* Output and update for atomic system: '<S4>/Boat_1' */
  {
    int32_T i_j;
    real32_T tmp_j;
    uint16_T u0_j;

    /* MultiPortSwitch: '<S9>/Multiport Switch' incorporates:
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      /* Reshape: '<S9>/Reshape' incorporates:
       *  Constant: '<S10>/Disarm'
       *  Constant: '<S10>/Disarm1'
       */
      rtb_VariantMergeForOutportactua[0] = CONTROL_PARAM.THROTTLE_BIAS;
      rtb_VariantMergeForOutportactua[1] = CONTROL_PARAM.SERVO_BIAS;
      break;

     case 2:
      /* Reshape: '<S9>/Reshape' incorporates:
       *  Constant: '<S11>/Standby'
       *  Constant: '<S11>/Standby1'
       */
      rtb_VariantMergeForOutportactua[0] = CONTROL_PARAM.THROTTLE_BIAS;
      rtb_VariantMergeForOutportactua[1] = CONTROL_PARAM.SERVO_BIAS;
      break;

     case 3:
      /* DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
       *  Bias: '<S7>/Bias'
       *  Gain: '<S7>/Gain'
       */
      tmp_j = fmodf(floorf(CONTROL_PARAM.THROTTLE_SCALE * rtb_Saturation +
                           (real32_T)CONTROL_PARAM.THROTTLE_BIAS), 65536.0F);
      u0_j = (uint16_T)(tmp_j < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                        -tmp_j : (int32_T)(uint16_T)tmp_j);

      /* Saturate: '<S7>/Saturation' */
      if (u0_j > 1950) {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 1950U;
      } else if (u0_j < 1000) {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 1000U;
      } else {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = u0_j;
      }

      /* DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
       *  Bias: '<S7>/Bias1'
       *  Gain: '<S7>/Gain1'
       */
      tmp_j = fmodf(floorf(CONTROL_PARAM.SERVO_SCALE * rtb_Saturation_i +
                           (real32_T)CONTROL_PARAM.SERVO_BIAS), 65536.0F);
      u0_j = (uint16_T)(tmp_j < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                        -tmp_j : (int32_T)(uint16_T)tmp_j);

      /* Saturate: '<S7>/Saturation' */
      if (u0_j > 1950) {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 1950U;
      } else if (u0_j < 1000) {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 1000U;
      } else {
        /* Reshape: '<S9>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = u0_j;
      }
      break;

     default:
      /* Reshape: '<S9>/Reshape' incorporates:
       *  Constant: '<S10>/Disarm'
       *  Constant: '<S10>/Disarm1'
       */
      rtb_VariantMergeForOutportactua[0] = CONTROL_PARAM.THROTTLE_BIAS;
      rtb_VariantMergeForOutportactua[1] = CONTROL_PARAM.SERVO_BIAS;
      break;
    }

    /* End of MultiPortSwitch: '<S9>/Multiport Switch' */

    /* Reshape: '<S9>/Reshape' */
    for (i_j = 0; i_j < 14; i_j++) {
      rtb_VariantMergeForOutportactua[i_j + 2] = 0U;
    }
  }

#elif AIRFRAME == 2

  /* Output and update for atomic system: '<S4>/Boat_2' */
  {
    real32_T rtb_steering_cmd;
    int32_T rtb_Switch;
    uint32_T rtb_thruster_n;
    real32_T tmp_e;
    uint16_T rtb_VariantMergeForOutportact_e;

    /* MultiPortSwitch: '<S13>/Multiport Switch' incorporates:
     *  Constant: '<S15>/Disarm'
     *  Constant: '<S15>/Disarm1'
     *  Inport: '<Root>/FMS_Out'
     */
    switch (Controller_U.FMS_Out.status) {
     case 1:
      /* Reshape: '<S13>/Reshape' incorporates:
       *  Constant: '<S15>/Disarm'
       *  Constant: '<S15>/Disarm1'
       */
      rtb_VariantMergeForOutportactua[0] = (uint16_T)((uint32_T)
        CONTROL_PARAM.SERVO_BIAS + CONTROL_PARAM.THROTTLE_BIAS);
      rtb_VariantMergeForOutportactua[1] = (uint16_T)((uint32_T)
        CONTROL_PARAM.SERVO_BIAS + CONTROL_PARAM.THROTTLE_BIAS);
      break;

     case 2:
      /* Reshape: '<S13>/Reshape' incorporates:
       *  Constant: '<S16>/Standby'
       *  Constant: '<S16>/Standby1'
       */
      rtb_VariantMergeForOutportactua[0] = (uint16_T)((uint32_T)
        CONTROL_PARAM.SERVO_BIAS + CONTROL_PARAM.THROTTLE_BIAS);
      rtb_VariantMergeForOutportactua[1] = (uint16_T)((uint32_T)
        CONTROL_PARAM.SERVO_BIAS + CONTROL_PARAM.THROTTLE_BIAS);
      break;

     case 3:
      /* Gain: '<S8>/Gain1' */
      rtb_steering_cmd = CONTROL_PARAM.SERVO_SCALE * rtb_Saturation_i;

      /* Switch: '<S14>/Switch' incorporates:
       *  Constant: '<S14>/Constant'
       *  Constant: '<S14>/Constant1'
       */
      if (rtb_Saturation >= 0.0F) {
        rtb_Switch = 1;
      } else {
        rtb_Switch = -1;
      }

      /* End of Switch: '<S14>/Switch' */

      /* Gain: '<S8>/Gain' */
      tmp_e = fmodf(floorf(CONTROL_PARAM.THROTTLE_SCALE * rtb_Saturation),
                    4.2949673E+9F);

      /* Bias: '<S8>/Bias' incorporates:
       *  Gain: '<S8>/Gain'
       */
      rtb_thruster_n = (tmp_e < 0.0F ? (uint32_T)-(int32_T)(uint32_T)-tmp_e :
                        (uint32_T)tmp_e) + CONTROL_PARAM.THROTTLE_BIAS;

      /* DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
       *  Gain: '<S8>/Gain2'
       *  Product: '<S8>/Multiply'
       */
      tmp_e = fmodf(floorf((real32_T)rtb_Switch * -rtb_steering_cmd),
                    4.2949673E+9F);

      /* DataTypeConversion: '<S8>/Data Type Conversion' incorporates:
       *  Bias: '<S8>/Bias1'
       *  DataTypeConversion: '<S8>/Data Type Conversion1'
       *  SignalConversion: '<S8>/TmpSignal ConversionAtAddInport1'
       *  Sum: '<S8>/Add'
       */
      rtb_VariantMergeForOutportact_e = (uint16_T)(((tmp_e < 0.0F ? (uint32_T)
        -(int32_T)(uint32_T)-tmp_e : (uint32_T)tmp_e) + rtb_thruster_n) +
        CONTROL_PARAM.SERVO_BIAS);

      /* Saturate: '<S8>/Saturation' */
      if (rtb_VariantMergeForOutportact_e > 2000) {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 2000U;
      } else if (rtb_VariantMergeForOutportact_e < 1000) {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = 1000U;
      } else {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[0] = rtb_VariantMergeForOutportact_e;
      }

      /* DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
       *  Product: '<S8>/Multiply'
       */
      tmp_e = fmodf(floorf((real32_T)rtb_Switch * rtb_steering_cmd),
                    4.2949673E+9F);

      /* DataTypeConversion: '<S8>/Data Type Conversion' incorporates:
       *  Bias: '<S8>/Bias1'
       *  DataTypeConversion: '<S8>/Data Type Conversion1'
       *  SignalConversion: '<S8>/TmpSignal ConversionAtAddInport1'
       *  Sum: '<S8>/Add'
       */
      rtb_VariantMergeForOutportact_e = (uint16_T)(((tmp_e < 0.0F ? (uint32_T)
        -(int32_T)(uint32_T)-tmp_e : (uint32_T)tmp_e) + rtb_thruster_n) +
        CONTROL_PARAM.SERVO_BIAS);

      /* Saturate: '<S8>/Saturation' */
      if (rtb_VariantMergeForOutportact_e > 2000) {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 2000U;
      } else if (rtb_VariantMergeForOutportact_e < 1000) {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = 1000U;
      } else {
        /* Reshape: '<S13>/Reshape' */
        rtb_VariantMergeForOutportactua[1] = rtb_VariantMergeForOutportact_e;
      }
      break;

     default:
      rtb_VariantMergeForOutportact_e = (uint16_T)((uint32_T)
        CONTROL_PARAM.SERVO_BIAS + CONTROL_PARAM.THROTTLE_BIAS);

      /* Reshape: '<S13>/Reshape' incorporates:
       *  Constant: '<S15>/Disarm'
       *  Constant: '<S15>/Disarm1'
       */
      rtb_VariantMergeForOutportactua[0] = rtb_VariantMergeForOutportact_e;
      rtb_VariantMergeForOutportactua[1] = rtb_VariantMergeForOutportact_e;
      break;
    }

    /* End of MultiPortSwitch: '<S13>/Multiport Switch' */

    /* Reshape: '<S13>/Reshape' */
    for (rtb_Switch = 0; rtb_Switch < 14; rtb_Switch++) {
      rtb_VariantMergeForOutportactua[rtb_Switch + 2] = 0U;
    }
  }

#endif

  /* End of Outputs for SubSystem: '<S2>/Control_Allocation' */

  /* Outport: '<Root>/Control_Out' incorporates:
   *  BusAssignment: '<S1>/Bus Assignment'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   */
  Controller_Y.Control_Out.timestamp =
    Controller_DW.DiscreteTimeIntegrator_DSTATE_i;
  for (i = 0; i < 16; i++) {
    Controller_Y.Control_Out.actuator_cmd[i] = rtb_VariantMergeForOutportactua[i];
  }

  /* End of Outport: '<Root>/Control_Out' */

  /* Update for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S24>/gain1'
   *  Inport: '<Root>/FMS_Out'
   *  Product: '<S24>/Multiply'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE += CONTROL_PARAM.VEL_I * rtb_u_err
    * 0.01F;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE >= CONTROL_PARAM.VEL_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE <= CONTROL_PARAM.VEL_I_MIN)
    {
      Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* End of Update for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/FMS_Out'
   */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;
  Controller_DW.DiscreteTimeIntegrator1_DSTATE += 0.01F * rtb_Gain;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)
    (Controller_U.FMS_Out.reset > 0);

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE_i += CONTROL_EXPORT.period;
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

  /* InitializeConditions for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */
  Controller_DW.DiscreteTimeIntegrator_DSTATE = Controller_ConstB.Constant;
  if (Controller_DW.DiscreteTimeIntegrator_DSTATE >= CONTROL_PARAM.VEL_I_MAX) {
    Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MAX;
  } else {
    if (Controller_DW.DiscreteTimeIntegrator_DSTATE <= CONTROL_PARAM.VEL_I_MIN)
    {
      Controller_DW.DiscreteTimeIntegrator_DSTATE = CONTROL_PARAM.VEL_I_MIN;
    }
  }

  Controller_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* End of InitializeConditions for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */

  /* InitializeConditions for DiscreteIntegrator: '<S26>/Discrete-Time Integrator1' */
  Controller_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
  Controller_DW.DiscreteTimeIntegrator1_PrevRes = 0;
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

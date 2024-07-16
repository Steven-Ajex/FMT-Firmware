/*
 * File: FMS.c
 *
 * Code generated for Simulink model 'FMS'.
 *
 * Model version                  : 1.2126
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Thu Jul 11 16:23:36 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "FMS.h"
#include "FMS_private.h"

/* Named constants for Chart: '<S425>/Motion Status' */
#define FMS_IN_Brake                   ((uint8_T)1U)
#define FMS_IN_Hold                    ((uint8_T)2U)
#define FMS_IN_Move                    ((uint8_T)3U)
#define FMS_IN_NO_ACTIVE_CHILD         ((uint8_T)0U)

/* Named constants for Chart: '<S435>/Motion State' */
#define FMS_IN_Brake_c                 ((uint8_T)1U)
#define FMS_IN_Hold_c                  ((uint8_T)2U)
#define FMS_IN_Move_o                  ((uint8_T)3U)
#define FMS_IN_NO_ACTIVE_CHILD_g       ((uint8_T)0U)

/* Named constants for Chart: '<S137>/Motion State' */
#define FMS_IN_Brake_f                 ((uint8_T)1U)
#define FMS_IN_Hold_j                  ((uint8_T)2U)
#define FMS_IN_Move_o4                 ((uint8_T)3U)
#define FMS_IN_NO_ACTIVE_CHILD_b       ((uint8_T)0U)

/* Named constants for Chart: '<S355>/Motion State' */
#define FMS_IN_Brake_o                 ((uint8_T)1U)
#define FMS_IN_Hold_d                  ((uint8_T)2U)
#define FMS_IN_Move_n                  ((uint8_T)3U)
#define FMS_IN_NO_ACTIVE_CHILD_h       ((uint8_T)0U)

/* Named constants for Chart: '<Root>/FMS State Machine' */
#define FMS_IN_Acro                    ((uint8_T)1U)
#define FMS_IN_Altitude                ((uint8_T)2U)
#define FMS_IN_Arm                     ((uint8_T)1U)
#define FMS_IN_Assist                  ((uint8_T)1U)
#define FMS_IN_Auto                    ((uint8_T)2U)
#define FMS_IN_Check                   ((uint8_T)1U)
#define FMS_IN_Connect                 ((uint8_T)1U)
#define FMS_IN_Disarm                  ((uint8_T)2U)
#define FMS_IN_Disarming               ((uint8_T)1U)
#define FMS_IN_Hold_h                  ((uint8_T)1U)
#define FMS_IN_Idle                    ((uint8_T)3U)
#define FMS_IN_InValidManualMode       ((uint8_T)1U)
#define FMS_IN_InvalidArmMode          ((uint8_T)3U)
#define FMS_IN_InvalidAssistMode       ((uint8_T)3U)
#define FMS_IN_InvalidAutoMode         ((uint8_T)1U)
#define FMS_IN_Land                    ((uint8_T)2U)
#define FMS_IN_Land_j                  ((uint8_T)3U)
#define FMS_IN_Listen                  ((uint8_T)2U)
#define FMS_IN_Loiter                  ((uint8_T)1U)
#define FMS_IN_Loiter_p                ((uint8_T)4U)
#define FMS_IN_Lost                    ((uint8_T)2U)
#define FMS_IN_Manual                  ((uint8_T)4U)
#define FMS_IN_Manual_g                ((uint8_T)2U)
#define FMS_IN_Mission                 ((uint8_T)2U)
#define FMS_IN_NextWP                  ((uint8_T)5U)
#define FMS_IN_Offboard                ((uint8_T)3U)
#define FMS_IN_Position                ((uint8_T)4U)
#define FMS_IN_Return                  ((uint8_T)3U)
#define FMS_IN_Return_h                ((uint8_T)6U)
#define FMS_IN_Run                     ((uint8_T)2U)
#define FMS_IN_Send                    ((uint8_T)3U)
#define FMS_IN_SetSpeed                ((uint8_T)7U)
#define FMS_IN_Stabilize               ((uint8_T)5U)
#define FMS_IN_Standby                 ((uint8_T)3U)
#define FMS_IN_SubMode                 ((uint8_T)5U)
#define FMS_IN_Takeoff                 ((uint8_T)4U)
#define FMS_IN_Takeoff_d               ((uint8_T)8U)
#define FMS_IN_Waypoint                ((uint8_T)9U)
#define FMS_event_DisarmEvent          (0)

/* Named constants for Chart: '<Root>/SafeMode' */
#define FMS_IN_Manual_e                ((uint8_T)3U)
#define FMS_IN_Mission_g               ((uint8_T)4U)
#define FMS_IN_Offboard_p              ((uint8_T)5U)
#define FMS_IN_Other                   ((uint8_T)6U)
#define FMS_IN_Position_f              ((uint8_T)7U)
#define FMS_IN_Stabilize_j             ((uint8_T)8U)

const FMS_Out_Bus FMS_rtZFMS_Out_Bus = {
  0U,                                  /* timestamp */
  0.0F,                                /* p_cmd */
  0.0F,                                /* q_cmd */
  0.0F,                                /* r_cmd */
  0.0F,                                /* phi_cmd */
  0.0F,                                /* theta_cmd */
  0.0F,                                /* psi_rate_cmd */
  0.0F,                                /* u_cmd */
  0.0F,                                /* v_cmd */
  0.0F,                                /* w_cmd */
  0.0F,                                /* ax_cmd */
  0.0F,                                /* ay_cmd */
  0.0F,                                /* az_cmd */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* actuator_cmd */
  0U,                                  /* throttle_cmd */
  0U,                                  /* cmd_mask */
  0U,                                  /* status */
  0U,                                  /* state */
  0U,                                  /* ctrl_mode */
  0U,                                  /* mode */
  0U,                                  /* reset */
  0U,                                  /* wp_consume */
  0U,                                  /* wp_current */
  0U,                                  /* reserved */

  {
    0.0F, 0.0F, 0.0F, 0.0F }
  /* home */
} ;                                    /* FMS_Out_Bus ground */

/* Exported block parameters */
struct_C5XIQzgZOXj5pNdNFngVXC FMS_PARAM = {
  0.15F,
  0.15F,
  0.1F,
  0.1F,
  1.2F,
  1.5F,
  2.0F,
  1.5F,
  2.0F,
  1.04719806F,
  0.523599F,
  15.0F,
  5.0F,
  5.0F,
  0.5F,
  0.5F,
  0.5F,
  0.7F,
  0.2F,
  120U,
  1U,
  1550U
} ;                                    /* Variable: FMS_PARAM
                                        * Referenced by:
                                        *   '<Root>/ACCEPT_R'
                                        *   '<S3>/Constant1'
                                        *   '<S27>/Constant'
                                        *   '<S477>/Constant'
                                        *   '<S338>/L1'
                                        *   '<S48>/Gain'
                                        *   '<S48>/Gain1'
                                        *   '<S127>/Gain'
                                        *   '<S127>/Gain1'
                                        *   '<S129>/Constant'
                                        *   '<S153>/L1'
                                        *   '<S234>/Saturation'
                                        *   '<S235>/Saturation1'
                                        *   '<S235>/Saturation2'
                                        *   '<S235>/Saturation3'
                                        *   '<S379>/Land_Speed'
                                        *   '<S380>/Saturation1'
                                        *   '<S466>/Takeoff_Speed'
                                        *   '<S467>/Gain2'
                                        *   '<S467>/Saturation1'
                                        *   '<S50>/Saturation1'
                                        *   '<S62>/Dead Zone'
                                        *   '<S62>/Gain'
                                        *   '<S63>/Dead Zone'
                                        *   '<S63>/Gain'
                                        *   '<S68>/Saturation'
                                        *   '<S84>/Saturation1'
                                        *   '<S96>/Saturation'
                                        *   '<S109>/Saturation1'
                                        *   '<S130>/Dead Zone'
                                        *   '<S130>/Gain'
                                        *   '<S131>/Dead Zone'
                                        *   '<S131>/Gain'
                                        *   '<S136>/Saturation'
                                        *   '<S288>/Gain2'
                                        *   '<S295>/Gain1'
                                        *   '<S295>/Gain2'
                                        *   '<S344>/Saturation1'
                                        *   '<S354>/Saturation'
                                        *   '<S366>/Saturation1'
                                        *   '<S381>/Constant'
                                        *   '<S384>/Gain2'
                                        *   '<S406>/Gain'
                                        *   '<S406>/Saturation1'
                                        *   '<S409>/Constant'
                                        *   '<S409>/vel'
                                        *   '<S409>/Switch'
                                        *   '<S53>/Gain2'
                                        *   '<S54>/Gain1'
                                        *   '<S60>/Constant'
                                        *   '<S71>/Gain2'
                                        *   '<S72>/Gain1'
                                        *   '<S79>/Constant'
                                        *   '<S87>/Gain2'
                                        *   '<S88>/Gain1'
                                        *   '<S94>/Constant'
                                        *   '<S99>/Gain2'
                                        *   '<S100>/Gain1'
                                        *   '<S107>/Constant'
                                        *   '<S112>/Gain2'
                                        *   '<S113>/Gain6'
                                        *   '<S124>/Constant'
                                        *   '<S125>/Constant'
                                        *   '<S139>/Gain2'
                                        *   '<S140>/Gain1'
                                        *   '<S147>/Constant'
                                        *   '<S172>/Gain'
                                        *   '<S172>/Saturation1'
                                        *   '<S175>/Constant'
                                        *   '<S175>/vel'
                                        *   '<S175>/Switch'
                                        *   '<S347>/Gain2'
                                        *   '<S348>/Gain1'
                                        *   '<S357>/Gain2'
                                        *   '<S358>/Gain1'
                                        *   '<S369>/Gain2'
                                        *   '<S370>/Gain6'
                                        *   '<S393>/Gain2'
                                        *   '<S393>/Saturation'
                                        *   '<S394>/Integrator'
                                        *   '<S56>/Land_Speed'
                                        *   '<S57>/Constant'
                                        *   '<S59>/Dead Zone'
                                        *   '<S59>/Gain'
                                        *   '<S76>/Dead Zone'
                                        *   '<S76>/Gain'
                                        *   '<S90>/Land_Speed'
                                        *   '<S91>/Constant'
                                        *   '<S93>/Dead Zone'
                                        *   '<S93>/Gain'
                                        *   '<S104>/Dead Zone'
                                        *   '<S104>/Gain'
                                        *   '<S118>/Dead Zone'
                                        *   '<S118>/Gain'
                                        *   '<S119>/Dead Zone'
                                        *   '<S119>/Gain'
                                        *   '<S144>/Dead Zone'
                                        *   '<S144>/Gain'
                                        *   '<S159>/Gain2'
                                        *   '<S159>/Saturation'
                                        *   '<S160>/Integrator'
                                        *   '<S350>/Dead Zone'
                                        *   '<S350>/Gain'
                                        *   '<S362>/Dead Zone'
                                        *   '<S362>/Gain'
                                        *   '<S374>/Dead Zone'
                                        *   '<S374>/Gain'
                                        *   '<S375>/Dead Zone'
                                        *   '<S375>/Gain'
                                        *   '<S424>/Saturation1'
                                        *   '<S434>/Saturation1'
                                        *   '<S190>/Saturation1'
                                        *   '<S200>/Saturation1'
                                        *   '<S427>/Gain2'
                                        *   '<S428>/Gain1'
                                        *   '<S437>/Gain2'
                                        *   '<S438>/Gain6'
                                        *   '<S193>/Gain2'
                                        *   '<S194>/Gain1'
                                        *   '<S203>/Gain2'
                                        *   '<S204>/Gain6'
                                        *   '<S430>/Dead Zone'
                                        *   '<S430>/Gain'
                                        *   '<S442>/Dead Zone'
                                        *   '<S442>/Gain'
                                        *   '<S443>/Dead Zone'
                                        *   '<S443>/Gain'
                                        *   '<S196>/Dead Zone'
                                        *   '<S196>/Gain'
                                        *   '<S208>/Dead Zone'
                                        *   '<S208>/Gain'
                                        *   '<S209>/Dead Zone'
                                        *   '<S209>/Gain'
                                        */

struct_c5vJTyPsTPqsN7zOhDCciB FMS_EXPORT = {
  4U,

  { 82, 111, 99, 107, 101, 116, 32, 70, 77, 83, 32, 118, 48, 46, 48, 46, 49, 0 }
} ;                                    /* Variable: FMS_EXPORT
                                        * Referenced by:
                                        *   '<S1>/Constant'
                                        *   '<S13>/Constant1'
                                        *   '<S475>/Constant'
                                        */

/* Block signals (default storage) */
B_FMS_T FMS_B;

/* Block states (default storage) */
DW_FMS_T FMS_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_FMS_T FMS_PrevZCX;

/* External inputs (root inport signals with default storage) */
ExtU_FMS_T FMS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FMS_T FMS_Y;

/* Real-time model */
RT_MODEL_FMS_T FMS_M_;
RT_MODEL_FMS_T *const FMS_M = &FMS_M_;

/* Forward declaration for local functions */
static void FMS_Stabilize(void);
static void FMS_Acro(void);
static void FMS_sf_msg_send_M(void);
static boolean_T FMS_CheckCmdValid(FMS_Cmd cmd_in, PilotMode mode_in, uint32_T
  ins_flag);
static boolean_T FMS_BottomRight(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle);
static boolean_T FMS_BottomLeft(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle);
static boolean_T FMS_sf_msg_pop_M(void);
static real32_T FMS_norm(const real32_T x[2]);
static void FMS_Mission(void);
static real_T FMS_getArmMode(PilotMode pilotMode);
static void FMS_enter_internal_Auto(void);
static void FMS_enter_internal_Arm(void);
static void FMS_SubMode(void);
static void FMS_exit_internal_Arm(void);
static void FMS_Arm(void);
static real_T FMS_ManualArmEvent(real32_T pilot_cmd_stick_throttle, uint32_T
  pilot_cmd_mode);
static void FMS_Vehicle(void);
static void FMS_c11_FMS(void);
static void FMS_sf_msg_discard_M(void);
static void initialize_msg_local_queues_for(void);

/*
 * System initialize for action system:
 *    '<S424>/Hold Control'
 *    '<S344>/Hold Control'
 *    '<S190>/Hold Control'
 *    '<S50>/Hold Control'
 *    '<S84>/Hold Control'
 */
void FMS_HoldControl_Init(DW_HoldControl_FMS_T *localDW)
{
  /* InitializeConditions for Delay: '<S427>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * System reset for action system:
 *    '<S424>/Hold Control'
 *    '<S344>/Hold Control'
 *    '<S190>/Hold Control'
 *    '<S50>/Hold Control'
 *    '<S84>/Hold Control'
 */
void FMS_HoldControl_Reset(DW_HoldControl_FMS_T *localDW)
{
  /* InitializeConditions for Delay: '<S427>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * Output and update for action system:
 *    '<S424>/Hold Control'
 *    '<S344>/Hold Control'
 *    '<S190>/Hold Control'
 *    '<S50>/Hold Control'
 *    '<S84>/Hold Control'
 */
void FMS_HoldControl(real32_T rtu_FMS_In, real32_T *rty_w_cmd_mPs,
                     DW_HoldControl_FMS_T *localDW)
{
  /* Delay: '<S427>/Delay' incorporates:
   *  Gain: '<S429>/Gain'
   */
  if (localDW->icLoad != 0) {
    localDW->Delay_DSTATE = -rtu_FMS_In;
  }

  /* Gain: '<S427>/Gain2' incorporates:
   *  Delay: '<S427>/Delay'
   *  Gain: '<S429>/Gain'
   *  Sum: '<S427>/Sum'
   */
  *rty_w_cmd_mPs = (localDW->Delay_DSTATE - (-rtu_FMS_In)) * FMS_PARAM.Z_P;

  /* Update for Delay: '<S427>/Delay' */
  localDW->icLoad = 0U;
}

/*
 * Output and update for action system:
 *    '<S424>/Brake Control'
 *    '<S344>/Brake Control'
 *    '<S354>/Brake Control'
 *    '<S190>/Brake Control'
 *    '<S136>/Brake Control'
 *    '<S50>/Brake Control'
 *    '<S68>/Brake Control'
 *    '<S84>/Brake Control'
 *    '<S96>/Brake Control'
 */
void FMS_BrakeControl(real32_T *rty_psi_rate_cmd_radPs)
{
  /* SignalConversion: '<S426>/OutportBuffer_InsertedFor_psi_rate_cmd_radPs_at_inport_0' incorporates:
   *  Constant: '<S426>/Brake Speed'
   */
  *rty_psi_rate_cmd_radPs = 0.0F;
}

/*
 * System initialize for action system:
 *    '<S424>/Move Control'
 *    '<S344>/Move Control'
 *    '<S190>/Move Control'
 */
void FMS_MoveControl_Init(DW_MoveControl_FMS_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S431>/Integrator1' */
  localDW->Integrator1_DSTATE = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S431>/Integrator' */
  localDW->Integrator_DSTATE = 0.0F;
}

/*
 * System reset for action system:
 *    '<S424>/Move Control'
 *    '<S344>/Move Control'
 *    '<S190>/Move Control'
 */
void FMS_MoveControl_Reset(DW_MoveControl_FMS_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S431>/Integrator1' */
  localDW->Integrator1_DSTATE = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S431>/Integrator' */
  localDW->Integrator_DSTATE = 0.0F;
}

/*
 * Output and update for action system:
 *    '<S424>/Move Control'
 *    '<S344>/Move Control'
 *    '<S190>/Move Control'
 */
void FMS_MoveControl(real32_T rtu_FMS_In, real32_T *rty_w_cmd_mPs, const
                     ConstB_MoveControl_FMS_T *localC, DW_MoveControl_FMS_T
                     *localDW)
{
  real32_T rtb_Add3_mj;
  real32_T rtb_Subtract3_k;
  real32_T rtb_a_g;
  real32_T rtb_Add4_k;
  real32_T rtb_a_m;

  /* Product: '<S432>/Multiply1' incorporates:
   *  Constant: '<S432>/const1'
   *  DiscreteIntegrator: '<S431>/Integrator'
   */
  rtb_Add3_mj = localDW->Integrator_DSTATE * 0.05F;

  /* DeadZone: '<S430>/Dead Zone' */
  if (rtu_FMS_In > FMS_PARAM.THROTTLE_DZ) {
    rtb_a_g = rtu_FMS_In - FMS_PARAM.THROTTLE_DZ;
  } else if (rtu_FMS_In >= -FMS_PARAM.THROTTLE_DZ) {
    rtb_a_g = 0.0F;
  } else {
    rtb_a_g = rtu_FMS_In - (-FMS_PARAM.THROTTLE_DZ);
  }

  /* End of DeadZone: '<S430>/Dead Zone' */

  /* Sum: '<S432>/Add' incorporates:
   *  DiscreteIntegrator: '<S431>/Integrator1'
   *  Gain: '<S428>/Gain1'
   *  Gain: '<S430>/Gain'
   *  Sum: '<S431>/Subtract'
   */
  rtb_Subtract3_k = (localDW->Integrator1_DSTATE - 1.0F / (1.0F -
    FMS_PARAM.THROTTLE_DZ) * rtb_a_g * -FMS_PARAM.VEL_Z_LIM) + rtb_Add3_mj;

  /* Signum: '<S432>/Sign' */
  if (rtb_Subtract3_k < 0.0F) {
    rtb_a_g = -1.0F;
  } else if (rtb_Subtract3_k > 0.0F) {
    rtb_a_g = 1.0F;
  } else {
    rtb_a_g = rtb_Subtract3_k;
  }

  /* End of Signum: '<S432>/Sign' */

  /* Sum: '<S432>/Add2' incorporates:
   *  Abs: '<S432>/Abs'
   *  Gain: '<S432>/Gain'
   *  Gain: '<S432>/Gain1'
   *  Product: '<S432>/Multiply2'
   *  Product: '<S432>/Multiply3'
   *  Sqrt: '<S432>/Sqrt'
   *  Sum: '<S432>/Add1'
   *  Sum: '<S432>/Subtract'
   */
  rtb_a_g = (sqrtf((8.0F * fabsf(rtb_Subtract3_k) + localC->d) * localC->d) -
             localC->d) * 0.5F * rtb_a_g + rtb_Add3_mj;

  /* Sum: '<S432>/Add4' */
  rtb_Add4_k = (rtb_Subtract3_k - rtb_a_g) + rtb_Add3_mj;

  /* Sum: '<S432>/Add3' */
  rtb_Add3_mj = rtb_Subtract3_k + localC->d;

  /* Sum: '<S432>/Subtract1' */
  rtb_Subtract3_k -= localC->d;

  /* Signum: '<S432>/Sign1' */
  if (rtb_Add3_mj < 0.0F) {
    rtb_Add3_mj = -1.0F;
  } else {
    if (rtb_Add3_mj > 0.0F) {
      rtb_Add3_mj = 1.0F;
    }
  }

  /* End of Signum: '<S432>/Sign1' */

  /* Signum: '<S432>/Sign2' */
  if (rtb_Subtract3_k < 0.0F) {
    rtb_Subtract3_k = -1.0F;
  } else {
    if (rtb_Subtract3_k > 0.0F) {
      rtb_Subtract3_k = 1.0F;
    }
  }

  /* End of Signum: '<S432>/Sign2' */

  /* Sum: '<S432>/Add5' incorporates:
   *  Gain: '<S432>/Gain2'
   *  Product: '<S432>/Multiply4'
   *  Sum: '<S432>/Subtract2'
   */
  rtb_a_g += (rtb_Add3_mj - rtb_Subtract3_k) * 0.5F * rtb_Add4_k;

  /* SignalConversion: '<S428>/OutportBufferForw_cmd_mPs' incorporates:
   *  DiscreteIntegrator: '<S431>/Integrator1'
   */
  *rty_w_cmd_mPs = localDW->Integrator1_DSTATE;

  /* Update for DiscreteIntegrator: '<S431>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S431>/Integrator'
   */
  localDW->Integrator1_DSTATE += 0.004F * localDW->Integrator_DSTATE;

  /* Sum: '<S432>/Add6' */
  rtb_Add3_mj = rtb_a_g + localC->d;

  /* Sum: '<S432>/Subtract3' */
  rtb_Subtract3_k = rtb_a_g - localC->d;

  /* Signum: '<S432>/Sign5' */
  if (rtb_a_g < 0.0F) {
    rtb_Add4_k = -1.0F;
  } else if (rtb_a_g > 0.0F) {
    rtb_Add4_k = 1.0F;
  } else {
    rtb_Add4_k = rtb_a_g;
  }

  /* End of Signum: '<S432>/Sign5' */

  /* Signum: '<S432>/Sign3' */
  if (rtb_Add3_mj < 0.0F) {
    rtb_Add3_mj = -1.0F;
  } else {
    if (rtb_Add3_mj > 0.0F) {
      rtb_Add3_mj = 1.0F;
    }
  }

  /* End of Signum: '<S432>/Sign3' */

  /* Signum: '<S432>/Sign4' */
  if (rtb_Subtract3_k < 0.0F) {
    rtb_Subtract3_k = -1.0F;
  } else {
    if (rtb_Subtract3_k > 0.0F) {
      rtb_Subtract3_k = 1.0F;
    }
  }

  /* End of Signum: '<S432>/Sign4' */

  /* Signum: '<S432>/Sign6' */
  if (rtb_a_g < 0.0F) {
    rtb_a_m = -1.0F;
  } else if (rtb_a_g > 0.0F) {
    rtb_a_m = 1.0F;
  } else {
    rtb_a_m = rtb_a_g;
  }

  /* End of Signum: '<S432>/Sign6' */

  /* Update for DiscreteIntegrator: '<S431>/Integrator' incorporates:
   *  Constant: '<S432>/const'
   *  Gain: '<S432>/Gain3'
   *  Product: '<S432>/Divide'
   *  Product: '<S432>/Multiply5'
   *  Product: '<S432>/Multiply6'
   *  Sum: '<S432>/Subtract4'
   *  Sum: '<S432>/Subtract5'
   *  Sum: '<S432>/Subtract6'
   */
  localDW->Integrator_DSTATE += ((rtb_a_g / localC->d - rtb_Add4_k) *
    localC->Gain4 * ((rtb_Add3_mj - rtb_Subtract3_k) * 0.5F) - rtb_a_m * 78.448F)
    * 0.004F;
}

/*
 * System initialize for atomic system:
 *    '<S425>/Motion Status'
 *    '<S345>/Motion Status'
 *    '<S191>/Motion Status'
 */
void FMS_MotionStatus_Init(DW_MotionStatus_FMS_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c10_FMS = 0U;
  localDW->is_c10_FMS = FMS_IN_NO_ACTIVE_CHILD;
}

/*
 * System reset for atomic system:
 *    '<S425>/Motion Status'
 *    '<S345>/Motion Status'
 *    '<S191>/Motion Status'
 */
void FMS_MotionStatus_Reset(DW_MotionStatus_FMS_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c10_FMS = 0U;
  localDW->is_c10_FMS = FMS_IN_NO_ACTIVE_CHILD;
}

/*
 * Output and update for atomic system:
 *    '<S425>/Motion Status'
 *    '<S345>/Motion Status'
 *    '<S191>/Motion Status'
 */
void FMS_MotionStatus(real32_T rtu_motion_req, real32_T rtu_speed, MotionState
                      *rty_state, DW_MotionStatus_FMS_T *localDW)
{
  /* Chart: '<S425>/Motion Status' */
  if (localDW->temporalCounter_i1 < 511U) {
    localDW->temporalCounter_i1++;
  }

  if (localDW->is_active_c10_FMS == 0U) {
    localDW->is_active_c10_FMS = 1U;
    localDW->is_c10_FMS = FMS_IN_Move;
    *rty_state = MotionState_Move;
  } else {
    switch (localDW->is_c10_FMS) {
     case FMS_IN_Brake:
      *rty_state = MotionState_Brake;
      if ((rtu_speed <= 0.15) || (localDW->temporalCounter_i1 >= 375U)) {
        localDW->is_c10_FMS = FMS_IN_Hold;
        *rty_state = MotionState_Hold;
      } else {
        if (rtu_motion_req == 1.0F) {
          localDW->is_c10_FMS = FMS_IN_Move;
          *rty_state = MotionState_Move;
        }
      }
      break;

     case FMS_IN_Hold:
      *rty_state = MotionState_Hold;
      if (rtu_motion_req == 1.0F) {
        localDW->is_c10_FMS = FMS_IN_Move;
        *rty_state = MotionState_Move;
      }
      break;

     default:
      *rty_state = MotionState_Move;
      if (rtu_motion_req == 0.0F) {
        localDW->is_c10_FMS = FMS_IN_Brake;
        localDW->temporalCounter_i1 = 0U;
        *rty_state = MotionState_Brake;
      }
      break;
    }
  }

  /* End of Chart: '<S425>/Motion Status' */
}

/*
 * System initialize for action system:
 *    '<S434>/Hold Control'
 *    '<S366>/Hold Control'
 *    '<S200>/Hold Control'
 *    '<S109>/Hold Control'
 */
void FMS_HoldControl_c_Init(DW_HoldControl_FMS_j_T *localDW)
{
  /* InitializeConditions for Delay: '<S437>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * System reset for action system:
 *    '<S434>/Hold Control'
 *    '<S366>/Hold Control'
 *    '<S200>/Hold Control'
 *    '<S109>/Hold Control'
 */
void FMS_HoldControl_k_Reset(DW_HoldControl_FMS_j_T *localDW)
{
  /* InitializeConditions for Delay: '<S437>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * Output and update for action system:
 *    '<S434>/Hold Control'
 *    '<S366>/Hold Control'
 *    '<S200>/Hold Control'
 *    '<S109>/Hold Control'
 */
void FMS_HoldControl_m(real32_T rtu_FMS_In, real32_T rtu_FMS_In_o, real32_T
  rtu_FMS_In_f, real32_T rty_uv_cmd_mPs[2], const ConstB_HoldControl_FMS_f_T
  *localC, DW_HoldControl_FMS_j_T *localDW)
{
  real32_T rtb_VectorConcatenate_d4[9];
  real32_T rtb_VectorConcatenate_bg[3];
  int32_T i;
  real32_T rtb_VectorConcatenate_bg_tmp;
  real32_T rtb_VectorConcatenate_bg_tmp_0;

  /* Delay: '<S437>/Delay' incorporates:
   *  SignalConversion: '<S437>/TmpSignal ConversionAtDelayInport2'
   */
  if (localDW->icLoad != 0) {
    localDW->Delay_DSTATE[0] = rtu_FMS_In;
    localDW->Delay_DSTATE[1] = rtu_FMS_In_o;
  }

  /* Trigonometry: '<S441>/Trigonometric Function1' incorporates:
   *  Gain: '<S440>/Gain'
   *  Trigonometry: '<S441>/Trigonometric Function3'
   */
  rtb_VectorConcatenate_bg_tmp_0 = arm_cos_f32(-rtu_FMS_In_f);
  rtb_VectorConcatenate_d4[0] = rtb_VectorConcatenate_bg_tmp_0;

  /* Trigonometry: '<S441>/Trigonometric Function' incorporates:
   *  Gain: '<S440>/Gain'
   *  Trigonometry: '<S441>/Trigonometric Function2'
   */
  rtb_VectorConcatenate_bg_tmp = arm_sin_f32(-rtu_FMS_In_f);
  rtb_VectorConcatenate_d4[1] = rtb_VectorConcatenate_bg_tmp;

  /* SignalConversion: '<S441>/ConcatBufferAtVector Concatenate1In3' incorporates:
   *  Constant: '<S441>/Constant3'
   */
  rtb_VectorConcatenate_d4[2] = 0.0F;

  /* Gain: '<S441>/Gain' */
  rtb_VectorConcatenate_d4[3] = -rtb_VectorConcatenate_bg_tmp;

  /* Trigonometry: '<S441>/Trigonometric Function3' */
  rtb_VectorConcatenate_d4[4] = rtb_VectorConcatenate_bg_tmp_0;

  /* SignalConversion: '<S441>/ConcatBufferAtVector Concatenate2In3' incorporates:
   *  Constant: '<S441>/Constant4'
   */
  rtb_VectorConcatenate_d4[5] = 0.0F;

  /* SignalConversion: '<S441>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_VectorConcatenate_d4[6] = localC->VectorConcatenate3[0];
  rtb_VectorConcatenate_d4[7] = localC->VectorConcatenate3[1];
  rtb_VectorConcatenate_d4[8] = localC->VectorConcatenate3[2];

  /* SignalConversion: '<S437>/TmpSignal ConversionAtMultiplyInport2' incorporates:
   *  Delay: '<S437>/Delay'
   *  SignalConversion: '<S437>/TmpSignal ConversionAtDelayInport2'
   *  Sum: '<S437>/Sum'
   */
  rtb_VectorConcatenate_bg_tmp_0 = localDW->Delay_DSTATE[0] - rtu_FMS_In;
  rtb_VectorConcatenate_bg_tmp = localDW->Delay_DSTATE[1] - rtu_FMS_In_o;

  /* Product: '<S437>/Multiply' incorporates:
   *  SignalConversion: '<S437>/TmpSignal ConversionAtMultiplyInport2'
   */
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_bg[i] = rtb_VectorConcatenate_d4[i + 3] *
      rtb_VectorConcatenate_bg_tmp + rtb_VectorConcatenate_d4[i] *
      rtb_VectorConcatenate_bg_tmp_0;
  }

  /* End of Product: '<S437>/Multiply' */

  /* Gain: '<S437>/Gain2' */
  rty_uv_cmd_mPs[0] = FMS_PARAM.XY_P * rtb_VectorConcatenate_bg[0];
  rty_uv_cmd_mPs[1] = FMS_PARAM.XY_P * rtb_VectorConcatenate_bg[1];

  /* Update for Delay: '<S437>/Delay' */
  localDW->icLoad = 0U;
}

/*
 * Output and update for action system:
 *    '<S434>/Brake Control'
 *    '<S366>/Brake Control'
 *    '<S200>/Brake Control'
 *    '<S109>/Brake Control'
 */
void FMS_BrakeControl_h(real32_T rty_uv_cmd_mPs[2])
{
  /* SignalConversion: '<S436>/OutportBuffer_InsertedFor_uv_cmd_mPs_at_inport_0' */
  rty_uv_cmd_mPs[0] = 0.0F;
  rty_uv_cmd_mPs[1] = 0.0F;
}

/*
 * System initialize for action system:
 *    '<S434>/Move Control'
 *    '<S366>/Move Control'
 *    '<S200>/Move Control'
 */
void FMS_MoveControl_l_Init(DW_MoveControl_FMS_f_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator1' */
  localDW->Integrator1_DSTATE[0] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator' */
  localDW->Integrator_DSTATE[0] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator1' */
  localDW->Integrator1_DSTATE[1] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator' */
  localDW->Integrator_DSTATE[1] = 0.0F;
}

/*
 * System reset for action system:
 *    '<S434>/Move Control'
 *    '<S366>/Move Control'
 *    '<S200>/Move Control'
 */
void FMS_MoveControl_i_Reset(DW_MoveControl_FMS_f_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator1' */
  localDW->Integrator1_DSTATE[0] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator' */
  localDW->Integrator_DSTATE[0] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator1' */
  localDW->Integrator1_DSTATE[1] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S444>/Integrator' */
  localDW->Integrator_DSTATE[1] = 0.0F;
}

/*
 * Output and update for action system:
 *    '<S434>/Move Control'
 *    '<S366>/Move Control'
 *    '<S200>/Move Control'
 */
void FMS_MoveControl_j(real32_T rtu_FMS_In, real32_T rtu_FMS_In_o, real32_T
  rty_uv_cmd_mPs[2], const ConstB_MoveControl_FMS_i_T *localC,
  DW_MoveControl_FMS_f_T *localDW)
{
  real32_T rtb_Subtract3_p;
  real32_T rtb_Add3_l_idx_0;
  real32_T rtb_Subtract3_l_idx_0;
  real32_T rtb_Add3_l_idx_1;
  real32_T rtb_Subtract3_l_idx_1;
  real32_T u;
  real32_T rtb_Subtract3_l_idx_0_0;

  /* SignalConversion: '<S438>/OutportBufferForuv_cmd_mPs' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator1'
   */
  rty_uv_cmd_mPs[0] = localDW->Integrator1_DSTATE[0];

  /* Product: '<S445>/Multiply1' incorporates:
   *  Constant: '<S445>/const1'
   *  DiscreteIntegrator: '<S444>/Integrator'
   */
  rtb_Add3_l_idx_0 = localDW->Integrator_DSTATE[0] * 0.05F;

  /* SignalConversion: '<S438>/OutportBufferForuv_cmd_mPs' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator1'
   */
  rty_uv_cmd_mPs[1] = localDW->Integrator1_DSTATE[1];

  /* Product: '<S445>/Multiply1' incorporates:
   *  Constant: '<S445>/const1'
   *  DiscreteIntegrator: '<S444>/Integrator'
   */
  rtb_Add3_l_idx_1 = localDW->Integrator_DSTATE[1] * 0.05F;

  /* DeadZone: '<S442>/Dead Zone' */
  if (rtu_FMS_In > FMS_PARAM.PITCH_DZ) {
    rtb_Subtract3_l_idx_1 = rtu_FMS_In - FMS_PARAM.PITCH_DZ;
  } else if (rtu_FMS_In >= -FMS_PARAM.PITCH_DZ) {
    rtb_Subtract3_l_idx_1 = 0.0F;
  } else {
    rtb_Subtract3_l_idx_1 = rtu_FMS_In - (-FMS_PARAM.PITCH_DZ);
  }

  /* End of DeadZone: '<S442>/Dead Zone' */

  /* Sum: '<S445>/Add' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator1'
   *  Gain: '<S438>/Gain6'
   *  Gain: '<S442>/Gain'
   *  Sum: '<S444>/Subtract'
   */
  rtb_Subtract3_l_idx_0 = (localDW->Integrator1_DSTATE[0] - 1.0F / (1.0F -
    FMS_PARAM.PITCH_DZ) * rtb_Subtract3_l_idx_1 * FMS_PARAM.VEL_XY_LIM) +
    rtb_Add3_l_idx_0;

  /* DeadZone: '<S443>/Dead Zone' */
  if (rtu_FMS_In_o > FMS_PARAM.ROLL_DZ) {
    rtb_Subtract3_l_idx_1 = rtu_FMS_In_o - FMS_PARAM.ROLL_DZ;
  } else if (rtu_FMS_In_o >= -FMS_PARAM.ROLL_DZ) {
    rtb_Subtract3_l_idx_1 = 0.0F;
  } else {
    rtb_Subtract3_l_idx_1 = rtu_FMS_In_o - (-FMS_PARAM.ROLL_DZ);
  }

  /* End of DeadZone: '<S443>/Dead Zone' */

  /* Sum: '<S445>/Add' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator1'
   *  Gain: '<S438>/Gain6'
   *  Gain: '<S443>/Gain'
   *  Sum: '<S444>/Subtract'
   */
  rtb_Subtract3_l_idx_1 = (localDW->Integrator1_DSTATE[1] - 1.0F / (1.0F -
    FMS_PARAM.ROLL_DZ) * rtb_Subtract3_l_idx_1 * FMS_PARAM.VEL_XY_LIM) +
    rtb_Add3_l_idx_1;

  /* Signum: '<S445>/Sign' */
  if (rtb_Subtract3_l_idx_0 < 0.0F) {
    rtb_Subtract3_l_idx_0_0 = -1.0F;
  } else if (rtb_Subtract3_l_idx_0 > 0.0F) {
    rtb_Subtract3_l_idx_0_0 = 1.0F;
  } else {
    rtb_Subtract3_l_idx_0_0 = rtb_Subtract3_l_idx_0;
  }

  /* Sum: '<S445>/Add2' incorporates:
   *  Abs: '<S445>/Abs'
   *  Gain: '<S445>/Gain'
   *  Gain: '<S445>/Gain1'
   *  Product: '<S445>/Multiply2'
   *  Product: '<S445>/Multiply3'
   *  Signum: '<S445>/Sign'
   *  Sqrt: '<S445>/Sqrt'
   *  Sum: '<S445>/Add1'
   *  Sum: '<S445>/Subtract'
   */
  rtb_Subtract3_l_idx_0_0 = (sqrtf((8.0F * fabsf(rtb_Subtract3_l_idx_0) +
    localC->d) * localC->d) - localC->d) * 0.5F * rtb_Subtract3_l_idx_0_0 +
    rtb_Add3_l_idx_0;

  /* Sum: '<S445>/Add3' incorporates:
   *  Signum: '<S445>/Sign'
   */
  u = rtb_Subtract3_l_idx_0 + localC->d;

  /* Sum: '<S445>/Subtract1' incorporates:
   *  Signum: '<S445>/Sign'
   */
  rtb_Subtract3_p = rtb_Subtract3_l_idx_0 - localC->d;

  /* Signum: '<S445>/Sign1' */
  if (u < 0.0F) {
    u = -1.0F;
  } else {
    if (u > 0.0F) {
      u = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign2' */
  if (rtb_Subtract3_p < 0.0F) {
    rtb_Subtract3_p = -1.0F;
  } else {
    if (rtb_Subtract3_p > 0.0F) {
      rtb_Subtract3_p = 1.0F;
    }
  }

  /* Sum: '<S445>/Add5' incorporates:
   *  Gain: '<S445>/Gain2'
   *  Product: '<S445>/Multiply4'
   *  Signum: '<S445>/Sign'
   *  Sum: '<S445>/Add2'
   *  Sum: '<S445>/Add4'
   *  Sum: '<S445>/Subtract2'
   */
  rtb_Subtract3_l_idx_0_0 += ((rtb_Subtract3_l_idx_0 - rtb_Subtract3_l_idx_0_0)
    + rtb_Add3_l_idx_0) * ((u - rtb_Subtract3_p) * 0.5F);

  /* Update for DiscreteIntegrator: '<S444>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator'
   */
  localDW->Integrator1_DSTATE[0] += 0.004F * localDW->Integrator_DSTATE[0];

  /* Signum: '<S445>/Sign3' incorporates:
   *  Sum: '<S445>/Add6'
   */
  u = rtb_Subtract3_l_idx_0_0 + localC->d;

  /* Signum: '<S445>/Sign4' incorporates:
   *  Sum: '<S445>/Subtract3'
   */
  rtb_Add3_l_idx_0 = rtb_Subtract3_l_idx_0_0 - localC->d;

  /* Signum: '<S445>/Sign5' */
  if (rtb_Subtract3_l_idx_0_0 < 0.0F) {
    rtb_Subtract3_l_idx_0 = -1.0F;
  } else if (rtb_Subtract3_l_idx_0_0 > 0.0F) {
    rtb_Subtract3_l_idx_0 = 1.0F;
  } else {
    rtb_Subtract3_l_idx_0 = rtb_Subtract3_l_idx_0_0;
  }

  /* Signum: '<S445>/Sign3' */
  if (u < 0.0F) {
    u = -1.0F;
  } else {
    if (u > 0.0F) {
      u = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign4' */
  if (rtb_Add3_l_idx_0 < 0.0F) {
    rtb_Add3_l_idx_0 = -1.0F;
  } else {
    if (rtb_Add3_l_idx_0 > 0.0F) {
      rtb_Add3_l_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign6' */
  if (rtb_Subtract3_l_idx_0_0 < 0.0F) {
    rtb_Subtract3_p = -1.0F;
  } else if (rtb_Subtract3_l_idx_0_0 > 0.0F) {
    rtb_Subtract3_p = 1.0F;
  } else {
    rtb_Subtract3_p = rtb_Subtract3_l_idx_0_0;
  }

  /* Update for DiscreteIntegrator: '<S444>/Integrator' incorporates:
   *  Constant: '<S445>/const'
   *  Gain: '<S445>/Gain3'
   *  Product: '<S445>/Divide'
   *  Product: '<S445>/Multiply5'
   *  Product: '<S445>/Multiply6'
   *  Sum: '<S445>/Subtract4'
   *  Sum: '<S445>/Subtract5'
   *  Sum: '<S445>/Subtract6'
   */
  localDW->Integrator_DSTATE[0] += ((rtb_Subtract3_l_idx_0_0 / localC->d -
    rtb_Subtract3_l_idx_0) * localC->Gain4 * ((u - rtb_Add3_l_idx_0) * 0.5F) -
    rtb_Subtract3_p * 58.836F) * 0.004F;

  /* Signum: '<S445>/Sign' */
  if (rtb_Subtract3_l_idx_1 < 0.0F) {
    rtb_Add3_l_idx_0 = -1.0F;
  } else if (rtb_Subtract3_l_idx_1 > 0.0F) {
    rtb_Add3_l_idx_0 = 1.0F;
  } else {
    rtb_Add3_l_idx_0 = rtb_Subtract3_l_idx_1;
  }

  /* Sum: '<S445>/Add2' incorporates:
   *  Abs: '<S445>/Abs'
   *  Gain: '<S445>/Gain'
   *  Gain: '<S445>/Gain1'
   *  Product: '<S445>/Multiply2'
   *  Product: '<S445>/Multiply3'
   *  Signum: '<S445>/Sign'
   *  Sqrt: '<S445>/Sqrt'
   *  Sum: '<S445>/Add1'
   *  Sum: '<S445>/Subtract'
   */
  rtb_Subtract3_l_idx_0_0 = (sqrtf((8.0F * fabsf(rtb_Subtract3_l_idx_1) +
    localC->d) * localC->d) - localC->d) * 0.5F * rtb_Add3_l_idx_0 +
    rtb_Add3_l_idx_1;

  /* Sum: '<S445>/Add3' incorporates:
   *  Signum: '<S445>/Sign'
   */
  u = rtb_Subtract3_l_idx_1 + localC->d;

  /* Sum: '<S445>/Subtract1' incorporates:
   *  Signum: '<S445>/Sign'
   */
  rtb_Subtract3_p = rtb_Subtract3_l_idx_1 - localC->d;

  /* Signum: '<S445>/Sign1' */
  if (u < 0.0F) {
    u = -1.0F;
  } else {
    if (u > 0.0F) {
      u = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign2' */
  if (rtb_Subtract3_p < 0.0F) {
    rtb_Subtract3_p = -1.0F;
  } else {
    if (rtb_Subtract3_p > 0.0F) {
      rtb_Subtract3_p = 1.0F;
    }
  }

  /* Sum: '<S445>/Add5' incorporates:
   *  Gain: '<S445>/Gain2'
   *  Product: '<S445>/Multiply4'
   *  Signum: '<S445>/Sign'
   *  Sum: '<S445>/Add2'
   *  Sum: '<S445>/Add4'
   *  Sum: '<S445>/Subtract2'
   */
  rtb_Subtract3_l_idx_0_0 += ((rtb_Subtract3_l_idx_1 - rtb_Subtract3_l_idx_0_0)
    + rtb_Add3_l_idx_1) * ((u - rtb_Subtract3_p) * 0.5F);

  /* Update for DiscreteIntegrator: '<S444>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S444>/Integrator'
   */
  localDW->Integrator1_DSTATE[1] += 0.004F * localDW->Integrator_DSTATE[1];

  /* Signum: '<S445>/Sign3' incorporates:
   *  Sum: '<S445>/Add6'
   */
  u = rtb_Subtract3_l_idx_0_0 + localC->d;

  /* Signum: '<S445>/Sign4' incorporates:
   *  Sum: '<S445>/Subtract3'
   */
  rtb_Add3_l_idx_0 = rtb_Subtract3_l_idx_0_0 - localC->d;

  /* Signum: '<S445>/Sign5' */
  if (rtb_Subtract3_l_idx_0_0 < 0.0F) {
    rtb_Subtract3_l_idx_0 = -1.0F;
  } else if (rtb_Subtract3_l_idx_0_0 > 0.0F) {
    rtb_Subtract3_l_idx_0 = 1.0F;
  } else {
    rtb_Subtract3_l_idx_0 = rtb_Subtract3_l_idx_0_0;
  }

  /* Signum: '<S445>/Sign3' */
  if (u < 0.0F) {
    u = -1.0F;
  } else {
    if (u > 0.0F) {
      u = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign4' */
  if (rtb_Add3_l_idx_0 < 0.0F) {
    rtb_Add3_l_idx_0 = -1.0F;
  } else {
    if (rtb_Add3_l_idx_0 > 0.0F) {
      rtb_Add3_l_idx_0 = 1.0F;
    }
  }

  /* Signum: '<S445>/Sign6' */
  if (rtb_Subtract3_l_idx_0_0 < 0.0F) {
    rtb_Subtract3_p = -1.0F;
  } else if (rtb_Subtract3_l_idx_0_0 > 0.0F) {
    rtb_Subtract3_p = 1.0F;
  } else {
    rtb_Subtract3_p = rtb_Subtract3_l_idx_0_0;
  }

  /* Update for DiscreteIntegrator: '<S444>/Integrator' incorporates:
   *  Constant: '<S445>/const'
   *  Gain: '<S445>/Gain3'
   *  Product: '<S445>/Divide'
   *  Product: '<S445>/Multiply5'
   *  Product: '<S445>/Multiply6'
   *  Sum: '<S445>/Subtract4'
   *  Sum: '<S445>/Subtract5'
   *  Sum: '<S445>/Subtract6'
   */
  localDW->Integrator_DSTATE[1] += ((rtb_Subtract3_l_idx_0_0 / localC->d -
    rtb_Subtract3_l_idx_0) * localC->Gain4 * ((u - rtb_Add3_l_idx_0) * 0.5F) -
    rtb_Subtract3_p * 58.836F) * 0.004F;
}

/*
 * System initialize for atomic system:
 *    '<S435>/Motion State'
 *    '<S367>/Motion State'
 *    '<S201>/Motion State'
 */
void FMS_MotionState_Init(DW_MotionState_FMS_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c23_FMS = 0U;
  localDW->is_c23_FMS = FMS_IN_NO_ACTIVE_CHILD_g;
}

/*
 * System reset for atomic system:
 *    '<S435>/Motion State'
 *    '<S367>/Motion State'
 *    '<S201>/Motion State'
 */
void FMS_MotionState_Reset(DW_MotionState_FMS_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c23_FMS = 0U;
  localDW->is_c23_FMS = FMS_IN_NO_ACTIVE_CHILD_g;
}

/*
 * Output and update for atomic system:
 *    '<S435>/Motion State'
 *    '<S367>/Motion State'
 *    '<S201>/Motion State'
 */
void FMS_MotionState(real32_T rtu_motion_req, real32_T rtu_speed, MotionState
                     *rty_state, DW_MotionState_FMS_T *localDW)
{
  /* Chart: '<S435>/Motion State' */
  if (localDW->temporalCounter_i1 < 1023U) {
    localDW->temporalCounter_i1++;
  }

  if (localDW->is_active_c23_FMS == 0U) {
    localDW->is_active_c23_FMS = 1U;
    localDW->is_c23_FMS = FMS_IN_Move_o;
    *rty_state = MotionState_Move;
  } else {
    switch (localDW->is_c23_FMS) {
     case FMS_IN_Brake_c:
      *rty_state = MotionState_Brake;
      if ((rtu_speed <= 0.2) || (localDW->temporalCounter_i1 >= 625U)) {
        localDW->is_c23_FMS = FMS_IN_Hold_c;
        *rty_state = MotionState_Hold;
      } else {
        if (rtu_motion_req == 1.0F) {
          localDW->is_c23_FMS = FMS_IN_Move_o;
          *rty_state = MotionState_Move;
        }
      }
      break;

     case FMS_IN_Hold_c:
      *rty_state = MotionState_Hold;
      if (rtu_motion_req == 1.0F) {
        localDW->is_c23_FMS = FMS_IN_Move_o;
        *rty_state = MotionState_Move;
      }
      break;

     default:
      *rty_state = MotionState_Move;
      if (rtu_motion_req == 0.0F) {
        localDW->is_c23_FMS = FMS_IN_Brake_c;
        localDW->temporalCounter_i1 = 0U;
        *rty_state = MotionState_Brake;
      }
      break;
    }
  }

  /* End of Chart: '<S435>/Motion State' */
}

/*
 * Output and update for atomic system:
 *    '<S411>/NearbyRefWP'
 *    '<S177>/NearbyRefWP'
 */
void FMS_NearbyRefWP(const real32_T rtu_P2[2], real32_T rtu_P3, real32_T
                     rtu_P3_d, real32_T rtu_L1, real32_T rty_P[2], real32_T
                     *rty_d)
{
  real32_T P3P2_idx_0;
  real32_T P3P2_idx_1;

  /* SignalConversion: '<S449>/TmpSignal ConversionAt SFunction Inport2' */
  P3P2_idx_0 = rtu_P2[0] - rtu_P3;
  P3P2_idx_1 = rtu_P2[1] - rtu_P3_d;
  P3P2_idx_0 = sqrtf(P3P2_idx_0 * P3P2_idx_0 + P3P2_idx_1 * P3P2_idx_1);
  if (P3P2_idx_0 <= rtu_L1) {
    *rty_d = P3P2_idx_0;
    rty_P[0] = rtu_P2[0];
    rty_P[1] = rtu_P2[1];
  } else {
    *rty_d = -1.0F;
    rty_P[0] = 0.0F;
    rty_P[1] = 0.0F;
  }
}

/*
 * System initialize for action system:
 *    '<S354>/Hold Control'
 *    '<S136>/Hold Control'
 *    '<S68>/Hold Control'
 *    '<S96>/Hold Control'
 */
void FMS_HoldControl_e_Init(DW_HoldControl_FMS_g_T *localDW)
{
  /* InitializeConditions for Delay: '<S357>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * System reset for action system:
 *    '<S354>/Hold Control'
 *    '<S136>/Hold Control'
 *    '<S68>/Hold Control'
 *    '<S96>/Hold Control'
 */
void FMS_HoldControl_kp_Reset(DW_HoldControl_FMS_g_T *localDW)
{
  /* InitializeConditions for Delay: '<S357>/Delay' */
  localDW->icLoad = 1U;
}

/*
 * Output and update for action system:
 *    '<S354>/Hold Control'
 *    '<S136>/Hold Control'
 *    '<S68>/Hold Control'
 *    '<S96>/Hold Control'
 */
void FMS_HoldControl_k(real32_T rtu_FMS_In, real32_T *rty_psi_rate_cmd_radPs,
  DW_HoldControl_FMS_g_T *localDW)
{
  real32_T rtb_psi_error_rad;
  real32_T rtb_Abs_l;

  /* Delay: '<S357>/Delay' */
  if (localDW->icLoad != 0) {
    localDW->Delay_DSTATE = rtu_FMS_In;
  }

  /* Sum: '<S357>/Sum' incorporates:
   *  Delay: '<S357>/Delay'
   */
  rtb_psi_error_rad = localDW->Delay_DSTATE - rtu_FMS_In;

  /* Abs: '<S360>/Abs' */
  rtb_Abs_l = fabsf(rtb_psi_error_rad);

  /* Switch: '<S360>/Switch' incorporates:
   *  Constant: '<S360>/Constant'
   *  Constant: '<S361>/Constant'
   *  Product: '<S360>/Multiply'
   *  RelationalOperator: '<S361>/Compare'
   *  Sum: '<S360>/Subtract'
   */
  if (rtb_Abs_l > 3.14159274F) {
    /* Signum: '<S360>/Sign' */
    if (rtb_psi_error_rad < 0.0F) {
      rtb_psi_error_rad = -1.0F;
    } else {
      if (rtb_psi_error_rad > 0.0F) {
        rtb_psi_error_rad = 1.0F;
      }
    }

    /* End of Signum: '<S360>/Sign' */
    rtb_psi_error_rad *= rtb_Abs_l - 6.28318548F;
  }

  /* End of Switch: '<S360>/Switch' */

  /* Gain: '<S357>/Gain2' */
  *rty_psi_rate_cmd_radPs = FMS_PARAM.YAW_P * rtb_psi_error_rad;

  /* Update for Delay: '<S357>/Delay' */
  localDW->icLoad = 0U;
}

/*
 * System initialize for action system:
 *    '<S354>/Move Control'
 *    '<S136>/Move Control'
 *    '<S68>/Move Control'
 *    '<S96>/Move Control'
 */
void FMS_MoveControl_j_Init(DW_MoveControl_FMS_c_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S363>/Integrator1' */
  localDW->Integrator1_DSTATE = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S363>/Integrator' */
  localDW->Integrator_DSTATE = 0.0F;
}

/*
 * System reset for action system:
 *    '<S354>/Move Control'
 *    '<S136>/Move Control'
 *    '<S68>/Move Control'
 *    '<S96>/Move Control'
 */
void FMS_MoveControl_l_Reset(DW_MoveControl_FMS_c_T *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S363>/Integrator1' */
  localDW->Integrator1_DSTATE = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S363>/Integrator' */
  localDW->Integrator_DSTATE = 0.0F;
}

/*
 * Output and update for action system:
 *    '<S354>/Move Control'
 *    '<S136>/Move Control'
 *    '<S68>/Move Control'
 *    '<S96>/Move Control'
 */
void FMS_MoveControl_b(real32_T rtu_FMS_In, real32_T *rty_psi_rate_cmd_radPs,
  const ConstB_MoveControl_FMS_f_T *localC, DW_MoveControl_FMS_c_T *localDW)
{
  real32_T rtb_Add3_ig;
  real32_T rtb_Subtract3_l;
  real32_T rtb_a_m;
  real32_T rtb_Add4_b;
  real32_T rtb_a_p4;

  /* Product: '<S364>/Multiply1' incorporates:
   *  Constant: '<S364>/const1'
   *  DiscreteIntegrator: '<S363>/Integrator'
   */
  rtb_Add3_ig = localDW->Integrator_DSTATE * 0.02F;

  /* DeadZone: '<S362>/Dead Zone' */
  if (rtu_FMS_In > FMS_PARAM.YAW_DZ) {
    rtb_a_m = rtu_FMS_In - FMS_PARAM.YAW_DZ;
  } else if (rtu_FMS_In >= -FMS_PARAM.YAW_DZ) {
    rtb_a_m = 0.0F;
  } else {
    rtb_a_m = rtu_FMS_In - (-FMS_PARAM.YAW_DZ);
  }

  /* End of DeadZone: '<S362>/Dead Zone' */

  /* Sum: '<S364>/Add' incorporates:
   *  DiscreteIntegrator: '<S363>/Integrator1'
   *  Gain: '<S358>/Gain1'
   *  Gain: '<S362>/Gain'
   *  Sum: '<S363>/Subtract'
   */
  rtb_Subtract3_l = (localDW->Integrator1_DSTATE - 1.0F / (1.0F -
    FMS_PARAM.YAW_DZ) * rtb_a_m * FMS_PARAM.YAW_RATE_LIM) + rtb_Add3_ig;

  /* Signum: '<S364>/Sign' */
  if (rtb_Subtract3_l < 0.0F) {
    rtb_a_m = -1.0F;
  } else if (rtb_Subtract3_l > 0.0F) {
    rtb_a_m = 1.0F;
  } else {
    rtb_a_m = rtb_Subtract3_l;
  }

  /* End of Signum: '<S364>/Sign' */

  /* Sum: '<S364>/Add2' incorporates:
   *  Abs: '<S364>/Abs'
   *  Gain: '<S364>/Gain'
   *  Gain: '<S364>/Gain1'
   *  Product: '<S364>/Multiply2'
   *  Product: '<S364>/Multiply3'
   *  Sqrt: '<S364>/Sqrt'
   *  Sum: '<S364>/Add1'
   *  Sum: '<S364>/Subtract'
   */
  rtb_a_m = (sqrtf((8.0F * fabsf(rtb_Subtract3_l) + localC->d) * localC->d) -
             localC->d) * 0.5F * rtb_a_m + rtb_Add3_ig;

  /* Sum: '<S364>/Add4' */
  rtb_Add4_b = (rtb_Subtract3_l - rtb_a_m) + rtb_Add3_ig;

  /* Sum: '<S364>/Add3' */
  rtb_Add3_ig = rtb_Subtract3_l + localC->d;

  /* Sum: '<S364>/Subtract1' */
  rtb_Subtract3_l -= localC->d;

  /* Signum: '<S364>/Sign1' */
  if (rtb_Add3_ig < 0.0F) {
    rtb_Add3_ig = -1.0F;
  } else {
    if (rtb_Add3_ig > 0.0F) {
      rtb_Add3_ig = 1.0F;
    }
  }

  /* End of Signum: '<S364>/Sign1' */

  /* Signum: '<S364>/Sign2' */
  if (rtb_Subtract3_l < 0.0F) {
    rtb_Subtract3_l = -1.0F;
  } else {
    if (rtb_Subtract3_l > 0.0F) {
      rtb_Subtract3_l = 1.0F;
    }
  }

  /* End of Signum: '<S364>/Sign2' */

  /* Sum: '<S364>/Add5' incorporates:
   *  Gain: '<S364>/Gain2'
   *  Product: '<S364>/Multiply4'
   *  Sum: '<S364>/Subtract2'
   */
  rtb_a_m += (rtb_Add3_ig - rtb_Subtract3_l) * 0.5F * rtb_Add4_b;

  /* SignalConversion: '<S358>/OutportBufferForpsi_rate_cmd_radPs' incorporates:
   *  DiscreteIntegrator: '<S363>/Integrator1'
   */
  *rty_psi_rate_cmd_radPs = localDW->Integrator1_DSTATE;

  /* Update for DiscreteIntegrator: '<S363>/Integrator1' incorporates:
   *  DiscreteIntegrator: '<S363>/Integrator'
   */
  localDW->Integrator1_DSTATE += 0.004F * localDW->Integrator_DSTATE;

  /* Sum: '<S364>/Add6' */
  rtb_Add3_ig = rtb_a_m + localC->d;

  /* Sum: '<S364>/Subtract3' */
  rtb_Subtract3_l = rtb_a_m - localC->d;

  /* Signum: '<S364>/Sign5' */
  if (rtb_a_m < 0.0F) {
    rtb_Add4_b = -1.0F;
  } else if (rtb_a_m > 0.0F) {
    rtb_Add4_b = 1.0F;
  } else {
    rtb_Add4_b = rtb_a_m;
  }

  /* End of Signum: '<S364>/Sign5' */

  /* Signum: '<S364>/Sign3' */
  if (rtb_Add3_ig < 0.0F) {
    rtb_Add3_ig = -1.0F;
  } else {
    if (rtb_Add3_ig > 0.0F) {
      rtb_Add3_ig = 1.0F;
    }
  }

  /* End of Signum: '<S364>/Sign3' */

  /* Signum: '<S364>/Sign4' */
  if (rtb_Subtract3_l < 0.0F) {
    rtb_Subtract3_l = -1.0F;
  } else {
    if (rtb_Subtract3_l > 0.0F) {
      rtb_Subtract3_l = 1.0F;
    }
  }

  /* End of Signum: '<S364>/Sign4' */

  /* Signum: '<S364>/Sign6' */
  if (rtb_a_m < 0.0F) {
    rtb_a_p4 = -1.0F;
  } else if (rtb_a_m > 0.0F) {
    rtb_a_p4 = 1.0F;
  } else {
    rtb_a_p4 = rtb_a_m;
  }

  /* End of Signum: '<S364>/Sign6' */

  /* Update for DiscreteIntegrator: '<S363>/Integrator' incorporates:
   *  Constant: '<S364>/const'
   *  Gain: '<S364>/Gain3'
   *  Product: '<S364>/Divide'
   *  Product: '<S364>/Multiply5'
   *  Product: '<S364>/Multiply6'
   *  Sum: '<S364>/Subtract4'
   *  Sum: '<S364>/Subtract5'
   *  Sum: '<S364>/Subtract6'
   */
  localDW->Integrator_DSTATE += ((rtb_a_m / localC->d - rtb_Add4_b) *
    localC->Gain4 * ((rtb_Add3_ig - rtb_Subtract3_l) * 0.5F) - rtb_a_p4 *
    15.707963F) * 0.004F;
}

/*
 * Output and update for action system:
 *    '<S38>/Unknown'
 *    '<S36>/Unknown'
 *    '<S35>/Unknown'
 *    '<S31>/Unknown'
 */
void FMS_Unknown(FMS_Out_Bus *rty_FMS_Out, const ConstB_Unknown_FMS_T *localC)
{
  int32_T i;

  /* BusAssignment: '<S340>/Bus Assignment' incorporates:
   *  Constant: '<S340>/Constant'
   *  Constant: '<S340>/Constant2'
   *  SignalConversion: '<S340>/TmpHiddenBufferAtBus AssignmentInport1'
   */
  *rty_FMS_Out = FMS_rtZFMS_Out_Bus;
  rty_FMS_Out->reset = 1U;
  rty_FMS_Out->status = localC->DataTypeConversion;
  rty_FMS_Out->state = localC->DataTypeConversion1;
  rty_FMS_Out->actuator_cmd[0] = 1000U;
  rty_FMS_Out->actuator_cmd[1] = 1000U;
  rty_FMS_Out->actuator_cmd[2] = 1000U;
  rty_FMS_Out->actuator_cmd[3] = 1000U;
  for (i = 0; i < 12; i++) {
    rty_FMS_Out->actuator_cmd[i + 4] = 0U;
  }

  /* End of BusAssignment: '<S340>/Bus Assignment' */
}

/*
 * System initialize for atomic system:
 *    '<S137>/Motion State'
 *    '<S69>/Motion State'
 *    '<S97>/Motion State'
 */
void FMS_MotionState_l_Init(DW_MotionState_FMS_g_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c19_FMS = 0U;
  localDW->is_c19_FMS = FMS_IN_NO_ACTIVE_CHILD_b;
}

/*
 * System reset for atomic system:
 *    '<S137>/Motion State'
 *    '<S69>/Motion State'
 *    '<S97>/Motion State'
 */
void FMS_MotionState_j_Reset(DW_MotionState_FMS_g_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c19_FMS = 0U;
  localDW->is_c19_FMS = FMS_IN_NO_ACTIVE_CHILD_b;
}

/*
 * Output and update for atomic system:
 *    '<S137>/Motion State'
 *    '<S69>/Motion State'
 *    '<S97>/Motion State'
 */
void FMS_MotionState_e(boolean_T rtu_motion_req, real32_T rtu_speed, MotionState
  *rty_state, DW_MotionState_FMS_g_T *localDW)
{
  /* Chart: '<S137>/Motion State' */
  if (localDW->temporalCounter_i1 < 255U) {
    localDW->temporalCounter_i1++;
  }

  if (localDW->is_active_c19_FMS == 0U) {
    localDW->is_active_c19_FMS = 1U;
    localDW->is_c19_FMS = FMS_IN_Move_o4;
    *rty_state = MotionState_Move;
  } else {
    switch (localDW->is_c19_FMS) {
     case FMS_IN_Brake_f:
      *rty_state = MotionState_Brake;
      if ((rtu_speed <= 0.1) || (localDW->temporalCounter_i1 >= 250U)) {
        localDW->is_c19_FMS = FMS_IN_Hold_j;
        *rty_state = MotionState_Hold;
      } else {
        if (rtu_motion_req) {
          localDW->is_c19_FMS = FMS_IN_Move_o4;
          *rty_state = MotionState_Move;
        }
      }
      break;

     case FMS_IN_Hold_j:
      *rty_state = MotionState_Hold;
      if (rtu_motion_req) {
        localDW->is_c19_FMS = FMS_IN_Move_o4;
        *rty_state = MotionState_Move;
      }
      break;

     default:
      *rty_state = MotionState_Move;
      if (!rtu_motion_req) {
        localDW->is_c19_FMS = FMS_IN_Brake_f;
        localDW->temporalCounter_i1 = 0U;
        *rty_state = MotionState_Brake;
      }
      break;
    }
  }

  /* End of Chart: '<S137>/Motion State' */
}

/*
 * Output and update for action system:
 *    '<S50>/Move Control'
 *    '<S84>/Move Control'
 */
void FMS_MoveControl_l(real32_T rtu_FMS_In, real32_T rtu_FMS_In_l, uint32_T
  rtu_FMS_In_i, real32_T *rty_w_cmd_mPs)
{
  real32_T rtb_Gain1_kl;

  /* DeadZone: '<S59>/Dead Zone' */
  if (rtu_FMS_In > FMS_PARAM.THROTTLE_DZ) {
    rtb_Gain1_kl = rtu_FMS_In - FMS_PARAM.THROTTLE_DZ;
  } else if (rtu_FMS_In >= -FMS_PARAM.THROTTLE_DZ) {
    rtb_Gain1_kl = 0.0F;
  } else {
    rtb_Gain1_kl = rtu_FMS_In - (-FMS_PARAM.THROTTLE_DZ);
  }

  /* End of DeadZone: '<S59>/Dead Zone' */

  /* Gain: '<S54>/Gain1' incorporates:
   *  Gain: '<S59>/Gain'
   */
  rtb_Gain1_kl = 1.0F / (1.0F - FMS_PARAM.THROTTLE_DZ) * rtb_Gain1_kl *
    -FMS_PARAM.VEL_Z_LIM;

  /* Switch: '<S54>/Switch' incorporates:
   *  Constant: '<S56>/Land_Speed'
   *  Constant: '<S57>/Constant'
   *  Constant: '<S58>/Constant'
   *  Gain: '<S56>/Gain'
   *  Logic: '<S54>/Logical Operator'
   *  MinMax: '<S56>/Min'
   *  RelationalOperator: '<S57>/Compare'
   *  RelationalOperator: '<S58>/Compare'
   *  S-Function (sfix_bitop): '<S54>/cmd_p valid'
   */
  if ((rtb_Gain1_kl > 0.0F) && ((rtu_FMS_In_i & 256U) != 0U) && (rtu_FMS_In_l <=
       FMS_PARAM.ASSIST_LAND_H)) {
    *rty_w_cmd_mPs = 0.5F * fminf(FMS_PARAM.LAND_SPEED, rtb_Gain1_kl);
  } else {
    *rty_w_cmd_mPs = rtb_Gain1_kl;
  }

  /* End of Switch: '<S54>/Switch' */
}

real_T rt_modd(real_T u0, real_T u1)
{
  real_T y;
  boolean_T yEq;
  real_T q;
  y = u0;
  if (u0 == 0.0) {
    y = 0.0;
  } else {
    if (u1 != 0.0) {
      y = fmod(u0, u1);
      yEq = (y == 0.0);
      if ((!yEq) && (u1 > floor(u1))) {
        q = fabs(u0 / u1);
        yEq = (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q);
      }

      if (yEq) {
        y = 0.0;
      } else {
        if ((u0 < 0.0) != (u1 < 0.0)) {
          y += u1;
        }
      }
    }
  }

  return y;
}

/* Output and update for function-call system: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
void F_VehicleArmAutoMissionLLA2FLAT(const real_T rtu_lla[3], const real_T
  rtu_llo[2], real_T rtu_href, real_T rtu_psio, real32_T rty_pos[3], const
  ConstB_VehicleArmAutoMissionL_T *localC)
{
  real_T rtb_Sum2_ee;
  real_T rtb_Gain_lq;
  real_T rtb_Sum3_l;
  real_T rtb_Sum_d;
  int32_T rtb_Compare_dy_0;
  real_T rtb_Sum_e_idx_0;

  /* Gain: '<S479>/deg2rad' */
  rtb_Sum_e_idx_0 = 0.017453292519943295 * rtu_llo[0];

  /* Trigonometry: '<S480>/Sin' */
  rtb_Sum2_ee = sin(rtb_Sum_e_idx_0);

  /* Math: '<S480>/Square1' */
  rtb_Sum2_ee *= rtb_Sum2_ee;

  /* Product: '<S480>/Multiply1' incorporates:
   *  Product: '<S480>/Multiply'
   */
  rtb_Gain_lq = localC->ff * rtb_Sum2_ee;

  /* Product: '<S480>/Divide' incorporates:
   *  Constant: '<S480>/Constant'
   *  Constant: '<S480>/R'
   *  Sqrt: '<S480>/Sqrt'
   *  Sum: '<S480>/Sum1'
   */
  rtb_Sum2_ee = 6.378137E+6 / sqrt(1.0 - rtb_Gain_lq);

  /* Product: '<S480>/Product3' incorporates:
   *  Constant: '<S480>/Constant1'
   *  Product: '<S480>/Multiply1'
   *  Sum: '<S480>/Sum2'
   */
  rtb_Gain_lq = 1.0 / (1.0 - rtb_Gain_lq) * localC->Sum4 * rtb_Sum2_ee;

  /* Product: '<S480>/Multiply2' incorporates:
   *  Trigonometry: '<S480>/Cos'
   */
  rtb_Sum2_ee *= cos(rtb_Sum_e_idx_0);

  /* Sum: '<S479>/Sum' */
  rtb_Sum_e_idx_0 = rtu_lla[0] - rtu_llo[0];

  /* Abs: '<S484>/Abs' incorporates:
   *  Abs: '<S487>/Abs1'
   *  Switch: '<S484>/Switch1'
   */
  rtb_Sum_d = fabs(rtb_Sum_e_idx_0);

  /* Switch: '<S484>/Switch1' incorporates:
   *  Abs: '<S484>/Abs'
   *  Bias: '<S484>/Bias2'
   *  Bias: '<S484>/Bias3'
   *  Constant: '<S481>/Constant'
   *  Constant: '<S481>/Constant1'
   *  Constant: '<S486>/Constant'
   *  Gain: '<S484>/Gain1'
   *  Product: '<S484>/Multiply'
   *  RelationalOperator: '<S486>/Compare'
   *  Switch: '<S481>/Switch'
   */
  if (rtb_Sum_d > 90.0) {
    /* Switch: '<S487>/Switch1' incorporates:
     *  Bias: '<S487>/Bias2'
     *  Bias: '<S487>/Bias3'
     *  Constant: '<S487>/Constant'
     *  Constant: '<S488>/Constant'
     *  Math: '<S487>/Math Function'
     *  RelationalOperator: '<S488>/Compare'
     */
    if (rtb_Sum_d > 180.0) {
      rtb_Sum_e_idx_0 = rt_modd(rtb_Sum_e_idx_0 + 180.0, 360.0) + -180.0;
    }

    /* End of Switch: '<S487>/Switch1' */

    /* Signum: '<S484>/Sign' */
    if (rtb_Sum_e_idx_0 < 0.0) {
      rtb_Sum_e_idx_0 = -1.0;
    } else {
      if (rtb_Sum_e_idx_0 > 0.0) {
        rtb_Sum_e_idx_0 = 1.0;
      }
    }

    /* End of Signum: '<S484>/Sign' */
    rtb_Sum_e_idx_0 *= -(rtb_Sum_d + -90.0) + 90.0;
    rtb_Compare_dy_0 = 180;
  } else {
    rtb_Compare_dy_0 = 0;
  }

  /* Sum: '<S481>/Sum' incorporates:
   *  Sum: '<S479>/Sum'
   */
  rtb_Sum_d = (rtu_lla[1] - rtu_llo[1]) + (real_T)rtb_Compare_dy_0;

  /* Product: '<S479>/Multiply' incorporates:
   *  Gain: '<S479>/deg2rad1'
   */
  rtb_Sum_e_idx_0 = 0.017453292519943295 * rtb_Sum_e_idx_0 * rtb_Gain_lq;

  /* Switch: '<S483>/Switch1' incorporates:
   *  Abs: '<S483>/Abs1'
   *  Bias: '<S483>/Bias2'
   *  Bias: '<S483>/Bias3'
   *  Constant: '<S483>/Constant'
   *  Constant: '<S485>/Constant'
   *  Math: '<S483>/Math Function'
   *  RelationalOperator: '<S485>/Compare'
   */
  if (fabs(rtb_Sum_d) > 180.0) {
    rtb_Sum_d = rt_modd(rtb_Sum_d + 180.0, 360.0) + -180.0;
  }

  /* End of Switch: '<S483>/Switch1' */

  /* Product: '<S479>/Multiply' incorporates:
   *  Gain: '<S479>/deg2rad1'
   */
  rtb_Sum_d = 0.017453292519943295 * rtb_Sum_d * rtb_Sum2_ee;

  /* Gain: '<S479>/deg2rad2' */
  rtb_Sum2_ee = 0.017453292519943295 * rtu_psio;

  /* Trigonometry: '<S482>/SinCos' */
  rtb_Sum3_l = sin(rtb_Sum2_ee);
  rtb_Gain_lq = cos(rtb_Sum2_ee);

  /* Sum: '<S482>/Sum2' incorporates:
   *  Product: '<S482>/Multiply1'
   *  Product: '<S482>/Multiply2'
   */
  rtb_Sum2_ee = rtb_Sum_e_idx_0 * rtb_Gain_lq + rtb_Sum_d * rtb_Sum3_l;

  /* Product: '<S482>/Multiply3' */
  rtb_Sum3_l *= rtb_Sum_e_idx_0;

  /* Product: '<S482>/Multiply4' */
  rtb_Gain_lq *= rtb_Sum_d;

  /* Sum: '<S482>/Sum3' */
  rtb_Sum3_l = rtb_Gain_lq - rtb_Sum3_l;

  /* DataTypeConversion: '<S476>/Data Type Conversion' incorporates:
   *  Gain: '<S479>/Gain'
   *  Sum: '<S479>/Sum1'
   */
  rty_pos[0] = (real32_T)rtb_Sum2_ee;
  rty_pos[1] = (real32_T)rtb_Sum3_l;
  rty_pos[2] = (real32_T)-(rtu_lla[2] + rtu_href);
}

/* Function for Chart: '<Root>/SafeMode' */
static void FMS_Stabilize(void)
{
  /* Inport: '<Root>/INS_Out' */
  if ((FMS_U.INS_Out.flag & 4U) != 0U) {
    FMS_B.target_mode = PilotMode_Stabilize;

    /* Delay: '<S15>/Delay' */
    switch (FMS_DW.Delay_DSTATE_c) {
     case PilotMode_Manual:
      FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
      break;

     case PilotMode_Acro:
      FMS_DW.is_c3_FMS = FMS_IN_Acro;
      break;

     case PilotMode_Stabilize:
      FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
      break;

     case PilotMode_Altitude:
      FMS_DW.is_c3_FMS = FMS_IN_Altitude;
      break;

     case PilotMode_Position:
      FMS_DW.is_c3_FMS = FMS_IN_Position_f;
      break;

     case PilotMode_Mission:
      FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
      break;

     case PilotMode_Offboard:
      FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
      break;

     default:
      FMS_DW.is_c3_FMS = FMS_IN_Other;
      break;
    }

    /* End of Delay: '<S15>/Delay' */
  } else {
    FMS_DW.is_c3_FMS = FMS_IN_Acro;
  }

  /* End of Inport: '<Root>/INS_Out' */
}

/* Function for Chart: '<Root>/SafeMode' */
static void FMS_Acro(void)
{
  /* Inport: '<Root>/INS_Out' */
  if ((FMS_U.INS_Out.flag & 4U) != 0U) {
    FMS_B.target_mode = PilotMode_Acro;

    /* Delay: '<S15>/Delay' */
    switch (FMS_DW.Delay_DSTATE_c) {
     case PilotMode_Manual:
      FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
      break;

     case PilotMode_Acro:
      FMS_DW.is_c3_FMS = FMS_IN_Acro;
      break;

     case PilotMode_Stabilize:
      FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
      break;

     case PilotMode_Altitude:
      FMS_DW.is_c3_FMS = FMS_IN_Altitude;
      break;

     case PilotMode_Position:
      FMS_DW.is_c3_FMS = FMS_IN_Position_f;
      break;

     case PilotMode_Mission:
      FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
      break;

     case PilotMode_Offboard:
      FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
      break;

     default:
      FMS_DW.is_c3_FMS = FMS_IN_Other;
      break;
    }

    /* End of Delay: '<S15>/Delay' */
  } else {
    FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
  }

  /* End of Inport: '<Root>/INS_Out' */
}

int32_T FMS_emplace(Queue_FMS_Cmd *q, const FMS_Cmd *dataIn)
{
  int32_T isEmplaced;
  int32_T newTail;
  Msg_FMS_Cmd *msg;
  newTail = (q->fTail + 1) % q->fCapacity;
  if (q->fHead == newTail) {
    isEmplaced = 0;
  } else {
    q->fTail = newTail;
    msg = &q->fArray[newTail];
    msg->fData = *dataIn;
    if (q->fHead == -1) {
      q->fHead = q->fTail;
    }

    isEmplaced = 1;
  }

  return isEmplaced;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_sf_msg_send_M(void)
{
  FMS_emplace(&FMS_DW.Queue_FMS_Cmd_b, &FMS_DW.M_msgReservedData);
}

/* Function for Chart: '<Root>/FMS State Machine' */
static boolean_T FMS_CheckCmdValid(FMS_Cmd cmd_in, PilotMode mode_in, uint32_T
  ins_flag)
{
  boolean_T valid;
  valid = false;
  if (!(mode_in == PilotMode_None)) {
    switch (cmd_in) {
     case FMS_Cmd_Takeoff:
     case FMS_Cmd_Land:
     case FMS_Cmd_Return:
     case FMS_Cmd_Pause:
      if (((ins_flag & 1U) != 0U) && ((ins_flag & 4U) != 0U) && ((ins_flag & 8U)
           != 0U) && ((ins_flag & 16U) != 0U) && ((ins_flag & 64U) != 0U) &&
          ((ins_flag & 128U) != 0U)) {
        valid = true;
      }
      break;

     case FMS_Cmd_PreArm:
      if (((ins_flag & 1U) == 0U) || ((ins_flag & 4U) == 0U) || ((ins_flag & 8U)
           == 0U)) {
      } else {
        switch (mode_in) {
         case PilotMode_Position:
         case PilotMode_Mission:
         case PilotMode_Offboard:
          if (((ins_flag & 16U) != 0U) && ((ins_flag & 64U) != 0U) && ((ins_flag
                & 128U) != 0U)) {
            valid = true;
          }
          break;

         case PilotMode_Altitude:
          if ((ins_flag & 128U) != 0U) {
            valid = true;
          }
          break;

         case PilotMode_Stabilize:
          valid = true;
          break;
        }
      }
      break;

     case FMS_Cmd_Continue:
      if ((mode_in == PilotMode_Offboard) || (mode_in == PilotMode_Mission)) {
        valid = true;
      }
      break;

     case FMS_Cmd_Disarm:
      valid = true;
      break;
    }
  }

  return valid;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static boolean_T FMS_BottomRight(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle)
{
  return (pilot_cmd_stick_throttle < -0.8) && (pilot_cmd_stick_yaw > 0.8);
}

/* Function for Chart: '<Root>/FMS State Machine' */
static boolean_T FMS_BottomLeft(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle)
{
  return (pilot_cmd_stick_throttle < -0.8) && (pilot_cmd_stick_yaw < -0.8);
}

int32_T FMS_pop(Queue_FMS_Cmd *q, Msg_FMS_Cmd *elementOut)
{
  int32_T isPop;
  if (q->fHead == -1) {
    isPop = 0;
  } else {
    *elementOut = q->fArray[q->fHead];
    isPop = 1;
    if (q->fHead == q->fTail) {
      q->fHead = -1;
      q->fTail = -1;
    } else {
      q->fHead = (q->fHead + 1) % q->fCapacity;
    }
  }

  return isPop;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static boolean_T FMS_sf_msg_pop_M(void)
{
  boolean_T isPresent;
  if (FMS_DW.M_isValid) {
    isPresent = true;
  } else {
    FMS_DW.M_msgHandle = FMS_pop(&FMS_DW.Queue_FMS_Cmd_b, &FMS_DW.Msg_FMS_Cmd_i
      [0]) != 0 ? (void *)&FMS_DW.Msg_FMS_Cmd_i[0] : NULL;
    if (FMS_DW.M_msgHandle != NULL) {
      FMS_DW.M_msgDataPtr = &((Msg_FMS_Cmd *)FMS_DW.M_msgHandle)->fData;
      isPresent = true;
      FMS_DW.M_msgReservedData = *(FMS_Cmd *)FMS_DW.M_msgDataPtr;
      FMS_DW.M_isValid = true;
    } else {
      isPresent = false;
      FMS_DW.M_isValid = false;
    }
  }

  return isPresent;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static real32_T FMS_norm(const real32_T x[2])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrtf(y);
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_Mission(void)
{
  real32_T tmp[2];
  uint32_T qY;
  int32_T tmp_0;
  if (FMS_DW.mission_timestamp_prev != FMS_DW.mission_timestamp_start) {
    FMS_DW.is_Mission = FMS_IN_NextWP;

    /* Inport: '<Root>/Mission_Data' */
    if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
      FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
    } else {
      FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
      qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
      if (qY > FMS_B.wp_index) {
        qY = 0U;
      }

      FMS_B.wp_consume = (uint8_T)qY;
    }
  } else {
    switch (FMS_DW.is_Mission) {
     case FMS_IN_Disarming:
      FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Vehicle = FMS_IN_Disarm;
      FMS_B.state = VehicleState_Disarm;
      break;

     case FMS_IN_Hold_d:
      /* Inport: '<Root>/Mission_Data' */
      if (FMS_DW.temporalCounter_i1 >= FMS_U.Mission_Data.param1[FMS_B.wp_index
          - 1] * 250.0F) {
        tmp_0 = (int32_T)(FMS_B.wp_index + 1U);
        if ((uint32_T)tmp_0 > 255U) {
          tmp_0 = 255;
        }

        FMS_B.wp_index = (uint8_T)tmp_0;
        FMS_DW.is_Mission = FMS_IN_NextWP;
        if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
          FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
        } else {
          FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
          qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
          if (qY > FMS_B.wp_index) {
            qY = 0U;
          }

          FMS_B.wp_consume = (uint8_T)qY;
        }
      }
      break;

     case FMS_IN_Land_j:
      if ((!FMS_B.AND) || (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1 = FMS_B.AND;
      if (FMS_DW.chartAbsoluteTimeCounter - FMS_DW.durationLastReferenceTick_1 >=
          500) {
        tmp_0 = (int32_T)(FMS_B.wp_index + 1U);
        if ((uint32_T)tmp_0 > 255U) {
          tmp_0 = 255;
        }

        FMS_B.wp_index = (uint8_T)tmp_0;
        FMS_DW.is_Mission = FMS_IN_NextWP;

        /* Inport: '<Root>/Mission_Data' */
        if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
          FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
        } else {
          FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
          qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
          if (qY > FMS_B.wp_index) {
            qY = 0U;
          }

          FMS_B.wp_consume = (uint8_T)qY;
        }
      }
      break;

     case FMS_IN_Loiter_p:
      break;

     case FMS_IN_NextWP:
      if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Takeoff) {
        FMS_DW.is_Mission = FMS_IN_Takeoff_d;

        /* Inport: '<Root>/Mission_Data' */
        FMS_B.lla[0] = (real_T)FMS_U.Mission_Data.x[FMS_B.wp_index - 1] * 1.0E-7;
        FMS_B.lla[1] = (real_T)FMS_U.Mission_Data.y[FMS_B.wp_index - 1] * 1.0E-7;
        FMS_B.lla[2] = -(FMS_U.Mission_Data.z[FMS_B.wp_index - 1] + FMS_DW.home
                         [2]);
        FMS_B.llo[0] = FMS_DW.llo[0];
        FMS_B.llo[1] = FMS_DW.llo[1];
        FMS_B.href = 0.0;
        FMS_B.psio = 0.0;

        /* Outputs for Function Call SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
        F_VehicleArmAutoMissionLLA2FLAT(FMS_B.lla, FMS_B.llo, FMS_B.href,
          FMS_B.psio, FMS_B.DataTypeConversion,
          &FMS_ConstB.VehicleArmAutoMissionLLA2FLAT);

        /* End of Outputs for SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.DataTypeConversion[0];
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.DataTypeConversion[1];
        FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.DataTypeConversion[2];
        FMS_B.state = VehicleState_Takeoff;
      } else if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Waypoint) {
        FMS_DW.is_Mission = FMS_IN_Waypoint;
        FMS_B.Cmd_In.cur_waypoint[0] = FMS_B.Cmd_In.sp_waypoint[0];
        FMS_B.Cmd_In.cur_waypoint[1] = FMS_B.Cmd_In.sp_waypoint[1];
        FMS_B.Cmd_In.cur_waypoint[2] = FMS_B.Cmd_In.sp_waypoint[2];

        /* Inport: '<Root>/Mission_Data' */
        FMS_B.lla[0] = (real_T)FMS_U.Mission_Data.x[FMS_B.wp_index - 1] * 1.0E-7;
        FMS_B.lla[1] = (real_T)FMS_U.Mission_Data.y[FMS_B.wp_index - 1] * 1.0E-7;
        FMS_B.lla[2] = -(FMS_U.Mission_Data.z[FMS_B.wp_index - 1] + FMS_DW.home
                         [2]);
        FMS_B.llo[0] = FMS_DW.llo[0];
        FMS_B.llo[1] = FMS_DW.llo[1];
        FMS_B.href = 0.0;
        FMS_B.psio = 0.0;

        /* Outputs for Function Call SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
        F_VehicleArmAutoMissionLLA2FLAT(FMS_B.lla, FMS_B.llo, FMS_B.href,
          FMS_B.psio, FMS_B.DataTypeConversion,
          &FMS_ConstB.VehicleArmAutoMissionLLA2FLAT);

        /* End of Outputs for SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.DataTypeConversion[0];
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.DataTypeConversion[1];
        FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.DataTypeConversion[2];
        FMS_B.state = VehicleState_Mission;
      } else if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_SetSpeed) {
        FMS_DW.is_Mission = FMS_IN_SetSpeed;

        /* Inport: '<Root>/Mission_Data' */
        FMS_B.Cmd_In.set_speed = FMS_U.Mission_Data.param2[FMS_B.wp_index - 1];
      } else if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Land) {
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
        FMS_DW.is_Mission = FMS_IN_Land_j;
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
        FMS_B.Cmd_In.sp_waypoint[2] = 0.0F;
        FMS_B.state = VehicleState_Land;
        FMS_DW.condWasTrueAtLastTimeStep_1 = FMS_B.AND;
      } else if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Return) {
        FMS_DW.is_Mission = FMS_IN_Return_h;
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_DW.home[0];
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_DW.home[1];
        FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
        FMS_B.state = VehicleState_Return;
      } else if (FMS_B.AND) {
        FMS_DW.is_Mission = FMS_IN_Disarming;
      } else {
        FMS_DW.is_Mission = FMS_IN_Loiter_p;
        FMS_B.state = VehicleState_Hold;
      }
      break;

     case FMS_IN_Return_h:
      tmp[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R -
        FMS_B.Cmd_In.sp_waypoint[0];
      tmp[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R -
        FMS_B.Cmd_In.sp_waypoint[1];
      if (FMS_norm(tmp) < 0.5F) {
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
        FMS_DW.is_Mission = FMS_IN_Land_j;
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
        FMS_B.Cmd_In.sp_waypoint[2] = 0.0F;
        FMS_B.state = VehicleState_Land;
        FMS_DW.condWasTrueAtLastTimeStep_1 = FMS_B.AND;
      }
      break;

     case FMS_IN_SetSpeed:
      tmp_0 = (int32_T)(FMS_B.wp_index + 1U);
      if ((uint32_T)tmp_0 > 255U) {
        tmp_0 = 255;
      }

      FMS_B.wp_index = (uint8_T)tmp_0;
      FMS_DW.is_Mission = FMS_IN_NextWP;

      /* Inport: '<Root>/Mission_Data' */
      if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
        FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
      } else {
        FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
        qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
        if (qY > FMS_B.wp_index) {
          qY = 0U;
        }

        FMS_B.wp_consume = (uint8_T)qY;
      }
      break;

     case FMS_IN_Takeoff_d:
      if (FMS_B.BusConversion_InsertedFor_FMSSt.h_R >= FMS_B.Cmd_In.sp_waypoint
          [2]) {
        tmp_0 = (int32_T)(FMS_B.wp_index + 1U);
        if ((uint32_T)tmp_0 > 255U) {
          tmp_0 = 255;
        }

        FMS_B.wp_index = (uint8_T)tmp_0;
        FMS_DW.is_Mission = FMS_IN_NextWP;

        /* Inport: '<Root>/Mission_Data' */
        if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
          FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
        } else {
          FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
          qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
          if (qY > FMS_B.wp_index) {
            qY = 0U;
          }

          FMS_B.wp_consume = (uint8_T)qY;
        }
      }
      break;

     case FMS_IN_Waypoint:
      tmp[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R -
        FMS_B.Cmd_In.sp_waypoint[0];
      tmp[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R -
        FMS_B.Cmd_In.sp_waypoint[1];

      /* Constant: '<Root>/ACCEPT_R' */
      if (FMS_norm(tmp) <= FMS_PARAM.ACCEPT_R) {
        FMS_B.Cmd_In.set_speed = 0.0F;

        /* Inport: '<Root>/Mission_Data' */
        if (FMS_U.Mission_Data.param1[FMS_B.wp_index - 1] > 0.0F) {
          FMS_DW.is_Mission = FMS_IN_Hold_d;
          FMS_DW.temporalCounter_i1 = 0U;
          FMS_B.state = VehicleState_Hold;
        } else {
          tmp_0 = (int32_T)(FMS_B.wp_index + 1U);
          if ((uint32_T)tmp_0 > 255U) {
            tmp_0 = 255;
          }

          FMS_B.wp_index = (uint8_T)tmp_0;
          FMS_DW.is_Mission = FMS_IN_NextWP;
          if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
            FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
          } else {
            FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
            qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
            if (qY > FMS_B.wp_index) {
              qY = 0U;
            }

            FMS_B.wp_consume = (uint8_T)qY;
          }
        }
      }

      /* End of Constant: '<Root>/ACCEPT_R' */
      break;
    }
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static real_T FMS_getArmMode(PilotMode pilotMode)
{
  real_T armMode;
  switch (pilotMode) {
   case PilotMode_Manual:
    armMode = 1.0;
    break;

   case PilotMode_Acro:
    armMode = 2.0;
    break;

   case PilotMode_Stabilize:
    armMode = 2.0;
    break;

   case PilotMode_Altitude:
    armMode = 2.0;
    break;

   case PilotMode_Position:
    armMode = 2.0;
    break;

   case PilotMode_Mission:
    armMode = 3.0;
    break;

   case PilotMode_Offboard:
    armMode = 3.0;
    break;

   default:
    armMode = 0.0;
    break;
  }

  return armMode;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_enter_internal_Auto(void)
{
  uint32_T qY;
  switch (FMS_B.target_mode) {
   case PilotMode_Offboard:
    FMS_DW.is_Auto = FMS_IN_Offboard;
    FMS_B.Cmd_In.offboard_psi_0 = FMS_B.BusConversion_InsertedFor_FMSSt.psi;
    if (FMS_B.LogicalOperator) {
      FMS_DW.is_Offboard = FMS_IN_Run;
      FMS_B.state = VehicleState_Offboard;
    } else {
      FMS_DW.is_Offboard = FMS_IN_Loiter;
      FMS_B.state = VehicleState_Hold;
    }
    break;

   case PilotMode_Mission:
    FMS_DW.is_Auto = FMS_IN_Mission;
    FMS_DW.llo[0] = FMS_B.BusConversion_InsertedFor_FMSSt.lat_0 *
      57.295779513082323;
    FMS_DW.llo[1] = FMS_B.BusConversion_InsertedFor_FMSSt.lon_0 *
      57.295779513082323;
    FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
    FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
    FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
    FMS_B.Cmd_In.set_speed = 0.0F;
    FMS_DW.is_Mission = FMS_IN_NextWP;

    /* Inport: '<Root>/Mission_Data' */
    if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
      FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index - 1];
    } else {
      FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
      qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
      if (qY > FMS_B.wp_index) {
        qY = 0U;
      }

      FMS_B.wp_consume = (uint8_T)qY;
    }

    /* End of Inport: '<Root>/Mission_Data' */
    break;

   default:
    FMS_DW.is_Auto = FMS_IN_InvalidAutoMode;
    break;
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_enter_internal_Arm(void)
{
  real_T tmp;
  tmp = FMS_getArmMode(FMS_B.target_mode);
  if (tmp == 3.0) {
    FMS_DW.is_Arm = FMS_IN_Auto;
    FMS_enter_internal_Auto();
  } else if (tmp == 2.0) {
    FMS_DW.is_Arm = FMS_IN_Assist;
    switch (FMS_B.target_mode) {
     case PilotMode_Acro:
      FMS_DW.is_Assist = FMS_IN_Acro;
      FMS_B.state = VehicleState_Acro;
      break;

     case PilotMode_Stabilize:
      FMS_DW.is_Assist = FMS_IN_Stabilize;
      FMS_B.state = VehicleState_Stabilize;
      break;

     case PilotMode_Altitude:
      FMS_DW.is_Assist = FMS_IN_Altitude;
      FMS_B.state = VehicleState_Altitude;
      break;

     case PilotMode_Position:
      FMS_DW.is_Assist = FMS_IN_Position;
      FMS_B.state = VehicleState_Position;
      break;

     default:
      FMS_DW.is_Assist = FMS_IN_InvalidAssistMode;
      break;
    }
  } else if (tmp == 1.0) {
    FMS_DW.is_Arm = FMS_IN_Manual;
    if (FMS_B.target_mode == PilotMode_Manual) {
      FMS_DW.is_Manual = FMS_IN_Manual_g;
      FMS_B.state = VehicleState_Manual;
    } else {
      FMS_DW.is_Manual = FMS_IN_InValidManualMode;
    }
  } else {
    FMS_DW.is_Arm = FMS_IN_InvalidArmMode;
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_SubMode(void)
{
  boolean_T b_sf_internal_predicateOutput;
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real_T tmp;
  FMS_B.stick_val[0] = FMS_DW.stick_val[0];
  FMS_B.stick_val[1] = FMS_DW.stick_val[1];
  FMS_B.stick_val[2] = FMS_DW.stick_val[2];
  FMS_B.stick_val[3] = FMS_DW.stick_val[3];
  FMS_B.pilot_cmd = FMS_B.BusConversion_InsertedFor_FMS_f;

  /* Outputs for Function Call SubSystem: '<S5>/Vehicle.StickMoved' */
  /* RelationalOperator: '<S489>/Compare' incorporates:
   *  Abs: '<S478>/Abs'
   *  Constant: '<S489>/Constant'
   *  MinMax: '<S478>/Max'
   *  Sum: '<S478>/Sum'
   */
  FMS_B.Compare_k = (fmax(fmax(fmax(fabs(FMS_B.stick_val[0] -
    FMS_B.pilot_cmd.stick_yaw), fabs(FMS_B.stick_val[1] -
    FMS_B.pilot_cmd.stick_throttle)), fabs(FMS_B.stick_val[2] -
    FMS_B.pilot_cmd.stick_roll)), fabs(FMS_B.stick_val[3] -
    FMS_B.pilot_cmd.stick_pitch)) >= 0.1);

  /* End of Outputs for SubSystem: '<S5>/Vehicle.StickMoved' */
  if ((FMS_B.Compare_k || ((FMS_B.BusConversion_InsertedFor_FMSSt.flag & 212U)
        != 212U)) && (FMS_B.target_mode != PilotMode_None)) {
    if (FMS_getArmMode(FMS_B.target_mode) == 3.0) {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Arm = FMS_IN_Auto;
      FMS_enter_internal_Auto();
    } else if (FMS_getArmMode(FMS_B.target_mode) == 2.0) {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Arm = FMS_IN_Assist;
      switch (FMS_B.target_mode) {
       case PilotMode_Acro:
        FMS_DW.is_Assist = FMS_IN_Acro;
        FMS_B.state = VehicleState_Acro;
        break;

       case PilotMode_Stabilize:
        FMS_DW.is_Assist = FMS_IN_Stabilize;
        FMS_B.state = VehicleState_Stabilize;
        break;

       case PilotMode_Altitude:
        FMS_DW.is_Assist = FMS_IN_Altitude;
        FMS_B.state = VehicleState_Altitude;
        break;

       case PilotMode_Position:
        FMS_DW.is_Assist = FMS_IN_Position;
        FMS_B.state = VehicleState_Position;
        break;

       default:
        FMS_DW.is_Assist = FMS_IN_InvalidAssistMode;
        break;
      }
    } else if (FMS_getArmMode(FMS_B.target_mode) == 1.0) {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Arm = FMS_IN_Manual;
      if (FMS_B.target_mode == PilotMode_Manual) {
        FMS_DW.is_Manual = FMS_IN_Manual_g;
        FMS_B.state = VehicleState_Manual;
      } else {
        FMS_DW.is_Manual = FMS_IN_InValidManualMode;
      }
    } else {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Arm = FMS_IN_InvalidArmMode;
    }
  } else {
    switch (FMS_DW.is_SubMode) {
     case FMS_IN_Hold_h:
      if (FMS_sf_msg_pop_M()) {
        b_sf_internal_predicateOutput = ((FMS_DW.M_msgReservedData ==
          FMS_Cmd_Continue) && (FMS_B.target_mode != PilotMode_None));
      } else {
        b_sf_internal_predicateOutput = false;
      }

      if (b_sf_internal_predicateOutput) {
        FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
        FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
        FMS_enter_internal_Arm();
      }
      break;

     case FMS_IN_Land:
      if ((!FMS_B.AND) || (!FMS_DW.condWasTrueAtLastTimeStep_1_k)) {
        FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_B.AND;
      if (FMS_DW.chartAbsoluteTimeCounter - FMS_DW.durationLastReferenceTick_1_k
          >= 125) {
        FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
        FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
        FMS_DW.is_Vehicle = FMS_IN_Disarm;
        FMS_B.state = VehicleState_Disarm;
      }
      break;

     case FMS_IN_Return:
      scale = 1.29246971E-26F;
      absxk = fabsf(FMS_B.BusConversion_InsertedFor_FMSSt.x_R - FMS_DW.home[0]);
      if (absxk > 1.29246971E-26F) {
        y = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        y = t * t;
      }

      absxk = fabsf(FMS_B.BusConversion_InsertedFor_FMSSt.y_R - FMS_DW.home[1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * sqrtf(y);

      /* Constant: '<Root>/ACCEPT_R' */
      if (y <= FMS_PARAM.ACCEPT_R) {
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_DW.home[0];
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_DW.home[1];
        FMS_B.Cmd_In.sp_waypoint[2] = 0.0F;
        FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
        FMS_DW.is_SubMode = FMS_IN_Land;
        FMS_B.state = VehicleState_Land;
        FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_B.AND;
      }

      /* End of Constant: '<Root>/ACCEPT_R' */
      break;

     case FMS_IN_Takeoff:
      if (FMS_B.BusConversion_InsertedFor_FMSSt.h_R >= FMS_B.Cmd_In.sp_waypoint
          [2]) {
        if (FMS_B.target_mode != PilotMode_None) {
          tmp = FMS_getArmMode(FMS_B.target_mode);
          if (tmp == 3.0) {
            FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_Auto;
            FMS_enter_internal_Auto();
          } else if (tmp == 2.0) {
            FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_Assist;
            switch (FMS_B.target_mode) {
             case PilotMode_Acro:
              FMS_DW.is_Assist = FMS_IN_Acro;
              FMS_B.state = VehicleState_Acro;
              break;

             case PilotMode_Stabilize:
              FMS_DW.is_Assist = FMS_IN_Stabilize;
              FMS_B.state = VehicleState_Stabilize;
              break;

             case PilotMode_Altitude:
              FMS_DW.is_Assist = FMS_IN_Altitude;
              FMS_B.state = VehicleState_Altitude;
              break;

             case PilotMode_Position:
              FMS_DW.is_Assist = FMS_IN_Position;
              FMS_B.state = VehicleState_Position;
              break;

             default:
              FMS_DW.is_Assist = FMS_IN_InvalidAssistMode;
              break;
            }
          } else if (tmp == 1.0) {
            FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_Manual;
            if (FMS_B.target_mode == PilotMode_Manual) {
              FMS_DW.is_Manual = FMS_IN_Manual_g;
              FMS_B.state = VehicleState_Manual;
            } else {
              FMS_DW.is_Manual = FMS_IN_InValidManualMode;
            }
          } else {
            FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_InvalidArmMode;
          }
        } else {
          FMS_DW.is_SubMode = FMS_IN_Hold_h;
          FMS_B.state = VehicleState_Hold;
        }
      }
      break;
    }
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_exit_internal_Arm(void)
{
  if (FMS_DW.is_Arm == FMS_IN_Auto) {
    if (FMS_DW.is_Auto == FMS_IN_Mission) {
      FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD_h;
    } else {
      FMS_DW.is_Offboard = FMS_IN_NO_ACTIVE_CHILD_h;
      FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD_h;
    }

    FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
  } else {
    FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD_h;
    FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD_h;
    FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
    FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_Arm(void)
{
  boolean_T sf_internal_predicateOutput;
  real_T tmp;
  if (FMS_sf_msg_pop_M()) {
    sf_internal_predicateOutput = (FMS_DW.M_msgReservedData == FMS_Cmd_Pause);
  } else {
    sf_internal_predicateOutput = false;
  }

  if (sf_internal_predicateOutput) {
    FMS_exit_internal_Arm();
    FMS_DW.is_Arm = FMS_IN_SubMode;
    FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
    FMS_DW.stick_val[1] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
    FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
    FMS_DW.stick_val[3] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
    FMS_DW.is_SubMode = FMS_IN_Hold_h;
    FMS_B.state = VehicleState_Hold;
  } else if ((FMS_DW.mode_prev != FMS_DW.mode_start) && (FMS_B.target_mode !=
              PilotMode_None)) {
    tmp = FMS_getArmMode(FMS_B.target_mode);
    if (tmp == 3.0) {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_Auto;
      FMS_enter_internal_Auto();
    } else if (tmp == 2.0) {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_Assist;
      switch (FMS_B.target_mode) {
       case PilotMode_Acro:
        FMS_DW.is_Assist = FMS_IN_Acro;
        FMS_B.state = VehicleState_Acro;
        break;

       case PilotMode_Stabilize:
        FMS_DW.is_Assist = FMS_IN_Stabilize;
        FMS_B.state = VehicleState_Stabilize;
        break;

       case PilotMode_Altitude:
        FMS_DW.is_Assist = FMS_IN_Altitude;
        FMS_B.state = VehicleState_Altitude;
        break;

       case PilotMode_Position:
        FMS_DW.is_Assist = FMS_IN_Position;
        FMS_B.state = VehicleState_Position;
        break;

       default:
        FMS_DW.is_Assist = FMS_IN_InvalidAssistMode;
        break;
      }
    } else if (tmp == 1.0) {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_Manual;
      if (FMS_B.target_mode == PilotMode_Manual) {
        FMS_DW.is_Manual = FMS_IN_Manual_g;
        FMS_B.state = VehicleState_Manual;
      } else {
        FMS_DW.is_Manual = FMS_IN_InValidManualMode;
      }
    } else {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_InvalidArmMode;
    }
  } else {
    if (FMS_sf_msg_pop_M()) {
      sf_internal_predicateOutput = (FMS_DW.M_msgReservedData == FMS_Cmd_Land);
    } else {
      sf_internal_predicateOutput = false;
    }

    if (sf_internal_predicateOutput) {
      FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
      FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
      FMS_B.Cmd_In.sp_waypoint[2] = 0.0F;
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_SubMode;
      FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
      FMS_DW.stick_val[1] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
      FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
      FMS_DW.stick_val[3] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
      FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
      FMS_DW.is_SubMode = FMS_IN_Land;
      FMS_B.state = VehicleState_Land;
      FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_B.AND;
    } else {
      if (FMS_sf_msg_pop_M()) {
        sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
          FMS_Cmd_Return);
      } else {
        sf_internal_predicateOutput = false;
      }

      if (sf_internal_predicateOutput) {
        FMS_B.Cmd_In.sp_waypoint[0] = FMS_DW.home[0];
        FMS_B.Cmd_In.sp_waypoint[1] = FMS_DW.home[1];
        FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
        FMS_exit_internal_Arm();
        FMS_DW.is_Arm = FMS_IN_SubMode;
        FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
        FMS_DW.stick_val[1] =
          FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
        FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
        FMS_DW.stick_val[3] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
        FMS_DW.is_SubMode = FMS_IN_Return;
        FMS_B.state = VehicleState_Return;
      } else {
        switch (FMS_DW.is_Arm) {
         case FMS_IN_Assist:
          if (FMS_B.Compare && ((int32_T)
                                (FMS_B.BusConversion_InsertedFor_FMSSt.flag &
                                 212U) == 212)) {
            FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.durationLastReferenceTick_1_n5 =
              FMS_DW.chartAbsoluteTimeCounter;
            FMS_DW.is_Vehicle = FMS_IN_Arm;
            FMS_DW.condWasTrueAtLastTimeStep_1_h = FMS_B.AND;
            FMS_DW.is_Arm = FMS_IN_SubMode;
            FMS_DW.stick_val[0] =
              FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
            FMS_DW.stick_val[1] =
              FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
            FMS_DW.stick_val[2] =
              FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
            FMS_DW.stick_val[3] =
              FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
            FMS_DW.is_SubMode = FMS_IN_Hold_h;
            FMS_B.state = VehicleState_Hold;
          } else {
            if (FMS_DW.is_Assist == FMS_IN_InvalidAssistMode) {
              FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD_h;
              FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
              FMS_DW.is_Vehicle = FMS_IN_Disarm;
              FMS_B.state = VehicleState_Disarm;
            }
          }
          break;

         case FMS_IN_Auto:
          switch (FMS_DW.is_Auto) {
           case FMS_IN_InvalidAutoMode:
            FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
            break;

           case FMS_IN_Mission:
            FMS_Mission();
            break;

           case FMS_IN_Offboard:
            switch (FMS_DW.is_Offboard) {
             case FMS_IN_Loiter:
              if (FMS_B.LogicalOperator) {
                FMS_DW.is_Offboard = FMS_IN_Run;
                FMS_B.state = VehicleState_Offboard;
              }
              break;

             case FMS_IN_Run:
              if (!FMS_B.LogicalOperator) {
                FMS_DW.is_Offboard = FMS_IN_Loiter;
                FMS_B.state = VehicleState_Hold;
              }
              break;
            }
            break;
          }
          break;

         case FMS_IN_InvalidArmMode:
          FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
          FMS_DW.is_Vehicle = FMS_IN_Disarm;
          FMS_B.state = VehicleState_Disarm;
          break;

         case FMS_IN_Manual:
          if (FMS_DW.is_Manual == FMS_IN_InValidManualMode) {
            FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
          }
          break;

         case FMS_IN_SubMode:
          FMS_SubMode();
          break;
        }
      }
    }
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static real_T FMS_ManualArmEvent(real32_T pilot_cmd_stick_throttle, uint32_T
  pilot_cmd_mode)
{
  real_T trigger;
  trigger = 0.0;
  switch (pilot_cmd_mode) {
   case PilotMode_Manual:
   case PilotMode_Acro:
   case PilotMode_Stabilize:
    if (pilot_cmd_stick_throttle > -0.7) {
      trigger = 1.0;
    }
    break;

   case PilotMode_Altitude:
   case PilotMode_Position:
    if (pilot_cmd_stick_throttle > 0.1) {
      trigger = 1.0;
    }
    break;
  }

  return trigger;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_Vehicle(void)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  int32_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  if (FMS_DW.mission_timestamp_prev != FMS_DW.mission_timestamp_start) {
    FMS_B.wp_consume = 0U;
    FMS_B.wp_index = 1U;
  }

  if (FMS_sf_msg_pop_M()) {
    sf_internal_predicateOutput = (FMS_DW.M_msgReservedData == FMS_Cmd_Disarm);
  } else {
    sf_internal_predicateOutput = false;
  }

  if (sf_internal_predicateOutput) {
    switch (FMS_DW.is_Vehicle) {
     case FMS_IN_Arm:
      FMS_exit_internal_Arm();
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD_h;
      break;

     case FMS_IN_Standby:
      FMS_DW.prep_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) ||
          (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
        FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
      FMS_DW.prep_mission_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) || (!FMS_DW.condWasTrueAtLastTimeStep_2))
      {
        FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD_h;
      break;

     default:
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD_h;
      break;
    }

    FMS_DW.is_Vehicle = FMS_IN_Disarm;
    FMS_B.state = VehicleState_Disarm;
  } else {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    switch (FMS_DW.is_Vehicle) {
     case FMS_IN_Arm:
      FMS_Arm();
      break;

     case FMS_IN_Disarm:
      if (FMS_sf_msg_pop_M()) {
        sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
          FMS_Cmd_PreArm);
      } else {
        sf_internal_predicateOutput = false;
      }

      if (sf_internal_predicateOutput) {
        guard1 = true;
      } else {
        if (FMS_sf_msg_pop_M()) {
          sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
            FMS_Cmd_Takeoff);
        } else {
          sf_internal_predicateOutput = false;
        }

        if (sf_internal_predicateOutput) {
          FMS_DW.prep_takeoff = 1.0;
          sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
          if ((!sf_internal_predicateOutput) ||
              (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
            FMS_DW.durationLastReferenceTick_1_n =
              FMS_DW.chartAbsoluteTimeCounter;
          }

          FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
          guard1 = true;
        }
      }
      break;

     case FMS_IN_Standby:
      if ((FMS_ManualArmEvent
           (FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle,
            FMS_B.BusConversion_InsertedFor_FMS_f.mode) == 1.0) &&
          (FMS_B.target_mode != PilotMode_None)) {
        guard2 = true;
      } else {
        sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
        if ((!sf_internal_predicateOutput) ||
            (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
          FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
        if (FMS_DW.chartAbsoluteTimeCounter -
            FMS_DW.durationLastReferenceTick_1_n >= 500) {
          guard3 = true;
        } else {
          if (FMS_sf_msg_pop_M()) {
            sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
              FMS_Cmd_Takeoff);
          } else {
            sf_internal_predicateOutput = false;
          }

          if (sf_internal_predicateOutput) {
            guard3 = true;
          } else if ((FMS_DW.temporalCounter_i1 >= 2500U) || (FMS_DW.sfEvent ==
                      FMS_event_DisarmEvent)) {
            FMS_DW.prep_takeoff = 0.0;
            sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
            if ((!sf_internal_predicateOutput) ||
                (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
              FMS_DW.durationLastReferenceTick_1_n =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
            FMS_DW.prep_mission_takeoff = 0.0;
            sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
            if ((!sf_internal_predicateOutput) ||
                (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
              FMS_DW.durationLastReferenceTick_2 =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
          } else {
            sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
            if ((!sf_internal_predicateOutput) ||
                (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
              FMS_DW.durationLastReferenceTick_2 =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
            if ((FMS_DW.chartAbsoluteTimeCounter -
                 FMS_DW.durationLastReferenceTick_2 >= 125) ||
                ((FMS_B.target_mode == PilotMode_Offboard) &&
                 FMS_B.LogicalOperator)) {
              guard2 = true;
            }
          }
        }
      }
      break;
    }

    if (guard3) {
      FMS_B.xy_R[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
      FMS_B.xy_R[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;

      /* Outputs for Function Call SubSystem: '<S5>/Vehicle.PrepTakeoff' */
      /* Reshape: '<S477>/Reshape' incorporates:
       *  Constant: '<S477>/Constant'
       */
      FMS_B.Reshape[0] = FMS_B.xy_R[0];
      FMS_B.Reshape[1] = FMS_B.xy_R[1];
      FMS_B.Reshape[2] = FMS_PARAM.TAKEOFF_H;

      /* End of Outputs for SubSystem: '<S5>/Vehicle.PrepTakeoff' */
      FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.Reshape[0];
      FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.Reshape[1];
      FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.Reshape[2];
      FMS_B.Cmd_In.sp_waypoint[2] += FMS_DW.home[2];
      FMS_DW.prep_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) ||
          (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
        FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
      FMS_DW.prep_mission_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) || (!FMS_DW.condWasTrueAtLastTimeStep_2))
      {
        FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
      FMS_DW.durationLastReferenceTick_1_n5 = FMS_DW.chartAbsoluteTimeCounter;
      FMS_DW.is_Vehicle = FMS_IN_Arm;
      FMS_DW.condWasTrueAtLastTimeStep_1_h = FMS_B.AND;
      FMS_DW.is_Arm = FMS_IN_SubMode;
      FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
      FMS_DW.stick_val[1] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
      FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
      FMS_DW.stick_val[3] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
      FMS_DW.is_SubMode = FMS_IN_Takeoff;
      FMS_B.state = VehicleState_Takeoff;
    }

    if (guard2) {
      FMS_DW.prep_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) ||
          (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
        FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
      FMS_DW.prep_mission_takeoff = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
      if ((!sf_internal_predicateOutput) || (!FMS_DW.condWasTrueAtLastTimeStep_2))
      {
        FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
      FMS_DW.durationLastReferenceTick_1_n5 = FMS_DW.chartAbsoluteTimeCounter;
      FMS_DW.is_Vehicle = FMS_IN_Arm;
      FMS_DW.condWasTrueAtLastTimeStep_1_h = FMS_B.AND;
      FMS_enter_internal_Arm();
    }

    if (guard1) {
      FMS_DW.condWasTrueAtLastTimeStep_2 = false;
      FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
      FMS_DW.condWasTrueAtLastTimeStep_1_b = false;
      FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
      FMS_DW.is_Vehicle = FMS_IN_Standby;
      FMS_DW.temporalCounter_i1 = 0U;
      guard4 = false;
      guard5 = false;
      guard6 = false;
      if (FMS_B.target_mode == PilotMode_Mission) {
        if ((FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) &&
            (FMS_U.Mission_Data.command[FMS_B.wp_index - 1] == (int32_T)
             NAV_Cmd_Takeoff)) {
          guard6 = true;
        } else {
          b_previousEvent = (int32_T)(FMS_B.wp_index + 1U);
          tmp = b_previousEvent;
          if ((uint32_T)b_previousEvent > 255U) {
            tmp = 255;
          }

          if ((tmp <= FMS_U.Mission_Data.valid_items) &&
              (FMS_U.Mission_Data.command[FMS_B.wp_index - 1] == (int32_T)
               NAV_Cmd_SetSpeed)) {
            if ((uint32_T)b_previousEvent > 255U) {
              b_previousEvent = 255;
            }

            if (FMS_U.Mission_Data.command[b_previousEvent - 1] == (int32_T)
                NAV_Cmd_Takeoff) {
              guard6 = true;
            } else {
              guard5 = true;
            }
          } else {
            guard5 = true;
          }
        }
      } else {
        guard4 = true;
      }

      if (guard6) {
        FMS_DW.prep_mission_takeoff = 1.0;
        FMS_DW.condWasTrueAtLastTimeStep_2 = (FMS_DW.prep_mission_takeoff == 1.0);
        FMS_DW.prep_takeoff = 0.0;
        FMS_DW.condWasTrueAtLastTimeStep_1_b = (FMS_DW.prep_takeoff == 1.0);
        guard4 = true;
      }

      if (guard5) {
        b_previousEvent = FMS_DW.sfEvent;
        FMS_DW.sfEvent = FMS_event_DisarmEvent;

        /* Chart: '<Root>/FMS State Machine' */
        FMS_c11_FMS();
        FMS_DW.sfEvent = b_previousEvent;
        if (FMS_DW.is_Vehicle != FMS_IN_Standby) {
        } else {
          guard4 = true;
        }
      }

      if (guard4) {
        FMS_DW.home[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
        FMS_DW.home[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
        FMS_DW.home[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
        FMS_DW.home[3] = FMS_B.BusConversion_InsertedFor_FMSSt.psi;
        FMS_B.state = VehicleState_Standby;
      }

      if (FMS_DW.is_Vehicle == FMS_IN_Standby) {
        sf_internal_predicateOutput = (FMS_DW.prep_takeoff == 1.0);
        if ((!sf_internal_predicateOutput) ||
            (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
          FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1_b = sf_internal_predicateOutput;
        sf_internal_predicateOutput = (FMS_DW.prep_mission_takeoff == 1.0);
        if ((!sf_internal_predicateOutput) ||
            (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
          FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_2 = sf_internal_predicateOutput;
      }
    }
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_c11_FMS(void)
{
  int32_T b_previousEvent;

  /* Chart: '<Root>/FMS State Machine' incorporates:
   *  Inport: '<Root>/Mission_Data'
   */
  if (FMS_DW.is_active_c11_FMS == 0U) {
    FMS_DW.mission_timestamp_prev = FMS_U.Mission_Data.timestamp;
    FMS_DW.mission_timestamp_start = FMS_U.Mission_Data.timestamp;
    FMS_DW.mode_prev = FMS_B.target_mode;
    FMS_DW.mode_start = FMS_B.target_mode;
    FMS_DW.cmd_prev = FMS_B.Switch1;
    FMS_DW.cmd_start = FMS_B.Switch1;
    FMS_DW.chartAbsoluteTimeCounter = 0;
    FMS_DW.is_active_c11_FMS = 1U;
    FMS_DW.is_active_Command_Listener = 1U;
    FMS_DW.is_Command_Listener = FMS_IN_Listen;
    FMS_DW.is_active_Combo_Stick = 1U;
    FMS_DW.durationLastReferenceTick_2_n = FMS_DW.chartAbsoluteTimeCounter;
    FMS_DW.durationLastReferenceTick_1_b = FMS_DW.chartAbsoluteTimeCounter;
    FMS_DW.is_Combo_Stick = FMS_IN_Idle;
    FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
    FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
    FMS_DW.is_active_Lost_Return = 1U;
    FMS_DW.is_Lost_Return = FMS_IN_Connect;
    FMS_DW.is_active_Vehicle = 1U;
    FMS_DW.is_Vehicle = FMS_IN_Disarm;
    FMS_B.state = VehicleState_Disarm;
  } else {
    if (FMS_DW.is_active_Command_Listener != 0U) {
      switch (FMS_DW.is_Command_Listener) {
       case FMS_IN_Check:
        if (FMS_DW.valid_cmd) {
          FMS_DW.is_Command_Listener = FMS_IN_Send;
          FMS_DW.M_msgReservedData = FMS_DW.save_cmd;
          FMS_sf_msg_send_M();
        } else {
          FMS_DW.is_Command_Listener = FMS_IN_Listen;
        }
        break;

       case FMS_IN_Listen:
        if ((FMS_DW.cmd_prev != FMS_DW.cmd_start) && (FMS_B.Switch1 !=
             FMS_Cmd_None)) {
          FMS_DW.save_cmd = FMS_B.Switch1;
          FMS_DW.is_Command_Listener = FMS_IN_Check;
          FMS_DW.valid_cmd = FMS_CheckCmdValid(FMS_DW.save_cmd,
            FMS_B.target_mode, FMS_B.BusConversion_InsertedFor_FMSSt.flag);
        }
        break;

       case FMS_IN_Send:
        FMS_DW.is_Command_Listener = FMS_IN_Listen;
        break;
      }
    }

    if (FMS_DW.is_active_Combo_Stick != 0U) {
      switch (FMS_DW.is_Combo_Stick) {
       case FMS_IN_Arm:
        if (!FMS_BottomRight(FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
                             FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle))
        {
          FMS_DW.durationLastReferenceTick_2_n = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.durationLastReferenceTick_1_b = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.is_Combo_Stick = FMS_IN_Idle;
          FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
          FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
        }
        break;

       case FMS_IN_Disarm:
        if (!FMS_BottomLeft(FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
                            FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle))
        {
          FMS_DW.durationLastReferenceTick_2_n = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.durationLastReferenceTick_1_b = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.is_Combo_Stick = FMS_IN_Idle;
          FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
          FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
        }
        break;

       case FMS_IN_Idle:
        if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_h0)) {
          FMS_DW.durationLastReferenceTick_1_b = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
        if (FMS_DW.chartAbsoluteTimeCounter -
            FMS_DW.durationLastReferenceTick_1_b > 375) {
          FMS_DW.is_Combo_Stick = FMS_IN_Arm;
          FMS_DW.M_msgReservedData = FMS_Cmd_PreArm;
          FMS_sf_msg_send_M();
        } else {
          if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2_b)) {
            FMS_DW.durationLastReferenceTick_2_n =
              FMS_DW.chartAbsoluteTimeCounter;
          }

          FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
          if (FMS_DW.chartAbsoluteTimeCounter -
              FMS_DW.durationLastReferenceTick_2_n > 375) {
            FMS_DW.is_Combo_Stick = FMS_IN_Disarm;
            b_previousEvent = FMS_DW.sfEvent;
            FMS_DW.sfEvent = FMS_event_DisarmEvent;
            if (FMS_DW.is_active_Vehicle != 0U) {
              FMS_Vehicle();
            }

            FMS_DW.sfEvent = b_previousEvent;
          } else {
            FMS_DW.bl = FMS_BottomLeft
              (FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
               FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle);
            if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2_b)) {
              FMS_DW.durationLastReferenceTick_2_n =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
            FMS_DW.br = FMS_BottomRight
              (FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
               FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle);
            if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_h0)) {
              FMS_DW.durationLastReferenceTick_1_b =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
          }
        }
        break;
      }
    }

    if (FMS_DW.is_active_Lost_Return != 0U) {
      switch (FMS_DW.is_Lost_Return) {
       case FMS_IN_Connect:
        if (FMS_B.LogicalOperator2) {
          FMS_DW.is_Lost_Return = FMS_IN_Lost;
          if ((FMS_B.BusConversion_InsertedFor_FMSSt.flag & 221U) != 0U) {
            FMS_DW.M_msgReservedData = FMS_Cmd_Return;
            FMS_sf_msg_send_M();
          }
        }
        break;

       case FMS_IN_Lost:
        if (!FMS_B.LogicalOperator2) {
          FMS_DW.is_Lost_Return = FMS_IN_Connect;
        }
        break;
      }
    }

    if (FMS_DW.is_active_Vehicle != 0U) {
      FMS_Vehicle();
    }
  }

  /* End of Chart: '<Root>/FMS State Machine' */
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_sf_msg_discard_M(void)
{
  if (FMS_DW.M_isValid) {
    FMS_DW.M_isValid = false;
  }
}

real32_T rt_remf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T q;
  if ((u1 != 0.0F) && (u1 != truncf(u1))) {
    q = fabsf(u0 / u1);
    if (fabsf(q - floorf(q + 0.5F)) <= FLT_EPSILON * q) {
      y = 0.0F;
    } else {
      y = fmodf(u0, u1);
    }
  } else {
    y = fmodf(u0, u1);
  }

  return y;
}

void FMS_initQueue(Queue_FMS_Cmd *q, QueuePolicy_T policy, int32_T capacity,
                   Msg_FMS_Cmd *qPool)
{
  q->fPolicy = policy;
  q->fCapacity = capacity;
  q->fHead = -1;
  q->fTail = -1;
  q->fArray = qPool;
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void initialize_msg_local_queues_for(void)
{
  FMS_initQueue((Queue_FMS_Cmd *)&FMS_DW.Queue_FMS_Cmd_b, MSG_FIFO_QUEUE, 10,
                (Msg_FMS_Cmd *)&FMS_DW.Msg_FMS_Cmd_i[1]);
}

/* Model step function */
void FMS_step(void)
{
  real32_T B;
  real32_T D;
  FMS_Cmd rtb_DataTypeConversion1_m;
  boolean_T rtb_FixPtRelationalOperator_me;
  uint8_T rtb_Switch_m;
  int8_T rtPrevAction;
  real32_T rtb_Add3_c;
  real32_T rtb_VectorConcatenate_i[9];
  real32_T rtb_Subtract3_o;
  real32_T rtb_a_l;
  real32_T rtb_Add4_e5;
  boolean_T rtb_Compare_on;
  MotionState rtb_state_c;
  real32_T rtb_Switch_dw[3];
  real32_T rtb_Rem_p;
  MotionState rtb_state_l;
  MotionState rtb_state_ki;
  real_T rtb_Switch1_p;
  real_T rtb_Gain;
  real_T rtb_Sum3;
  uint16_T rtb_y_md;
  uint16_T rtb_y_c1;
  real32_T rtb_TmpSignalConversionAtDela_a[2];
  real32_T rtb_VectorConcatenate_k[9];
  real32_T rtb_Sqrt_b;
  boolean_T rtb_LogicalOperator_es;
  real32_T rtb_TmpSignalConversionAtMath_c[3];
  real32_T rtb_Sum_ff[2];
  real32_T rtb_VectorConcatenate_ar[3];
  boolean_T tmp[3];
  real32_T tmp_0[3];
  boolean_T tmp_1[3];
  boolean_T tmp_2[3];
  int32_T rtb_Compare_bv_0;
  real32_T rtb_MathFunction_iq_idx_1;
  real32_T rtb_MathFunction_iq_idx_0;
  real32_T rtb_TmpSignalConversionAtMath_0;
  real32_T rtb_MathFunction_f_idx_2;
  real32_T rtb_MathFunction_f_idx_0;
  real32_T rtb_P_l_idx_0;
  real_T rtb_Multiply_l5_idx_0;
  real32_T u1_tmp;
  uint32_T tmp_3;
  uint32_T tmp_4;
  uint32_T tmp_5;
  boolean_T guard1 = false;

  /* DiscreteIntegrator: '<S13>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *  RelationalOperator: '<S17>/FixPt Relational Operator'
   *  UnitDelay: '<S17>/Delay Input1'
   *
   * Block description for '<S17>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Pilot_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE) {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = 0U;
  }

  /* Switch: '<S15>/Switch' incorporates:
   *  Constant: '<S16>/Constant'
   *  Constant: '<S22>/Constant'
   *  Constant: '<S23>/Constant'
   *  DataTypeConversion: '<S15>/Data Type Conversion2'
   *  Delay: '<S15>/Delay'
   *  DiscreteIntegrator: '<S13>/Discrete-Time Integrator1'
   *  Inport: '<Root>/GCS_Cmd'
   *  Inport: '<Root>/Pilot_Cmd'
   *  Logic: '<S15>/Logical Operator'
   *  Logic: '<S15>/Logical Operator1'
   *  RelationalOperator: '<S16>/Compare'
   *  RelationalOperator: '<S22>/Compare'
   *  RelationalOperator: '<S23>/Compare'
   *  RelationalOperator: '<S24>/FixPt Relational Operator'
   *  Switch: '<S15>/Switch1'
   *  UnitDelay: '<S24>/Delay Input1'
   *
   * Block description for '<S24>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if ((FMS_U.Pilot_Cmd.mode != 0U) && (FMS_DW.DiscreteTimeIntegrator1_DSTAT_b <
       500U)) {
    FMS_DW.Delay_DSTATE_c = (PilotMode)FMS_U.Pilot_Cmd.mode;
  } else {
    if ((FMS_U.GCS_Cmd.mode != FMS_DW.DelayInput1_DSTATE_f) &&
        (FMS_U.GCS_Cmd.mode != 0U)) {
      /* Switch: '<S15>/Switch1' incorporates:
       *  DataTypeConversion: '<S15>/Data Type Conversion1'
       *  Delay: '<S15>/Delay'
       *  Inport: '<Root>/GCS_Cmd'
       */
      FMS_DW.Delay_DSTATE_c = (PilotMode)FMS_U.GCS_Cmd.mode;
    }
  }

  /* End of Switch: '<S15>/Switch' */

  /* Chart: '<Root>/SafeMode' incorporates:
   *  Delay: '<S15>/Delay'
   *  Inport: '<Root>/INS_Out'
   */
  if (FMS_DW.is_active_c3_FMS == 0U) {
    FMS_DW.is_active_c3_FMS = 1U;
    switch (FMS_DW.Delay_DSTATE_c) {
     case PilotMode_Manual:
      FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
      break;

     case PilotMode_Acro:
      FMS_DW.is_c3_FMS = FMS_IN_Acro;
      break;

     case PilotMode_Stabilize:
      FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
      break;

     case PilotMode_Altitude:
      FMS_DW.is_c3_FMS = FMS_IN_Altitude;
      break;

     case PilotMode_Position:
      FMS_DW.is_c3_FMS = FMS_IN_Position_f;
      break;

     case PilotMode_Mission:
      FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
      break;

     case PilotMode_Offboard:
      FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
      break;

     default:
      FMS_DW.is_c3_FMS = FMS_IN_Other;
      break;
    }
  } else {
    switch (FMS_DW.is_c3_FMS) {
     case FMS_IN_Acro:
      FMS_Acro();
      break;

     case FMS_IN_Altitude:
      if (((FMS_U.INS_Out.flag & 4U) != 0U) && ((FMS_U.INS_Out.flag & 128U) !=
           0U)) {
        FMS_B.target_mode = PilotMode_Altitude;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Manual:
          FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
          break;

         case PilotMode_Acro:
          FMS_DW.is_c3_FMS = FMS_IN_Acro;
          break;

         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
          break;

         case PilotMode_Altitude:
          FMS_DW.is_c3_FMS = FMS_IN_Altitude;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_f;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
      }
      break;

     case FMS_IN_Manual_e:
      FMS_B.target_mode = PilotMode_Manual;
      switch (FMS_DW.Delay_DSTATE_c) {
       case PilotMode_Manual:
        FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
        break;

       case PilotMode_Acro:
        FMS_DW.is_c3_FMS = FMS_IN_Acro;
        break;

       case PilotMode_Stabilize:
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
        break;

       case PilotMode_Altitude:
        FMS_DW.is_c3_FMS = FMS_IN_Altitude;
        break;

       case PilotMode_Position:
        FMS_DW.is_c3_FMS = FMS_IN_Position_f;
        break;

       case PilotMode_Mission:
        FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
        break;

       case PilotMode_Offboard:
        FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
        break;

       default:
        FMS_DW.is_c3_FMS = FMS_IN_Other;
        break;
      }
      break;

     case FMS_IN_Mission_g:
      if (((FMS_U.INS_Out.flag & 4U) != 0U) && ((FMS_U.INS_Out.flag & 16U) != 0U)
          && ((FMS_U.INS_Out.flag & 32U) != 0U) && ((FMS_U.INS_Out.flag & 64U)
           != 0U) && ((FMS_U.INS_Out.flag & 128U) != 0U)) {
        FMS_B.target_mode = PilotMode_Mission;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Manual:
          FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
          break;

         case PilotMode_Acro:
          FMS_DW.is_c3_FMS = FMS_IN_Acro;
          break;

         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
          break;

         case PilotMode_Altitude:
          FMS_DW.is_c3_FMS = FMS_IN_Altitude;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_f;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Position_f;
      }
      break;

     case FMS_IN_Offboard_p:
      if (((FMS_U.INS_Out.flag & 4U) != 0U) && ((FMS_U.INS_Out.flag & 16U) != 0U)
          && ((FMS_U.INS_Out.flag & 64U) != 0U) && ((FMS_U.INS_Out.flag & 128U)
           != 0U)) {
        FMS_B.target_mode = PilotMode_Offboard;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Manual:
          FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
          break;

         case PilotMode_Acro:
          FMS_DW.is_c3_FMS = FMS_IN_Acro;
          break;

         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
          break;

         case PilotMode_Altitude:
          FMS_DW.is_c3_FMS = FMS_IN_Altitude;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_f;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Position_f;
      }
      break;

     case FMS_IN_Other:
      FMS_B.target_mode = FMS_DW.Delay_DSTATE_c;
      switch (FMS_DW.Delay_DSTATE_c) {
       case PilotMode_Manual:
        FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
        break;

       case PilotMode_Acro:
        FMS_DW.is_c3_FMS = FMS_IN_Acro;
        break;

       case PilotMode_Stabilize:
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
        break;

       case PilotMode_Altitude:
        FMS_DW.is_c3_FMS = FMS_IN_Altitude;
        break;

       case PilotMode_Position:
        FMS_DW.is_c3_FMS = FMS_IN_Position_f;
        break;

       case PilotMode_Mission:
        FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
        break;

       case PilotMode_Offboard:
        FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
        break;

       default:
        FMS_DW.is_c3_FMS = FMS_IN_Other;
        break;
      }
      break;

     case FMS_IN_Position_f:
      if (((FMS_U.INS_Out.flag & 4U) != 0U) && ((FMS_U.INS_Out.flag & 16U) != 0U)
          && ((FMS_U.INS_Out.flag & 64U) != 0U) && ((FMS_U.INS_Out.flag & 128U)
           != 0U)) {
        FMS_B.target_mode = PilotMode_Position;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Manual:
          FMS_DW.is_c3_FMS = FMS_IN_Manual_e;
          break;

         case PilotMode_Acro:
          FMS_DW.is_c3_FMS = FMS_IN_Acro;
          break;

         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_j;
          break;

         case PilotMode_Altitude:
          FMS_DW.is_c3_FMS = FMS_IN_Altitude;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_f;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission_g;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard_p;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Altitude;
      }
      break;

     default:
      FMS_Stabilize();
      break;
    }
  }

  /* End of Chart: '<Root>/SafeMode' */

  /* DataTypeConversion: '<S14>/Data Type Conversion1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   */
  rtb_DataTypeConversion1_m = (FMS_Cmd)FMS_U.GCS_Cmd.cmd_1;

  /* Switch: '<S14>/Switch1' incorporates:
   *  Constant: '<S14>/Constant1'
   *  DataTypeConversion: '<S14>/Data Type Conversion2'
   *  Inport: '<Root>/GCS_Cmd'
   *  Inport: '<Root>/Pilot_Cmd'
   *  RelationalOperator: '<S19>/FixPt Relational Operator'
   *  RelationalOperator: '<S20>/FixPt Relational Operator'
   *  Switch: '<S14>/Switch2'
   *  UnitDelay: '<S19>/Delay Input1'
   *  UnitDelay: '<S20>/Delay Input1'
   *
   * Block description for '<S19>/Delay Input1':
   *
   *  Store in Global RAM
   *
   * Block description for '<S20>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Pilot_Cmd.cmd_1 != FMS_DW.DelayInput1_DSTATE_i) {
    FMS_B.Switch1 = (FMS_Cmd)FMS_U.Pilot_Cmd.cmd_1;
  } else if (FMS_U.GCS_Cmd.cmd_1 != FMS_DW.DelayInput1_DSTATE_p) {
    /* Switch: '<S14>/Switch2' */
    FMS_B.Switch1 = rtb_DataTypeConversion1_m;
  } else {
    FMS_B.Switch1 = FMS_Cmd_None;
  }

  /* End of Switch: '<S14>/Switch1' */

  /* BusCreator: '<Root>/BusConversion_InsertedFor_FMS State Machine_at_inport_2' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   */
  FMS_B.BusConversion_InsertedFor_FMS_f = FMS_U.Pilot_Cmd;

  /* RelationalOperator: '<S26>/FixPt Relational Operator' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *  UnitDelay: '<S26>/Delay Input1'
   *
   * Block description for '<S26>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_FixPtRelationalOperator_me = (FMS_U.Pilot_Cmd.timestamp !=
    FMS_DW.DelayInput1_DSTATE_h);

  /* DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Inport: '<Root>/GCS_Cmd'
   *  Logic: '<S3>/Logical Operator'
   *  Logic: '<S3>/Logical Operator1'
   *  RelationalOperator: '<S25>/FixPt Relational Operator'
   *  UnitDelay: '<S25>/Delay Input1'
   *
   * Block description for '<S25>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if ((FMS_U.GCS_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE_d) ||
      rtb_FixPtRelationalOperator_me || (FMS_PARAM.LOST_RETURN_EN == 0)) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE = 0.0F;
  }

  if (FMS_DW.DiscreteTimeIntegrator_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator_DSTATE = 0.0F;
    }
  }

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  Constant: '<S27>/Constant'
   *  Constant: '<S3>/Constant1'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   *  RelationalOperator: '<S27>/Compare'
   */
  FMS_B.LogicalOperator2 = ((FMS_DW.DiscreteTimeIntegrator_DSTATE >=
    FMS_PARAM.LOST_RETURN_TIME) && (FMS_PARAM.LOST_RETURN_EN != 0));

  /* DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' */
  if (rtb_FixPtRelationalOperator_me) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
  }

  if (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator1_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
    }
  }

  /* RelationalOperator: '<S28>/Compare' incorporates:
   *  Constant: '<S28>/Constant'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
   */
  FMS_B.Compare = (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 1.0F);

  /* BusCreator: '<Root>/BusConversion_InsertedFor_FMS State Machine_at_inport_5' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  FMS_B.BusConversion_InsertedFor_FMSSt = FMS_U.INS_Out;

  /* Logic: '<S500>/AND' incorporates:
   *  Constant: '<S500>/Lower Limit'
   *  Constant: '<S500>/Upper Limit'
   *  Inport: '<Root>/INS_Out'
   *  RelationalOperator: '<S500>/Lower Test'
   *  RelationalOperator: '<S500>/Upper Test'
   */
  FMS_B.AND = ((0.0F <= FMS_U.INS_Out.h_AGL) && (FMS_U.INS_Out.h_AGL <= 0.15F));

  /* DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S11>/Constant1'
   *  Delay: '<S11>/Delay'
   *  Inport: '<Root>/Auto_Cmd'
   *  RelationalOperator: '<S10>/FixPt Relational Operator'
   *  Switch: '<S11>/Switch'
   *  UnitDelay: '<S10>/Delay Input1'
   *
   * Block description for '<S10>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Auto_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE_c) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_b = 0U;
    rtb_Switch_m = 1U;
  } else {
    rtb_Switch_m = FMS_DW.Delay_DSTATE_o;
  }

  /* Logic: '<S1>/Logical Operator' incorporates:
   *  Constant: '<S12>/Upper Limit'
   *  Constant: '<S9>/Constant'
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   *  Inport: '<Root>/Auto_Cmd'
   *  RelationalOperator: '<S12>/Upper Test'
   *  RelationalOperator: '<S9>/Compare'
   */
  FMS_B.LogicalOperator = ((FMS_DW.DiscreteTimeIntegrator_DSTATE_b < 1000U) &&
    (rtb_Switch_m != 0) && (FMS_U.Auto_Cmd.frame <= 2));

  /* Chart: '<Root>/FMS State Machine' incorporates:
   *  Inport: '<Root>/Mission_Data'
   */
  FMS_DW.chartAbsoluteTimeCounter++;
  if ((!FMS_B.AND) || (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
    FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1 = FMS_B.AND;
  if ((!FMS_B.AND) || (!FMS_DW.condWasTrueAtLastTimeStep_1_k)) {
    FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_B.AND;
  rtb_FixPtRelationalOperator_me = (FMS_DW.prep_takeoff == 1.0);
  if ((!rtb_FixPtRelationalOperator_me) ||
      (!FMS_DW.condWasTrueAtLastTimeStep_1_b)) {
    FMS_DW.durationLastReferenceTick_1_n = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1_b = rtb_FixPtRelationalOperator_me;
  rtb_FixPtRelationalOperator_me = (FMS_DW.prep_mission_takeoff == 1.0);
  if ((!rtb_FixPtRelationalOperator_me) || (!FMS_DW.condWasTrueAtLastTimeStep_2))
  {
    FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_2 = rtb_FixPtRelationalOperator_me;
  if ((!FMS_B.AND) || (!FMS_DW.condWasTrueAtLastTimeStep_1_h)) {
    FMS_DW.durationLastReferenceTick_1_n5 = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1_h = FMS_B.AND;
  if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_h0)) {
    FMS_DW.durationLastReferenceTick_1_b = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1_h0 = FMS_DW.br;
  if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2_b)) {
    FMS_DW.durationLastReferenceTick_2_n = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_2_b = FMS_DW.bl;
  if (FMS_DW.temporalCounter_i1 < MAX_uint32_T) {
    FMS_DW.temporalCounter_i1++;
  }

  FMS_DW.sfEvent = -1;
  FMS_DW.mission_timestamp_prev = FMS_DW.mission_timestamp_start;
  FMS_DW.mission_timestamp_start = FMS_U.Mission_Data.timestamp;
  FMS_DW.mode_prev = FMS_DW.mode_start;
  FMS_DW.mode_start = FMS_B.target_mode;
  FMS_DW.cmd_prev = FMS_DW.cmd_start;
  FMS_DW.cmd_start = FMS_B.Switch1;
  FMS_DW.M_isValid = false;
  FMS_c11_FMS();
  FMS_sf_msg_discard_M();

  /* End of Chart: '<Root>/FMS State Machine' */

  /* Outputs for Atomic SubSystem: '<Root>/FMS Commander' */
  /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
  /* SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1' */
  rtb_Switch_dw[0] = FMS_B.Cmd_In.sp_waypoint[0];
  rtb_Switch_dw[1] = FMS_B.Cmd_In.sp_waypoint[1];
  rtb_Switch_dw[2] = FMS_B.Cmd_In.sp_waypoint[2];

  /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

  /* SwitchCase: '<S29>/Switch Case' incorporates:
   *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy6Inport1'
   */
  rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem;

  /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
  switch (FMS_B.state) {
   case VehicleState_Disarm:
   case VehicleState_None:
    FMS_DW.SwitchCase_ActiveSubsystem = 0;
    break;

   case VehicleState_Standby:
    FMS_DW.SwitchCase_ActiveSubsystem = 1;
    break;

   default:
    FMS_DW.SwitchCase_ActiveSubsystem = 2;
    break;
  }

  /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
  if ((rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem) && (rtPrevAction == 2))
  {
    /* Disable for SwitchCase: '<S31>/Switch Case' */
    switch (FMS_DW.SwitchCase_ActiveSubsystem_b) {
     case 0:
      /* Disable for SwitchCase: '<S38>/Switch Case' */
      switch (FMS_DW.SwitchCase_ActiveSubsystem_at) {
       case 0:
       case 1:
       case 4:
        break;

       case 2:
        /* Disable for SwitchCase: '<S434>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_g = -1;

        /* Disable for SwitchCase: '<S424>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_ld = -1;
        break;

       case 3:
        /* Disable for SwitchCase: '<S366>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_p = -1;

        /* Disable for SwitchCase: '<S344>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_pp = -1;

        /* Disable for SwitchCase: '<S354>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_bn = -1;
        break;
      }

      FMS_DW.SwitchCase_ActiveSubsystem_at = -1;
      break;

     case 1:
      /* Disable for SwitchCase: '<S36>/Switch Case' */
      if (FMS_DW.SwitchCase_ActiveSubsystem_i == 1) {
        /* Disable for Resettable SubSystem: '<S149>/Mission_SubSystem' */
        /* Disable for SwitchCase: '<S200>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

        /* Disable for SwitchCase: '<S190>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_e = -1;

        /* End of Disable for SubSystem: '<S149>/Mission_SubSystem' */
      }

      FMS_DW.SwitchCase_ActiveSubsystem_i = -1;
      break;

     case 2:
      /* Disable for SwitchCase: '<S35>/Switch Case' */
      switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
       case 0:
       case 4:
        break;

       case 1:
        /* Disable for SwitchCase: '<S136>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_fs = -1;
        break;

       case 2:
        /* Disable for SwitchCase: '<S50>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_m = -1;

        /* Disable for SwitchCase: '<S68>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_h = -1;
        break;

       case 3:
        /* Disable for SwitchCase: '<S84>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_o = -1;

        /* Disable for SwitchCase: '<S109>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_l = -1;

        /* Disable for SwitchCase: '<S96>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_j = -1;
        break;
      }

      FMS_DW.SwitchCase_ActiveSubsystem_f = -1;
      break;

     case 3:
     case 4:
      break;
    }

    FMS_DW.SwitchCase_ActiveSubsystem_b = -1;

    /* End of Disable for SwitchCase: '<S31>/Switch Case' */
  }

  switch (FMS_DW.SwitchCase_ActiveSubsystem) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S29>/Disarm' incorporates:
     *  ActionPort: '<S33>/Action Port'
     */
    /* Outport: '<Root>/FMS_Out' incorporates:
     *  BusAssignment: '<S32>/Bus Assignment'
     *  BusAssignment: '<S33>/Bus Assignment'
     *  Constant: '<S33>/Constant'
     */
    FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

    /* BusAssignment: '<S33>/Bus Assignment' incorporates:
     *  BusAssignment: '<S32>/Bus Assignment'
     *  Constant: '<S33>/Constant2'
     *  Outport: '<Root>/FMS_Out'
     */
    FMS_Y.FMS_Out.reset = 1U;
    FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_m;
    FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_a;

    /* End of Outputs for SubSystem: '<S29>/Disarm' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S29>/Standby' incorporates:
     *  ActionPort: '<S34>/Action Port'
     */
    /* Outport: '<Root>/FMS_Out' incorporates:
     *  BusAssignment: '<S32>/Bus Assignment'
     *  BusAssignment: '<S34>/Bus Assignment'
     *  Constant: '<S34>/Constant'
     */
    FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

    /* BusAssignment: '<S34>/Bus Assignment' incorporates:
     *  BusAssignment: '<S32>/Bus Assignment'
     *  Constant: '<S34>/Constant2'
     *  Outport: '<Root>/FMS_Out'
     */
    FMS_Y.FMS_Out.reset = 1U;
    FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion2_h;
    FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_f;

    /* End of Outputs for SubSystem: '<S29>/Standby' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S29>/Arm' incorporates:
     *  ActionPort: '<S31>/Action Port'
     */
    /* SwitchCase: '<S31>/Switch Case' */
    rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_b;

    /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
    switch (FMS_B.state) {
     case VehicleState_Land:
     case VehicleState_Return:
     case VehicleState_Takeoff:
     case VehicleState_Hold:
      FMS_DW.SwitchCase_ActiveSubsystem_b = 0;
      break;

     case VehicleState_Offboard:
     case VehicleState_Mission:
      FMS_DW.SwitchCase_ActiveSubsystem_b = 1;
      break;

     case VehicleState_Acro:
     case VehicleState_Stabilize:
     case VehicleState_Altitude:
     case VehicleState_Position:
      FMS_DW.SwitchCase_ActiveSubsystem_b = 2;
      break;

     case VehicleState_Manual:
      FMS_DW.SwitchCase_ActiveSubsystem_b = 3;
      break;

     default:
      FMS_DW.SwitchCase_ActiveSubsystem_b = 4;
      break;
    }

    /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
    if (rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem_b) {
      switch (rtPrevAction) {
       case 0:
        /* Disable for SwitchCase: '<S38>/Switch Case' */
        switch (FMS_DW.SwitchCase_ActiveSubsystem_at) {
         case 0:
         case 1:
         case 4:
          break;

         case 2:
          /* Disable for SwitchCase: '<S434>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_g = -1;

          /* Disable for SwitchCase: '<S424>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_ld = -1;
          break;

         case 3:
          /* Disable for SwitchCase: '<S366>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_p = -1;

          /* Disable for SwitchCase: '<S344>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_pp = -1;

          /* Disable for SwitchCase: '<S354>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_bn = -1;
          break;
        }

        FMS_DW.SwitchCase_ActiveSubsystem_at = -1;
        break;

       case 1:
        /* Disable for SwitchCase: '<S36>/Switch Case' */
        if (FMS_DW.SwitchCase_ActiveSubsystem_i == 1) {
          /* Disable for Resettable SubSystem: '<S149>/Mission_SubSystem' */
          /* Disable for SwitchCase: '<S200>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

          /* Disable for SwitchCase: '<S190>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_e = -1;

          /* End of Disable for SubSystem: '<S149>/Mission_SubSystem' */
        }

        FMS_DW.SwitchCase_ActiveSubsystem_i = -1;
        break;

       case 2:
        /* Disable for SwitchCase: '<S35>/Switch Case' */
        switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
         case 0:
         case 4:
          break;

         case 1:
          /* Disable for SwitchCase: '<S136>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_fs = -1;
          break;

         case 2:
          /* Disable for SwitchCase: '<S50>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_m = -1;

          /* Disable for SwitchCase: '<S68>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_h = -1;
          break;

         case 3:
          /* Disable for SwitchCase: '<S84>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_o = -1;

          /* Disable for SwitchCase: '<S109>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_l = -1;

          /* Disable for SwitchCase: '<S96>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_j = -1;
          break;
        }

        FMS_DW.SwitchCase_ActiveSubsystem_f = -1;
        break;

       case 3:
       case 4:
        break;
      }
    }

    switch (FMS_DW.SwitchCase_ActiveSubsystem_b) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S31>/SubMode' incorporates:
       *  ActionPort: '<S38>/Action Port'
       */
      /* SwitchCase: '<S38>/Switch Case' incorporates:
       *  Math: '<S456>/Math Function'
       *  Product: '<S458>/Divide'
       *  Sum: '<S412>/Subtract'
       */
      rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_at;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      switch (FMS_B.state) {
       case VehicleState_Takeoff:
        FMS_DW.SwitchCase_ActiveSubsystem_at = 0;
        break;

       case VehicleState_Land:
        FMS_DW.SwitchCase_ActiveSubsystem_at = 1;
        break;

       case VehicleState_Return:
        FMS_DW.SwitchCase_ActiveSubsystem_at = 2;
        break;

       case VehicleState_Hold:
        FMS_DW.SwitchCase_ActiveSubsystem_at = 3;
        break;

       default:
        FMS_DW.SwitchCase_ActiveSubsystem_at = 4;
        break;
      }

      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
      if (rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem_at) {
        switch (rtPrevAction) {
         case 0:
         case 1:
         case 4:
          break;

         case 2:
          /* Disable for SwitchCase: '<S434>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_g = -1;

          /* Disable for SwitchCase: '<S424>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_ld = -1;
          break;

         case 3:
          /* Disable for SwitchCase: '<S366>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_p = -1;

          /* Disable for SwitchCase: '<S344>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_pp = -1;

          /* Disable for SwitchCase: '<S354>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_bn = -1;
          break;
        }
      }

      switch (FMS_DW.SwitchCase_ActiveSubsystem_at) {
       case 0:
        if (FMS_DW.SwitchCase_ActiveSubsystem_at != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S38>/Takeoff' incorporates:
           *  ActionPort: '<S339>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S38>/Switch Case' incorporates:
           *  Delay: '<S473>/cur_waypoint'
           *  DiscreteIntegrator: '<S469>/Integrator'
           *  DiscreteIntegrator: '<S469>/Integrator1'
           */
          FMS_DW.icLoad_j1 = 1U;
          FMS_DW.Integrator1_DSTATE_a = 0.0F;
          FMS_DW.Integrator_DSTATE_m = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S38>/Takeoff' */
        }

        /* Outputs for IfAction SubSystem: '<S38>/Takeoff' incorporates:
         *  ActionPort: '<S339>/Action Port'
         */
        /* Delay: '<S473>/cur_waypoint' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad_j1 != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.cur_waypoint_DSTATE[0] = FMS_U.INS_Out.x_R;
          FMS_DW.cur_waypoint_DSTATE[1] = FMS_U.INS_Out.y_R;
          FMS_DW.cur_waypoint_DSTATE[2] = FMS_U.INS_Out.h_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Product: '<S473>/Divide' incorporates:
         *  Delay: '<S473>/cur_waypoint'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         *  Sum: '<S473>/Sum1'
         *  Sum: '<S473>/Sum2'
         */
        rtb_Add3_c = 1.0F / (FMS_B.Cmd_In.sp_waypoint[2] -
                             FMS_DW.cur_waypoint_DSTATE[2]) * (FMS_U.INS_Out.h_R
          - FMS_DW.cur_waypoint_DSTATE[2]);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Saturate: '<S473>/Saturation' */
        if (rtb_Add3_c > 1.0F) {
          rtb_Add3_c = 1.0F;
        } else {
          if (rtb_Add3_c < 0.0F) {
            rtb_Add3_c = 0.0F;
          }
        }

        /* End of Saturate: '<S473>/Saturation' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Trigonometry: '<S474>/Trigonometric Function1' incorporates:
         *  Gain: '<S472>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Trigonometry: '<S474>/Trigonometric Function3'
         */
        rtb_MathFunction_f_idx_0 = arm_cos_f32(-FMS_U.INS_Out.psi);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_VectorConcatenate_i[0] = rtb_MathFunction_f_idx_0;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Trigonometry: '<S474>/Trigonometric Function' incorporates:
         *  Gain: '<S472>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Trigonometry: '<S474>/Trigonometric Function2'
         */
        rtb_a_l = arm_sin_f32(-FMS_U.INS_Out.psi);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_VectorConcatenate_i[1] = rtb_a_l;

        /* SignalConversion: '<S474>/ConcatBufferAtVector Concatenate1In3' incorporates:
         *  Constant: '<S474>/Constant3'
         */
        rtb_VectorConcatenate_i[2] = 0.0F;

        /* Gain: '<S474>/Gain' */
        rtb_VectorConcatenate_i[3] = -rtb_a_l;

        /* Trigonometry: '<S474>/Trigonometric Function3' */
        rtb_VectorConcatenate_i[4] = rtb_MathFunction_f_idx_0;

        /* SignalConversion: '<S474>/ConcatBufferAtVector Concatenate2In3' incorporates:
         *  Constant: '<S474>/Constant4'
         */
        rtb_VectorConcatenate_i[5] = 0.0F;

        /* SignalConversion: '<S474>/ConcatBufferAtVector ConcatenateIn3' */
        rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_fb[0];
        rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_fb[1];
        rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_fb[2];

        /* Saturate: '<S467>/Saturation1' */
        rtb_Add4_e5 = FMS_PARAM.VEL_XY_LIM / 5.0F;
        rtb_MathFunction_f_idx_0 = -FMS_PARAM.VEL_XY_LIM / 5.0F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* SignalConversion: '<S467>/TmpSignal ConversionAtMultiplyInport2' incorporates:
         *  Delay: '<S473>/cur_waypoint'
         *  Inport: '<Root>/INS_Out'
         *  Product: '<S473>/Multiply'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         *  Sum: '<S467>/Sum'
         *  Sum: '<S473>/Sum3'
         *  Sum: '<S473>/Sum4'
         */
        rtb_Subtract3_o = ((FMS_B.Cmd_In.sp_waypoint[0] -
                            FMS_DW.cur_waypoint_DSTATE[0]) * rtb_Add3_c +
                           FMS_DW.cur_waypoint_DSTATE[0]) - FMS_U.INS_Out.x_R;
        rtb_a_l = ((FMS_B.Cmd_In.sp_waypoint[1] - FMS_DW.cur_waypoint_DSTATE[1])
                   * rtb_Add3_c + FMS_DW.cur_waypoint_DSTATE[1]) -
          FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Product: '<S467>/Multiply' */
        for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
          rtb_VectorConcatenate_ar[rtb_Compare_bv_0] =
            rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
            rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o;
        }

        /* Saturate: '<S467>/Saturation1' incorporates:
         *  Gain: '<S467>/Gain2'
         *  Product: '<S467>/Multiply'
         */
        rtb_Sqrt_b = FMS_PARAM.XY_P * rtb_VectorConcatenate_ar[0];
        rtb_Add3_c = FMS_PARAM.XY_P * rtb_VectorConcatenate_ar[1];

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S339>/Bus Assignment1'
         *  Constant: '<S339>/Constant1'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S339>/Constant'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_ld;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_dh;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_d;
        FMS_Y.FMS_Out.psi_rate_cmd = 0.0F;

        /* Saturate: '<S467>/Saturation1' */
        if (rtb_Sqrt_b > rtb_Add4_e5) {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_Add4_e5;
        } else if (rtb_Sqrt_b < rtb_MathFunction_f_idx_0) {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_MathFunction_f_idx_0;
        } else {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_Sqrt_b;
        }

        if (rtb_Add3_c > rtb_Add4_e5) {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_Add4_e5;
        } else if (rtb_Add3_c < rtb_MathFunction_f_idx_0) {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_MathFunction_f_idx_0;
        } else {
          /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_Add3_c;
        }

        /* BusAssignment: '<S339>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  DiscreteIntegrator: '<S469>/Integrator1'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.w_cmd = FMS_DW.Integrator1_DSTATE_a;

        /* Product: '<S470>/Multiply1' incorporates:
         *  Constant: '<S470>/const1'
         *  DiscreteIntegrator: '<S469>/Integrator'
         */
        rtb_Add3_c = FMS_DW.Integrator_DSTATE_m * 0.35F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S466>/Switch' incorporates:
         *  Constant: '<S466>/Takeoff_Speed'
         *  Constant: '<S468>/Constant'
         *  Gain: '<S466>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  RelationalOperator: '<S468>/Compare'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         *  Sum: '<S466>/Sum'
         */
        if (FMS_U.INS_Out.h_R - FMS_B.Cmd_In.cur_waypoint[2] <= 0.2F) {
          rtb_a_l = 1.5F * -FMS_PARAM.TAKEOFF_SPEED;
        } else {
          rtb_a_l = -FMS_PARAM.TAKEOFF_SPEED;
        }

        /* End of Switch: '<S466>/Switch' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S470>/Add' incorporates:
         *  DiscreteIntegrator: '<S469>/Integrator1'
         *  Sum: '<S469>/Subtract'
         */
        rtb_Subtract3_o = (FMS_DW.Integrator1_DSTATE_a - rtb_a_l) + rtb_Add3_c;

        /* Signum: '<S470>/Sign' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_Subtract3_o > 0.0F) {
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_Add4_e5 = rtb_Subtract3_o;
        }

        /* End of Signum: '<S470>/Sign' */

        /* Sum: '<S470>/Add2' incorporates:
         *  Abs: '<S470>/Abs'
         *  Gain: '<S470>/Gain'
         *  Gain: '<S470>/Gain1'
         *  Product: '<S470>/Multiply2'
         *  Product: '<S470>/Multiply3'
         *  Sqrt: '<S470>/Sqrt'
         *  Sum: '<S470>/Add1'
         *  Sum: '<S470>/Subtract'
         */
        rtb_a_l = (sqrtf((8.0F * fabsf(rtb_Subtract3_o) + FMS_ConstB.d_m) *
                         FMS_ConstB.d_m) - FMS_ConstB.d_m) * 0.5F * rtb_Add4_e5
          + rtb_Add3_c;

        /* Sum: '<S470>/Add4' */
        rtb_Add4_e5 = (rtb_Subtract3_o - rtb_a_l) + rtb_Add3_c;

        /* Sum: '<S470>/Add3' */
        rtb_Add3_c = rtb_Subtract3_o + FMS_ConstB.d_m;

        /* Sum: '<S470>/Subtract1' */
        rtb_Subtract3_o -= FMS_ConstB.d_m;

        /* Signum: '<S470>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S470>/Sign1' */

        /* Signum: '<S470>/Sign2' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S470>/Sign2' */

        /* Sum: '<S470>/Add5' incorporates:
         *  Gain: '<S470>/Gain2'
         *  Product: '<S470>/Multiply4'
         *  Sum: '<S470>/Subtract2'
         */
        rtb_a_l += (rtb_Add3_c - rtb_Subtract3_o) * 0.5F * rtb_Add4_e5;

        /* Update for Delay: '<S473>/cur_waypoint' */
        FMS_DW.icLoad_j1 = 0U;

        /* Update for DiscreteIntegrator: '<S469>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S469>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_a += 0.004F * FMS_DW.Integrator_DSTATE_m;

        /* Sum: '<S470>/Subtract3' */
        rtb_Add3_c = rtb_a_l - FMS_ConstB.d_m;

        /* Sum: '<S470>/Add6' */
        rtb_Subtract3_o = rtb_a_l + FMS_ConstB.d_m;

        /* Signum: '<S470>/Sign5' incorporates:
         *  Signum: '<S470>/Sign6'
         */
        if (rtb_a_l < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;

          /* Signum: '<S470>/Sign6' */
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;

          /* Signum: '<S470>/Sign6' */
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_MathFunction_f_idx_0 = rtb_a_l;

          /* Signum: '<S470>/Sign6' */
          rtb_Add4_e5 = rtb_a_l;
        }

        /* End of Signum: '<S470>/Sign5' */

        /* Signum: '<S470>/Sign3' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S470>/Sign3' */

        /* Signum: '<S470>/Sign4' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S470>/Sign4' */

        /* Update for DiscreteIntegrator: '<S469>/Integrator' incorporates:
         *  Constant: '<S470>/const'
         *  Gain: '<S470>/Gain3'
         *  Product: '<S470>/Divide'
         *  Product: '<S470>/Multiply5'
         *  Product: '<S470>/Multiply6'
         *  Sum: '<S470>/Subtract4'
         *  Sum: '<S470>/Subtract5'
         *  Sum: '<S470>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_m += ((rtb_a_l / FMS_ConstB.d_m -
          rtb_MathFunction_f_idx_0) * FMS_ConstB.Gain4_a * ((rtb_Subtract3_o -
          rtb_Add3_c) * 0.5F) - rtb_Add4_e5 * 9.806F) * 0.004F;

        /* End of Outputs for SubSystem: '<S38>/Takeoff' */
        break;

       case 1:
        if (FMS_DW.SwitchCase_ActiveSubsystem_at != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S38>/Land' incorporates:
           *  ActionPort: '<S337>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S38>/Switch Case' incorporates:
           *  DiscreteIntegrator: '<S382>/Integrator'
           *  DiscreteIntegrator: '<S382>/Integrator1'
           */
          FMS_DW.Integrator1_DSTATE_j = 0.0F;
          FMS_DW.Integrator_DSTATE_d = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S38>/Land' */
        }

        /* Outputs for IfAction SubSystem: '<S38>/Land' incorporates:
         *  ActionPort: '<S337>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Trigonometry: '<S387>/Trigonometric Function1' incorporates:
         *  Gain: '<S386>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_VectorConcatenate_i[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

        /* Trigonometry: '<S387>/Trigonometric Function' incorporates:
         *  Gain: '<S386>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_VectorConcatenate_i[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SignalConversion: '<S387>/ConcatBufferAtVector Concatenate1In3' incorporates:
         *  Constant: '<S387>/Constant3'
         */
        rtb_VectorConcatenate_i[2] = 0.0F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Gain: '<S387>/Gain' incorporates:
         *  Gain: '<S386>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Trigonometry: '<S387>/Trigonometric Function2'
         */
        rtb_VectorConcatenate_i[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

        /* Trigonometry: '<S387>/Trigonometric Function3' incorporates:
         *  Gain: '<S386>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_VectorConcatenate_i[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SignalConversion: '<S387>/ConcatBufferAtVector Concatenate2In3' incorporates:
         *  Constant: '<S387>/Constant4'
         */
        rtb_VectorConcatenate_i[5] = 0.0F;

        /* SignalConversion: '<S387>/ConcatBufferAtVector ConcatenateIn3' */
        rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_f[0];
        rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_f[1];
        rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_f[2];

        /* Saturate: '<S380>/Saturation1' */
        rtb_MathFunction_f_idx_0 = FMS_PARAM.VEL_XY_LIM / 5.0F;
        rtb_Add4_e5 = -FMS_PARAM.VEL_XY_LIM / 5.0F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* SignalConversion: '<S384>/TmpSignal ConversionAtMultiplyInport2' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         *  Sum: '<S384>/Sum'
         */
        rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[0] - FMS_U.INS_Out.x_R;
        rtb_a_l = FMS_B.Cmd_In.sp_waypoint[1] - FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Product: '<S384>/Multiply' incorporates:
         *  SignalConversion: '<S384>/TmpSignal ConversionAtMultiplyInport2'
         */
        for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
          rtb_VectorConcatenate_ar[rtb_Compare_bv_0] =
            rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
            rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o;
        }

        /* End of Product: '<S384>/Multiply' */

        /* Saturate: '<S380>/Saturation1' incorporates:
         *  Gain: '<S384>/Gain2'
         */
        rtb_Sqrt_b = FMS_PARAM.XY_P * rtb_VectorConcatenate_ar[0];
        rtb_Add3_c = FMS_PARAM.XY_P * rtb_VectorConcatenate_ar[1];

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S337>/Bus Assignment1'
         *  Constant: '<S337>/Constant1'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S337>/Constant'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_c;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_h;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_m;
        FMS_Y.FMS_Out.psi_rate_cmd = 0.0F;

        /* Saturate: '<S380>/Saturation1' */
        if (rtb_Sqrt_b > rtb_MathFunction_f_idx_0) {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_MathFunction_f_idx_0;
        } else if (rtb_Sqrt_b < rtb_Add4_e5) {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_Add4_e5;
        } else {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_Sqrt_b;
        }

        if (rtb_Add3_c > rtb_MathFunction_f_idx_0) {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_MathFunction_f_idx_0;
        } else if (rtb_Add3_c < rtb_Add4_e5) {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_Add4_e5;
        } else {
          /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_Add3_c;
        }

        /* BusAssignment: '<S337>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  DiscreteIntegrator: '<S382>/Integrator1'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.w_cmd = FMS_DW.Integrator1_DSTATE_j;

        /* Product: '<S383>/Multiply1' incorporates:
         *  Constant: '<S383>/const1'
         *  DiscreteIntegrator: '<S382>/Integrator'
         */
        rtb_Add3_c = FMS_DW.Integrator_DSTATE_d * 0.35F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S379>/Switch' incorporates:
         *  Constant: '<S379>/Land_Speed'
         *  Constant: '<S381>/Constant'
         *  Gain: '<S379>/Gain'
         *  Inport: '<Root>/INS_Out'
         *  Logic: '<S379>/Logical Operator'
         *  RelationalOperator: '<S381>/Compare'
         *  S-Function (sfix_bitop): '<S379>/cmd_p valid'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (((FMS_U.INS_Out.flag & 256U) != 0U) && (FMS_U.INS_Out.h_AGL <=
             FMS_PARAM.ASSIST_LAND_H)) {
          rtb_a_l = 0.5F * FMS_PARAM.LAND_SPEED;
        } else {
          rtb_a_l = FMS_PARAM.LAND_SPEED;
        }

        /* End of Switch: '<S379>/Switch' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S383>/Add' incorporates:
         *  DiscreteIntegrator: '<S382>/Integrator1'
         *  Sum: '<S382>/Subtract'
         */
        rtb_Subtract3_o = (FMS_DW.Integrator1_DSTATE_j - rtb_a_l) + rtb_Add3_c;

        /* Signum: '<S383>/Sign' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_Subtract3_o > 0.0F) {
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_Add4_e5 = rtb_Subtract3_o;
        }

        /* End of Signum: '<S383>/Sign' */

        /* Sum: '<S383>/Add2' incorporates:
         *  Abs: '<S383>/Abs'
         *  Gain: '<S383>/Gain'
         *  Gain: '<S383>/Gain1'
         *  Product: '<S383>/Multiply2'
         *  Product: '<S383>/Multiply3'
         *  Sqrt: '<S383>/Sqrt'
         *  Sum: '<S383>/Add1'
         *  Sum: '<S383>/Subtract'
         */
        rtb_a_l = (sqrtf((8.0F * fabsf(rtb_Subtract3_o) + FMS_ConstB.d_p) *
                         FMS_ConstB.d_p) - FMS_ConstB.d_p) * 0.5F * rtb_Add4_e5
          + rtb_Add3_c;

        /* Sum: '<S383>/Add4' */
        rtb_Add4_e5 = (rtb_Subtract3_o - rtb_a_l) + rtb_Add3_c;

        /* Sum: '<S383>/Add3' */
        rtb_Add3_c = rtb_Subtract3_o + FMS_ConstB.d_p;

        /* Sum: '<S383>/Subtract1' */
        rtb_Subtract3_o -= FMS_ConstB.d_p;

        /* Signum: '<S383>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S383>/Sign1' */

        /* Signum: '<S383>/Sign2' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S383>/Sign2' */

        /* Sum: '<S383>/Add5' incorporates:
         *  Gain: '<S383>/Gain2'
         *  Product: '<S383>/Multiply4'
         *  Sum: '<S383>/Subtract2'
         */
        rtb_a_l += (rtb_Add3_c - rtb_Subtract3_o) * 0.5F * rtb_Add4_e5;

        /* Update for DiscreteIntegrator: '<S382>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S382>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_j += 0.004F * FMS_DW.Integrator_DSTATE_d;

        /* Sum: '<S383>/Subtract3' */
        rtb_Add3_c = rtb_a_l - FMS_ConstB.d_p;

        /* Sum: '<S383>/Add6' */
        rtb_Subtract3_o = rtb_a_l + FMS_ConstB.d_p;

        /* Signum: '<S383>/Sign5' incorporates:
         *  Signum: '<S383>/Sign6'
         */
        if (rtb_a_l < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;

          /* Signum: '<S383>/Sign6' */
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;

          /* Signum: '<S383>/Sign6' */
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_MathFunction_f_idx_0 = rtb_a_l;

          /* Signum: '<S383>/Sign6' */
          rtb_Add4_e5 = rtb_a_l;
        }

        /* End of Signum: '<S383>/Sign5' */

        /* Signum: '<S383>/Sign3' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S383>/Sign3' */

        /* Signum: '<S383>/Sign4' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S383>/Sign4' */

        /* Update for DiscreteIntegrator: '<S382>/Integrator' incorporates:
         *  Constant: '<S383>/const'
         *  Gain: '<S383>/Gain3'
         *  Product: '<S383>/Divide'
         *  Product: '<S383>/Multiply5'
         *  Product: '<S383>/Multiply6'
         *  Sum: '<S383>/Subtract4'
         *  Sum: '<S383>/Subtract5'
         *  Sum: '<S383>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_d += ((rtb_a_l / FMS_ConstB.d_p -
          rtb_MathFunction_f_idx_0) * FMS_ConstB.Gain4_e * ((rtb_Subtract3_o -
          rtb_Add3_c) * 0.5F) - rtb_Add4_e5 * 9.806F) * 0.004F;

        /* End of Outputs for SubSystem: '<S38>/Land' */
        break;

       case 2:
        if (FMS_DW.SwitchCase_ActiveSubsystem_at != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S38>/Return' incorporates:
           *  ActionPort: '<S338>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S38>/Switch Case' incorporates:
           *  Delay: '<S390>/Delay'
           *  Delay: '<S391>/Delay'
           *  Delay: '<S413>/Delay'
           *  DiscreteIntegrator: '<S394>/Integrator'
           *  DiscreteIntegrator: '<S394>/Integrator1'
           *  DiscreteIntegrator: '<S409>/Acceleration_Speed'
           *  DiscreteIntegrator: '<S414>/Discrete-Time Integrator'
           *  DiscreteIntegrator: '<S461>/Discrete-Time Integrator'
           */
          FMS_DW.icLoad_l = 1U;
          FMS_DW.DiscreteTimeIntegrator_DSTATE_m = 0U;
          FMS_DW.Acceleration_Speed_DSTATE_j = 0.0F;
          FMS_DW.Acceleration_Speed_PrevResetS_j = 0;
          FMS_DW.l1_heading_e = 0.0F;
          FMS_DW.icLoad_j = 1U;
          FMS_DW.Integrator1_IC_LOADING_j = 1U;
          FMS_DW.icLoad_c = 1U;
          FMS_DW.Integrator_DSTATE_bs = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S38>/Return' */

          /* SystemReset for IfAction SubSystem: '<S38>/Return' incorporates:
           *  ActionPort: '<S338>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S38>/Switch Case' incorporates:
           *  Chart: '<S425>/Motion Status'
           *  Chart: '<S435>/Motion State'
           */
          FMS_MotionState_Reset(&FMS_DW.sf_MotionState);
          FMS_MotionStatus_Reset(&FMS_DW.sf_MotionStatus);

          /* End of SystemReset for SubSystem: '<S38>/Return' */
        }

        /* Outputs for IfAction SubSystem: '<S38>/Return' incorporates:
         *  ActionPort: '<S338>/Action Port'
         */
        /* Delay: '<S413>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad_l != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_e[0] = FMS_U.INS_Out.x_R;
          FMS_DW.Delay_DSTATE_e[1] = FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S409>/Switch2' incorporates:
         *  Constant: '<S409>/vel'
         *  Constant: '<S418>/Constant'
         *  RelationalOperator: '<S418>/Compare'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        if (FMS_B.Cmd_In.set_speed > 0.0F) {
          rtb_Add4_e5 = FMS_B.Cmd_In.set_speed;
        } else {
          rtb_Add4_e5 = FMS_PARAM.CRUISE_SPEED;
        }

        /* End of Switch: '<S409>/Switch2' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* RelationalOperator: '<S408>/Compare' incorporates:
         *  Constant: '<S465>/Constant'
         *  DiscreteIntegrator: '<S414>/Discrete-Time Integrator'
         *  RelationalOperator: '<S465>/Compare'
         */
        rtb_Compare_on = (FMS_DW.DiscreteTimeIntegrator_DSTATE_m <= 3);

        /* DiscreteIntegrator: '<S409>/Acceleration_Speed' */
        if (rtb_Compare_on || (FMS_DW.Acceleration_Speed_PrevResetS_j != 0)) {
          FMS_DW.Acceleration_Speed_DSTATE_j = 0.0F;
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S435>/Motion State' incorporates:
         *  Constant: '<S435>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S435>/Square'
         *  Math: '<S435>/Square1'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sqrt: '<S435>/Sqrt'
         *  Sum: '<S435>/Add'
         */
        FMS_MotionState(0.0F, sqrtf(FMS_U.INS_Out.vn * FMS_U.INS_Out.vn +
          FMS_U.INS_Out.ve * FMS_U.INS_Out.ve), &rtb_state_c,
                        &FMS_DW.sf_MotionState);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S434>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_g;
        FMS_DW.SwitchCase_ActiveSubsystem_g = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_g = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_g = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_g = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_g) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_g != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S434>/Hold Control' incorporates:
             *  ActionPort: '<S437>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S434>/Switch Case' */
            FMS_HoldControl_k_Reset(&FMS_DW.HoldControl_m);

            /* End of SystemReset for SubSystem: '<S434>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S434>/Hold Control' incorporates:
           *  ActionPort: '<S437>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_m(FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                            FMS_U.INS_Out.psi, FMS_B.Merge_a,
                            &FMS_ConstB.HoldControl_m, &FMS_DW.HoldControl_m);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S434>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S434>/Brake Control' incorporates:
           *  ActionPort: '<S436>/Action Port'
           */
          FMS_BrakeControl_h(FMS_B.Merge_a);

          /* End of Outputs for SubSystem: '<S434>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_g != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S434>/Move Control' incorporates:
             *  ActionPort: '<S438>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S434>/Switch Case' */
            FMS_MoveControl_i_Reset(&FMS_DW.MoveControl_j);

            /* End of SystemReset for SubSystem: '<S434>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S434>/Move Control' incorporates:
           *  ActionPort: '<S438>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_j(FMS_U.Pilot_Cmd.stick_pitch,
                            FMS_U.Pilot_Cmd.stick_roll, FMS_B.Merge_a,
                            &FMS_ConstB.MoveControl_j, &FMS_DW.MoveControl_j);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S434>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S434>/Switch Case' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S425>/Motion Status' incorporates:
         *  Abs: '<S425>/Abs'
         *  Constant: '<S425>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        FMS_MotionStatus(0.0F, fabsf(FMS_U.INS_Out.vd), &rtb_state_c,
                         &FMS_DW.sf_MotionStatus);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S424>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_ld;
        FMS_DW.SwitchCase_ActiveSubsystem_ld = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_ld = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_ld = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_ld = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_ld) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_ld != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S424>/Hold Control' incorporates:
             *  ActionPort: '<S427>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S424>/Switch Case' */
            FMS_HoldControl_Reset(&FMS_DW.HoldControl);

            /* End of SystemReset for SubSystem: '<S424>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S424>/Hold Control' incorporates:
           *  ActionPort: '<S427>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl(FMS_U.INS_Out.h_R, &FMS_B.Merge_jj,
                          &FMS_DW.HoldControl);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S424>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S424>/Brake Control' incorporates:
           *  ActionPort: '<S426>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_jj);

          /* End of Outputs for SubSystem: '<S424>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_ld != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S424>/Move Control' incorporates:
             *  ActionPort: '<S428>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S424>/Switch Case' */
            FMS_MoveControl_Reset(&FMS_DW.MoveControl);

            /* End of SystemReset for SubSystem: '<S424>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S424>/Move Control' incorporates:
           *  ActionPort: '<S428>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl(FMS_U.Pilot_Cmd.stick_throttle, &FMS_B.Merge_jj,
                          &FMS_ConstB.MoveControl, &FMS_DW.MoveControl);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S424>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S424>/Switch Case' */

        /* Switch: '<S389>/Switch' incorporates:
         *  Product: '<S413>/Multiply'
         *  Sum: '<S413>/Sum'
         */
        if (rtb_Compare_on) {
          /* Saturate: '<S434>/Saturation1' */
          if (FMS_B.Merge_a[0] > FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[0] = FMS_PARAM.VEL_XY_LIM;
          } else if (FMS_B.Merge_a[0] < -FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[0] = -FMS_PARAM.VEL_XY_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[0] = FMS_B.Merge_a[0];
          }

          if (FMS_B.Merge_a[1] > FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[1] = FMS_PARAM.VEL_XY_LIM;
          } else if (FMS_B.Merge_a[1] < -FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[1] = -FMS_PARAM.VEL_XY_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[1] = FMS_B.Merge_a[1];
          }

          /* End of Saturate: '<S434>/Saturation1' */

          /* Saturate: '<S424>/Saturation1' */
          if (FMS_B.Merge_jj > FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_PARAM.VEL_Z_LIM;
          } else if (FMS_B.Merge_jj < -FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = -FMS_PARAM.VEL_Z_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_B.Merge_jj;
          }

          /* End of Saturate: '<S424>/Saturation1' */
        } else {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S413>/Sum' incorporates:
           *  Delay: '<S413>/Delay'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[0] - FMS_DW.Delay_DSTATE_e
            [0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* SignalConversion: '<S462>/TmpSignal ConversionAtMath FunctionInport1' */
          rtb_TmpSignalConversionAtMath_c[0] = rtb_Subtract3_o;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S409>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_P_l_idx_0 = FMS_U.INS_Out.x_R - FMS_B.Cmd_In.sp_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_Add3_c = rtb_Subtract3_o;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S413>/Sum' incorporates:
           *  Delay: '<S413>/Delay'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[1] - FMS_DW.Delay_DSTATE_e
            [1];

          /* Sum: '<S409>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_TmpSignalConversionAtMath_0 = FMS_U.INS_Out.y_R -
            FMS_B.Cmd_In.sp_waypoint[1];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Sqrt: '<S421>/Sqrt' incorporates:
           *  Math: '<S421>/Square'
           *  Sum: '<S409>/Sum'
           *  Sum: '<S421>/Sum of Elements'
           */
          rtb_Sqrt_b = sqrtf(rtb_P_l_idx_0 * rtb_P_l_idx_0 +
                             rtb_TmpSignalConversionAtMath_0 *
                             rtb_TmpSignalConversionAtMath_0);

          /* SignalConversion: '<S464>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_c[0];
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_c[1];
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_c[2];

          /* SignalConversion: '<S464>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S464>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Gain: '<S463>/Gain' incorporates:
           *  DiscreteIntegrator: '<S461>/Discrete-Time Integrator'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S461>/Add'
           */
          rtb_a_l = -(FMS_U.INS_Out.psi - FMS_DW.l1_heading_e);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Trigonometry: '<S464>/Trigonometric Function3' incorporates:
           *  Trigonometry: '<S464>/Trigonometric Function1'
           */
          rtb_MathFunction_f_idx_0 = arm_cos_f32(rtb_a_l);
          rtb_VectorConcatenate_i[4] = rtb_MathFunction_f_idx_0;

          /* Trigonometry: '<S464>/Trigonometric Function2' incorporates:
           *  Trigonometry: '<S464>/Trigonometric Function'
           */
          rtb_a_l = arm_sin_f32(rtb_a_l);

          /* Gain: '<S464>/Gain' incorporates:
           *  Trigonometry: '<S464>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -rtb_a_l;

          /* SignalConversion: '<S464>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S464>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S464>/Trigonometric Function' */
          rtb_VectorConcatenate_i[1] = rtb_a_l;

          /* Trigonometry: '<S464>/Trigonometric Function1' */
          rtb_VectorConcatenate_i[0] = rtb_MathFunction_f_idx_0;

          /* Switch: '<S420>/Switch2' incorporates:
           *  Constant: '<S409>/Constant2'
           *  DiscreteIntegrator: '<S409>/Acceleration_Speed'
           *  RelationalOperator: '<S420>/LowerRelop1'
           *  RelationalOperator: '<S420>/UpperRelop'
           *  Switch: '<S420>/Switch'
           */
          if (FMS_DW.Acceleration_Speed_DSTATE_j > rtb_Add4_e5) {
            rtb_a_l = rtb_Add4_e5;
          } else if (FMS_DW.Acceleration_Speed_DSTATE_j < 0.0F) {
            /* Switch: '<S420>/Switch' incorporates:
             *  Constant: '<S409>/Constant2'
             */
            rtb_a_l = 0.0F;
          } else {
            rtb_a_l = FMS_DW.Acceleration_Speed_DSTATE_j;
          }

          /* End of Switch: '<S420>/Switch2' */

          /* Switch: '<S409>/Switch' */
          if (rtb_Sqrt_b > FMS_PARAM.L1) {
            rtb_Sqrt_b = rtb_Add4_e5;
          } else {
            /* Gain: '<S409>/Gain' */
            rtb_Sqrt_b *= 0.5F;

            /* Switch: '<S419>/Switch2' incorporates:
             *  Constant: '<S409>/Constant1'
             *  RelationalOperator: '<S419>/LowerRelop1'
             *  RelationalOperator: '<S419>/UpperRelop'
             *  Switch: '<S419>/Switch'
             */
            if (rtb_Sqrt_b > rtb_Add4_e5) {
              rtb_Sqrt_b = rtb_Add4_e5;
            } else {
              if (rtb_Sqrt_b < 0.5F) {
                /* Switch: '<S419>/Switch' incorporates:
                 *  Constant: '<S409>/Constant1'
                 */
                rtb_Sqrt_b = 0.5F;
              }
            }

            /* End of Switch: '<S419>/Switch2' */
          }

          /* End of Switch: '<S409>/Switch' */

          /* Switch: '<S409>/Switch1' incorporates:
           *  Sum: '<S409>/Sum1'
           */
          if (rtb_a_l - rtb_Sqrt_b < 0.0F) {
            rtb_Sqrt_b = rtb_a_l;
          }

          /* End of Switch: '<S409>/Switch1' */

          /* Sum: '<S462>/Sum of Elements' incorporates:
           *  Math: '<S462>/Math Function'
           */
          rtb_a_l = rtb_TmpSignalConversionAtMath_c[0] *
            rtb_TmpSignalConversionAtMath_c[0] + rtb_Subtract3_o *
            rtb_Subtract3_o;

          /* Math: '<S462>/Math Function1' incorporates:
           *  Sum: '<S462>/Sum of Elements'
           *
           * About '<S462>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_a_l < 0.0F) {
            rtb_a_l = -sqrtf(fabsf(rtb_a_l));
          } else {
            rtb_a_l = sqrtf(rtb_a_l);
          }

          /* End of Math: '<S462>/Math Function1' */

          /* Switch: '<S462>/Switch' incorporates:
           *  Constant: '<S462>/Constant'
           *  Product: '<S462>/Product'
           */
          if (rtb_a_l <= 0.0F) {
            rtb_Add3_c = 0.0F;
            rtb_Subtract3_o = 0.0F;
            rtb_a_l = 1.0F;
          }

          /* End of Switch: '<S462>/Switch' */

          /* Product: '<S460>/Multiply2' incorporates:
           *  Product: '<S462>/Divide'
           */
          rtb_MathFunction_f_idx_0 = rtb_Add3_c / rtb_a_l * rtb_Sqrt_b;
          rtb_Sqrt_b *= rtb_Subtract3_o / rtb_a_l;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S415>/Sum1' incorporates:
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Add3_c = FMS_B.Cmd_In.sp_waypoint[0] - FMS_B.Cmd_In.cur_waypoint[0];
          rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[1] -
            FMS_B.Cmd_In.cur_waypoint[1];

          /* Sum: '<S415>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_a_l = FMS_U.INS_Out.x_R - FMS_B.Cmd_In.cur_waypoint[0];
          rtb_MathFunction_iq_idx_0 = FMS_U.INS_Out.y_R -
            FMS_B.Cmd_In.cur_waypoint[1];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Product: '<S415>/Divide' incorporates:
           *  Math: '<S416>/Square'
           *  Math: '<S417>/Square'
           *  Sqrt: '<S416>/Sqrt'
           *  Sqrt: '<S417>/Sqrt'
           *  Sum: '<S415>/Sum'
           *  Sum: '<S415>/Sum1'
           *  Sum: '<S416>/Sum of Elements'
           *  Sum: '<S417>/Sum of Elements'
           */
          rtb_a_l = sqrtf(rtb_a_l * rtb_a_l + rtb_MathFunction_iq_idx_0 *
                          rtb_MathFunction_iq_idx_0) / sqrtf(rtb_Add3_c *
            rtb_Add3_c + rtb_Subtract3_o * rtb_Subtract3_o);

          /* Saturate: '<S415>/Saturation' */
          if (rtb_a_l > 1.0F) {
            rtb_a_l = 1.0F;
          } else {
            if (rtb_a_l < 0.0F) {
              rtb_a_l = 0.0F;
            }
          }

          /* End of Saturate: '<S415>/Saturation' */

          /* Product: '<S413>/Multiply' */
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_VectorConcatenate_ar[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_Sqrt_b +
              rtb_VectorConcatenate_i[rtb_Compare_bv_0] *
              rtb_MathFunction_f_idx_0;
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Gain: '<S406>/Gain' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S415>/Multiply'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Sum: '<S406>/Sum2'
           *  Sum: '<S415>/Add'
           *  Sum: '<S415>/Subtract'
           */
          rtb_Sqrt_b = (FMS_U.INS_Out.h_R - ((FMS_B.Cmd_In.sp_waypoint[2] -
            FMS_B.Cmd_In.cur_waypoint[2]) * rtb_a_l + FMS_B.Cmd_In.cur_waypoint
            [2])) * FMS_PARAM.Z_P;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[0] = rtb_VectorConcatenate_ar[0];
          rtb_TmpSignalConversionAtMath_c[1] = rtb_VectorConcatenate_ar[1];

          /* Saturate: '<S406>/Saturation1' incorporates:
           *  Product: '<S413>/Multiply'
           */
          if (rtb_Sqrt_b > FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_PARAM.VEL_Z_LIM;
          } else if (rtb_Sqrt_b < -FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = -FMS_PARAM.VEL_Z_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[2] = rtb_Sqrt_b;
          }

          /* End of Saturate: '<S406>/Saturation1' */
        }

        /* End of Switch: '<S389>/Switch' */

        /* Delay: '<S391>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad_j != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_a = FMS_U.INS_Out.psi;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* DiscreteIntegrator: '<S394>/Integrator1' incorporates:
         *  Delay: '<S391>/Delay'
         */
        if (FMS_DW.Integrator1_IC_LOADING_j != 0) {
          FMS_DW.Integrator1_DSTATE_e = FMS_DW.Delay_DSTATE_a;
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Math: '<S398>/Rem' incorporates:
         *  Constant: '<S398>/Constant1'
         *  DiscreteIntegrator: '<S394>/Integrator1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sum: '<S393>/Sum'
         */
        rtb_Add3_c = rt_remf(FMS_DW.Integrator1_DSTATE_e - FMS_U.INS_Out.psi,
                             6.28318548F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Switch: '<S398>/Switch' incorporates:
         *  Abs: '<S398>/Abs'
         *  Constant: '<S398>/Constant'
         *  Constant: '<S399>/Constant'
         *  Product: '<S398>/Multiply'
         *  RelationalOperator: '<S399>/Compare'
         *  Sum: '<S398>/Add'
         */
        if (fabsf(rtb_Add3_c) > 3.14159274F) {
          /* Signum: '<S398>/Sign' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Subtract3_o = -1.0F;
          } else if (rtb_Add3_c > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          } else {
            rtb_Subtract3_o = rtb_Add3_c;
          }

          /* End of Signum: '<S398>/Sign' */
          rtb_Add3_c -= 6.28318548F * rtb_Subtract3_o;
        }

        /* End of Switch: '<S398>/Switch' */

        /* Gain: '<S393>/Gain2' */
        rtb_Add3_c *= FMS_PARAM.YAW_P;

        /* Saturate: '<S393>/Saturation' */
        if (rtb_Add3_c > FMS_PARAM.YAW_RATE_LIM) {
          rtb_Add3_c = FMS_PARAM.YAW_RATE_LIM;
        } else {
          if (rtb_Add3_c < -FMS_PARAM.YAW_RATE_LIM) {
            rtb_Add3_c = -FMS_PARAM.YAW_RATE_LIM;
          }
        }

        /* End of Saturate: '<S393>/Saturation' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S338>/Bus Assignment1'
         *  Constant: '<S338>/Constant2'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S338>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_o;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_m;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_p;
        FMS_Y.FMS_Out.u_cmd = rtb_TmpSignalConversionAtMath_c[0];
        FMS_Y.FMS_Out.v_cmd = rtb_TmpSignalConversionAtMath_c[1];
        FMS_Y.FMS_Out.w_cmd = rtb_TmpSignalConversionAtMath_c[2];
        FMS_Y.FMS_Out.psi_rate_cmd = rtb_Add3_c;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S455>/Sum of Elements' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S455>/Math Function'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sum: '<S453>/Sum of Elements'
         */
        rtb_a_l = FMS_U.INS_Out.vn * FMS_U.INS_Out.vn + FMS_U.INS_Out.ve *
          FMS_U.INS_Out.ve;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S455>/Math Function1' incorporates:
         *  Sum: '<S455>/Sum of Elements'
         *
         * About '<S455>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_a_l < 0.0F) {
          rtb_Add3_c = -sqrtf(fabsf(rtb_a_l));
        } else {
          rtb_Add3_c = sqrtf(rtb_a_l);
        }

        /* End of Math: '<S455>/Math Function1' */

        /* Switch: '<S455>/Switch' incorporates:
         *  Constant: '<S455>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Product: '<S455>/Product'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (rtb_Add3_c > 0.0F) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[0] = FMS_U.INS_Out.vn;
          rtb_TmpSignalConversionAtMath_c[1] = FMS_U.INS_Out.ve;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[2] = rtb_Add3_c;
        } else {
          rtb_TmpSignalConversionAtMath_c[0] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[1] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[2] = 1.0F;
        }

        /* End of Switch: '<S455>/Switch' */

        /* Delay: '<S390>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad_c != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_p[0] = FMS_U.INS_Out.x_R;
          FMS_DW.Delay_DSTATE_p[1] = FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S338>/Sum' incorporates:
         *  Delay: '<S390>/Delay'
         *  MATLAB Function: '<S411>/OutRegionRegWP'
         *  MATLAB Function: '<S411>/SearchL1RefWP'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_P_l_idx_0 = FMS_B.Cmd_In.sp_waypoint[0] - FMS_DW.Delay_DSTATE_p[0];
        rtb_TmpSignalConversionAtMath_0 = FMS_B.Cmd_In.sp_waypoint[1] -
          FMS_DW.Delay_DSTATE_p[1];

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S405>/Sum of Elements' incorporates:
         *  Math: '<S405>/Math Function'
         *  Sum: '<S338>/Sum'
         */
        rtb_Subtract3_o = rtb_TmpSignalConversionAtMath_0 *
          rtb_TmpSignalConversionAtMath_0 + rtb_P_l_idx_0 * rtb_P_l_idx_0;

        /* Math: '<S405>/Math Function1' incorporates:
         *  Sum: '<S405>/Sum of Elements'
         *
         * About '<S405>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Add3_c = -sqrtf(fabsf(rtb_Subtract3_o));
        } else {
          rtb_Add3_c = sqrtf(rtb_Subtract3_o);
        }

        /* End of Math: '<S405>/Math Function1' */

        /* Switch: '<S405>/Switch' incorporates:
         *  Constant: '<S405>/Constant'
         *  Product: '<S405>/Product'
         *  Sum: '<S338>/Sum'
         */
        if (rtb_Add3_c > 0.0F) {
          rtb_MathFunction_f_idx_0 = rtb_TmpSignalConversionAtMath_0;
          rtb_Sqrt_b = rtb_P_l_idx_0;
          rtb_MathFunction_f_idx_2 = rtb_Add3_c;
        } else {
          rtb_MathFunction_f_idx_0 = 0.0F;
          rtb_Sqrt_b = 0.0F;
          rtb_MathFunction_f_idx_2 = 1.0F;
        }

        /* End of Switch: '<S405>/Switch' */

        /* Product: '<S455>/Divide' */
        rtb_Sum_ff[0] = rtb_TmpSignalConversionAtMath_c[0] /
          rtb_TmpSignalConversionAtMath_c[2];
        rtb_Sum_ff[1] = rtb_TmpSignalConversionAtMath_c[1] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Sum: '<S458>/Sum of Elements' incorporates:
         *  Math: '<S458>/Math Function'
         *  SignalConversion: '<S458>/TmpSignal ConversionAtMath FunctionInport1'
         */
        rtb_Subtract3_o = rtb_Sum_ff[1] * rtb_Sum_ff[1] + rtb_Sum_ff[0] *
          rtb_Sum_ff[0];

        /* Math: '<S458>/Math Function1' incorporates:
         *  Sum: '<S458>/Sum of Elements'
         *
         * About '<S458>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Add3_c = -sqrtf(fabsf(rtb_Subtract3_o));
        } else {
          rtb_Add3_c = sqrtf(rtb_Subtract3_o);
        }

        /* End of Math: '<S458>/Math Function1' */

        /* Switch: '<S458>/Switch' incorporates:
         *  Constant: '<S458>/Constant'
         *  Product: '<S458>/Product'
         */
        if (rtb_Add3_c > 0.0F) {
          rtb_TmpSignalConversionAtMath_c[0] = rtb_Sum_ff[1];
          rtb_TmpSignalConversionAtMath_c[1] = rtb_Sum_ff[0];
          rtb_TmpSignalConversionAtMath_c[2] = rtb_Add3_c;
        } else {
          rtb_TmpSignalConversionAtMath_c[0] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[1] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[2] = 1.0F;
        }

        /* End of Switch: '<S458>/Switch' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S411>/NearbyRefWP' incorporates:
         *  Constant: '<S338>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        FMS_NearbyRefWP(&rtb_Switch_dw[0], FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                        FMS_PARAM.L1, rtb_Sum_ff, &rtb_Rem_p);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* MATLAB Function: '<S411>/SearchL1RefWP' incorporates:
         *  Constant: '<S338>/L1'
         *  Delay: '<S390>/Delay'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_Subtract3_o = rtb_P_l_idx_0 * rtb_P_l_idx_0 +
          rtb_TmpSignalConversionAtMath_0 * rtb_TmpSignalConversionAtMath_0;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        B = (rtb_P_l_idx_0 * (FMS_DW.Delay_DSTATE_p[0] - FMS_U.INS_Out.x_R) +
             rtb_TmpSignalConversionAtMath_0 * (FMS_DW.Delay_DSTATE_p[1] -
              FMS_U.INS_Out.y_R)) * 2.0F;
        D = B * B - (((((FMS_U.INS_Out.x_R * FMS_U.INS_Out.x_R +
                         FMS_U.INS_Out.y_R * FMS_U.INS_Out.y_R) +
                        FMS_DW.Delay_DSTATE_p[0] * FMS_DW.Delay_DSTATE_p[0]) +
                       FMS_DW.Delay_DSTATE_p[1] * FMS_DW.Delay_DSTATE_p[1]) -
                      (FMS_U.INS_Out.x_R * FMS_DW.Delay_DSTATE_p[0] +
                       FMS_U.INS_Out.y_R * FMS_DW.Delay_DSTATE_p[1]) * 2.0F) -
                     FMS_PARAM.L1 * FMS_PARAM.L1) * (4.0F * rtb_Subtract3_o);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_Add3_c = -1.0F;
        rtb_MathFunction_iq_idx_0 = 0.0F;
        rtb_MathFunction_iq_idx_1 = 0.0F;
        guard1 = false;
        if (D > 0.0F) {
          u1_tmp = sqrtf(D);
          D = (-B + u1_tmp) / (2.0F * rtb_Subtract3_o);
          rtb_Subtract3_o = (-B - u1_tmp) / (2.0F * rtb_Subtract3_o);
          if ((D >= 0.0F) && (D <= 1.0F) && (rtb_Subtract3_o >= 0.0F) &&
              (rtb_Subtract3_o <= 1.0F)) {
            rtb_Add3_c = fmaxf(D, rtb_Subtract3_o);
            guard1 = true;
          } else if ((D >= 0.0F) && (D <= 1.0F)) {
            rtb_Add3_c = D;
            guard1 = true;
          } else {
            if ((rtb_Subtract3_o >= 0.0F) && (rtb_Subtract3_o <= 1.0F)) {
              rtb_Add3_c = rtb_Subtract3_o;
              guard1 = true;
            }
          }
        } else {
          if (D == 0.0F) {
            D = -B / (2.0F * rtb_Subtract3_o);
            if ((D >= 0.0F) && (D <= 1.0F)) {
              rtb_Add3_c = D;
              guard1 = true;
            }
          }
        }

        if (guard1) {
          rtb_MathFunction_iq_idx_0 = rtb_P_l_idx_0 * rtb_Add3_c +
            FMS_DW.Delay_DSTATE_p[0];
          rtb_MathFunction_iq_idx_1 = rtb_TmpSignalConversionAtMath_0 *
            rtb_Add3_c + FMS_DW.Delay_DSTATE_p[1];
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S411>/OutRegionRegWP' incorporates:
         *  Delay: '<S390>/Delay'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_Subtract3_o = ((FMS_U.INS_Out.y_R - FMS_DW.Delay_DSTATE_p[1]) *
                           rtb_TmpSignalConversionAtMath_0 + (FMS_U.INS_Out.x_R
          - FMS_DW.Delay_DSTATE_p[0]) * rtb_P_l_idx_0) / (rtb_P_l_idx_0 *
          rtb_P_l_idx_0 + rtb_TmpSignalConversionAtMath_0 *
          rtb_TmpSignalConversionAtMath_0);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_FixPtRelationalOperator_me = (rtb_Subtract3_o <= 0.0F);
        rtb_LogicalOperator_es = (rtb_Subtract3_o >= 1.0F);
        if (rtb_FixPtRelationalOperator_me) {
          rtb_P_l_idx_0 = FMS_DW.Delay_DSTATE_p[0];
        } else if (rtb_LogicalOperator_es) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_P_l_idx_0 = FMS_B.Cmd_In.sp_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        } else {
          rtb_P_l_idx_0 = rtb_Subtract3_o * rtb_P_l_idx_0 +
            FMS_DW.Delay_DSTATE_p[0];
        }

        /* Switch: '<S411>/Switch1' incorporates:
         *  Constant: '<S448>/Constant'
         *  RelationalOperator: '<S448>/Compare'
         */
        if (rtb_Rem_p <= 0.0F) {
          /* Switch: '<S411>/Switch' incorporates:
           *  Constant: '<S447>/Constant'
           *  MATLAB Function: '<S411>/SearchL1RefWP'
           *  RelationalOperator: '<S447>/Compare'
           */
          if (rtb_Add3_c >= 0.0F) {
            rtb_Sum_ff[0] = rtb_MathFunction_iq_idx_0;
            rtb_Sum_ff[1] = rtb_MathFunction_iq_idx_1;
          } else {
            rtb_Sum_ff[0] = rtb_P_l_idx_0;

            /* MATLAB Function: '<S411>/OutRegionRegWP' incorporates:
             *  Delay: '<S390>/Delay'
             *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
             */
            if (rtb_FixPtRelationalOperator_me) {
              rtb_Sum_ff[1] = FMS_DW.Delay_DSTATE_p[1];
            } else if (rtb_LogicalOperator_es) {
              /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
              rtb_Sum_ff[1] = FMS_B.Cmd_In.sp_waypoint[1];

              /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            } else {
              rtb_Sum_ff[1] = rtb_Subtract3_o * rtb_TmpSignalConversionAtMath_0
                + FMS_DW.Delay_DSTATE_p[1];
            }
          }

          /* End of Switch: '<S411>/Switch' */
        }

        /* End of Switch: '<S411>/Switch1' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S412>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_Rem_p = rtb_Sum_ff[0] - FMS_U.INS_Out.x_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_Sum_ff[0] = rtb_Rem_p;
        rtb_TmpSignalConversionAtDela_a[0] = rtb_Rem_p * rtb_Rem_p;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S412>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S456>/Math Function'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_Rem_p = rtb_Sum_ff[1] - FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S456>/Math Function' incorporates:
         *  Math: '<S454>/Square'
         */
        rtb_Subtract3_o = rtb_Rem_p * rtb_Rem_p;

        /* Sum: '<S456>/Sum of Elements' incorporates:
         *  Math: '<S456>/Math Function'
         */
        rtb_Add3_c = rtb_Subtract3_o + rtb_TmpSignalConversionAtDela_a[0];

        /* Math: '<S456>/Math Function1' incorporates:
         *  Sum: '<S456>/Sum of Elements'
         *
         * About '<S456>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -sqrtf(fabsf(rtb_Add3_c));
        } else {
          rtb_Add3_c = sqrtf(rtb_Add3_c);
        }

        /* End of Math: '<S456>/Math Function1' */

        /* Switch: '<S456>/Switch' incorporates:
         *  Constant: '<S456>/Constant'
         *  Product: '<S456>/Product'
         */
        if (rtb_Add3_c > 0.0F) {
          rtb_Switch_dw[0] = rtb_Sum_ff[0];
          rtb_Switch_dw[1] = rtb_Rem_p;
          rtb_Switch_dw[2] = rtb_Add3_c;
        } else {
          rtb_Switch_dw[0] = 0.0F;
          rtb_Switch_dw[1] = 0.0F;
          rtb_Switch_dw[2] = 1.0F;
        }

        /* End of Switch: '<S456>/Switch' */

        /* Product: '<S456>/Divide' */
        rtb_TmpSignalConversionAtMath_0 = rtb_Switch_dw[0] / rtb_Switch_dw[2];
        rtb_P_l_idx_0 = rtb_Switch_dw[1] / rtb_Switch_dw[2];

        /* Sum: '<S459>/Sum of Elements' incorporates:
         *  Math: '<S459>/Math Function'
         *  SignalConversion: '<S459>/TmpSignal ConversionAtMath FunctionInport1'
         */
        rtb_Add3_c = rtb_P_l_idx_0 * rtb_P_l_idx_0 +
          rtb_TmpSignalConversionAtMath_0 * rtb_TmpSignalConversionAtMath_0;

        /* Math: '<S459>/Math Function1' incorporates:
         *  Sum: '<S459>/Sum of Elements'
         *
         * About '<S459>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -sqrtf(fabsf(rtb_Add3_c));
        } else {
          rtb_Add3_c = sqrtf(rtb_Add3_c);
        }

        /* End of Math: '<S459>/Math Function1' */

        /* Switch: '<S459>/Switch' incorporates:
         *  Constant: '<S459>/Constant'
         *  Product: '<S459>/Product'
         */
        if (rtb_Add3_c > 0.0F) {
          rtb_Switch_dw[0] = rtb_P_l_idx_0;
          rtb_Switch_dw[1] = rtb_TmpSignalConversionAtMath_0;
          rtb_Switch_dw[2] = rtb_Add3_c;
        } else {
          rtb_Switch_dw[0] = 0.0F;
          rtb_Switch_dw[1] = 0.0F;
          rtb_Switch_dw[2] = 1.0F;
        }

        /* End of Switch: '<S459>/Switch' */

        /* Product: '<S459>/Divide' */
        rtb_TmpSignalConversionAtMath_0 = rtb_Switch_dw[0] / rtb_Switch_dw[2];

        /* Math: '<S454>/Square' */
        rtb_TmpSignalConversionAtDela_a[0] = rtb_Sum_ff[0] * rtb_Sum_ff[0];

        /* Product: '<S405>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_MathFunction_f_idx_0 /
          rtb_MathFunction_f_idx_2;
        rtb_Sum_ff[0] = rtb_TmpSignalConversionAtMath_c[0] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Product: '<S459>/Divide' incorporates:
         *  Product: '<S458>/Divide'
         */
        rtb_P_l_idx_0 = rtb_Switch_dw[1] / rtb_Switch_dw[2];

        /* Product: '<S458>/Divide' */
        rtb_Rem_p = rtb_TmpSignalConversionAtMath_c[1] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Product: '<S405>/Divide' */
        rtb_MathFunction_iq_idx_1 = rtb_Sqrt_b / rtb_MathFunction_f_idx_2;

        /* Sqrt: '<S453>/Sqrt' */
        rtb_Add3_c = sqrtf(rtb_a_l);

        /* Gain: '<S412>/Gain' incorporates:
         *  Math: '<S412>/Square'
         */
        rtb_a_l = rtb_Add3_c * rtb_Add3_c * 2.0F;

        /* Sum: '<S457>/Subtract' incorporates:
         *  Product: '<S457>/Multiply'
         *  Product: '<S457>/Multiply1'
         */
        rtb_Add3_c = rtb_TmpSignalConversionAtMath_0 * rtb_Rem_p - rtb_P_l_idx_0
          * rtb_Sum_ff[0];

        /* Signum: '<S452>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S452>/Sign1' */

        /* Switch: '<S452>/Switch2' incorporates:
         *  Constant: '<S452>/Constant4'
         */
        if (rtb_Add3_c == 0.0F) {
          rtb_Add3_c = 1.0F;
        }

        /* End of Switch: '<S452>/Switch2' */

        /* DotProduct: '<S452>/Dot Product' */
        rtb_Rem_p = rtb_Sum_ff[0] * rtb_TmpSignalConversionAtMath_0 + rtb_Rem_p *
          rtb_P_l_idx_0;

        /* Trigonometry: '<S452>/Acos' incorporates:
         *  DotProduct: '<S452>/Dot Product'
         */
        if (rtb_Rem_p > 1.0F) {
          rtb_Rem_p = 1.0F;
        } else {
          if (rtb_Rem_p < -1.0F) {
            rtb_Rem_p = -1.0F;
          }
        }

        /* Product: '<S452>/Multiply' incorporates:
         *  Trigonometry: '<S452>/Acos'
         */
        rtb_Add3_c *= acosf(rtb_Rem_p);

        /* Saturate: '<S412>/Saturation' */
        if (rtb_Add3_c > 1.57079637F) {
          rtb_Add3_c = 1.57079637F;
        } else {
          if (rtb_Add3_c < -1.57079637F) {
            rtb_Add3_c = -1.57079637F;
          }
        }

        /* End of Saturate: '<S412>/Saturation' */

        /* Product: '<S412>/Divide' incorporates:
         *  Constant: '<S338>/L1'
         *  Constant: '<S412>/Constant'
         *  MinMax: '<S412>/Max'
         *  MinMax: '<S412>/Min'
         *  Product: '<S412>/Multiply1'
         *  Sqrt: '<S454>/Sqrt'
         *  Sum: '<S454>/Sum of Elements'
         *  Trigonometry: '<S412>/Sin'
         */
        rtb_a_l = arm_sin_f32(rtb_Add3_c) * rtb_a_l / fminf(FMS_PARAM.L1, fmaxf
          (sqrtf(rtb_Subtract3_o + rtb_TmpSignalConversionAtDela_a[0]), 0.5F));

        /* Sum: '<S403>/Subtract' incorporates:
         *  Product: '<S403>/Multiply'
         *  Product: '<S403>/Multiply1'
         */
        rtb_MathFunction_f_idx_2 = rtb_MathFunction_iq_idx_0 *
          FMS_ConstB.Divide_d[1] - rtb_MathFunction_iq_idx_1 *
          FMS_ConstB.Divide_d[0];

        /* Signum: '<S392>/Sign1' */
        if (rtb_MathFunction_f_idx_2 < 0.0F) {
          rtb_MathFunction_f_idx_2 = -1.0F;
        } else {
          if (rtb_MathFunction_f_idx_2 > 0.0F) {
            rtb_MathFunction_f_idx_2 = 1.0F;
          }
        }

        /* End of Signum: '<S392>/Sign1' */

        /* Switch: '<S392>/Switch2' incorporates:
         *  Constant: '<S392>/Constant4'
         */
        if (rtb_MathFunction_f_idx_2 == 0.0F) {
          rtb_MathFunction_f_idx_2 = 1.0F;
        }

        /* End of Switch: '<S392>/Switch2' */

        /* DotProduct: '<S392>/Dot Product' */
        rtb_Sqrt_b = FMS_ConstB.Divide_d[0] * rtb_MathFunction_iq_idx_0 +
          FMS_ConstB.Divide_d[1] * rtb_MathFunction_iq_idx_1;

        /* Trigonometry: '<S392>/Acos' incorporates:
         *  DotProduct: '<S392>/Dot Product'
         */
        if (rtb_Sqrt_b > 1.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          if (rtb_Sqrt_b < -1.0F) {
            rtb_Sqrt_b = -1.0F;
          }
        }

        /* Product: '<S392>/Multiply' incorporates:
         *  Trigonometry: '<S392>/Acos'
         */
        rtb_MathFunction_f_idx_2 *= acosf(rtb_Sqrt_b);

        /* Math: '<S395>/Rem' incorporates:
         *  Constant: '<S395>/Constant1'
         *  Delay: '<S391>/Delay'
         *  Sum: '<S391>/Sum2'
         */
        rtb_Add3_c = rt_remf(rtb_MathFunction_f_idx_2 - FMS_DW.Delay_DSTATE_a,
                             6.28318548F);

        /* Switch: '<S395>/Switch' incorporates:
         *  Abs: '<S395>/Abs'
         *  Constant: '<S395>/Constant'
         *  Constant: '<S401>/Constant'
         *  Product: '<S395>/Multiply'
         *  RelationalOperator: '<S401>/Compare'
         *  Sum: '<S395>/Add'
         */
        if (fabsf(rtb_Add3_c) > 3.14159274F) {
          /* Signum: '<S395>/Sign' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Subtract3_o = -1.0F;
          } else if (rtb_Add3_c > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          } else {
            rtb_Subtract3_o = rtb_Add3_c;
          }

          /* End of Signum: '<S395>/Sign' */
          rtb_Add3_c -= 6.28318548F * rtb_Subtract3_o;
        }

        /* End of Switch: '<S395>/Switch' */

        /* Sum: '<S391>/Sum' incorporates:
         *  Delay: '<S391>/Delay'
         */
        rtb_Subtract3_o = rtb_Add3_c + FMS_DW.Delay_DSTATE_a;

        /* Product: '<S400>/Multiply1' incorporates:
         *  Constant: '<S400>/const1'
         *  DiscreteIntegrator: '<S394>/Integrator'
         */
        rtb_Add3_c = FMS_DW.Integrator_DSTATE_bs * 0.785398185F;

        /* Sum: '<S400>/Add' incorporates:
         *  DiscreteIntegrator: '<S394>/Integrator1'
         *  Sum: '<S394>/Subtract'
         */
        rtb_Subtract3_o = (FMS_DW.Integrator1_DSTATE_e - rtb_Subtract3_o) +
          rtb_Add3_c;

        /* Signum: '<S400>/Sign' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else if (rtb_Subtract3_o > 0.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          rtb_MathFunction_f_idx_0 = rtb_Subtract3_o;
        }

        /* End of Signum: '<S400>/Sign' */

        /* Sum: '<S400>/Add2' incorporates:
         *  Abs: '<S400>/Abs'
         *  Gain: '<S400>/Gain'
         *  Gain: '<S400>/Gain1'
         *  Product: '<S400>/Multiply2'
         *  Product: '<S400>/Multiply3'
         *  Sqrt: '<S400>/Sqrt'
         *  Sum: '<S400>/Add1'
         *  Sum: '<S400>/Subtract'
         */
        rtb_Rem_p = (sqrtf((8.0F * fabsf(rtb_Subtract3_o) + FMS_ConstB.d_le) *
                           FMS_ConstB.d_le) - FMS_ConstB.d_le) * 0.5F *
          rtb_MathFunction_f_idx_0 + rtb_Add3_c;

        /* Sum: '<S400>/Add4' */
        rtb_MathFunction_f_idx_0 = (rtb_Subtract3_o - rtb_Rem_p) + rtb_Add3_c;

        /* Sum: '<S400>/Add3' */
        rtb_Add3_c = rtb_Subtract3_o + FMS_ConstB.d_le;

        /* Sum: '<S400>/Subtract1' */
        rtb_Subtract3_o -= FMS_ConstB.d_le;

        /* Signum: '<S400>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S400>/Sign1' */

        /* Signum: '<S400>/Sign2' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S400>/Sign2' */

        /* Sum: '<S400>/Add5' incorporates:
         *  Gain: '<S400>/Gain2'
         *  Product: '<S400>/Multiply4'
         *  Sum: '<S400>/Subtract2'
         */
        rtb_Rem_p += (rtb_Add3_c - rtb_Subtract3_o) * 0.5F *
          rtb_MathFunction_f_idx_0;

        /* Sum: '<S400>/Add6' */
        rtb_Add3_c = rtb_Rem_p + FMS_ConstB.d_le;

        /* Sum: '<S400>/Subtract3' */
        rtb_Subtract3_o = rtb_Rem_p - FMS_ConstB.d_le;

        /* Product: '<S400>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_Rem_p / FMS_ConstB.d_le;

        /* Signum: '<S400>/Sign5' incorporates:
         *  Signum: '<S400>/Sign6'
         */
        if (rtb_Rem_p < 0.0F) {
          rtb_MathFunction_iq_idx_1 = -1.0F;

          /* Signum: '<S400>/Sign6' */
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else if (rtb_Rem_p > 0.0F) {
          rtb_MathFunction_iq_idx_1 = 1.0F;

          /* Signum: '<S400>/Sign6' */
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          rtb_MathFunction_iq_idx_1 = rtb_Rem_p;

          /* Signum: '<S400>/Sign6' */
          rtb_MathFunction_f_idx_0 = rtb_Rem_p;
        }

        /* End of Signum: '<S400>/Sign5' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S391>/Sum1' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_MathFunction_f_idx_2 -= FMS_U.INS_Out.psi;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S396>/Rem' incorporates:
         *  Constant: '<S396>/Constant1'
         */
        rtb_Rem_p = rt_remf(rtb_MathFunction_f_idx_2, 6.28318548F);

        /* Switch: '<S396>/Switch' incorporates:
         *  Abs: '<S396>/Abs'
         *  Constant: '<S396>/Constant'
         *  Constant: '<S402>/Constant'
         *  Product: '<S396>/Multiply'
         *  RelationalOperator: '<S402>/Compare'
         *  Sum: '<S396>/Add'
         */
        if (fabsf(rtb_Rem_p) > 3.14159274F) {
          /* Signum: '<S396>/Sign' */
          if (rtb_Rem_p < 0.0F) {
            rtb_Sqrt_b = -1.0F;
          } else if (rtb_Rem_p > 0.0F) {
            rtb_Sqrt_b = 1.0F;
          } else {
            rtb_Sqrt_b = rtb_Rem_p;
          }

          /* End of Signum: '<S396>/Sign' */
          rtb_Rem_p -= 6.28318548F * rtb_Sqrt_b;
        }

        /* End of Switch: '<S396>/Switch' */

        /* Abs: '<S389>/Abs' */
        rtb_Rem_p = fabsf(rtb_Rem_p);

        /* Update for Delay: '<S413>/Delay' */
        FMS_DW.icLoad_l = 0U;

        /* Update for DiscreteIntegrator: '<S414>/Discrete-Time Integrator' incorporates:
         *  Constant: '<S407>/Constant'
         *  RelationalOperator: '<S407>/Compare'
         */
        FMS_DW.DiscreteTimeIntegrator_DSTATE_m = (uint8_T)((uint32_T)(rtb_Rem_p <=
          0.17453292F) + FMS_DW.DiscreteTimeIntegrator_DSTATE_m);
        if (FMS_DW.DiscreteTimeIntegrator_DSTATE_m >= 100) {
          FMS_DW.DiscreteTimeIntegrator_DSTATE_m = 100U;
        } else {
          if (FMS_DW.DiscreteTimeIntegrator_DSTATE_m <= 0) {
            FMS_DW.DiscreteTimeIntegrator_DSTATE_m = 0U;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S414>/Discrete-Time Integrator' */

        /* Update for DiscreteIntegrator: '<S409>/Acceleration_Speed' incorporates:
         *  Constant: '<S409>/Constant'
         */
        FMS_DW.Acceleration_Speed_DSTATE_j += 0.004F * FMS_PARAM.CRUISE_ACC;
        FMS_DW.Acceleration_Speed_PrevResetS_j = (int8_T)rtb_Compare_on;

        /* Product: '<S413>/Divide1' */
        rtb_Sqrt_b = rtb_a_l / rtb_Add4_e5;

        /* Saturate: '<S413>/Saturation' */
        if (rtb_Sqrt_b > 0.314159274F) {
          rtb_Sqrt_b = 0.314159274F;
        } else {
          if (rtb_Sqrt_b < -0.314159274F) {
            rtb_Sqrt_b = -0.314159274F;
          }
        }

        /* End of Saturate: '<S413>/Saturation' */

        /* Update for DiscreteIntegrator: '<S461>/Discrete-Time Integrator' */
        FMS_DW.l1_heading_e += 0.004F * rtb_Sqrt_b;

        /* Update for Delay: '<S391>/Delay' */
        FMS_DW.icLoad_j = 0U;

        /* Update for DiscreteIntegrator: '<S394>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S394>/Integrator'
         */
        FMS_DW.Integrator1_IC_LOADING_j = 0U;
        FMS_DW.Integrator1_DSTATE_e += 0.004F * FMS_DW.Integrator_DSTATE_bs;

        /* Update for Delay: '<S390>/Delay' */
        FMS_DW.icLoad_c = 0U;

        /* Signum: '<S400>/Sign3' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S400>/Sign3' */

        /* Signum: '<S400>/Sign4' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S400>/Sign4' */

        /* Update for DiscreteIntegrator: '<S394>/Integrator' incorporates:
         *  Constant: '<S400>/const'
         *  Gain: '<S400>/Gain3'
         *  Product: '<S400>/Multiply5'
         *  Product: '<S400>/Multiply6'
         *  Sum: '<S400>/Subtract4'
         *  Sum: '<S400>/Subtract5'
         *  Sum: '<S400>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_bs += ((rtb_MathFunction_iq_idx_0 -
          rtb_MathFunction_iq_idx_1) * FMS_ConstB.Gain4_np * ((rtb_Add3_c -
          rtb_Subtract3_o) * 0.5F) - rtb_MathFunction_f_idx_0 * 1.04719758F) *
          0.004F;
        if (FMS_DW.Integrator_DSTATE_bs >= FMS_PARAM.YAW_RATE_LIM) {
          FMS_DW.Integrator_DSTATE_bs = FMS_PARAM.YAW_RATE_LIM;
        } else {
          if (FMS_DW.Integrator_DSTATE_bs <= -FMS_PARAM.YAW_RATE_LIM) {
            FMS_DW.Integrator_DSTATE_bs = -FMS_PARAM.YAW_RATE_LIM;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S394>/Integrator' */
        /* End of Outputs for SubSystem: '<S38>/Return' */
        break;

       case 3:
        if (FMS_DW.SwitchCase_ActiveSubsystem_at != rtPrevAction) {
          /* SystemReset for IfAction SubSystem: '<S38>/Hold' incorporates:
           *  ActionPort: '<S336>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S38>/Switch Case' incorporates:
           *  Chart: '<S345>/Motion Status'
           *  Chart: '<S355>/Motion State'
           *  Chart: '<S367>/Motion State'
           */
          FMS_MotionStatus_Reset(&FMS_DW.sf_MotionStatus_j);
          FMS_DW.temporalCounter_i1_ai = 0U;
          FMS_DW.is_active_c15_FMS = 0U;
          FMS_DW.is_c15_FMS = FMS_IN_NO_ACTIVE_CHILD_h;
          FMS_MotionState_Reset(&FMS_DW.sf_MotionState_g);

          /* End of SystemReset for SubSystem: '<S38>/Hold' */
        }

        /* Outputs for IfAction SubSystem: '<S38>/Hold' incorporates:
         *  ActionPort: '<S336>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S345>/Motion Status' incorporates:
         *  Abs: '<S345>/Abs'
         *  Constant: '<S345>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        FMS_MotionStatus(0.0F, fabsf(FMS_U.INS_Out.vd), &rtb_state_c,
                         &FMS_DW.sf_MotionStatus_j);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Chart: '<S355>/Motion State' incorporates:
         *  Abs: '<S355>/Abs'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.temporalCounter_i1_ai < 255U) {
          FMS_DW.temporalCounter_i1_ai++;
        }

        if (FMS_DW.is_active_c15_FMS == 0U) {
          FMS_DW.is_active_c15_FMS = 1U;
          FMS_DW.is_c15_FMS = FMS_IN_Move_n;
          rtb_state_ki = MotionState_Move;
        } else {
          switch (FMS_DW.is_c15_FMS) {
           case FMS_IN_Brake_o:
            rtb_state_ki = MotionState_Brake;

            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            if ((fabsf(FMS_U.INS_Out.r) <= 0.1) || (FMS_DW.temporalCounter_i1_ai
                 >= 250U)) {
              FMS_DW.is_c15_FMS = FMS_IN_Hold_d;
              rtb_state_ki = MotionState_Hold;
            }

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            break;

           case FMS_IN_Hold_d:
            rtb_state_ki = MotionState_Hold;
            break;

           default:
            FMS_DW.is_c15_FMS = FMS_IN_Brake_o;
            FMS_DW.temporalCounter_i1_ai = 0U;
            rtb_state_ki = MotionState_Brake;
            break;
          }
        }

        /* End of Chart: '<S355>/Motion State' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S367>/Motion State' incorporates:
         *  Constant: '<S367>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S367>/Square'
         *  Math: '<S367>/Square1'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sqrt: '<S367>/Sqrt'
         *  Sum: '<S367>/Add'
         */
        FMS_MotionState(0.0F, sqrtf(FMS_U.INS_Out.vn * FMS_U.INS_Out.vn +
          FMS_U.INS_Out.ve * FMS_U.INS_Out.ve), &rtb_state_l,
                        &FMS_DW.sf_MotionState_g);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S366>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_p;
        FMS_DW.SwitchCase_ActiveSubsystem_p = -1;
        switch (rtb_state_l) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_p = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_p = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_p = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_p) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_p != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S366>/Hold Control' incorporates:
             *  ActionPort: '<S369>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S366>/Switch Case' */
            FMS_HoldControl_k_Reset(&FMS_DW.HoldControl_f);

            /* End of SystemReset for SubSystem: '<S366>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S366>/Hold Control' incorporates:
           *  ActionPort: '<S369>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_m(FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                            FMS_U.INS_Out.psi, FMS_B.Merge_o,
                            &FMS_ConstB.HoldControl_f, &FMS_DW.HoldControl_f);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S366>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S366>/Brake Control' incorporates:
           *  ActionPort: '<S368>/Action Port'
           */
          FMS_BrakeControl_h(FMS_B.Merge_o);

          /* End of Outputs for SubSystem: '<S366>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_p != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S366>/Move Control' incorporates:
             *  ActionPort: '<S370>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S366>/Switch Case' */
            FMS_MoveControl_i_Reset(&FMS_DW.MoveControl_i);

            /* End of SystemReset for SubSystem: '<S366>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S366>/Move Control' incorporates:
           *  ActionPort: '<S370>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_j(FMS_U.Pilot_Cmd.stick_pitch,
                            FMS_U.Pilot_Cmd.stick_roll, FMS_B.Merge_o,
                            &FMS_ConstB.MoveControl_i, &FMS_DW.MoveControl_i);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S366>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S366>/Switch Case' */

        /* SwitchCase: '<S344>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_pp;
        FMS_DW.SwitchCase_ActiveSubsystem_pp = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_pp = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_pp = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_pp = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_pp) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_pp != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S344>/Hold Control' incorporates:
             *  ActionPort: '<S347>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S344>/Switch Case' */
            FMS_HoldControl_Reset(&FMS_DW.HoldControl_n);

            /* End of SystemReset for SubSystem: '<S344>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S344>/Hold Control' incorporates:
           *  ActionPort: '<S347>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl(FMS_U.INS_Out.h_R, &FMS_B.Merge_ey,
                          &FMS_DW.HoldControl_n);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S344>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S344>/Brake Control' incorporates:
           *  ActionPort: '<S346>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_ey);

          /* End of Outputs for SubSystem: '<S344>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_pp != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S344>/Move Control' incorporates:
             *  ActionPort: '<S348>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S344>/Switch Case' */
            FMS_MoveControl_Reset(&FMS_DW.MoveControl_n);

            /* End of SystemReset for SubSystem: '<S344>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S344>/Move Control' incorporates:
           *  ActionPort: '<S348>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl(FMS_U.Pilot_Cmd.stick_throttle, &FMS_B.Merge_ey,
                          &FMS_ConstB.MoveControl_n, &FMS_DW.MoveControl_n);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S344>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S344>/Switch Case' */

        /* SwitchCase: '<S354>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_bn;
        FMS_DW.SwitchCase_ActiveSubsystem_bn = -1;
        switch (rtb_state_ki) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_bn = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_bn = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_bn = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_bn) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_bn != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S354>/Hold Control' incorporates:
             *  ActionPort: '<S357>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S354>/Switch Case' */
            FMS_HoldControl_kp_Reset(&FMS_DW.HoldControl_k);

            /* End of SystemReset for SubSystem: '<S354>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S354>/Hold Control' incorporates:
           *  ActionPort: '<S357>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_k(FMS_U.INS_Out.psi, &FMS_B.Merge_n1,
                            &FMS_DW.HoldControl_k);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S354>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S354>/Brake Control' incorporates:
           *  ActionPort: '<S356>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_n1);

          /* End of Outputs for SubSystem: '<S354>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_bn != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S354>/Move Control' incorporates:
             *  ActionPort: '<S358>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S354>/Switch Case' */
            FMS_MoveControl_l_Reset(&FMS_DW.MoveControl_b);

            /* End of SystemReset for SubSystem: '<S354>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S354>/Move Control' incorporates:
           *  ActionPort: '<S358>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_b(FMS_U.Pilot_Cmd.stick_yaw, &FMS_B.Merge_n1,
                            &FMS_ConstB.MoveControl_b, &FMS_DW.MoveControl_b);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S354>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S354>/Switch Case' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S336>/Bus Assignment'
         *  Constant: '<S336>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S336>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_g;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_i;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_j;

        /* Saturate: '<S354>/Saturation' */
        if (FMS_B.Merge_n1 > FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_PARAM.YAW_RATE_LIM;
        } else if (FMS_B.Merge_n1 < -FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -FMS_PARAM.YAW_RATE_LIM;
        } else {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_B.Merge_n1;
        }

        /* End of Saturate: '<S354>/Saturation' */

        /* Saturate: '<S366>/Saturation1' */
        if (FMS_B.Merge_o[0] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (FMS_B.Merge_o[0] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_B.Merge_o[0];
        }

        if (FMS_B.Merge_o[1] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (FMS_B.Merge_o[1] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = FMS_B.Merge_o[1];
        }

        /* End of Saturate: '<S366>/Saturation1' */

        /* Saturate: '<S344>/Saturation1' */
        if (FMS_B.Merge_ey > FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = FMS_PARAM.VEL_Z_LIM;
        } else if (FMS_B.Merge_ey < -FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = -FMS_PARAM.VEL_Z_LIM;
        } else {
          /* BusAssignment: '<S336>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = FMS_B.Merge_ey;
        }

        /* End of Saturate: '<S344>/Saturation1' */
        /* End of Outputs for SubSystem: '<S38>/Hold' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S38>/Unknown' incorporates:
         *  ActionPort: '<S340>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_g);

        /* End of Outputs for SubSystem: '<S38>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S38>/Switch Case' */
      /* End of Outputs for SubSystem: '<S31>/SubMode' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S31>/Auto' incorporates:
       *  ActionPort: '<S36>/Action Port'
       */
      /* SwitchCase: '<S36>/Switch Case' incorporates:
       *  Math: '<S220>/Square'
       *  Math: '<S222>/Math Function'
       *  Sum: '<S178>/Subtract'
       *  Sum: '<S235>/Sum1'
       */
      rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_i;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      switch (FMS_B.state) {
       case VehicleState_Offboard:
        FMS_DW.SwitchCase_ActiveSubsystem_i = 0;
        break;

       case VehicleState_Mission:
        FMS_DW.SwitchCase_ActiveSubsystem_i = 1;
        break;

       default:
        FMS_DW.SwitchCase_ActiveSubsystem_i = 2;
        break;
      }

      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
      if ((rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem_i) && (rtPrevAction
           == 1)) {
        /* Disable for Resettable SubSystem: '<S149>/Mission_SubSystem' */
        /* Disable for SwitchCase: '<S200>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

        /* Disable for SwitchCase: '<S190>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_e = -1;

        /* End of Disable for SubSystem: '<S149>/Mission_SubSystem' */
      }

      switch (FMS_DW.SwitchCase_ActiveSubsystem_i) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S36>/Offboard' incorporates:
         *  ActionPort: '<S150>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Gain: '<S314>/rad2deg' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_Multiply_l5_idx_0 = 57.295779513082323 * FMS_U.INS_Out.lat_0;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Gain: '<S317>/deg2rad' */
        rtb_Switch1_p = 0.017453292519943295 * rtb_Multiply_l5_idx_0;

        /* Trigonometry: '<S318>/Sin' */
        rtb_Gain = sin(rtb_Switch1_p);

        /* Math: '<S318>/Square1' */
        rtb_Gain *= rtb_Gain;

        /* Product: '<S318>/Multiply1' incorporates:
         *  Product: '<S318>/Multiply'
         */
        rtb_Sum3 = FMS_ConstB.ff * rtb_Gain;

        /* Product: '<S318>/Divide' incorporates:
         *  Constant: '<S318>/Constant'
         *  Constant: '<S318>/R'
         *  Sqrt: '<S318>/Sqrt'
         *  Sum: '<S318>/Sum1'
         */
        rtb_Gain = 6.378137E+6 / sqrt(1.0 - rtb_Sum3);

        /* Product: '<S318>/Product3' incorporates:
         *  Constant: '<S318>/Constant1'
         *  Product: '<S318>/Multiply1'
         *  Sum: '<S318>/Sum2'
         */
        rtb_Sum3 = 1.0 / (1.0 - rtb_Sum3) * FMS_ConstB.Sum4 * rtb_Gain;

        /* Product: '<S318>/Multiply2' incorporates:
         *  Trigonometry: '<S318>/Cos'
         */
        rtb_Gain *= cos(rtb_Switch1_p);

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S317>/Sum' incorporates:
         *  Gain: '<S314>/Gain'
         *  Inport: '<Root>/Auto_Cmd'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        rtb_Multiply_l5_idx_0 = 1.0000000000287557E-7 * (real_T)
          FMS_U.Auto_Cmd.lat_cmd - rtb_Multiply_l5_idx_0;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Abs: '<S322>/Abs' incorporates:
         *  Abs: '<S325>/Abs1'
         *  Switch: '<S322>/Switch1'
         */
        rtb_Switch1_p = fabs(rtb_Multiply_l5_idx_0);

        /* Switch: '<S322>/Switch1' incorporates:
         *  Abs: '<S322>/Abs'
         *  Bias: '<S322>/Bias2'
         *  Bias: '<S322>/Bias3'
         *  Constant: '<S319>/Constant'
         *  Constant: '<S319>/Constant1'
         *  Constant: '<S324>/Constant'
         *  Gain: '<S322>/Gain1'
         *  Product: '<S322>/Multiply'
         *  RelationalOperator: '<S324>/Compare'
         *  Switch: '<S319>/Switch'
         */
        if (rtb_Switch1_p > 90.0) {
          /* Switch: '<S325>/Switch1' incorporates:
           *  Bias: '<S325>/Bias2'
           *  Bias: '<S325>/Bias3'
           *  Constant: '<S325>/Constant'
           *  Constant: '<S326>/Constant'
           *  Math: '<S325>/Math Function'
           *  RelationalOperator: '<S326>/Compare'
           */
          if (rtb_Switch1_p > 180.0) {
            rtb_Multiply_l5_idx_0 = rt_modd(rtb_Multiply_l5_idx_0 + 180.0, 360.0)
              + -180.0;
          }

          /* End of Switch: '<S325>/Switch1' */

          /* Signum: '<S322>/Sign' */
          if (rtb_Multiply_l5_idx_0 < 0.0) {
            rtb_Multiply_l5_idx_0 = -1.0;
          } else {
            if (rtb_Multiply_l5_idx_0 > 0.0) {
              rtb_Multiply_l5_idx_0 = 1.0;
            }
          }

          /* End of Signum: '<S322>/Sign' */
          rtb_Multiply_l5_idx_0 *= -(rtb_Switch1_p + -90.0) + 90.0;
          rtb_Compare_bv_0 = 180;
        } else {
          rtb_Compare_bv_0 = 0;
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S319>/Sum' incorporates:
         *  Gain: '<S314>/Gain1'
         *  Gain: '<S314>/rad2deg'
         *  Inport: '<Root>/Auto_Cmd'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sum: '<S317>/Sum'
         */
        rtb_Switch1_p = (1.0000000000287557E-7 * (real_T)FMS_U.Auto_Cmd.lon_cmd
                         - 57.295779513082323 * FMS_U.INS_Out.lon_0) + (real_T)
          rtb_Compare_bv_0;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Product: '<S317>/Multiply' incorporates:
         *  Gain: '<S317>/deg2rad1'
         */
        rtb_Multiply_l5_idx_0 = 0.017453292519943295 * rtb_Multiply_l5_idx_0 *
          rtb_Sum3;

        /* Switch: '<S321>/Switch1' incorporates:
         *  Abs: '<S321>/Abs1'
         *  Bias: '<S321>/Bias2'
         *  Bias: '<S321>/Bias3'
         *  Constant: '<S321>/Constant'
         *  Constant: '<S323>/Constant'
         *  Math: '<S321>/Math Function'
         *  RelationalOperator: '<S323>/Compare'
         */
        if (fabs(rtb_Switch1_p) > 180.0) {
          rtb_Switch1_p = rt_modd(rtb_Switch1_p + 180.0, 360.0) + -180.0;
        }

        /* End of Switch: '<S321>/Switch1' */

        /* Product: '<S317>/Multiply' incorporates:
         *  Gain: '<S317>/deg2rad1'
         */
        rtb_Gain *= 0.017453292519943295 * rtb_Switch1_p;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MultiPortSwitch: '<S295>/Index Vector' incorporates:
         *  Inport: '<Root>/Auto_Cmd'
         *  Product: '<S299>/Multiply1'
         *  Product: '<S300>/Multiply3'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        switch (FMS_U.Auto_Cmd.frame) {
         case 0:
          /* SignalConversion: '<S308>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S308>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Trigonometry: '<S308>/Trigonometric Function3' incorporates:
           *  Gain: '<S307>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* Gain: '<S308>/Gain' incorporates:
           *  Gain: '<S307>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Trigonometry: '<S308>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S308>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S308>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S308>/Trigonometric Function' incorporates:
           *  Gain: '<S307>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S308>/Trigonometric Function1' incorporates:
           *  Gain: '<S307>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S308>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_i[0];

          /* Saturate: '<S299>/Saturation' incorporates:
           *  Constant: '<S305>/Constant'
           *  Constant: '<S306>/Constant'
           *  Constant: '<S316>/Constant'
           *  DataTypeConversion: '<S314>/Data Type Conversion1'
           *  Inport: '<Root>/INS_Out'
           *  Logic: '<S298>/Logical Operator'
           *  Product: '<S299>/Multiply'
           *  Product: '<S320>/Multiply1'
           *  Product: '<S320>/Multiply2'
           *  RelationalOperator: '<S305>/Compare'
           *  RelationalOperator: '<S306>/Compare'
           *  RelationalOperator: '<S316>/Compare'
           *  S-Function (sfix_bitop): '<S298>/lat_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/x_cmd valid'
           *  S-Function (sfix_bitop): '<S313>/lat_cmd valid'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S299>/Sum1'
           *  Sum: '<S320>/Sum2'
           *  Switch: '<S301>/Switch'
           */
          if ((FMS_U.Auto_Cmd.cmd_mask & 1024U) > 0U) {
            rtb_a_l = (real32_T)(rtb_Multiply_l5_idx_0 * FMS_ConstB.SinCos_o2 +
                                 rtb_Gain * FMS_ConstB.SinCos_o1);
          } else {
            rtb_a_l = FMS_U.Auto_Cmd.x_cmd;
          }

          rtb_Sqrt_b = ((FMS_U.Auto_Cmd.cmd_mask & 128U) > 0U) ||
            ((FMS_U.Auto_Cmd.cmd_mask & 1024U) > 0U) ? rtb_a_l -
            FMS_U.INS_Out.x_R : 0.0F;
          if (rtb_Sqrt_b > 4.0F) {
            rtb_Sqrt_b = 4.0F;
          } else {
            if (rtb_Sqrt_b < -4.0F) {
              rtb_Sqrt_b = -4.0F;
            }
          }

          /* SignalConversion: '<S308>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_i[1];

          /* Saturate: '<S299>/Saturation' incorporates:
           *  Constant: '<S305>/Constant'
           *  Constant: '<S306>/Constant'
           *  Constant: '<S316>/Constant'
           *  DataTypeConversion: '<S314>/Data Type Conversion1'
           *  Inport: '<Root>/INS_Out'
           *  Logic: '<S298>/Logical Operator'
           *  Product: '<S299>/Multiply'
           *  Product: '<S320>/Multiply3'
           *  Product: '<S320>/Multiply4'
           *  RelationalOperator: '<S305>/Compare'
           *  RelationalOperator: '<S306>/Compare'
           *  RelationalOperator: '<S316>/Compare'
           *  S-Function (sfix_bitop): '<S298>/lon_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/y_cmd valid'
           *  S-Function (sfix_bitop): '<S313>/lon_cmd valid'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S299>/Sum1'
           *  Sum: '<S320>/Sum3'
           *  Switch: '<S301>/Switch'
           */
          if ((FMS_U.Auto_Cmd.cmd_mask & 2048U) > 0U) {
            rtb_a_l = (real32_T)(rtb_Gain * FMS_ConstB.SinCos_o2 -
                                 rtb_Multiply_l5_idx_0 * FMS_ConstB.SinCos_o1);
          } else {
            rtb_a_l = FMS_U.Auto_Cmd.y_cmd;
          }

          rtb_Add3_c = ((FMS_U.Auto_Cmd.cmd_mask & 256U) > 0U) ||
            ((FMS_U.Auto_Cmd.cmd_mask & 2048U) > 0U) ? rtb_a_l -
            FMS_U.INS_Out.y_R : 0.0F;
          if (rtb_Add3_c > 4.0F) {
            rtb_Add3_c = 4.0F;
          } else {
            if (rtb_Add3_c < -4.0F) {
              rtb_Add3_c = -4.0F;
            }
          }

          /* SignalConversion: '<S308>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_i[2];

          /* Saturate: '<S299>/Saturation' incorporates:
           *  Constant: '<S305>/Constant'
           *  Constant: '<S306>/Constant'
           *  Constant: '<S316>/Constant'
           *  DataTypeConversion: '<S314>/Data Type Conversion'
           *  DataTypeConversion: '<S314>/Data Type Conversion1'
           *  Gain: '<S302>/Gain'
           *  Gain: '<S314>/Gain2'
           *  Gain: '<S317>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  Logic: '<S298>/Logical Operator'
           *  Product: '<S299>/Multiply'
           *  RelationalOperator: '<S305>/Compare'
           *  RelationalOperator: '<S306>/Compare'
           *  RelationalOperator: '<S316>/Compare'
           *  S-Function (sfix_bitop): '<S298>/alt_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/z_cmd valid'
           *  S-Function (sfix_bitop): '<S313>/alt_cmd valid'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S299>/Sum1'
           *  Sum: '<S317>/Sum1'
           *  Switch: '<S301>/Switch'
           */
          if ((FMS_U.Auto_Cmd.cmd_mask & 4096U) > 0U) {
            rtb_a_l = (real32_T)-(FMS_U.Auto_Cmd.alt_cmd + -FMS_U.INS_Out.alt_0);
          } else {
            rtb_a_l = FMS_U.Auto_Cmd.z_cmd;
          }

          rtb_Subtract3_o = ((FMS_U.Auto_Cmd.cmd_mask & 512U) > 0U) ||
            ((FMS_U.Auto_Cmd.cmd_mask & 4096U) > 0U) ? rtb_a_l -
            (-FMS_U.INS_Out.h_R) : 0.0F;
          if (rtb_Subtract3_o > 2.0F) {
            rtb_Subtract3_o = 2.0F;
          } else {
            if (rtb_Subtract3_o < -2.0F) {
              rtb_Subtract3_o = -2.0F;
            }
          }

          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_Switch_dw[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] * rtb_Subtract3_o +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_Add3_c +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Sqrt_b);
          }

          /* SignalConversion: '<S242>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S242>/Constant4'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  Product: '<S299>/Multiply1'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Trigonometry: '<S242>/Trigonometric Function3' incorporates:
           *  Gain: '<S241>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* Gain: '<S242>/Gain' incorporates:
           *  Gain: '<S241>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Trigonometry: '<S242>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S242>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S242>/Constant3'
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S242>/Trigonometric Function' incorporates:
           *  Gain: '<S241>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S242>/Trigonometric Function1' incorporates:
           *  Gain: '<S241>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S242>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_e1[0];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/ax_cmd valid'
           */
          rtb_Subtract3_o = (FMS_U.Auto_Cmd.cmd_mask & 65536U) > 0U ?
            FMS_U.Auto_Cmd.ax_cmd : 0.0F;

          /* SignalConversion: '<S242>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_e1[1];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/ay_cmd valid'
           */
          rtb_a_l = (FMS_U.Auto_Cmd.cmd_mask & 131072U) > 0U ?
            FMS_U.Auto_Cmd.ay_cmd : 0.0F;

          /* SignalConversion: '<S242>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_e1[2];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/az_cmd valid'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_U.Auto_Cmd.cmd_mask & 262144U) > 0U ?
            FMS_U.Auto_Cmd.az_cmd : 0.0F;

          /* MultiPortSwitch: '<S232>/Index Vector' incorporates:
           *  Product: '<S238>/Multiply'
           */
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_TmpSignalConversionAtMath_c[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
              rtb_MathFunction_iq_idx_0 +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o);
          }
          break;

         case 1:
          /* SignalConversion: '<S312>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S312>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Gain: '<S310>/Gain' incorporates:
           *  Gain: '<S243>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Sum: '<S300>/Subtract'
           */
          rtb_Add3_c = -(FMS_U.INS_Out.psi - FMS_B.Cmd_In.offboard_psi_0);

          /* Trigonometry: '<S312>/Trigonometric Function3' incorporates:
           *  Gain: '<S310>/Gain'
           *  Trigonometry: '<S312>/Trigonometric Function1'
           */
          rtb_MathFunction_f_idx_0 = arm_cos_f32(rtb_Add3_c);
          rtb_VectorConcatenate_i[4] = rtb_MathFunction_f_idx_0;

          /* Trigonometry: '<S312>/Trigonometric Function2' incorporates:
           *  Gain: '<S310>/Gain'
           *  Trigonometry: '<S312>/Trigonometric Function'
           */
          rtb_a_l = arm_sin_f32(rtb_Add3_c);

          /* Gain: '<S312>/Gain' incorporates:
           *  Trigonometry: '<S312>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -rtb_a_l;

          /* SignalConversion: '<S312>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S312>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S312>/Trigonometric Function' */
          rtb_VectorConcatenate_i[1] = rtb_a_l;

          /* Trigonometry: '<S312>/Trigonometric Function1' */
          rtb_VectorConcatenate_i[0] = rtb_MathFunction_f_idx_0;

          /* SignalConversion: '<S312>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_e[0];

          /* SignalConversion: '<S311>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_k[6] = FMS_ConstB.VectorConcatenate3_n[0];

          /* SignalConversion: '<S312>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_e[1];

          /* SignalConversion: '<S311>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_k[7] = FMS_ConstB.VectorConcatenate3_n[1];

          /* SignalConversion: '<S312>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_e[2];

          /* SignalConversion: '<S311>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_k[8] = FMS_ConstB.VectorConcatenate3_n[2];

          /* SignalConversion: '<S311>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S311>/Constant4'
           */
          rtb_VectorConcatenate_k[5] = 0.0F;

          /* Trigonometry: '<S311>/Trigonometric Function3' incorporates:
           *  Gain: '<S309>/Gain'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Trigonometry: '<S311>/Trigonometric Function1'
           */
          rtb_Subtract3_o = arm_cos_f32(-FMS_B.Cmd_In.offboard_psi_0);
          rtb_VectorConcatenate_k[4] = rtb_Subtract3_o;

          /* Trigonometry: '<S311>/Trigonometric Function2' incorporates:
           *  Gain: '<S309>/Gain'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Trigonometry: '<S311>/Trigonometric Function'
           */
          rtb_Add4_e5 = arm_sin_f32(-FMS_B.Cmd_In.offboard_psi_0);

          /* Gain: '<S311>/Gain' incorporates:
           *  Trigonometry: '<S311>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_k[3] = -rtb_Add4_e5;

          /* SignalConversion: '<S311>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S311>/Constant3'
           */
          rtb_VectorConcatenate_k[2] = 0.0F;

          /* Trigonometry: '<S311>/Trigonometric Function' */
          rtb_VectorConcatenate_k[1] = rtb_Add4_e5;

          /* Trigonometry: '<S311>/Trigonometric Function1' */
          rtb_VectorConcatenate_k[0] = rtb_Subtract3_o;

          /* RelationalOperator: '<S316>/Compare' incorporates:
           *  Constant: '<S316>/Constant'
           *  S-Function (sfix_bitop): '<S313>/alt_cmd valid'
           *  S-Function (sfix_bitop): '<S313>/lat_cmd valid'
           *  S-Function (sfix_bitop): '<S313>/lon_cmd valid'
           */
          tmp[0] = ((FMS_U.Auto_Cmd.cmd_mask & 1024U) > 0U);
          tmp[1] = ((FMS_U.Auto_Cmd.cmd_mask & 2048U) > 0U);
          tmp[2] = ((FMS_U.Auto_Cmd.cmd_mask & 4096U) > 0U);

          /* DataTypeConversion: '<S314>/Data Type Conversion1' incorporates:
           *  DataTypeConversion: '<S314>/Data Type Conversion'
           *  Gain: '<S314>/Gain2'
           *  Gain: '<S317>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S320>/Multiply1'
           *  Product: '<S320>/Multiply2'
           *  Product: '<S320>/Multiply3'
           *  Product: '<S320>/Multiply4'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S317>/Sum1'
           *  Sum: '<S320>/Sum2'
           *  Sum: '<S320>/Sum3'
           */
          rtb_VectorConcatenate_ar[0] = (real32_T)(rtb_Multiply_l5_idx_0 *
            FMS_ConstB.SinCos_o2 + rtb_Gain * FMS_ConstB.SinCos_o1);
          rtb_VectorConcatenate_ar[1] = (real32_T)(rtb_Gain *
            FMS_ConstB.SinCos_o2 - rtb_Multiply_l5_idx_0 * FMS_ConstB.SinCos_o1);
          rtb_VectorConcatenate_ar[2] = (real32_T)-(FMS_U.Auto_Cmd.alt_cmd +
            -FMS_U.INS_Out.alt_0);

          /* Switch: '<S301>/Switch' */
          tmp_0[0] = FMS_U.Auto_Cmd.x_cmd;
          tmp_0[1] = FMS_U.Auto_Cmd.y_cmd;
          tmp_0[2] = FMS_U.Auto_Cmd.z_cmd;

          /* RelationalOperator: '<S305>/Compare' incorporates:
           *  Constant: '<S305>/Constant'
           *  S-Function (sfix_bitop): '<S298>/x_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/y_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/z_cmd valid'
           */
          tmp_1[0] = ((FMS_U.Auto_Cmd.cmd_mask & 128U) > 0U);
          tmp_1[1] = ((FMS_U.Auto_Cmd.cmd_mask & 256U) > 0U);
          tmp_1[2] = ((FMS_U.Auto_Cmd.cmd_mask & 512U) > 0U);

          /* RelationalOperator: '<S306>/Compare' incorporates:
           *  Constant: '<S306>/Constant'
           *  S-Function (sfix_bitop): '<S298>/alt_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/lat_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/lon_cmd valid'
           */
          tmp_2[0] = ((FMS_U.Auto_Cmd.cmd_mask & 1024U) > 0U);
          tmp_2[1] = ((FMS_U.Auto_Cmd.cmd_mask & 2048U) > 0U);
          tmp_2[2] = ((FMS_U.Auto_Cmd.cmd_mask & 4096U) > 0U);
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            /* Sum: '<S300>/Sum2' incorporates:
             *  Product: '<S300>/Multiply2'
             *  Switch: '<S301>/Switch'
             */
            if (tmp[rtb_Compare_bv_0]) {
              rtb_a_l = rtb_VectorConcatenate_ar[rtb_Compare_bv_0];
            } else {
              rtb_a_l = tmp_0[rtb_Compare_bv_0];
            }

            /* Saturate: '<S300>/Saturation1' incorporates:
             *  Gain: '<S302>/Gain'
             *  Inport: '<Root>/INS_Out'
             *  Logic: '<S298>/Logical Operator'
             *  Product: '<S300>/Multiply'
             *  Product: '<S300>/Multiply2'
             *  SignalConversion: '<S30>/Signal Copy1'
             *  Sum: '<S300>/Sum2'
             */
            rtb_Sqrt_b = tmp_1[rtb_Compare_bv_0] || tmp_2[rtb_Compare_bv_0] ?
              rtb_a_l - ((rtb_VectorConcatenate_k[rtb_Compare_bv_0 + 3] *
                          FMS_U.INS_Out.y_R +
                          rtb_VectorConcatenate_k[rtb_Compare_bv_0] *
                          FMS_U.INS_Out.x_R) +
                         rtb_VectorConcatenate_k[rtb_Compare_bv_0 + 6] *
                         -FMS_U.INS_Out.h_R) : 0.0F;
            if (rtb_Sqrt_b > FMS_ConstP.pooled22[rtb_Compare_bv_0]) {
              rtb_TmpSignalConversionAtMath_c[rtb_Compare_bv_0] =
                FMS_ConstP.pooled22[rtb_Compare_bv_0];
            } else if (rtb_Sqrt_b < FMS_ConstP.pooled23[rtb_Compare_bv_0]) {
              rtb_TmpSignalConversionAtMath_c[rtb_Compare_bv_0] =
                FMS_ConstP.pooled23[rtb_Compare_bv_0];
            } else {
              rtb_TmpSignalConversionAtMath_c[rtb_Compare_bv_0] = rtb_Sqrt_b;
            }

            /* End of Saturate: '<S300>/Saturation1' */
          }

          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_Switch_dw[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
              rtb_TmpSignalConversionAtMath_c[2] +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] *
               rtb_TmpSignalConversionAtMath_c[1] +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] *
               rtb_TmpSignalConversionAtMath_c[0]);
          }

          /* SignalConversion: '<S244>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S244>/Constant4'
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  Product: '<S300>/Multiply3'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Trigonometry: '<S244>/Trigonometric Function3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(rtb_Add3_c);

          /* Gain: '<S244>/Gain' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           *  Trigonometry: '<S244>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(rtb_Add3_c);

          /* SignalConversion: '<S244>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S244>/Constant3'
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S244>/Trigonometric Function' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(rtb_Add3_c);

          /* Trigonometry: '<S244>/Trigonometric Function1' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(rtb_Add3_c);

          /* SignalConversion: '<S244>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_o[0];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/ax_cmd valid'
           */
          rtb_Subtract3_o = (FMS_U.Auto_Cmd.cmd_mask & 65536U) > 0U ?
            FMS_U.Auto_Cmd.ax_cmd : 0.0F;

          /* SignalConversion: '<S244>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_o[1];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/ay_cmd valid'
           */
          rtb_a_l = (FMS_U.Auto_Cmd.cmd_mask & 131072U) > 0U ?
            FMS_U.Auto_Cmd.ay_cmd : 0.0F;

          /* SignalConversion: '<S244>/ConcatBufferAtVector ConcatenateIn3' incorporates:
           *  MultiPortSwitch: '<S232>/Index Vector'
           */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_o[2];

          /* Product: '<S232>/Multiply' incorporates:
           *  Constant: '<S240>/Constant'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/az_cmd valid'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_U.Auto_Cmd.cmd_mask & 262144U) > 0U ?
            FMS_U.Auto_Cmd.az_cmd : 0.0F;

          /* MultiPortSwitch: '<S232>/Index Vector' incorporates:
           *  Product: '<S239>/Multiply3'
           */
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_TmpSignalConversionAtMath_c[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
              rtb_MathFunction_iq_idx_0 +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o);
          }
          break;

         default:
          /* SignalConversion: '<S304>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_nj[0];
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_nj[1];
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_nj[2];

          /* SignalConversion: '<S304>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S304>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Trigonometry: '<S304>/Trigonometric Function3' incorporates:
           *  Gain: '<S303>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* Gain: '<S304>/Gain' incorporates:
           *  Gain: '<S303>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Trigonometry: '<S304>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S304>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S304>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S304>/Trigonometric Function' incorporates:
           *  Gain: '<S303>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S304>/Trigonometric Function1' incorporates:
           *  Gain: '<S303>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* S-Function (sfix_bitop): '<S313>/lat_cmd valid' incorporates:
           *  S-Function (sfix_bitop): '<S298>/lat_cmd valid'
           */
          tmp_3 = FMS_U.Auto_Cmd.cmd_mask & 1024U;

          /* RelationalOperator: '<S316>/Compare' incorporates:
           *  Constant: '<S316>/Constant'
           *  S-Function (sfix_bitop): '<S313>/lat_cmd valid'
           */
          tmp[0] = (tmp_3 > 0U);

          /* S-Function (sfix_bitop): '<S313>/lon_cmd valid' incorporates:
           *  S-Function (sfix_bitop): '<S298>/lon_cmd valid'
           */
          tmp_4 = FMS_U.Auto_Cmd.cmd_mask & 2048U;

          /* RelationalOperator: '<S316>/Compare' incorporates:
           *  Constant: '<S316>/Constant'
           *  S-Function (sfix_bitop): '<S313>/lon_cmd valid'
           */
          tmp[1] = (tmp_4 > 0U);

          /* S-Function (sfix_bitop): '<S313>/alt_cmd valid' incorporates:
           *  S-Function (sfix_bitop): '<S298>/alt_cmd valid'
           */
          tmp_5 = FMS_U.Auto_Cmd.cmd_mask & 4096U;

          /* RelationalOperator: '<S316>/Compare' incorporates:
           *  Constant: '<S316>/Constant'
           *  S-Function (sfix_bitop): '<S313>/alt_cmd valid'
           */
          tmp[2] = (tmp_5 > 0U);

          /* DataTypeConversion: '<S314>/Data Type Conversion1' incorporates:
           *  DataTypeConversion: '<S314>/Data Type Conversion'
           *  Gain: '<S314>/Gain2'
           *  Gain: '<S317>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S320>/Multiply1'
           *  Product: '<S320>/Multiply2'
           *  Product: '<S320>/Multiply3'
           *  Product: '<S320>/Multiply4'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S317>/Sum1'
           *  Sum: '<S320>/Sum2'
           *  Sum: '<S320>/Sum3'
           */
          rtb_VectorConcatenate_ar[0] = (real32_T)(rtb_Multiply_l5_idx_0 *
            FMS_ConstB.SinCos_o2 + rtb_Gain * FMS_ConstB.SinCos_o1);
          rtb_VectorConcatenate_ar[1] = (real32_T)(rtb_Gain *
            FMS_ConstB.SinCos_o2 - rtb_Multiply_l5_idx_0 * FMS_ConstB.SinCos_o1);
          rtb_VectorConcatenate_ar[2] = (real32_T)-(FMS_U.Auto_Cmd.alt_cmd +
            -FMS_U.INS_Out.alt_0);

          /* Switch: '<S301>/Switch' */
          tmp_0[0] = FMS_U.Auto_Cmd.x_cmd;
          tmp_0[1] = FMS_U.Auto_Cmd.y_cmd;
          tmp_0[2] = FMS_U.Auto_Cmd.z_cmd;

          /* RelationalOperator: '<S305>/Compare' incorporates:
           *  Constant: '<S305>/Constant'
           *  S-Function (sfix_bitop): '<S298>/x_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/y_cmd valid'
           *  S-Function (sfix_bitop): '<S298>/z_cmd valid'
           */
          tmp_1[0] = ((FMS_U.Auto_Cmd.cmd_mask & 128U) > 0U);
          tmp_1[1] = ((FMS_U.Auto_Cmd.cmd_mask & 256U) > 0U);
          tmp_1[2] = ((FMS_U.Auto_Cmd.cmd_mask & 512U) > 0U);

          /* RelationalOperator: '<S306>/Compare' incorporates:
           *  Constant: '<S306>/Constant'
           */
          tmp_2[0] = (tmp_3 > 0U);
          tmp_2[1] = (tmp_4 > 0U);
          tmp_2[2] = (tmp_5 > 0U);

          /* Sum: '<S297>/Sum2' */
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            /* Switch: '<S301>/Switch' incorporates:
             *  Product: '<S297>/Multiply2'
             */
            if (tmp[rtb_Compare_bv_0]) {
              rtb_a_l = rtb_VectorConcatenate_ar[rtb_Compare_bv_0];
            } else {
              rtb_a_l = tmp_0[rtb_Compare_bv_0];
            }

            /* Saturate: '<S297>/Saturation1' incorporates:
             *  Gain: '<S302>/Gain'
             *  Inport: '<Root>/INS_Out'
             *  Logic: '<S298>/Logical Operator'
             *  Product: '<S297>/Multiply'
             *  Product: '<S297>/Multiply2'
             *  SignalConversion: '<S30>/Signal Copy1'
             */
            rtb_Sqrt_b = tmp_1[rtb_Compare_bv_0] || tmp_2[rtb_Compare_bv_0] ?
              rtb_a_l - ((rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] *
                          FMS_U.INS_Out.y_R +
                          rtb_VectorConcatenate_i[rtb_Compare_bv_0] *
                          FMS_U.INS_Out.x_R) +
                         rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
                         -FMS_U.INS_Out.h_R) : 0.0F;
            if (rtb_Sqrt_b > FMS_ConstP.pooled22[rtb_Compare_bv_0]) {
              rtb_Switch_dw[rtb_Compare_bv_0] =
                FMS_ConstP.pooled22[rtb_Compare_bv_0];
            } else if (rtb_Sqrt_b < FMS_ConstP.pooled23[rtb_Compare_bv_0]) {
              rtb_Switch_dw[rtb_Compare_bv_0] =
                FMS_ConstP.pooled23[rtb_Compare_bv_0];
            } else {
              rtb_Switch_dw[rtb_Compare_bv_0] = rtb_Sqrt_b;
            }

            /* End of Saturate: '<S297>/Saturation1' */
          }

          /* End of Sum: '<S297>/Sum2' */

          /* MultiPortSwitch: '<S232>/Index Vector' incorporates:
           *  Constant: '<S240>/Constant'
           *  Product: '<S232>/Multiply'
           *  RelationalOperator: '<S240>/Compare'
           *  S-Function (sfix_bitop): '<S237>/ax_cmd valid'
           *  S-Function (sfix_bitop): '<S237>/ay_cmd valid'
           *  S-Function (sfix_bitop): '<S237>/az_cmd valid'
           */
          rtb_TmpSignalConversionAtMath_c[0] = (FMS_U.Auto_Cmd.cmd_mask & 65536U)
            > 0U ? FMS_U.Auto_Cmd.ax_cmd : 0.0F;
          rtb_TmpSignalConversionAtMath_c[1] = (FMS_U.Auto_Cmd.cmd_mask &
            131072U) > 0U ? FMS_U.Auto_Cmd.ay_cmd : 0.0F;
          rtb_TmpSignalConversionAtMath_c[2] = (FMS_U.Auto_Cmd.cmd_mask &
            262144U) > 0U ? FMS_U.Auto_Cmd.az_cmd : 0.0F;
          break;
        }

        /* End of MultiPortSwitch: '<S295>/Index Vector' */

        /* Sum: '<S291>/Sum1' incorporates:
         *  Constant: '<S291>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S291>/Math Function'
         *  SignalConversion: '<S30>/Signal Copy'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_MathFunction_f_idx_0 = rt_remf(FMS_U.Auto_Cmd.psi_cmd, 6.28318548F)
          - FMS_U.INS_Out.psi;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Abs: '<S292>/Abs' */
        rtb_Add3_c = fabsf(rtb_MathFunction_f_idx_0);

        /* Switch: '<S292>/Switch' incorporates:
         *  Constant: '<S292>/Constant'
         *  Constant: '<S293>/Constant'
         *  Product: '<S292>/Multiply'
         *  RelationalOperator: '<S293>/Compare'
         *  Sum: '<S292>/Subtract'
         */
        if (rtb_Add3_c > 3.14159274F) {
          /* Signum: '<S292>/Sign' */
          if (rtb_MathFunction_f_idx_0 < 0.0F) {
            rtb_MathFunction_f_idx_0 = -1.0F;
          } else {
            if (rtb_MathFunction_f_idx_0 > 0.0F) {
              rtb_MathFunction_f_idx_0 = 1.0F;
            }
          }

          /* End of Signum: '<S292>/Sign' */
          rtb_MathFunction_f_idx_0 *= rtb_Add3_c - 6.28318548F;
        }

        /* End of Switch: '<S292>/Switch' */

        /* Saturate: '<S291>/Saturation' */
        if (rtb_MathFunction_f_idx_0 > 0.314159274F) {
          rtb_MathFunction_f_idx_0 = 0.314159274F;
        } else {
          if (rtb_MathFunction_f_idx_0 < -0.314159274F) {
            rtb_MathFunction_f_idx_0 = -0.314159274F;
          }
        }

        /* End of Saturate: '<S291>/Saturation' */

        /* Gain: '<S288>/Gain2' */
        rtb_MathFunction_f_idx_0 *= FMS_PARAM.YAW_P;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S234>/Sum' incorporates:
         *  Constant: '<S290>/Constant'
         *  Constant: '<S294>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  Product: '<S288>/Multiply2'
         *  Product: '<S289>/Multiply1'
         *  RelationalOperator: '<S290>/Compare'
         *  RelationalOperator: '<S294>/Compare'
         *  S-Function (sfix_bitop): '<S288>/psi_cmd valid'
         *  S-Function (sfix_bitop): '<S289>/psi_rate_cmd valid'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        rtb_Sqrt_b = ((FMS_U.Auto_Cmd.cmd_mask & 32U) > 0U ?
                      rtb_MathFunction_f_idx_0 : 0.0F) +
          ((FMS_U.Auto_Cmd.cmd_mask & 64U) > 0U ? FMS_U.Auto_Cmd.psi_rate_cmd :
           0.0F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Gain: '<S295>/Gain1' */
        rtb_Add3_c = FMS_PARAM.XY_P * rtb_Switch_dw[0];
        rtb_Add4_e5 = FMS_PARAM.XY_P * rtb_Switch_dw[1];

        /* Gain: '<S295>/Gain2' */
        rtb_MathFunction_f_idx_0 = FMS_PARAM.Z_P * rtb_Switch_dw[2];

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MultiPortSwitch: '<S296>/Index Vector' incorporates:
         *  Constant: '<S331>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  Product: '<S296>/Multiply'
         *  Product: '<S329>/Multiply'
         *  Product: '<S330>/Multiply3'
         *  RelationalOperator: '<S331>/Compare'
         *  S-Function (sfix_bitop): '<S328>/u_cmd valid'
         *  S-Function (sfix_bitop): '<S328>/v_cmd valid'
         *  S-Function (sfix_bitop): '<S328>/w_cmd valid'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        switch (FMS_U.Auto_Cmd.frame) {
         case 0:
          /* SignalConversion: '<S333>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S333>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Trigonometry: '<S333>/Trigonometric Function3' incorporates:
           *  Gain: '<S332>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* Gain: '<S333>/Gain' incorporates:
           *  Gain: '<S332>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Trigonometry: '<S333>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S333>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S333>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S333>/Trigonometric Function' incorporates:
           *  Gain: '<S332>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S333>/Trigonometric Function1' incorporates:
           *  Gain: '<S332>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* SignalConversion: '<S333>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_l[0];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/u_cmd valid'
           */
          rtb_Subtract3_o = (FMS_U.Auto_Cmd.cmd_mask & 8192U) > 0U ?
            FMS_U.Auto_Cmd.u_cmd : 0.0F;

          /* SignalConversion: '<S333>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_l[1];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/v_cmd valid'
           */
          rtb_a_l = (FMS_U.Auto_Cmd.cmd_mask & 16384U) > 0U ?
            FMS_U.Auto_Cmd.v_cmd : 0.0F;

          /* SignalConversion: '<S333>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_l[2];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/w_cmd valid'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_U.Auto_Cmd.cmd_mask & 32768U) > 0U ?
            FMS_U.Auto_Cmd.w_cmd : 0.0F;
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_Switch_dw[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
              rtb_MathFunction_iq_idx_0 +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o);
          }
          break;

         case 1:
          /* SignalConversion: '<S335>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S335>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Gain: '<S334>/Gain' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Sum: '<S330>/Subtract'
           */
          rtb_Subtract3_o = -(FMS_U.INS_Out.psi - FMS_B.Cmd_In.offboard_psi_0);

          /* Trigonometry: '<S335>/Trigonometric Function3' incorporates:
           *  Gain: '<S334>/Gain'
           */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(rtb_Subtract3_o);

          /* Gain: '<S335>/Gain' incorporates:
           *  Gain: '<S334>/Gain'
           *  Trigonometry: '<S335>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(rtb_Subtract3_o);

          /* SignalConversion: '<S335>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S335>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S335>/Trigonometric Function' incorporates:
           *  Gain: '<S334>/Gain'
           */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(rtb_Subtract3_o);

          /* Trigonometry: '<S335>/Trigonometric Function1' incorporates:
           *  Gain: '<S334>/Gain'
           */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(rtb_Subtract3_o);

          /* SignalConversion: '<S335>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3_iz[0];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/u_cmd valid'
           */
          rtb_Subtract3_o = (FMS_U.Auto_Cmd.cmd_mask & 8192U) > 0U ?
            FMS_U.Auto_Cmd.u_cmd : 0.0F;

          /* SignalConversion: '<S335>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3_iz[1];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/v_cmd valid'
           */
          rtb_a_l = (FMS_U.Auto_Cmd.cmd_mask & 16384U) > 0U ?
            FMS_U.Auto_Cmd.v_cmd : 0.0F;

          /* SignalConversion: '<S335>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3_iz[2];

          /* Product: '<S296>/Multiply' incorporates:
           *  Constant: '<S331>/Constant'
           *  RelationalOperator: '<S331>/Compare'
           *  S-Function (sfix_bitop): '<S328>/w_cmd valid'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_U.Auto_Cmd.cmd_mask & 32768U) > 0U ?
            FMS_U.Auto_Cmd.w_cmd : 0.0F;
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_Switch_dw[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 6] *
              rtb_MathFunction_iq_idx_0 +
              (rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_a_l +
               rtb_VectorConcatenate_i[rtb_Compare_bv_0] * rtb_Subtract3_o);
          }
          break;

         default:
          rtb_Switch_dw[0] = (FMS_U.Auto_Cmd.cmd_mask & 8192U) > 0U ?
            FMS_U.Auto_Cmd.u_cmd : 0.0F;
          rtb_Switch_dw[1] = (FMS_U.Auto_Cmd.cmd_mask & 16384U) > 0U ?
            FMS_U.Auto_Cmd.v_cmd : 0.0F;
          rtb_Switch_dw[2] = (FMS_U.Auto_Cmd.cmd_mask & 32768U) > 0U ?
            FMS_U.Auto_Cmd.w_cmd : 0.0F;
          break;
        }

        /* End of MultiPortSwitch: '<S296>/Index Vector' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_Switch_dw[0] += rtb_Add3_c;
        rtb_Switch_dw[1] += rtb_Add4_e5;

        /* Sum: '<S235>/Sum1' */
        rtb_Add3_c = rtb_MathFunction_f_idx_0 + rtb_Switch_dw[2];

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S252>/Switch' incorporates:
         *  Constant: '<S267>/Constant'
         *  Constant: '<S268>/Constant'
         *  Constant: '<S269>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  RelationalOperator: '<S267>/Compare'
         *  RelationalOperator: '<S268>/Compare'
         *  RelationalOperator: '<S269>/Compare'
         *  S-Function (sfix_bitop): '<S252>/x_u_cmd'
         *  S-Function (sfix_bitop): '<S252>/y_v_cmd'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        if (FMS_U.Auto_Cmd.frame < 2) {
          /* Logic: '<S252>/Logical Operator' incorporates:
           *  Constant: '<S268>/Constant'
           *  Constant: '<S269>/Constant'
           *  RelationalOperator: '<S268>/Compare'
           *  RelationalOperator: '<S269>/Compare'
           *  S-Function (sfix_bitop): '<S252>/x_u_cmd'
           *  S-Function (sfix_bitop): '<S252>/y_v_cmd'
           */
          rtb_LogicalOperator_es = (((FMS_U.Auto_Cmd.cmd_mask & 8320U) > 0U) ||
            ((FMS_U.Auto_Cmd.cmd_mask & 16640U) > 0U));
          rtb_FixPtRelationalOperator_me = rtb_LogicalOperator_es;
        } else {
          rtb_LogicalOperator_es = ((FMS_U.Auto_Cmd.cmd_mask & 8320U) > 0U);
          rtb_FixPtRelationalOperator_me = ((FMS_U.Auto_Cmd.cmd_mask & 16640U) >
            0U);
        }

        /* End of Switch: '<S252>/Switch' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Outputs for Atomic SubSystem: '<S233>/u_cmd_valid' */
        /* MATLAB Function: '<S264>/bit_shift' incorporates:
         *  DataTypeConversion: '<S233>/Data Type Conversion6'
         */
        rtb_y_md = (uint16_T)(rtb_LogicalOperator_es << 6);

        /* End of Outputs for SubSystem: '<S233>/u_cmd_valid' */

        /* Outputs for Atomic SubSystem: '<S233>/v_cmd_valid' */
        /* MATLAB Function: '<S265>/bit_shift' incorporates:
         *  DataTypeConversion: '<S233>/Data Type Conversion7'
         */
        rtb_y_c1 = (uint16_T)(rtb_FixPtRelationalOperator_me << 7);

        /* End of Outputs for SubSystem: '<S233>/v_cmd_valid' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S253>/Switch' incorporates:
         *  Constant: '<S271>/Constant'
         *  Constant: '<S272>/Constant'
         *  Constant: '<S274>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  RelationalOperator: '<S271>/Compare'
         *  RelationalOperator: '<S272>/Compare'
         *  RelationalOperator: '<S274>/Compare'
         *  S-Function (sfix_bitop): '<S253>/ax_cmd'
         *  S-Function (sfix_bitop): '<S253>/ay_cmd'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        if (FMS_U.Auto_Cmd.frame < 2) {
          /* Logic: '<S253>/Logical Operator' incorporates:
           *  Constant: '<S272>/Constant'
           *  Constant: '<S274>/Constant'
           *  RelationalOperator: '<S272>/Compare'
           *  RelationalOperator: '<S274>/Compare'
           *  S-Function (sfix_bitop): '<S253>/ax_cmd'
           *  S-Function (sfix_bitop): '<S253>/ay_cmd'
           */
          rtb_LogicalOperator_es = (((FMS_U.Auto_Cmd.cmd_mask & 65536U) > 0U) ||
            ((FMS_U.Auto_Cmd.cmd_mask & 131072U) > 0U));
          rtb_FixPtRelationalOperator_me = rtb_LogicalOperator_es;
        } else {
          rtb_LogicalOperator_es = ((FMS_U.Auto_Cmd.cmd_mask & 65536U) > 0U);
          rtb_FixPtRelationalOperator_me = ((FMS_U.Auto_Cmd.cmd_mask & 131072U) >
            0U);
        }

        /* End of Switch: '<S253>/Switch' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S150>/Bus Assignment'
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S150>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S150>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Inport: '<Root>/Auto_Cmd'
         *  Outport: '<Root>/FMS_Out'
         *  SignalConversion: '<S30>/Signal Copy'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_a;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_n;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_k;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        FMS_Y.FMS_Out.p_cmd = FMS_U.Auto_Cmd.p_cmd;
        FMS_Y.FMS_Out.q_cmd = FMS_U.Auto_Cmd.q_cmd;
        FMS_Y.FMS_Out.r_cmd = FMS_U.Auto_Cmd.r_cmd;
        FMS_Y.FMS_Out.phi_cmd = FMS_U.Auto_Cmd.phi_cmd;
        FMS_Y.FMS_Out.theta_cmd = FMS_U.Auto_Cmd.theta_cmd;
        FMS_Y.FMS_Out.throttle_cmd = FMS_U.Auto_Cmd.throttle_cmd;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        FMS_Y.FMS_Out.ax_cmd = rtb_TmpSignalConversionAtMath_c[0];
        FMS_Y.FMS_Out.ay_cmd = rtb_TmpSignalConversionAtMath_c[1];
        FMS_Y.FMS_Out.az_cmd = rtb_TmpSignalConversionAtMath_c[2];

        /* Saturate: '<S234>/Saturation' */
        if (rtb_Sqrt_b > FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_PARAM.YAW_RATE_LIM;
        } else if (rtb_Sqrt_b < -FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -FMS_PARAM.YAW_RATE_LIM;
        } else {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = rtb_Sqrt_b;
        }

        /* End of Saturate: '<S234>/Saturation' */

        /* Saturate: '<S235>/Saturation2' */
        if (rtb_Switch_dw[0] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (rtb_Switch_dw[0] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = rtb_Switch_dw[0];
        }

        /* End of Saturate: '<S235>/Saturation2' */

        /* Saturate: '<S235>/Saturation1' */
        if (rtb_Switch_dw[1] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (rtb_Switch_dw[1] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = rtb_Switch_dw[1];
        }

        /* End of Saturate: '<S235>/Saturation1' */

        /* Saturate: '<S235>/Saturation3' */
        if (rtb_Add3_c > FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = FMS_PARAM.VEL_Z_LIM;
        } else if (rtb_Add3_c < -FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = -FMS_PARAM.VEL_Z_LIM;
        } else {
          /* BusAssignment: '<S150>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = rtb_Add3_c;
        }

        /* End of Saturate: '<S235>/Saturation3' */

        /* Outputs for Atomic SubSystem: '<S233>/q_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Outputs for Atomic SubSystem: '<S233>/r_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/phi_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/theta_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/psi_rate_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/w_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/ax_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/ay_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/az_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S233>/throttle_cmd_valid' */
        /* BusAssignment: '<S150>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S245>/Constant'
         *  Constant: '<S246>/Constant'
         *  Constant: '<S247>/Constant'
         *  Constant: '<S248>/Constant'
         *  Constant: '<S249>/Constant'
         *  Constant: '<S250>/Constant'
         *  Constant: '<S251>/Constant'
         *  Constant: '<S270>/Constant'
         *  Constant: '<S273>/Constant'
         *  DataTypeConversion: '<S233>/Data Type Conversion10'
         *  DataTypeConversion: '<S233>/Data Type Conversion9'
         *  Inport: '<Root>/Auto_Cmd'
         *  MATLAB Function: '<S254>/bit_shift'
         *  MATLAB Function: '<S255>/bit_shift'
         *  MATLAB Function: '<S256>/bit_shift'
         *  MATLAB Function: '<S258>/bit_shift'
         *  MATLAB Function: '<S259>/bit_shift'
         *  MATLAB Function: '<S260>/bit_shift'
         *  MATLAB Function: '<S261>/bit_shift'
         *  MATLAB Function: '<S262>/bit_shift'
         *  MATLAB Function: '<S263>/bit_shift'
         *  MATLAB Function: '<S266>/bit_shift'
         *  Outport: '<Root>/FMS_Out'
         *  RelationalOperator: '<S245>/Compare'
         *  RelationalOperator: '<S246>/Compare'
         *  RelationalOperator: '<S247>/Compare'
         *  RelationalOperator: '<S248>/Compare'
         *  RelationalOperator: '<S249>/Compare'
         *  RelationalOperator: '<S250>/Compare'
         *  RelationalOperator: '<S251>/Compare'
         *  RelationalOperator: '<S270>/Compare'
         *  RelationalOperator: '<S273>/Compare'
         *  S-Function (sfix_bitop): '<S233>/p_cmd'
         *  S-Function (sfix_bitop): '<S233>/phi_cmd'
         *  S-Function (sfix_bitop): '<S233>/psi_psi_rate_cmd'
         *  S-Function (sfix_bitop): '<S233>/q_cmd'
         *  S-Function (sfix_bitop): '<S233>/r_cmd'
         *  S-Function (sfix_bitop): '<S233>/theta_cmd'
         *  S-Function (sfix_bitop): '<S233>/throttle_cmd'
         *  S-Function (sfix_bitop): '<S252>/z_w_cmd'
         *  S-Function (sfix_bitop): '<S253>/az_cmd'
         *  SignalConversion: '<S30>/Signal Copy'
         *  Sum: '<S233>/Add'
         */
        FMS_Y.FMS_Out.cmd_mask = (uint16_T)((((((((((((uint32_T)(uint16_T)
          ((uint32_T)(((FMS_U.Auto_Cmd.cmd_mask & 2U) > 0U) << 1) +
           ((FMS_U.Auto_Cmd.cmd_mask & 1U) > 0U)) + (uint16_T)
          (((FMS_U.Auto_Cmd.cmd_mask & 4U) > 0U) << 2)) +
          (((FMS_U.Auto_Cmd.cmd_mask & 8U) > 0U) << 3)) +
          (((FMS_U.Auto_Cmd.cmd_mask & 16U) > 0U) << 4)) +
          (((FMS_U.Auto_Cmd.cmd_mask & 96U) > 0U) << 5)) + rtb_y_md) + rtb_y_c1)
          + (((FMS_U.Auto_Cmd.cmd_mask & 33280U) > 0U) << 8)) +
          (rtb_LogicalOperator_es << 9)) + (rtb_FixPtRelationalOperator_me << 10))
          + (((FMS_U.Auto_Cmd.cmd_mask & 262144U) > 0U) << 11)) +
          (((FMS_U.Auto_Cmd.cmd_mask & 524288U) > 0U) << 12));

        /* End of Outputs for SubSystem: '<S233>/throttle_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/az_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/ay_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/ax_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/w_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/psi_rate_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/theta_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/phi_cmd_valid' */
        /* End of Outputs for SubSystem: '<S233>/r_cmd_valid' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        /* End of Outputs for SubSystem: '<S233>/q_cmd_valid' */
        /* End of Outputs for SubSystem: '<S36>/Offboard' */
        break;

       case 1:
        if (FMS_DW.SwitchCase_ActiveSubsystem_i != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S36>/Mission' incorporates:
           *  ActionPort: '<S149>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S36>/Switch Case' incorporates:
           *  UnitDelay: '<S152>/Delay Input1'
           *
           * Block description for '<S152>/Delay Input1':
           *
           *  Store in Global RAM
           */
          FMS_DW.DelayInput1_DSTATE_pe = 0U;

          /* End of InitializeConditions for SubSystem: '<S36>/Mission' */

          /* SystemReset for IfAction SubSystem: '<S36>/Mission' incorporates:
           *  ActionPort: '<S149>/Action Port'
           */
          /* SystemReset for Resettable SubSystem: '<S149>/Mission_SubSystem' */
          /* SystemReset for SwitchCase: '<S36>/Switch Case' incorporates:
           *  Chart: '<S191>/Motion Status'
           *  Chart: '<S201>/Motion State'
           *  Delay: '<S157>/Delay'
           *  Delay: '<S179>/Delay'
           *  DiscreteIntegrator: '<S160>/Integrator'
           *  DiscreteIntegrator: '<S160>/Integrator1'
           *  DiscreteIntegrator: '<S175>/Acceleration_Speed'
           *  DiscreteIntegrator: '<S180>/Discrete-Time Integrator'
           *  DiscreteIntegrator: '<S227>/Discrete-Time Integrator'
           */
          FMS_DW.icLoad = 1U;
          FMS_DW.DiscreteTimeIntegrator_DSTATE_k = 0U;
          FMS_DW.Acceleration_Speed_DSTATE = 0.0F;
          FMS_DW.Acceleration_Speed_PrevResetSta = 0;
          FMS_DW.l1_heading = 0.0F;
          FMS_DW.icLoad_k = 1U;
          FMS_DW.Integrator1_IC_LOADING = 1U;
          FMS_DW.Integrator_DSTATE_i = 0.0F;
          FMS_MotionState_Reset(&FMS_DW.sf_MotionState_n);
          FMS_MotionStatus_Reset(&FMS_DW.sf_MotionStatus_jt);

          /* End of SystemReset for SubSystem: '<S149>/Mission_SubSystem' */
          /* End of SystemReset for SubSystem: '<S36>/Mission' */
        }

        /* Outputs for IfAction SubSystem: '<S36>/Mission' incorporates:
         *  ActionPort: '<S149>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* RelationalOperator: '<S152>/FixPt Relational Operator' incorporates:
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy5Inport1'
         *  UnitDelay: '<S152>/Delay Input1'
         *
         * Block description for '<S152>/Delay Input1':
         *
         *  Store in Global RAM
         */
        rtb_FixPtRelationalOperator_me = (FMS_B.wp_index !=
          FMS_DW.DelayInput1_DSTATE_pe);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Outputs for Resettable SubSystem: '<S149>/Mission_SubSystem' incorporates:
         *  ResetPort: '<S153>/Reset'
         */
        if (rtb_FixPtRelationalOperator_me &&
            (FMS_PrevZCX.Mission_SubSystem_Reset_ZCE != POS_ZCSIG)) {
          /* Disable for SwitchCase: '<S200>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

          /* Disable for SwitchCase: '<S190>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_e = -1;

          /* InitializeConditions for Delay: '<S179>/Delay' */
          FMS_DW.icLoad = 1U;

          /* InitializeConditions for DiscreteIntegrator: '<S180>/Discrete-Time Integrator' */
          FMS_DW.DiscreteTimeIntegrator_DSTATE_k = 0U;

          /* InitializeConditions for DiscreteIntegrator: '<S175>/Acceleration_Speed' */
          FMS_DW.Acceleration_Speed_DSTATE = 0.0F;
          FMS_DW.Acceleration_Speed_PrevResetSta = 0;

          /* InitializeConditions for DiscreteIntegrator: '<S227>/Discrete-Time Integrator' */
          FMS_DW.l1_heading = 0.0F;

          /* InitializeConditions for Delay: '<S157>/Delay' */
          FMS_DW.icLoad_k = 1U;

          /* InitializeConditions for DiscreteIntegrator: '<S160>/Integrator1' */
          FMS_DW.Integrator1_IC_LOADING = 1U;

          /* InitializeConditions for DiscreteIntegrator: '<S160>/Integrator' */
          FMS_DW.Integrator_DSTATE_i = 0.0F;

          /* SystemReset for Chart: '<S201>/Motion State' */
          FMS_MotionState_Reset(&FMS_DW.sf_MotionState_n);

          /* SystemReset for Chart: '<S191>/Motion Status' */
          FMS_MotionStatus_Reset(&FMS_DW.sf_MotionStatus_jt);
        }

        FMS_PrevZCX.Mission_SubSystem_Reset_ZCE = rtb_FixPtRelationalOperator_me;

        /* Delay: '<S179>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE[0] = FMS_U.INS_Out.x_R;
          FMS_DW.Delay_DSTATE[1] = FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S175>/Switch2' incorporates:
         *  Constant: '<S175>/vel'
         *  Constant: '<S184>/Constant'
         *  RelationalOperator: '<S184>/Compare'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        if (FMS_B.Cmd_In.set_speed > 0.0F) {
          rtb_Add4_e5 = FMS_B.Cmd_In.set_speed;
        } else {
          rtb_Add4_e5 = FMS_PARAM.CRUISE_SPEED;
        }

        /* End of Switch: '<S175>/Switch2' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* DiscreteIntegrator: '<S180>/Discrete-Time Integrator' incorporates:
         *  UnitDelay: '<S152>/Delay Input1'
         *
         * Block description for '<S152>/Delay Input1':
         *
         *  Store in Global RAM
         */
        FMS_DW.DelayInput1_DSTATE_pe = FMS_DW.DiscreteTimeIntegrator_DSTATE_k;

        /* RelationalOperator: '<S174>/Compare' incorporates:
         *  Constant: '<S231>/Constant'
         *  RelationalOperator: '<S231>/Compare'
         *  UnitDelay: '<S152>/Delay Input1'
         *
         * Block description for '<S152>/Delay Input1':
         *
         *  Store in Global RAM
         */
        rtb_Compare_on = (FMS_DW.DelayInput1_DSTATE_pe <= 3);

        /* DiscreteIntegrator: '<S175>/Acceleration_Speed' */
        if (rtb_Compare_on || (FMS_DW.Acceleration_Speed_PrevResetSta != 0)) {
          FMS_DW.Acceleration_Speed_DSTATE = 0.0F;
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S201>/Motion State' incorporates:
         *  Constant: '<S201>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S201>/Square'
         *  Math: '<S201>/Square1'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sqrt: '<S201>/Sqrt'
         *  Sum: '<S201>/Add'
         */
        FMS_MotionState(0.0F, sqrtf(FMS_U.INS_Out.vn * FMS_U.INS_Out.vn +
          FMS_U.INS_Out.ve * FMS_U.INS_Out.ve), &rtb_state_c,
                        &FMS_DW.sf_MotionState_n);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S200>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_a;
        FMS_DW.SwitchCase_ActiveSubsystem_a = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_a = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_a = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_a = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_a) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_a != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S200>/Hold Control' incorporates:
             *  ActionPort: '<S203>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S200>/Switch Case' */
            FMS_HoldControl_k_Reset(&FMS_DW.HoldControl_d);

            /* End of SystemReset for SubSystem: '<S200>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S200>/Hold Control' incorporates:
           *  ActionPort: '<S203>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_m(FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                            FMS_U.INS_Out.psi, FMS_B.Merge_n,
                            &FMS_ConstB.HoldControl_d, &FMS_DW.HoldControl_d);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S200>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S200>/Brake Control' incorporates:
           *  ActionPort: '<S202>/Action Port'
           */
          FMS_BrakeControl_h(FMS_B.Merge_n);

          /* End of Outputs for SubSystem: '<S200>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_a != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S200>/Move Control' incorporates:
             *  ActionPort: '<S204>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S200>/Switch Case' */
            FMS_MoveControl_i_Reset(&FMS_DW.MoveControl_c);

            /* End of SystemReset for SubSystem: '<S200>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S200>/Move Control' incorporates:
           *  ActionPort: '<S204>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_j(FMS_U.Pilot_Cmd.stick_pitch,
                            FMS_U.Pilot_Cmd.stick_roll, FMS_B.Merge_n,
                            &FMS_ConstB.MoveControl_c, &FMS_DW.MoveControl_c);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S200>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S200>/Switch Case' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S191>/Motion Status' incorporates:
         *  Abs: '<S191>/Abs'
         *  Constant: '<S191>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        FMS_MotionStatus(0.0F, fabsf(FMS_U.INS_Out.vd), &rtb_state_c,
                         &FMS_DW.sf_MotionStatus_jt);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S190>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_e;
        FMS_DW.SwitchCase_ActiveSubsystem_e = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_e = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_e = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_e = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_e) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_e != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S190>/Hold Control' incorporates:
             *  ActionPort: '<S193>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S190>/Switch Case' */
            FMS_HoldControl_Reset(&FMS_DW.HoldControl_a);

            /* End of SystemReset for SubSystem: '<S190>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S190>/Hold Control' incorporates:
           *  ActionPort: '<S193>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl(FMS_U.INS_Out.h_R, &FMS_B.Merge_e,
                          &FMS_DW.HoldControl_a);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S190>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S190>/Brake Control' incorporates:
           *  ActionPort: '<S192>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_e);

          /* End of Outputs for SubSystem: '<S190>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_e != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S190>/Move Control' incorporates:
             *  ActionPort: '<S194>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S190>/Switch Case' */
            FMS_MoveControl_Reset(&FMS_DW.MoveControl_m);

            /* End of SystemReset for SubSystem: '<S190>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S190>/Move Control' incorporates:
           *  ActionPort: '<S194>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl(FMS_U.Pilot_Cmd.stick_throttle, &FMS_B.Merge_e,
                          &FMS_ConstB.MoveControl_m, &FMS_DW.MoveControl_m);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S190>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S190>/Switch Case' */

        /* Switch: '<S155>/Switch' incorporates:
         *  Product: '<S179>/Multiply'
         *  Sum: '<S179>/Sum'
         */
        if (rtb_Compare_on) {
          /* Saturate: '<S200>/Saturation1' */
          if (FMS_B.Merge_n[0] > FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[0] = FMS_PARAM.VEL_XY_LIM;
          } else if (FMS_B.Merge_n[0] < -FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[0] = -FMS_PARAM.VEL_XY_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[0] = FMS_B.Merge_n[0];
          }

          if (FMS_B.Merge_n[1] > FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[1] = FMS_PARAM.VEL_XY_LIM;
          } else if (FMS_B.Merge_n[1] < -FMS_PARAM.VEL_XY_LIM) {
            rtb_TmpSignalConversionAtMath_c[1] = -FMS_PARAM.VEL_XY_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[1] = FMS_B.Merge_n[1];
          }

          /* End of Saturate: '<S200>/Saturation1' */

          /* Saturate: '<S190>/Saturation1' */
          if (FMS_B.Merge_e > FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_PARAM.VEL_Z_LIM;
          } else if (FMS_B.Merge_e < -FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = -FMS_PARAM.VEL_Z_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_B.Merge_e;
          }

          /* End of Saturate: '<S190>/Saturation1' */
        } else {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S179>/Sum' incorporates:
           *  Delay: '<S179>/Delay'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[0] - FMS_DW.Delay_DSTATE[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* SignalConversion: '<S228>/TmpSignal ConversionAtMath FunctionInport1' */
          rtb_TmpSignalConversionAtMath_c[0] = rtb_Subtract3_o;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S175>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_MathFunction_f_idx_0 = FMS_U.INS_Out.x_R -
            FMS_B.Cmd_In.sp_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_Sum_ff[0] = rtb_Subtract3_o;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S179>/Sum' incorporates:
           *  Delay: '<S179>/Delay'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[1] - FMS_DW.Delay_DSTATE[1];

          /* Sum: '<S175>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Add3_c = FMS_U.INS_Out.y_R - FMS_B.Cmd_In.sp_waypoint[1];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Sqrt: '<S187>/Sqrt' incorporates:
           *  Math: '<S187>/Square'
           *  Sum: '<S175>/Sum'
           *  Sum: '<S187>/Sum of Elements'
           */
          rtb_Add3_c = sqrtf(rtb_MathFunction_f_idx_0 * rtb_MathFunction_f_idx_0
                             + rtb_Add3_c * rtb_Add3_c);

          /* SignalConversion: '<S230>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_i[6] = FMS_ConstB.VectorConcatenate3[0];
          rtb_VectorConcatenate_i[7] = FMS_ConstB.VectorConcatenate3[1];
          rtb_VectorConcatenate_i[8] = FMS_ConstB.VectorConcatenate3[2];

          /* SignalConversion: '<S230>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S230>/Constant4'
           */
          rtb_VectorConcatenate_i[5] = 0.0F;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Gain: '<S229>/Gain' incorporates:
           *  DiscreteIntegrator: '<S227>/Discrete-Time Integrator'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  Sum: '<S227>/Add'
           */
          rtb_a_l = -(FMS_U.INS_Out.psi - FMS_DW.l1_heading);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Trigonometry: '<S230>/Trigonometric Function3' */
          rtb_VectorConcatenate_i[4] = arm_cos_f32(rtb_a_l);

          /* Gain: '<S230>/Gain' incorporates:
           *  Trigonometry: '<S230>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_i[3] = -arm_sin_f32(rtb_a_l);

          /* SignalConversion: '<S230>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S230>/Constant3'
           */
          rtb_VectorConcatenate_i[2] = 0.0F;

          /* Trigonometry: '<S230>/Trigonometric Function' */
          rtb_VectorConcatenate_i[1] = arm_sin_f32(rtb_a_l);

          /* Trigonometry: '<S230>/Trigonometric Function1' */
          rtb_VectorConcatenate_i[0] = arm_cos_f32(rtb_a_l);

          /* Switch: '<S186>/Switch2' incorporates:
           *  Constant: '<S175>/Constant2'
           *  DiscreteIntegrator: '<S175>/Acceleration_Speed'
           *  RelationalOperator: '<S186>/LowerRelop1'
           *  RelationalOperator: '<S186>/UpperRelop'
           *  Switch: '<S186>/Switch'
           */
          if (FMS_DW.Acceleration_Speed_DSTATE > rtb_Add4_e5) {
            rtb_a_l = rtb_Add4_e5;
          } else if (FMS_DW.Acceleration_Speed_DSTATE < 0.0F) {
            /* Switch: '<S186>/Switch' incorporates:
             *  Constant: '<S175>/Constant2'
             */
            rtb_a_l = 0.0F;
          } else {
            rtb_a_l = FMS_DW.Acceleration_Speed_DSTATE;
          }

          /* End of Switch: '<S186>/Switch2' */

          /* Switch: '<S175>/Switch' */
          if (rtb_Add3_c > FMS_PARAM.L1) {
            rtb_Sqrt_b = rtb_Add4_e5;
          } else {
            /* Gain: '<S175>/Gain' */
            rtb_Sqrt_b = 0.5F * rtb_Add3_c;

            /* Switch: '<S185>/Switch2' incorporates:
             *  Constant: '<S175>/Constant1'
             *  RelationalOperator: '<S185>/LowerRelop1'
             *  RelationalOperator: '<S185>/UpperRelop'
             *  Switch: '<S185>/Switch'
             */
            if (rtb_Sqrt_b > rtb_Add4_e5) {
              rtb_Sqrt_b = rtb_Add4_e5;
            } else {
              if (rtb_Sqrt_b < 0.5F) {
                /* Switch: '<S185>/Switch' incorporates:
                 *  Constant: '<S175>/Constant1'
                 */
                rtb_Sqrt_b = 0.5F;
              }
            }

            /* End of Switch: '<S185>/Switch2' */
          }

          /* End of Switch: '<S175>/Switch' */

          /* Switch: '<S175>/Switch1' incorporates:
           *  Sum: '<S175>/Sum1'
           */
          if (rtb_a_l - rtb_Sqrt_b < 0.0F) {
            rtb_Sqrt_b = rtb_a_l;
          }

          /* End of Switch: '<S175>/Switch1' */

          /* Sum: '<S228>/Sum of Elements' incorporates:
           *  Math: '<S228>/Math Function'
           */
          rtb_a_l = rtb_TmpSignalConversionAtMath_c[0] *
            rtb_TmpSignalConversionAtMath_c[0] + rtb_Subtract3_o *
            rtb_Subtract3_o;

          /* Math: '<S228>/Math Function1' incorporates:
           *  Sum: '<S228>/Sum of Elements'
           *
           * About '<S228>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_a_l < 0.0F) {
            rtb_a_l = -sqrtf(fabsf(rtb_a_l));
          } else {
            rtb_a_l = sqrtf(rtb_a_l);
          }

          /* End of Math: '<S228>/Math Function1' */

          /* Switch: '<S228>/Switch' incorporates:
           *  Constant: '<S228>/Constant'
           *  Product: '<S228>/Product'
           */
          if (rtb_a_l > 0.0F) {
            rtb_Add3_c = rtb_Sum_ff[0];
          } else {
            rtb_Add3_c = 0.0F;
            rtb_Subtract3_o = 0.0F;
            rtb_a_l = 1.0F;
          }

          /* End of Switch: '<S228>/Switch' */

          /* Product: '<S226>/Multiply2' incorporates:
           *  Product: '<S228>/Divide'
           */
          rtb_MathFunction_f_idx_0 = rtb_Add3_c / rtb_a_l * rtb_Sqrt_b;
          rtb_Sqrt_b *= rtb_Subtract3_o / rtb_a_l;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S181>/Sum1' incorporates:
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_Rem_p = FMS_B.Cmd_In.sp_waypoint[0] - FMS_B.Cmd_In.cur_waypoint[0];
          rtb_a_l = FMS_B.Cmd_In.sp_waypoint[1] - FMS_B.Cmd_In.cur_waypoint[1];

          /* Sum: '<S181>/Sum' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           */
          rtb_MathFunction_iq_idx_0 = FMS_U.INS_Out.x_R -
            FMS_B.Cmd_In.cur_waypoint[0];
          rtb_Add3_c = FMS_U.INS_Out.y_R - FMS_B.Cmd_In.cur_waypoint[1];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Product: '<S181>/Divide' incorporates:
           *  Math: '<S182>/Square'
           *  Math: '<S183>/Square'
           *  Sqrt: '<S182>/Sqrt'
           *  Sqrt: '<S183>/Sqrt'
           *  Sum: '<S181>/Sum'
           *  Sum: '<S181>/Sum1'
           *  Sum: '<S182>/Sum of Elements'
           *  Sum: '<S183>/Sum of Elements'
           */
          rtb_a_l = sqrtf(rtb_MathFunction_iq_idx_0 * rtb_MathFunction_iq_idx_0
                          + rtb_Add3_c * rtb_Add3_c) / sqrtf(rtb_Rem_p *
            rtb_Rem_p + rtb_a_l * rtb_a_l);

          /* Saturate: '<S181>/Saturation' */
          if (rtb_a_l > 1.0F) {
            rtb_a_l = 1.0F;
          } else {
            if (rtb_a_l < 0.0F) {
              rtb_a_l = 0.0F;
            }
          }

          /* End of Saturate: '<S181>/Saturation' */

          /* Product: '<S179>/Multiply' */
          for (rtb_Compare_bv_0 = 0; rtb_Compare_bv_0 < 3; rtb_Compare_bv_0++) {
            rtb_VectorConcatenate_ar[rtb_Compare_bv_0] =
              rtb_VectorConcatenate_i[rtb_Compare_bv_0 + 3] * rtb_Sqrt_b +
              rtb_VectorConcatenate_i[rtb_Compare_bv_0] *
              rtb_MathFunction_f_idx_0;
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Gain: '<S172>/Gain' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S181>/Multiply'
           *  SignalConversion: '<S30>/Signal Copy1'
           *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Sum: '<S172>/Sum2'
           *  Sum: '<S181>/Add'
           *  Sum: '<S181>/Subtract'
           */
          rtb_Sqrt_b = (FMS_U.INS_Out.h_R - ((FMS_B.Cmd_In.sp_waypoint[2] -
            FMS_B.Cmd_In.cur_waypoint[2]) * rtb_a_l + FMS_B.Cmd_In.cur_waypoint
            [2])) * FMS_PARAM.Z_P;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[0] = rtb_VectorConcatenate_ar[0];
          rtb_TmpSignalConversionAtMath_c[1] = rtb_VectorConcatenate_ar[1];

          /* Saturate: '<S172>/Saturation1' incorporates:
           *  Product: '<S179>/Multiply'
           */
          if (rtb_Sqrt_b > FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = FMS_PARAM.VEL_Z_LIM;
          } else if (rtb_Sqrt_b < -FMS_PARAM.VEL_Z_LIM) {
            rtb_TmpSignalConversionAtMath_c[2] = -FMS_PARAM.VEL_Z_LIM;
          } else {
            rtb_TmpSignalConversionAtMath_c[2] = rtb_Sqrt_b;
          }

          /* End of Saturate: '<S172>/Saturation1' */
        }

        /* End of Switch: '<S155>/Switch' */

        /* Delay: '<S157>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.icLoad_k != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_h = FMS_U.INS_Out.psi;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* DiscreteIntegrator: '<S160>/Integrator1' incorporates:
         *  Delay: '<S157>/Delay'
         */
        if (FMS_DW.Integrator1_IC_LOADING != 0) {
          FMS_DW.Integrator1_DSTATE_p = FMS_DW.Delay_DSTATE_h;
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Math: '<S164>/Rem' incorporates:
         *  Constant: '<S164>/Constant1'
         *  DiscreteIntegrator: '<S160>/Integrator1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sum: '<S159>/Sum'
         */
        rtb_Subtract3_o = rt_remf(FMS_DW.Integrator1_DSTATE_p -
          FMS_U.INS_Out.psi, 6.28318548F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Switch: '<S164>/Switch' incorporates:
         *  Abs: '<S164>/Abs'
         *  Constant: '<S164>/Constant'
         *  Constant: '<S165>/Constant'
         *  Product: '<S164>/Multiply'
         *  RelationalOperator: '<S165>/Compare'
         *  Sum: '<S164>/Add'
         */
        if (fabsf(rtb_Subtract3_o) > 3.14159274F) {
          /* Signum: '<S164>/Sign' */
          if (rtb_Subtract3_o < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else if (rtb_Subtract3_o > 0.0F) {
            rtb_Add3_c = 1.0F;
          } else {
            rtb_Add3_c = rtb_Subtract3_o;
          }

          /* End of Signum: '<S164>/Sign' */
          rtb_Subtract3_o -= 6.28318548F * rtb_Add3_c;
        }

        /* End of Switch: '<S164>/Switch' */

        /* Gain: '<S159>/Gain2' */
        rtb_Subtract3_o *= FMS_PARAM.YAW_P;

        /* Saturate: '<S159>/Saturation' */
        if (rtb_Subtract3_o > FMS_PARAM.YAW_RATE_LIM) {
          rtb_Subtract3_o = FMS_PARAM.YAW_RATE_LIM;
        } else {
          if (rtb_Subtract3_o < -FMS_PARAM.YAW_RATE_LIM) {
            rtb_Subtract3_o = -FMS_PARAM.YAW_RATE_LIM;
          }
        }

        /* End of Saturate: '<S159>/Saturation' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S153>/Bus Assignment'
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S153>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S153>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_l;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_b;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_by;
        FMS_Y.FMS_Out.u_cmd = rtb_TmpSignalConversionAtMath_c[0];
        FMS_Y.FMS_Out.v_cmd = rtb_TmpSignalConversionAtMath_c[1];
        FMS_Y.FMS_Out.w_cmd = rtb_TmpSignalConversionAtMath_c[2];
        FMS_Y.FMS_Out.psi_rate_cmd = rtb_Subtract3_o;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S221>/Sum of Elements' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S221>/Math Function'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_a_l = FMS_U.INS_Out.vn * FMS_U.INS_Out.vn + FMS_U.INS_Out.ve *
          FMS_U.INS_Out.ve;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S221>/Math Function1' incorporates:
         *  Sum: '<S221>/Sum of Elements'
         *
         * About '<S221>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_a_l < 0.0F) {
          rtb_Subtract3_o = -sqrtf(fabsf(rtb_a_l));
        } else {
          rtb_Subtract3_o = sqrtf(rtb_a_l);
        }

        /* End of Math: '<S221>/Math Function1' */

        /* Switch: '<S221>/Switch' incorporates:
         *  Constant: '<S221>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Product: '<S221>/Product'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (rtb_Subtract3_o > 0.0F) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[0] = FMS_U.INS_Out.vn;
          rtb_TmpSignalConversionAtMath_c[1] = FMS_U.INS_Out.ve;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_TmpSignalConversionAtMath_c[2] = rtb_Subtract3_o;
        } else {
          rtb_TmpSignalConversionAtMath_c[0] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[1] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[2] = 1.0F;
        }

        /* End of Switch: '<S221>/Switch' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S153>/Sum' incorporates:
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_Sqrt_b = FMS_B.Cmd_In.sp_waypoint[0] - FMS_B.Cmd_In.cur_waypoint[0];
        rtb_TmpSignalConversionAtDela_a[0] = FMS_B.Cmd_In.sp_waypoint[0] -
          FMS_B.Cmd_In.cur_waypoint[0];
        rtb_MathFunction_f_idx_0 = FMS_B.Cmd_In.sp_waypoint[1] -
          FMS_B.Cmd_In.cur_waypoint[1];
        rtb_TmpSignalConversionAtDela_a[1] = FMS_B.Cmd_In.sp_waypoint[1] -
          FMS_B.Cmd_In.cur_waypoint[1];

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S171>/Sum of Elements' incorporates:
         *  Math: '<S171>/Math Function'
         *  Sum: '<S153>/Sum'
         */
        rtb_a_l = rtb_MathFunction_f_idx_0 * rtb_MathFunction_f_idx_0 +
          rtb_Sqrt_b * rtb_Sqrt_b;

        /* Math: '<S171>/Math Function1' incorporates:
         *  Sum: '<S171>/Sum of Elements'
         *
         * About '<S171>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_a_l < 0.0F) {
          rtb_Subtract3_o = -sqrtf(fabsf(rtb_a_l));
        } else {
          rtb_Subtract3_o = sqrtf(rtb_a_l);
        }

        /* End of Math: '<S171>/Math Function1' */

        /* Switch: '<S171>/Switch' incorporates:
         *  Constant: '<S171>/Constant'
         *  Product: '<S171>/Product'
         */
        if (rtb_Subtract3_o > 0.0F) {
          rtb_MathFunction_f_idx_2 = rtb_Subtract3_o;
        } else {
          rtb_MathFunction_f_idx_0 = 0.0F;
          rtb_Sqrt_b = 0.0F;
          rtb_MathFunction_f_idx_2 = 1.0F;
        }

        /* End of Switch: '<S171>/Switch' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S177>/NearbyRefWP' incorporates:
         *  Constant: '<S153>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        FMS_NearbyRefWP(&rtb_Switch_dw[0], FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                        FMS_PARAM.L1, rtb_TmpSignalConversionAtDela_a, &rtb_a_l);

        /* MATLAB Function: '<S177>/SearchL1RefWP' incorporates:
         *  Constant: '<S153>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_Add3_c = FMS_B.Cmd_In.sp_waypoint[0] - FMS_B.Cmd_In.cur_waypoint[0];
        rtb_Subtract3_o = FMS_B.Cmd_In.sp_waypoint[1] -
          FMS_B.Cmd_In.cur_waypoint[1];

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_Subtract3_o = rtb_Add3_c * rtb_Add3_c + rtb_Subtract3_o *
          rtb_Subtract3_o;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        B = ((FMS_B.Cmd_In.sp_waypoint[0] - FMS_B.Cmd_In.cur_waypoint[0]) *
             (FMS_B.Cmd_In.cur_waypoint[0] - FMS_U.INS_Out.x_R) +
             (FMS_B.Cmd_In.sp_waypoint[1] - FMS_B.Cmd_In.cur_waypoint[1]) *
             (FMS_B.Cmd_In.cur_waypoint[1] - FMS_U.INS_Out.y_R)) * 2.0F;
        D = B * B - (((((FMS_U.INS_Out.x_R * FMS_U.INS_Out.x_R +
                         FMS_U.INS_Out.y_R * FMS_U.INS_Out.y_R) +
                        FMS_B.Cmd_In.cur_waypoint[0] *
                        FMS_B.Cmd_In.cur_waypoint[0]) +
                       FMS_B.Cmd_In.cur_waypoint[1] * FMS_B.Cmd_In.cur_waypoint
                       [1]) - (FMS_U.INS_Out.x_R * FMS_B.Cmd_In.cur_waypoint[0]
          + FMS_U.INS_Out.y_R * FMS_B.Cmd_In.cur_waypoint[1]) * 2.0F) -
                     FMS_PARAM.L1 * FMS_PARAM.L1) * (4.0F * rtb_Subtract3_o);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_Add3_c = -1.0F;
        rtb_MathFunction_iq_idx_0 = 0.0F;
        rtb_MathFunction_iq_idx_1 = 0.0F;
        guard1 = false;
        if (D > 0.0F) {
          u1_tmp = sqrtf(D);
          D = (-B + u1_tmp) / (2.0F * rtb_Subtract3_o);
          rtb_Subtract3_o = (-B - u1_tmp) / (2.0F * rtb_Subtract3_o);
          if ((D >= 0.0F) && (D <= 1.0F) && (rtb_Subtract3_o >= 0.0F) &&
              (rtb_Subtract3_o <= 1.0F)) {
            rtb_Add3_c = fmaxf(D, rtb_Subtract3_o);
            guard1 = true;
          } else if ((D >= 0.0F) && (D <= 1.0F)) {
            rtb_Add3_c = D;
            guard1 = true;
          } else {
            if ((rtb_Subtract3_o >= 0.0F) && (rtb_Subtract3_o <= 1.0F)) {
              rtb_Add3_c = rtb_Subtract3_o;
              guard1 = true;
            }
          }
        } else {
          if (D == 0.0F) {
            D = -B / (2.0F * rtb_Subtract3_o);
            if ((D >= 0.0F) && (D <= 1.0F)) {
              rtb_Add3_c = D;
              guard1 = true;
            }
          }
        }

        if (guard1) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_MathFunction_iq_idx_0 = (FMS_B.Cmd_In.sp_waypoint[0] -
            FMS_B.Cmd_In.cur_waypoint[0]) * rtb_Add3_c +
            FMS_B.Cmd_In.cur_waypoint[0];
          rtb_MathFunction_iq_idx_1 = (FMS_B.Cmd_In.sp_waypoint[1] -
            FMS_B.Cmd_In.cur_waypoint[1]) * rtb_Add3_c +
            FMS_B.Cmd_In.cur_waypoint[1];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S177>/OutRegionRegWP' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_TmpSignalConversionAtMath_0 = FMS_B.Cmd_In.sp_waypoint[0] -
          FMS_B.Cmd_In.cur_waypoint[0];
        rtb_Rem_p = FMS_B.Cmd_In.sp_waypoint[1] - FMS_B.Cmd_In.cur_waypoint[1];
        rtb_Subtract3_o = ((FMS_U.INS_Out.y_R - FMS_B.Cmd_In.cur_waypoint[1]) *
                           rtb_Rem_p + (FMS_U.INS_Out.x_R -
          FMS_B.Cmd_In.cur_waypoint[0]) * rtb_TmpSignalConversionAtMath_0) /
          (rtb_TmpSignalConversionAtMath_0 * rtb_TmpSignalConversionAtMath_0 +
           rtb_Rem_p * rtb_Rem_p);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_FixPtRelationalOperator_me = (rtb_Subtract3_o <= 0.0F);
        rtb_LogicalOperator_es = (rtb_Subtract3_o >= 1.0F);
        if (rtb_FixPtRelationalOperator_me) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_P_l_idx_0 = FMS_B.Cmd_In.cur_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        } else if (rtb_LogicalOperator_es) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_P_l_idx_0 = FMS_B.Cmd_In.sp_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        } else {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_P_l_idx_0 = rtb_Subtract3_o * rtb_TmpSignalConversionAtMath_0 +
            FMS_B.Cmd_In.cur_waypoint[0];

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* Switch: '<S177>/Switch1' incorporates:
         *  Constant: '<S214>/Constant'
         *  RelationalOperator: '<S214>/Compare'
         */
        if (rtb_a_l <= 0.0F) {
          /* Switch: '<S177>/Switch' incorporates:
           *  Constant: '<S213>/Constant'
           *  MATLAB Function: '<S177>/SearchL1RefWP'
           *  RelationalOperator: '<S213>/Compare'
           */
          if (rtb_Add3_c >= 0.0F) {
            rtb_TmpSignalConversionAtDela_a[0] = rtb_MathFunction_iq_idx_0;
            rtb_TmpSignalConversionAtDela_a[1] = rtb_MathFunction_iq_idx_1;
          } else {
            rtb_TmpSignalConversionAtDela_a[0] = rtb_P_l_idx_0;

            /* MATLAB Function: '<S177>/OutRegionRegWP' incorporates:
             *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy3Inport1'
             */
            if (rtb_FixPtRelationalOperator_me) {
              /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
              rtb_TmpSignalConversionAtDela_a[1] = FMS_B.Cmd_In.cur_waypoint[1];

              /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            } else if (rtb_LogicalOperator_es) {
              /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
              rtb_TmpSignalConversionAtDela_a[1] = FMS_B.Cmd_In.sp_waypoint[1];

              /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            } else {
              /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
              rtb_TmpSignalConversionAtDela_a[1] = rtb_Subtract3_o * rtb_Rem_p +
                FMS_B.Cmd_In.cur_waypoint[1];

              /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            }
          }

          /* End of Switch: '<S177>/Switch' */
        }

        /* End of Switch: '<S177>/Switch1' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S178>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_MathFunction_iq_idx_0 = rtb_TmpSignalConversionAtDela_a[0] -
          FMS_U.INS_Out.x_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        rtb_TmpSignalConversionAtDela_a[0] = rtb_MathFunction_iq_idx_0 *
          rtb_MathFunction_iq_idx_0;
        rtb_Add3_c = rtb_MathFunction_iq_idx_0;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S178>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S222>/Math Function'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_MathFunction_iq_idx_0 = rtb_TmpSignalConversionAtDela_a[1] -
          FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S222>/Math Function' incorporates:
         *  Math: '<S220>/Square'
         */
        rtb_a_l = rtb_MathFunction_iq_idx_0 * rtb_MathFunction_iq_idx_0;

        /* Sum: '<S222>/Sum of Elements' incorporates:
         *  Math: '<S222>/Math Function'
         */
        rtb_Subtract3_o = rtb_a_l + rtb_TmpSignalConversionAtDela_a[0];

        /* Math: '<S222>/Math Function1' incorporates:
         *  Sum: '<S222>/Sum of Elements'
         *
         * About '<S222>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -sqrtf(fabsf(rtb_Subtract3_o));
        } else {
          rtb_Subtract3_o = sqrtf(rtb_Subtract3_o);
        }

        /* End of Math: '<S222>/Math Function1' */

        /* Switch: '<S222>/Switch' incorporates:
         *  Constant: '<S222>/Constant'
         *  Product: '<S222>/Product'
         */
        if (rtb_Subtract3_o > 0.0F) {
          rtb_Switch_dw[0] = rtb_Add3_c;
          rtb_Switch_dw[1] = rtb_MathFunction_iq_idx_0;
          rtb_Switch_dw[2] = rtb_Subtract3_o;
        } else {
          rtb_Switch_dw[0] = 0.0F;
          rtb_Switch_dw[1] = 0.0F;
          rtb_Switch_dw[2] = 1.0F;
        }

        /* End of Switch: '<S222>/Switch' */

        /* Product: '<S221>/Divide' */
        rtb_TmpSignalConversionAtMath_0 = rtb_TmpSignalConversionAtMath_c[0] /
          rtb_TmpSignalConversionAtMath_c[2];
        rtb_P_l_idx_0 = rtb_TmpSignalConversionAtMath_c[1] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Sum: '<S224>/Sum of Elements' incorporates:
         *  Math: '<S224>/Math Function'
         *  SignalConversion: '<S224>/TmpSignal ConversionAtMath FunctionInport1'
         */
        rtb_Subtract3_o = rtb_P_l_idx_0 * rtb_P_l_idx_0 +
          rtb_TmpSignalConversionAtMath_0 * rtb_TmpSignalConversionAtMath_0;

        /* Math: '<S224>/Math Function1' incorporates:
         *  Sum: '<S224>/Sum of Elements'
         *
         * About '<S224>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -sqrtf(fabsf(rtb_Subtract3_o));
        } else {
          rtb_Subtract3_o = sqrtf(rtb_Subtract3_o);
        }

        /* End of Math: '<S224>/Math Function1' */

        /* Switch: '<S224>/Switch' incorporates:
         *  Constant: '<S224>/Constant'
         *  Product: '<S224>/Product'
         */
        if (rtb_Subtract3_o > 0.0F) {
          rtb_TmpSignalConversionAtMath_c[0] = rtb_P_l_idx_0;
          rtb_TmpSignalConversionAtMath_c[1] = rtb_TmpSignalConversionAtMath_0;
          rtb_TmpSignalConversionAtMath_c[2] = rtb_Subtract3_o;
        } else {
          rtb_TmpSignalConversionAtMath_c[0] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[1] = 0.0F;
          rtb_TmpSignalConversionAtMath_c[2] = 1.0F;
        }

        /* End of Switch: '<S224>/Switch' */

        /* Product: '<S222>/Divide' */
        rtb_TmpSignalConversionAtMath_0 = rtb_Switch_dw[0] / rtb_Switch_dw[2];
        rtb_P_l_idx_0 = rtb_Switch_dw[1] / rtb_Switch_dw[2];

        /* Sum: '<S225>/Sum of Elements' incorporates:
         *  Math: '<S225>/Math Function'
         *  SignalConversion: '<S225>/TmpSignal ConversionAtMath FunctionInport1'
         */
        rtb_Subtract3_o = rtb_P_l_idx_0 * rtb_P_l_idx_0 +
          rtb_TmpSignalConversionAtMath_0 * rtb_TmpSignalConversionAtMath_0;

        /* Math: '<S225>/Math Function1' incorporates:
         *  Sum: '<S225>/Sum of Elements'
         *
         * About '<S225>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -sqrtf(fabsf(rtb_Subtract3_o));
        } else {
          rtb_Subtract3_o = sqrtf(rtb_Subtract3_o);
        }

        /* End of Math: '<S225>/Math Function1' */

        /* Switch: '<S225>/Switch' incorporates:
         *  Constant: '<S225>/Constant'
         *  Product: '<S225>/Product'
         */
        if (rtb_Subtract3_o > 0.0F) {
          rtb_Switch_dw[0] = rtb_P_l_idx_0;
          rtb_Switch_dw[1] = rtb_TmpSignalConversionAtMath_0;
          rtb_Switch_dw[2] = rtb_Subtract3_o;
        } else {
          rtb_Switch_dw[0] = 0.0F;
          rtb_Switch_dw[1] = 0.0F;
          rtb_Switch_dw[2] = 1.0F;
        }

        /* End of Switch: '<S225>/Switch' */

        /* Product: '<S225>/Divide' */
        rtb_TmpSignalConversionAtMath_0 = rtb_Switch_dw[0] / rtb_Switch_dw[2];

        /* Product: '<S224>/Divide' */
        rtb_TmpSignalConversionAtDela_a[0] = rtb_TmpSignalConversionAtMath_c[0] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Product: '<S171>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_MathFunction_f_idx_0 /
          rtb_MathFunction_f_idx_2;
        rtb_Add3_c *= rtb_Add3_c;

        /* Product: '<S225>/Divide' incorporates:
         *  Math: '<S220>/Square'
         */
        rtb_P_l_idx_0 = rtb_Switch_dw[1] / rtb_Switch_dw[2];

        /* Product: '<S224>/Divide' */
        rtb_TmpSignalConversionAtDela_a[1] = rtb_TmpSignalConversionAtMath_c[1] /
          rtb_TmpSignalConversionAtMath_c[2];

        /* Product: '<S171>/Divide' */
        rtb_MathFunction_iq_idx_1 = rtb_Sqrt_b / rtb_MathFunction_f_idx_2;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sqrt: '<S219>/Sqrt' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S219>/Square'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sum: '<S219>/Sum of Elements'
         */
        rtb_Subtract3_o = sqrtf(FMS_U.INS_Out.vn * FMS_U.INS_Out.vn +
          FMS_U.INS_Out.ve * FMS_U.INS_Out.ve);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Gain: '<S178>/Gain' incorporates:
         *  Math: '<S178>/Square'
         */
        rtb_Sqrt_b = rtb_Subtract3_o * rtb_Subtract3_o * 2.0F;

        /* Sum: '<S223>/Subtract' incorporates:
         *  Product: '<S223>/Multiply'
         *  Product: '<S223>/Multiply1'
         */
        rtb_Subtract3_o = rtb_TmpSignalConversionAtMath_0 *
          rtb_TmpSignalConversionAtDela_a[1] - rtb_P_l_idx_0 *
          rtb_TmpSignalConversionAtDela_a[0];

        /* Signum: '<S218>/Sign1' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S218>/Sign1' */

        /* Switch: '<S218>/Switch2' incorporates:
         *  Constant: '<S218>/Constant4'
         */
        if (rtb_Subtract3_o == 0.0F) {
          rtb_Subtract3_o = 1.0F;
        }

        /* End of Switch: '<S218>/Switch2' */

        /* DotProduct: '<S218>/Dot Product' */
        rtb_MathFunction_f_idx_0 = rtb_TmpSignalConversionAtDela_a[0] *
          rtb_TmpSignalConversionAtMath_0 + rtb_TmpSignalConversionAtDela_a[1] *
          rtb_P_l_idx_0;

        /* Trigonometry: '<S218>/Acos' incorporates:
         *  DotProduct: '<S218>/Dot Product'
         */
        if (rtb_MathFunction_f_idx_0 > 1.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          if (rtb_MathFunction_f_idx_0 < -1.0F) {
            rtb_MathFunction_f_idx_0 = -1.0F;
          }
        }

        /* Product: '<S218>/Multiply' incorporates:
         *  Trigonometry: '<S218>/Acos'
         */
        rtb_Subtract3_o *= acosf(rtb_MathFunction_f_idx_0);

        /* Saturate: '<S178>/Saturation' */
        if (rtb_Subtract3_o > 1.57079637F) {
          rtb_Subtract3_o = 1.57079637F;
        } else {
          if (rtb_Subtract3_o < -1.57079637F) {
            rtb_Subtract3_o = -1.57079637F;
          }
        }

        /* End of Saturate: '<S178>/Saturation' */

        /* Product: '<S178>/Divide' incorporates:
         *  Constant: '<S153>/L1'
         *  Constant: '<S178>/Constant'
         *  MinMax: '<S178>/Max'
         *  MinMax: '<S178>/Min'
         *  Product: '<S178>/Multiply1'
         *  Sqrt: '<S220>/Sqrt'
         *  Sum: '<S220>/Sum of Elements'
         *  Trigonometry: '<S178>/Sin'
         */
        rtb_MathFunction_f_idx_2 = arm_sin_f32(rtb_Subtract3_o) * rtb_Sqrt_b /
          fminf(FMS_PARAM.L1, fmaxf(sqrtf(rtb_a_l + rtb_Add3_c), 0.5F));

        /* Sum: '<S169>/Subtract' incorporates:
         *  Product: '<S169>/Multiply'
         *  Product: '<S169>/Multiply1'
         */
        rtb_Rem_p = rtb_MathFunction_iq_idx_0 * FMS_ConstB.Divide[1] -
          rtb_MathFunction_iq_idx_1 * FMS_ConstB.Divide[0];

        /* Signum: '<S158>/Sign1' */
        if (rtb_Rem_p < 0.0F) {
          rtb_Rem_p = -1.0F;
        } else {
          if (rtb_Rem_p > 0.0F) {
            rtb_Rem_p = 1.0F;
          }
        }

        /* End of Signum: '<S158>/Sign1' */

        /* Switch: '<S158>/Switch2' incorporates:
         *  Constant: '<S158>/Constant4'
         */
        if (rtb_Rem_p == 0.0F) {
          rtb_Rem_p = 1.0F;
        }

        /* End of Switch: '<S158>/Switch2' */

        /* DotProduct: '<S158>/Dot Product' */
        rtb_Sqrt_b = FMS_ConstB.Divide[0] * rtb_MathFunction_iq_idx_0 +
          FMS_ConstB.Divide[1] * rtb_MathFunction_iq_idx_1;

        /* Trigonometry: '<S158>/Acos' incorporates:
         *  DotProduct: '<S158>/Dot Product'
         */
        if (rtb_Sqrt_b > 1.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          if (rtb_Sqrt_b < -1.0F) {
            rtb_Sqrt_b = -1.0F;
          }
        }

        /* Product: '<S158>/Multiply' incorporates:
         *  Trigonometry: '<S158>/Acos'
         */
        rtb_Rem_p *= acosf(rtb_Sqrt_b);

        /* Math: '<S161>/Rem' incorporates:
         *  Constant: '<S161>/Constant1'
         *  Delay: '<S157>/Delay'
         *  Sum: '<S157>/Sum2'
         */
        rtb_Subtract3_o = rt_remf(rtb_Rem_p - FMS_DW.Delay_DSTATE_h, 6.28318548F);

        /* Switch: '<S161>/Switch' incorporates:
         *  Abs: '<S161>/Abs'
         *  Constant: '<S161>/Constant'
         *  Constant: '<S167>/Constant'
         *  Product: '<S161>/Multiply'
         *  RelationalOperator: '<S167>/Compare'
         *  Sum: '<S161>/Add'
         */
        if (fabsf(rtb_Subtract3_o) > 3.14159274F) {
          /* Signum: '<S161>/Sign' */
          if (rtb_Subtract3_o < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else if (rtb_Subtract3_o > 0.0F) {
            rtb_Add3_c = 1.0F;
          } else {
            rtb_Add3_c = rtb_Subtract3_o;
          }

          /* End of Signum: '<S161>/Sign' */
          rtb_Subtract3_o -= 6.28318548F * rtb_Add3_c;
        }

        /* End of Switch: '<S161>/Switch' */

        /* Sum: '<S157>/Sum' incorporates:
         *  Delay: '<S157>/Delay'
         */
        rtb_Add3_c = rtb_Subtract3_o + FMS_DW.Delay_DSTATE_h;

        /* Product: '<S166>/Multiply1' incorporates:
         *  Constant: '<S166>/const1'
         *  DiscreteIntegrator: '<S160>/Integrator'
         */
        rtb_Subtract3_o = FMS_DW.Integrator_DSTATE_i * 0.785398185F;

        /* Sum: '<S166>/Add' incorporates:
         *  DiscreteIntegrator: '<S160>/Integrator1'
         *  Sum: '<S160>/Subtract'
         */
        rtb_MathFunction_f_idx_0 = (FMS_DW.Integrator1_DSTATE_p - rtb_Add3_c) +
          rtb_Subtract3_o;

        /* Signum: '<S166>/Sign' */
        if (rtb_MathFunction_f_idx_0 < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else if (rtb_MathFunction_f_idx_0 > 0.0F) {
          rtb_Add3_c = 1.0F;
        } else {
          rtb_Add3_c = rtb_MathFunction_f_idx_0;
        }

        /* End of Signum: '<S166>/Sign' */

        /* Sum: '<S166>/Add2' incorporates:
         *  Abs: '<S166>/Abs'
         *  Gain: '<S166>/Gain'
         *  Gain: '<S166>/Gain1'
         *  Product: '<S166>/Multiply2'
         *  Product: '<S166>/Multiply3'
         *  Sqrt: '<S166>/Sqrt'
         *  Sum: '<S166>/Add1'
         *  Sum: '<S166>/Subtract'
         */
        rtb_a_l = (sqrtf((8.0F * fabsf(rtb_MathFunction_f_idx_0) +
                          FMS_ConstB.d_j) * FMS_ConstB.d_j) - FMS_ConstB.d_j) *
          0.5F * rtb_Add3_c + rtb_Subtract3_o;

        /* Sum: '<S166>/Add4' */
        rtb_Subtract3_o += rtb_MathFunction_f_idx_0 - rtb_a_l;

        /* Sum: '<S166>/Add3' */
        rtb_Add3_c = rtb_MathFunction_f_idx_0 + FMS_ConstB.d_j;

        /* Sum: '<S166>/Subtract1' */
        rtb_MathFunction_f_idx_0 -= FMS_ConstB.d_j;

        /* Signum: '<S166>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S166>/Sign1' */

        /* Signum: '<S166>/Sign2' */
        if (rtb_MathFunction_f_idx_0 < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else {
          if (rtb_MathFunction_f_idx_0 > 0.0F) {
            rtb_MathFunction_f_idx_0 = 1.0F;
          }
        }

        /* End of Signum: '<S166>/Sign2' */

        /* Sum: '<S166>/Add5' incorporates:
         *  Gain: '<S166>/Gain2'
         *  Product: '<S166>/Multiply4'
         *  Sum: '<S166>/Subtract2'
         */
        rtb_a_l += (rtb_Add3_c - rtb_MathFunction_f_idx_0) * 0.5F *
          rtb_Subtract3_o;

        /* Sum: '<S166>/Add6' */
        rtb_Add3_c = rtb_a_l + FMS_ConstB.d_j;

        /* Sum: '<S166>/Subtract3' */
        rtb_Subtract3_o = rtb_a_l - FMS_ConstB.d_j;

        /* Product: '<S166>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_a_l / FMS_ConstB.d_j;

        /* Signum: '<S166>/Sign5' incorporates:
         *  Signum: '<S166>/Sign6'
         */
        if (rtb_a_l < 0.0F) {
          rtb_MathFunction_iq_idx_1 = -1.0F;

          /* Signum: '<S166>/Sign6' */
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_MathFunction_iq_idx_1 = 1.0F;

          /* Signum: '<S166>/Sign6' */
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          rtb_MathFunction_iq_idx_1 = rtb_a_l;

          /* Signum: '<S166>/Sign6' */
          rtb_MathFunction_f_idx_0 = rtb_a_l;
        }

        /* End of Signum: '<S166>/Sign5' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S157>/Sum1' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtb_Rem_p -= FMS_U.INS_Out.psi;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S162>/Rem' incorporates:
         *  Constant: '<S162>/Constant1'
         */
        rtb_a_l = rt_remf(rtb_Rem_p, 6.28318548F);

        /* Switch: '<S162>/Switch' incorporates:
         *  Abs: '<S162>/Abs'
         *  Constant: '<S162>/Constant'
         *  Constant: '<S168>/Constant'
         *  Product: '<S162>/Multiply'
         *  RelationalOperator: '<S168>/Compare'
         *  Sum: '<S162>/Add'
         */
        if (fabsf(rtb_a_l) > 3.14159274F) {
          /* Signum: '<S162>/Sign' */
          if (rtb_a_l < 0.0F) {
            rtb_Sqrt_b = -1.0F;
          } else if (rtb_a_l > 0.0F) {
            rtb_Sqrt_b = 1.0F;
          } else {
            rtb_Sqrt_b = rtb_a_l;
          }

          /* End of Signum: '<S162>/Sign' */
          rtb_a_l -= 6.28318548F * rtb_Sqrt_b;
        }

        /* End of Switch: '<S162>/Switch' */

        /* Abs: '<S155>/Abs' */
        rtb_a_l = fabsf(rtb_a_l);

        /* Update for Delay: '<S179>/Delay' */
        FMS_DW.icLoad = 0U;

        /* Update for DiscreteIntegrator: '<S180>/Discrete-Time Integrator' incorporates:
         *  Constant: '<S173>/Constant'
         *  RelationalOperator: '<S173>/Compare'
         */
        FMS_DW.DiscreteTimeIntegrator_DSTATE_k = (uint8_T)((uint32_T)(rtb_a_l <=
          0.17453292F) + FMS_DW.DiscreteTimeIntegrator_DSTATE_k);
        if (FMS_DW.DiscreteTimeIntegrator_DSTATE_k >= 100) {
          FMS_DW.DiscreteTimeIntegrator_DSTATE_k = 100U;
        } else {
          if (FMS_DW.DiscreteTimeIntegrator_DSTATE_k <= 0) {
            FMS_DW.DiscreteTimeIntegrator_DSTATE_k = 0U;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S180>/Discrete-Time Integrator' */

        /* Update for DiscreteIntegrator: '<S175>/Acceleration_Speed' incorporates:
         *  Constant: '<S175>/Constant'
         */
        FMS_DW.Acceleration_Speed_DSTATE += 0.004F * FMS_PARAM.CRUISE_ACC;
        FMS_DW.Acceleration_Speed_PrevResetSta = (int8_T)rtb_Compare_on;

        /* Product: '<S179>/Divide1' */
        rtb_Sqrt_b = rtb_MathFunction_f_idx_2 / rtb_Add4_e5;

        /* Saturate: '<S179>/Saturation' */
        if (rtb_Sqrt_b > 0.314159274F) {
          rtb_Sqrt_b = 0.314159274F;
        } else {
          if (rtb_Sqrt_b < -0.314159274F) {
            rtb_Sqrt_b = -0.314159274F;
          }
        }

        /* End of Saturate: '<S179>/Saturation' */

        /* Update for DiscreteIntegrator: '<S227>/Discrete-Time Integrator' */
        FMS_DW.l1_heading += 0.004F * rtb_Sqrt_b;

        /* Update for Delay: '<S157>/Delay' */
        FMS_DW.icLoad_k = 0U;

        /* Update for DiscreteIntegrator: '<S160>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S160>/Integrator'
         */
        FMS_DW.Integrator1_IC_LOADING = 0U;
        FMS_DW.Integrator1_DSTATE_p += 0.004F * FMS_DW.Integrator_DSTATE_i;

        /* Signum: '<S166>/Sign3' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S166>/Sign3' */

        /* Signum: '<S166>/Sign4' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S166>/Sign4' */

        /* Update for DiscreteIntegrator: '<S160>/Integrator' incorporates:
         *  Constant: '<S166>/const'
         *  Gain: '<S166>/Gain3'
         *  Product: '<S166>/Multiply5'
         *  Product: '<S166>/Multiply6'
         *  Sum: '<S166>/Subtract4'
         *  Sum: '<S166>/Subtract5'
         *  Sum: '<S166>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_i += ((rtb_MathFunction_iq_idx_0 -
          rtb_MathFunction_iq_idx_1) * FMS_ConstB.Gain4_c * ((rtb_Add3_c -
          rtb_Subtract3_o) * 0.5F) - rtb_MathFunction_f_idx_0 * 1.04719758F) *
          0.004F;
        if (FMS_DW.Integrator_DSTATE_i >= FMS_PARAM.YAW_RATE_LIM) {
          FMS_DW.Integrator_DSTATE_i = FMS_PARAM.YAW_RATE_LIM;
        } else {
          if (FMS_DW.Integrator_DSTATE_i <= -FMS_PARAM.YAW_RATE_LIM) {
            FMS_DW.Integrator_DSTATE_i = -FMS_PARAM.YAW_RATE_LIM;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S160>/Integrator' */
        /* End of Outputs for SubSystem: '<S149>/Mission_SubSystem' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Update for UnitDelay: '<S152>/Delay Input1' incorporates:
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy5Inport1'
         *
         * Block description for '<S152>/Delay Input1':
         *
         *  Store in Global RAM
         */
        FMS_DW.DelayInput1_DSTATE_pe = FMS_B.wp_index;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        /* End of Outputs for SubSystem: '<S36>/Mission' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S36>/Unknown' incorporates:
         *  ActionPort: '<S151>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_d);

        /* End of Outputs for SubSystem: '<S36>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S36>/Switch Case' */
      /* End of Outputs for SubSystem: '<S31>/Auto' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S31>/Assist' incorporates:
       *  ActionPort: '<S35>/Action Port'
       */
      /* SwitchCase: '<S35>/Switch Case' */
      rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_f;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      switch (FMS_B.state) {
       case VehicleState_Acro:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 0;
        break;

       case VehicleState_Stabilize:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 1;
        break;

       case VehicleState_Altitude:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 2;
        break;

       case VehicleState_Position:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 3;
        break;

       default:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 4;
        break;
      }

      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
      if (rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem_f) {
        switch (rtPrevAction) {
         case 0:
         case 4:
          break;

         case 1:
          /* Disable for SwitchCase: '<S136>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_fs = -1;
          break;

         case 2:
          /* Disable for SwitchCase: '<S50>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_m = -1;

          /* Disable for SwitchCase: '<S68>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_h = -1;
          break;

         case 3:
          /* Disable for SwitchCase: '<S84>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_o = -1;

          /* Disable for SwitchCase: '<S109>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_l = -1;

          /* Disable for SwitchCase: '<S96>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_j = -1;
          break;
        }
      }

      switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S35>/Acro' incorporates:
         *  ActionPort: '<S40>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S40>/Bus Assignment'
         *  Constant: '<S40>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S40>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Gain: '<S45>/Gain'
         *  Gain: '<S45>/Gain1'
         *  Gain: '<S45>/Gain2'
         *  Inport: '<Root>/Pilot_Cmd'
         *  Outport: '<Root>/FMS_Out'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_j;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_do;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_b;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        FMS_Y.FMS_Out.p_cmd = 3.14159274F * FMS_U.Pilot_Cmd.stick_roll;
        FMS_Y.FMS_Out.q_cmd = -3.14159274F * FMS_U.Pilot_Cmd.stick_pitch;
        FMS_Y.FMS_Out.r_cmd = 1.57079637F * FMS_U.Pilot_Cmd.stick_yaw;

        /* Saturate: '<S46>/Saturation' incorporates:
         *  Constant: '<S46>/Constant4'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         *  Sum: '<S46>/Sum'
         */
        if (FMS_U.Pilot_Cmd.stick_throttle + 1.0F > 2.0F) {
          rtb_a_l = 2.0F;
        } else if (FMS_U.Pilot_Cmd.stick_throttle + 1.0F < 0.0F) {
          rtb_a_l = 0.0F;
        } else {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_throttle + 1.0F;
        }

        /* End of Saturate: '<S46>/Saturation' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* BusAssignment: '<S40>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Constant: '<S46>/Constant5'
         *  Gain: '<S46>/Gain2'
         *  Outport: '<Root>/FMS_Out'
         *  Sum: '<S46>/Add'
         */
        FMS_Y.FMS_Out.throttle_cmd = (uint16_T)((uint16_T)fmodf(floorf(500.0F *
          rtb_a_l), 65536.0F) + 1000U);

        /* End of Outputs for SubSystem: '<S35>/Acro' */
        break;

       case 1:
        if (FMS_DW.SwitchCase_ActiveSubsystem_f != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S35>/Stabilize' incorporates:
           *  ActionPort: '<S43>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S35>/Switch Case' incorporates:
           *  DiscreteIntegrator: '<S132>/Integrator'
           *  DiscreteIntegrator: '<S132>/Integrator1'
           *  DiscreteIntegrator: '<S133>/Integrator'
           *  DiscreteIntegrator: '<S133>/Integrator1'
           */
          FMS_DW.Integrator1_DSTATE_l = 0.0F;
          FMS_DW.Integrator1_DSTATE_h = 0.0F;
          FMS_DW.Integrator_DSTATE_a = 0.0F;
          FMS_DW.Integrator_DSTATE_c = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S35>/Stabilize' */

          /* SystemReset for IfAction SubSystem: '<S35>/Stabilize' incorporates:
           *  ActionPort: '<S43>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S35>/Switch Case' incorporates:
           *  Chart: '<S137>/Motion State'
           */
          FMS_MotionState_j_Reset(&FMS_DW.sf_MotionState_e);

          /* End of SystemReset for SubSystem: '<S35>/Stabilize' */
        }

        /* Outputs for IfAction SubSystem: '<S35>/Stabilize' incorporates:
         *  ActionPort: '<S43>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Logic: '<S127>/Logical Operator' incorporates:
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy7Inport1'
         */
        rtb_FixPtRelationalOperator_me = !FMS_B.Compare;

        /* Chart: '<S137>/Motion State' incorporates:
         *  Abs: '<S137>/Abs'
         *  Abs: '<S137>/Abs1'
         *  Constant: '<S147>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  RelationalOperator: '<S147>/Compare'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        FMS_MotionState_e(fabsf(FMS_U.Pilot_Cmd.stick_yaw) > FMS_PARAM.YAW_DZ,
                          fabsf(FMS_U.INS_Out.r), &rtb_state_c,
                          &FMS_DW.sf_MotionState_e);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S136>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_fs;
        FMS_DW.SwitchCase_ActiveSubsystem_fs = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_fs = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_fs = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_fs = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_fs) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_fs != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S136>/Hold Control' incorporates:
             *  ActionPort: '<S139>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S136>/Switch Case' */
            FMS_HoldControl_kp_Reset(&FMS_DW.HoldControl_h);

            /* End of SystemReset for SubSystem: '<S136>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S136>/Hold Control' incorporates:
           *  ActionPort: '<S139>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_k(FMS_U.INS_Out.psi, &FMS_B.Merge_j,
                            &FMS_DW.HoldControl_h);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S136>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S136>/Brake Control' incorporates:
           *  ActionPort: '<S138>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_j);

          /* End of Outputs for SubSystem: '<S136>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_fs != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S136>/Move Control' incorporates:
             *  ActionPort: '<S140>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S136>/Switch Case' */
            FMS_MoveControl_l_Reset(&FMS_DW.MoveControl_k);

            /* End of SystemReset for SubSystem: '<S136>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S136>/Move Control' incorporates:
           *  ActionPort: '<S140>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_b(FMS_U.Pilot_Cmd.stick_yaw, &FMS_B.Merge_j,
                            &FMS_ConstB.MoveControl_k, &FMS_DW.MoveControl_k);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S136>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S136>/Switch Case' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S129>/Switch' incorporates:
         *  Constant: '<S129>/Constant'
         *  Constant: '<S129>/Constant4'
         *  Constant: '<S129>/Constant5'
         *  Gain: '<S129>/Gain2'
         *  Inport: '<Root>/Pilot_Cmd'
         *  Saturate: '<S129>/Saturation'
         *  SignalConversion: '<S30>/Signal Copy2'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy7Inport1'
         *  Sum: '<S129>/Add'
         *  Sum: '<S129>/Sum'
         */
        if (FMS_B.Compare) {
          rtb_y_md = FMS_PARAM.LAND_LOCK_THRO;
        } else {
          if (FMS_U.Pilot_Cmd.stick_throttle + 1.0F > 2.0F) {
            /* Saturate: '<S129>/Saturation' */
            rtb_a_l = 2.0F;
          } else if (FMS_U.Pilot_Cmd.stick_throttle + 1.0F < 0.0F) {
            /* Saturate: '<S129>/Saturation' */
            rtb_a_l = 0.0F;
          } else {
            /* Saturate: '<S129>/Saturation' incorporates:
             *  Constant: '<S129>/Constant4'
             *  Inport: '<Root>/Pilot_Cmd'
             *  SignalConversion: '<S30>/Signal Copy2'
             *  Sum: '<S129>/Sum'
             */
            rtb_a_l = FMS_U.Pilot_Cmd.stick_throttle + 1.0F;
          }

          rtb_y_md = (uint16_T)((uint16_T)fmodf(floorf(500.0F * rtb_a_l),
            65536.0F) + 1000U);
        }

        /* End of Switch: '<S129>/Switch' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S43>/Bus Assignment'
         *  Constant: '<S43>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S43>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  DataTypeConversion: '<S127>/Data Type Conversion'
         *  DiscreteIntegrator: '<S132>/Integrator1'
         *  DiscreteIntegrator: '<S133>/Integrator1'
         *  Outport: '<Root>/FMS_Out'
         *  Product: '<S127>/Multiply'
         *  Product: '<S127>/Multiply1'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_ba;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_k;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_o;
        FMS_Y.FMS_Out.phi_cmd = (real32_T)rtb_FixPtRelationalOperator_me *
          FMS_DW.Integrator1_DSTATE_l;
        FMS_Y.FMS_Out.theta_cmd = (real32_T)rtb_FixPtRelationalOperator_me *
          FMS_DW.Integrator1_DSTATE_h;

        /* Saturate: '<S136>/Saturation' */
        if (FMS_B.Merge_j > FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S43>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_PARAM.YAW_RATE_LIM;
        } else if (FMS_B.Merge_j < -FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S43>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -FMS_PARAM.YAW_RATE_LIM;
        } else {
          /* BusAssignment: '<S43>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_B.Merge_j;
        }

        /* End of Saturate: '<S136>/Saturation' */

        /* BusAssignment: '<S43>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.throttle_cmd = rtb_y_md;

        /* Product: '<S135>/Multiply1' incorporates:
         *  Constant: '<S135>/const1'
         *  DiscreteIntegrator: '<S133>/Integrator'
         */
        rtb_Add4_e5 = FMS_DW.Integrator_DSTATE_a * 0.04F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S131>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_pitch > FMS_PARAM.PITCH_DZ) {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - FMS_PARAM.PITCH_DZ;
        } else if (FMS_U.Pilot_Cmd.stick_pitch >= -FMS_PARAM.PITCH_DZ) {
          rtb_a_l = 0.0F;
        } else {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - (-FMS_PARAM.PITCH_DZ);
        }

        /* End of DeadZone: '<S131>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S135>/Add' incorporates:
         *  DiscreteIntegrator: '<S133>/Integrator1'
         *  Gain: '<S127>/Gain1'
         *  Gain: '<S131>/Gain'
         *  Sum: '<S133>/Subtract'
         */
        rtb_a_l = (FMS_DW.Integrator1_DSTATE_h - 1.0F / (1.0F -
                    FMS_PARAM.PITCH_DZ) * rtb_a_l * -FMS_PARAM.ROLL_PITCH_LIM) +
          rtb_Add4_e5;

        /* Signum: '<S135>/Sign' */
        if (rtb_a_l < 0.0F) {
          rtb_Sqrt_b = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          rtb_Sqrt_b = rtb_a_l;
        }

        /* End of Signum: '<S135>/Sign' */

        /* Sum: '<S135>/Add2' incorporates:
         *  Abs: '<S135>/Abs'
         *  Gain: '<S135>/Gain'
         *  Gain: '<S135>/Gain1'
         *  Product: '<S135>/Multiply2'
         *  Product: '<S135>/Multiply3'
         *  Sqrt: '<S135>/Sqrt'
         *  Sum: '<S135>/Add1'
         *  Sum: '<S135>/Subtract'
         */
        rtb_Sqrt_b = (sqrtf((8.0F * fabsf(rtb_a_l) + FMS_ConstB.d_l) *
                            FMS_ConstB.d_l) - FMS_ConstB.d_l) * 0.5F *
          rtb_Sqrt_b + rtb_Add4_e5;

        /* Sum: '<S135>/Add4' */
        rtb_Subtract3_o = (rtb_a_l - rtb_Sqrt_b) + rtb_Add4_e5;

        /* Sum: '<S135>/Add3' */
        rtb_Add3_c = rtb_a_l + FMS_ConstB.d_l;

        /* Sum: '<S135>/Subtract1' */
        rtb_a_l -= FMS_ConstB.d_l;

        /* Signum: '<S135>/Sign1' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S135>/Sign1' */

        /* Signum: '<S135>/Sign2' */
        if (rtb_a_l < 0.0F) {
          rtb_a_l = -1.0F;
        } else {
          if (rtb_a_l > 0.0F) {
            rtb_a_l = 1.0F;
          }
        }

        /* End of Signum: '<S135>/Sign2' */

        /* Sum: '<S135>/Add5' incorporates:
         *  Gain: '<S135>/Gain2'
         *  Product: '<S135>/Multiply4'
         *  Sum: '<S135>/Subtract2'
         */
        rtb_Sqrt_b += (rtb_Add3_c - rtb_a_l) * 0.5F * rtb_Subtract3_o;

        /* Sum: '<S135>/Add6' */
        rtb_Add3_c = rtb_Sqrt_b + FMS_ConstB.d_l;

        /* Sum: '<S135>/Subtract3' */
        rtb_Subtract3_o = rtb_Sqrt_b - FMS_ConstB.d_l;

        /* Product: '<S135>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_Sqrt_b / FMS_ConstB.d_l;

        /* Signum: '<S135>/Sign5' incorporates:
         *  Signum: '<S135>/Sign6'
         */
        if (rtb_Sqrt_b < 0.0F) {
          rtb_MathFunction_iq_idx_1 = -1.0F;

          /* Signum: '<S135>/Sign6' */
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else if (rtb_Sqrt_b > 0.0F) {
          rtb_MathFunction_iq_idx_1 = 1.0F;

          /* Signum: '<S135>/Sign6' */
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          rtb_MathFunction_iq_idx_1 = rtb_Sqrt_b;

          /* Signum: '<S135>/Sign6' */
          rtb_MathFunction_f_idx_0 = rtb_Sqrt_b;
        }

        /* End of Signum: '<S135>/Sign5' */

        /* Product: '<S134>/Multiply1' incorporates:
         *  Constant: '<S134>/const1'
         *  DiscreteIntegrator: '<S132>/Integrator'
         */
        rtb_Add4_e5 = FMS_DW.Integrator_DSTATE_c * 0.04F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S130>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_roll > FMS_PARAM.ROLL_DZ) {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - FMS_PARAM.ROLL_DZ;
        } else if (FMS_U.Pilot_Cmd.stick_roll >= -FMS_PARAM.ROLL_DZ) {
          rtb_a_l = 0.0F;
        } else {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - (-FMS_PARAM.ROLL_DZ);
        }

        /* End of DeadZone: '<S130>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S134>/Add' incorporates:
         *  DiscreteIntegrator: '<S132>/Integrator1'
         *  Gain: '<S127>/Gain'
         *  Gain: '<S130>/Gain'
         *  Sum: '<S132>/Subtract'
         */
        rtb_a_l = (FMS_DW.Integrator1_DSTATE_l - 1.0F / (1.0F -
                    FMS_PARAM.ROLL_DZ) * rtb_a_l * FMS_PARAM.ROLL_PITCH_LIM) +
          rtb_Add4_e5;

        /* Signum: '<S134>/Sign' */
        if (rtb_a_l < 0.0F) {
          rtb_Sqrt_b = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          rtb_Sqrt_b = rtb_a_l;
        }

        /* End of Signum: '<S134>/Sign' */

        /* Sum: '<S134>/Add2' incorporates:
         *  Abs: '<S134>/Abs'
         *  Gain: '<S134>/Gain'
         *  Gain: '<S134>/Gain1'
         *  Product: '<S134>/Multiply2'
         *  Product: '<S134>/Multiply3'
         *  Sqrt: '<S134>/Sqrt'
         *  Sum: '<S134>/Add1'
         *  Sum: '<S134>/Subtract'
         */
        rtb_Sqrt_b = (sqrtf((8.0F * fabsf(rtb_a_l) + FMS_ConstB.d_h) *
                            FMS_ConstB.d_h) - FMS_ConstB.d_h) * 0.5F *
          rtb_Sqrt_b + rtb_Add4_e5;

        /* Sum: '<S134>/Add4' */
        rtb_MathFunction_f_idx_2 = (rtb_a_l - rtb_Sqrt_b) + rtb_Add4_e5;

        /* Sum: '<S134>/Add3' */
        rtb_Add4_e5 = rtb_a_l + FMS_ConstB.d_h;

        /* Sum: '<S134>/Subtract1' */
        rtb_a_l -= FMS_ConstB.d_h;

        /* Signum: '<S134>/Sign1' */
        if (rtb_Add4_e5 < 0.0F) {
          rtb_Add4_e5 = -1.0F;
        } else {
          if (rtb_Add4_e5 > 0.0F) {
            rtb_Add4_e5 = 1.0F;
          }
        }

        /* End of Signum: '<S134>/Sign1' */

        /* Signum: '<S134>/Sign2' */
        if (rtb_a_l < 0.0F) {
          rtb_a_l = -1.0F;
        } else {
          if (rtb_a_l > 0.0F) {
            rtb_a_l = 1.0F;
          }
        }

        /* End of Signum: '<S134>/Sign2' */

        /* Sum: '<S134>/Add5' incorporates:
         *  Gain: '<S134>/Gain2'
         *  Product: '<S134>/Multiply4'
         *  Sum: '<S134>/Subtract2'
         */
        rtb_Sqrt_b += (rtb_Add4_e5 - rtb_a_l) * 0.5F * rtb_MathFunction_f_idx_2;

        /* Update for DiscreteIntegrator: '<S132>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S132>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_l += 0.004F * FMS_DW.Integrator_DSTATE_c;

        /* Update for DiscreteIntegrator: '<S133>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S133>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_h += 0.004F * FMS_DW.Integrator_DSTATE_a;

        /* Signum: '<S135>/Sign3' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S135>/Sign3' */

        /* Signum: '<S135>/Sign4' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S135>/Sign4' */

        /* Update for DiscreteIntegrator: '<S133>/Integrator' incorporates:
         *  Constant: '<S135>/const'
         *  Gain: '<S135>/Gain3'
         *  Product: '<S135>/Multiply5'
         *  Product: '<S135>/Multiply6'
         *  Sum: '<S135>/Subtract4'
         *  Sum: '<S135>/Subtract5'
         *  Sum: '<S135>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_a += ((rtb_MathFunction_iq_idx_0 -
          rtb_MathFunction_iq_idx_1) * FMS_ConstB.Gain4_j * ((rtb_Add3_c -
          rtb_Subtract3_o) * 0.5F) - rtb_MathFunction_f_idx_0 * 12.566371F) *
          0.004F;

        /* Sum: '<S134>/Subtract3' */
        rtb_Add3_c = rtb_Sqrt_b - FMS_ConstB.d_h;

        /* Sum: '<S134>/Add6' */
        rtb_Subtract3_o = rtb_Sqrt_b + FMS_ConstB.d_h;

        /* Signum: '<S134>/Sign5' incorporates:
         *  Signum: '<S134>/Sign6'
         */
        if (rtb_Sqrt_b < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;

          /* Signum: '<S134>/Sign6' */
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_Sqrt_b > 0.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;

          /* Signum: '<S134>/Sign6' */
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_MathFunction_f_idx_0 = rtb_Sqrt_b;

          /* Signum: '<S134>/Sign6' */
          rtb_Add4_e5 = rtb_Sqrt_b;
        }

        /* End of Signum: '<S134>/Sign5' */

        /* Signum: '<S134>/Sign3' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S134>/Sign3' */

        /* Signum: '<S134>/Sign4' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S134>/Sign4' */

        /* Update for DiscreteIntegrator: '<S132>/Integrator' incorporates:
         *  Constant: '<S134>/const'
         *  Gain: '<S134>/Gain3'
         *  Product: '<S134>/Divide'
         *  Product: '<S134>/Multiply5'
         *  Product: '<S134>/Multiply6'
         *  Sum: '<S134>/Subtract4'
         *  Sum: '<S134>/Subtract5'
         *  Sum: '<S134>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_c += ((rtb_Sqrt_b / FMS_ConstB.d_h -
          rtb_MathFunction_f_idx_0) * FMS_ConstB.Gain4_n * ((rtb_Subtract3_o -
          rtb_Add3_c) * 0.5F) - rtb_Add4_e5 * 12.566371F) * 0.004F;

        /* End of Outputs for SubSystem: '<S35>/Stabilize' */
        break;

       case 2:
        if (FMS_DW.SwitchCase_ActiveSubsystem_f != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S35>/Altitude' incorporates:
           *  ActionPort: '<S41>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S35>/Switch Case' incorporates:
           *  DiscreteIntegrator: '<S64>/Integrator'
           *  DiscreteIntegrator: '<S64>/Integrator1'
           *  DiscreteIntegrator: '<S65>/Integrator'
           *  DiscreteIntegrator: '<S65>/Integrator1'
           */
          FMS_DW.Integrator1_DSTATE_f = 0.0F;
          FMS_DW.Integrator1_DSTATE_o = 0.0F;
          FMS_DW.Integrator_DSTATE_b = 0.0F;
          FMS_DW.Integrator_DSTATE_bp = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S35>/Altitude' */

          /* SystemReset for IfAction SubSystem: '<S35>/Altitude' incorporates:
           *  ActionPort: '<S41>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S35>/Switch Case' incorporates:
           *  Chart: '<S51>/Motion Status'
           *  Chart: '<S69>/Motion State'
           */
          FMS_DW.temporalCounter_i1_a = 0U;
          FMS_DW.is_active_c17_FMS = 0U;
          FMS_DW.is_c17_FMS = FMS_IN_NO_ACTIVE_CHILD_h;
          FMS_MotionState_j_Reset(&FMS_DW.sf_MotionState_k);

          /* End of SystemReset for SubSystem: '<S35>/Altitude' */
        }

        /* Outputs for IfAction SubSystem: '<S35>/Altitude' incorporates:
         *  ActionPort: '<S41>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* RelationalOperator: '<S60>/Compare' incorporates:
         *  Abs: '<S51>/Abs1'
         *  Constant: '<S60>/Constant'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtb_FixPtRelationalOperator_me = (fabsf(FMS_U.Pilot_Cmd.stick_throttle) >
          FMS_PARAM.THROTTLE_DZ);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Chart: '<S51>/Motion Status' incorporates:
         *  Abs: '<S51>/Abs'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.temporalCounter_i1_a < 511U) {
          FMS_DW.temporalCounter_i1_a++;
        }

        if (FMS_DW.is_active_c17_FMS == 0U) {
          FMS_DW.is_active_c17_FMS = 1U;
          FMS_DW.is_c17_FMS = FMS_IN_Move_n;
          rtb_state_c = MotionState_Move;
        } else {
          switch (FMS_DW.is_c17_FMS) {
           case FMS_IN_Brake_o:
            rtb_state_c = MotionState_Brake;

            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            if ((fabsf(FMS_U.INS_Out.vd) <= 0.15) ||
                (FMS_DW.temporalCounter_i1_a >= 375U)) {
              FMS_DW.is_c17_FMS = FMS_IN_Hold_d;
              rtb_state_c = MotionState_Hold;
            } else {
              if (rtb_FixPtRelationalOperator_me) {
                FMS_DW.is_c17_FMS = FMS_IN_Move_n;
                rtb_state_c = MotionState_Move;
              }
            }

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            break;

           case FMS_IN_Hold_d:
            rtb_state_c = MotionState_Hold;
            if (rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c17_FMS = FMS_IN_Move_n;
              rtb_state_c = MotionState_Move;
            }
            break;

           default:
            rtb_state_c = MotionState_Move;
            if (!rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c17_FMS = FMS_IN_Brake_o;
              FMS_DW.temporalCounter_i1_a = 0U;
              rtb_state_c = MotionState_Brake;
            }
            break;
          }
        }

        /* End of Chart: '<S51>/Motion Status' */

        /* SwitchCase: '<S50>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_m;
        FMS_DW.SwitchCase_ActiveSubsystem_m = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_m = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_m = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_m = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_m) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_m != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S50>/Hold Control' incorporates:
             *  ActionPort: '<S53>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S50>/Switch Case' */
            FMS_HoldControl_Reset(&FMS_DW.HoldControl_k2);

            /* End of SystemReset for SubSystem: '<S50>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S50>/Hold Control' incorporates:
           *  ActionPort: '<S53>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl(FMS_U.INS_Out.h_R, &FMS_B.Merge_l,
                          &FMS_DW.HoldControl_k2);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S50>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S50>/Brake Control' incorporates:
           *  ActionPort: '<S52>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_l);

          /* End of Outputs for SubSystem: '<S50>/Brake Control' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S50>/Move Control' incorporates:
           *  ActionPort: '<S54>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_l(FMS_U.Pilot_Cmd.stick_throttle, FMS_U.INS_Out.h_AGL,
                            FMS_U.INS_Out.flag, &FMS_B.Merge_l);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S50>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S50>/Switch Case' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Switch: '<S47>/Switch' incorporates:
         *  Constant: '<S47>/Constant'
         *  Saturate: '<S50>/Saturation1'
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy7Inport1'
         */
        if (FMS_B.Compare) {
          rtb_Add3_c = 0.5F;
        } else if (FMS_B.Merge_l > FMS_PARAM.VEL_Z_LIM) {
          /* Saturate: '<S50>/Saturation1' */
          rtb_Add3_c = FMS_PARAM.VEL_Z_LIM;
        } else if (FMS_B.Merge_l < -FMS_PARAM.VEL_Z_LIM) {
          /* Saturate: '<S50>/Saturation1' */
          rtb_Add3_c = -FMS_PARAM.VEL_Z_LIM;
        } else {
          /* Saturate: '<S50>/Saturation1' */
          rtb_Add3_c = FMS_B.Merge_l;
        }

        /* End of Switch: '<S47>/Switch' */

        /* Logic: '<S48>/Logical Operator' incorporates:
         *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy7Inport1'
         */
        rtb_FixPtRelationalOperator_me = !FMS_B.Compare;

        /* Chart: '<S69>/Motion State' incorporates:
         *  Abs: '<S69>/Abs'
         *  Abs: '<S69>/Abs1'
         *  Constant: '<S79>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  RelationalOperator: '<S79>/Compare'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        FMS_MotionState_e(fabsf(FMS_U.Pilot_Cmd.stick_yaw) > FMS_PARAM.YAW_DZ,
                          fabsf(FMS_U.INS_Out.r), &rtb_state_c,
                          &FMS_DW.sf_MotionState_k);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SwitchCase: '<S68>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_h;
        FMS_DW.SwitchCase_ActiveSubsystem_h = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_h = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_h = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_h = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_h) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_h != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S68>/Hold Control' incorporates:
             *  ActionPort: '<S71>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S68>/Switch Case' */
            FMS_HoldControl_kp_Reset(&FMS_DW.HoldControl_o);

            /* End of SystemReset for SubSystem: '<S68>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S68>/Hold Control' incorporates:
           *  ActionPort: '<S71>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_k(FMS_U.INS_Out.psi, &FMS_B.Merge_m,
                            &FMS_DW.HoldControl_o);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S68>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S68>/Brake Control' incorporates:
           *  ActionPort: '<S70>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_m);

          /* End of Outputs for SubSystem: '<S68>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_h != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S68>/Move Control' incorporates:
             *  ActionPort: '<S72>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S68>/Switch Case' */
            FMS_MoveControl_l_Reset(&FMS_DW.MoveControl_cr);

            /* End of SystemReset for SubSystem: '<S68>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S68>/Move Control' incorporates:
           *  ActionPort: '<S72>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_b(FMS_U.Pilot_Cmd.stick_yaw, &FMS_B.Merge_m,
                            &FMS_ConstB.MoveControl_cr, &FMS_DW.MoveControl_cr);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S68>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S68>/Switch Case' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S41>/Bus Assignment'
         *  Constant: '<S41>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S41>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  DataTypeConversion: '<S48>/Data Type Conversion'
         *  DiscreteIntegrator: '<S64>/Integrator1'
         *  DiscreteIntegrator: '<S65>/Integrator1'
         *  Outport: '<Root>/FMS_Out'
         *  Product: '<S48>/Multiply'
         *  Product: '<S48>/Multiply1'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion1_d;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion2_n;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion3;
        FMS_Y.FMS_Out.phi_cmd = (real32_T)rtb_FixPtRelationalOperator_me *
          FMS_DW.Integrator1_DSTATE_f;
        FMS_Y.FMS_Out.theta_cmd = (real32_T)rtb_FixPtRelationalOperator_me *
          FMS_DW.Integrator1_DSTATE_o;

        /* Saturate: '<S68>/Saturation' */
        if (FMS_B.Merge_m > FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S41>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_PARAM.YAW_RATE_LIM;
        } else if (FMS_B.Merge_m < -FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S41>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -FMS_PARAM.YAW_RATE_LIM;
        } else {
          /* BusAssignment: '<S41>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_B.Merge_m;
        }

        /* End of Saturate: '<S68>/Saturation' */

        /* BusAssignment: '<S41>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.w_cmd = rtb_Add3_c;

        /* Product: '<S67>/Multiply1' incorporates:
         *  Constant: '<S67>/const1'
         *  DiscreteIntegrator: '<S65>/Integrator'
         */
        rtb_Add4_e5 = FMS_DW.Integrator_DSTATE_b * 0.04F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S63>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_pitch > FMS_PARAM.PITCH_DZ) {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - FMS_PARAM.PITCH_DZ;
        } else if (FMS_U.Pilot_Cmd.stick_pitch >= -FMS_PARAM.PITCH_DZ) {
          rtb_a_l = 0.0F;
        } else {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - (-FMS_PARAM.PITCH_DZ);
        }

        /* End of DeadZone: '<S63>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S67>/Add' incorporates:
         *  DiscreteIntegrator: '<S65>/Integrator1'
         *  Gain: '<S48>/Gain1'
         *  Gain: '<S63>/Gain'
         *  Sum: '<S65>/Subtract'
         */
        rtb_a_l = (FMS_DW.Integrator1_DSTATE_o - 1.0F / (1.0F -
                    FMS_PARAM.PITCH_DZ) * rtb_a_l * -FMS_PARAM.ROLL_PITCH_LIM) +
          rtb_Add4_e5;

        /* Signum: '<S67>/Sign' */
        if (rtb_a_l < 0.0F) {
          rtb_Sqrt_b = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          rtb_Sqrt_b = rtb_a_l;
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
        rtb_Sqrt_b = (sqrtf((8.0F * fabsf(rtb_a_l) + FMS_ConstB.d_c) *
                            FMS_ConstB.d_c) - FMS_ConstB.d_c) * 0.5F *
          rtb_Sqrt_b + rtb_Add4_e5;

        /* Sum: '<S67>/Add4' */
        rtb_Subtract3_o = (rtb_a_l - rtb_Sqrt_b) + rtb_Add4_e5;

        /* Sum: '<S67>/Add3' */
        rtb_Add3_c = rtb_a_l + FMS_ConstB.d_c;

        /* Sum: '<S67>/Subtract1' */
        rtb_a_l -= FMS_ConstB.d_c;

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
        if (rtb_a_l < 0.0F) {
          rtb_a_l = -1.0F;
        } else {
          if (rtb_a_l > 0.0F) {
            rtb_a_l = 1.0F;
          }
        }

        /* End of Signum: '<S67>/Sign2' */

        /* Sum: '<S67>/Add5' incorporates:
         *  Gain: '<S67>/Gain2'
         *  Product: '<S67>/Multiply4'
         *  Sum: '<S67>/Subtract2'
         */
        rtb_Sqrt_b += (rtb_Add3_c - rtb_a_l) * 0.5F * rtb_Subtract3_o;

        /* Sum: '<S67>/Add6' */
        rtb_Add3_c = rtb_Sqrt_b + FMS_ConstB.d_c;

        /* Sum: '<S67>/Subtract3' */
        rtb_Subtract3_o = rtb_Sqrt_b - FMS_ConstB.d_c;

        /* Product: '<S67>/Divide' */
        rtb_MathFunction_iq_idx_0 = rtb_Sqrt_b / FMS_ConstB.d_c;

        /* Signum: '<S67>/Sign5' incorporates:
         *  Signum: '<S67>/Sign6'
         */
        if (rtb_Sqrt_b < 0.0F) {
          rtb_MathFunction_iq_idx_1 = -1.0F;

          /* Signum: '<S67>/Sign6' */
          rtb_MathFunction_f_idx_0 = -1.0F;
        } else if (rtb_Sqrt_b > 0.0F) {
          rtb_MathFunction_iq_idx_1 = 1.0F;

          /* Signum: '<S67>/Sign6' */
          rtb_MathFunction_f_idx_0 = 1.0F;
        } else {
          rtb_MathFunction_iq_idx_1 = rtb_Sqrt_b;

          /* Signum: '<S67>/Sign6' */
          rtb_MathFunction_f_idx_0 = rtb_Sqrt_b;
        }

        /* End of Signum: '<S67>/Sign5' */

        /* Product: '<S66>/Multiply1' incorporates:
         *  Constant: '<S66>/const1'
         *  DiscreteIntegrator: '<S64>/Integrator'
         */
        rtb_Add4_e5 = FMS_DW.Integrator_DSTATE_bp * 0.04F;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S62>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_roll > FMS_PARAM.ROLL_DZ) {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - FMS_PARAM.ROLL_DZ;
        } else if (FMS_U.Pilot_Cmd.stick_roll >= -FMS_PARAM.ROLL_DZ) {
          rtb_a_l = 0.0F;
        } else {
          rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - (-FMS_PARAM.ROLL_DZ);
        }

        /* End of DeadZone: '<S62>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Sum: '<S66>/Add' incorporates:
         *  DiscreteIntegrator: '<S64>/Integrator1'
         *  Gain: '<S48>/Gain'
         *  Gain: '<S62>/Gain'
         *  Sum: '<S64>/Subtract'
         */
        rtb_a_l = (FMS_DW.Integrator1_DSTATE_f - 1.0F / (1.0F -
                    FMS_PARAM.ROLL_DZ) * rtb_a_l * FMS_PARAM.ROLL_PITCH_LIM) +
          rtb_Add4_e5;

        /* Signum: '<S66>/Sign' */
        if (rtb_a_l < 0.0F) {
          rtb_Sqrt_b = -1.0F;
        } else if (rtb_a_l > 0.0F) {
          rtb_Sqrt_b = 1.0F;
        } else {
          rtb_Sqrt_b = rtb_a_l;
        }

        /* End of Signum: '<S66>/Sign' */

        /* Sum: '<S66>/Add2' incorporates:
         *  Abs: '<S66>/Abs'
         *  Gain: '<S66>/Gain'
         *  Gain: '<S66>/Gain1'
         *  Product: '<S66>/Multiply2'
         *  Product: '<S66>/Multiply3'
         *  Sqrt: '<S66>/Sqrt'
         *  Sum: '<S66>/Add1'
         *  Sum: '<S66>/Subtract'
         */
        rtb_Sqrt_b = (sqrtf((8.0F * fabsf(rtb_a_l) + FMS_ConstB.d_e) *
                            FMS_ConstB.d_e) - FMS_ConstB.d_e) * 0.5F *
          rtb_Sqrt_b + rtb_Add4_e5;

        /* Sum: '<S66>/Add4' */
        rtb_MathFunction_f_idx_2 = (rtb_a_l - rtb_Sqrt_b) + rtb_Add4_e5;

        /* Sum: '<S66>/Add3' */
        rtb_Add4_e5 = rtb_a_l + FMS_ConstB.d_e;

        /* Sum: '<S66>/Subtract1' */
        rtb_a_l -= FMS_ConstB.d_e;

        /* Signum: '<S66>/Sign1' */
        if (rtb_Add4_e5 < 0.0F) {
          rtb_Add4_e5 = -1.0F;
        } else {
          if (rtb_Add4_e5 > 0.0F) {
            rtb_Add4_e5 = 1.0F;
          }
        }

        /* End of Signum: '<S66>/Sign1' */

        /* Signum: '<S66>/Sign2' */
        if (rtb_a_l < 0.0F) {
          rtb_a_l = -1.0F;
        } else {
          if (rtb_a_l > 0.0F) {
            rtb_a_l = 1.0F;
          }
        }

        /* End of Signum: '<S66>/Sign2' */

        /* Sum: '<S66>/Add5' incorporates:
         *  Gain: '<S66>/Gain2'
         *  Product: '<S66>/Multiply4'
         *  Sum: '<S66>/Subtract2'
         */
        rtb_Sqrt_b += (rtb_Add4_e5 - rtb_a_l) * 0.5F * rtb_MathFunction_f_idx_2;

        /* Update for DiscreteIntegrator: '<S64>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S64>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_f += 0.004F * FMS_DW.Integrator_DSTATE_bp;

        /* Update for DiscreteIntegrator: '<S65>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S65>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_o += 0.004F * FMS_DW.Integrator_DSTATE_b;

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
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S67>/Sign4' */

        /* Update for DiscreteIntegrator: '<S65>/Integrator' incorporates:
         *  Constant: '<S67>/const'
         *  Gain: '<S67>/Gain3'
         *  Product: '<S67>/Multiply5'
         *  Product: '<S67>/Multiply6'
         *  Sum: '<S67>/Subtract4'
         *  Sum: '<S67>/Subtract5'
         *  Sum: '<S67>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_b += ((rtb_MathFunction_iq_idx_0 -
          rtb_MathFunction_iq_idx_1) * FMS_ConstB.Gain4_m * ((rtb_Add3_c -
          rtb_Subtract3_o) * 0.5F) - rtb_MathFunction_f_idx_0 * 12.566371F) *
          0.004F;

        /* Sum: '<S66>/Subtract3' */
        rtb_Add3_c = rtb_Sqrt_b - FMS_ConstB.d_e;

        /* Sum: '<S66>/Add6' */
        rtb_Subtract3_o = rtb_Sqrt_b + FMS_ConstB.d_e;

        /* Signum: '<S66>/Sign5' incorporates:
         *  Signum: '<S66>/Sign6'
         */
        if (rtb_Sqrt_b < 0.0F) {
          rtb_MathFunction_f_idx_0 = -1.0F;

          /* Signum: '<S66>/Sign6' */
          rtb_Add4_e5 = -1.0F;
        } else if (rtb_Sqrt_b > 0.0F) {
          rtb_MathFunction_f_idx_0 = 1.0F;

          /* Signum: '<S66>/Sign6' */
          rtb_Add4_e5 = 1.0F;
        } else {
          rtb_MathFunction_f_idx_0 = rtb_Sqrt_b;

          /* Signum: '<S66>/Sign6' */
          rtb_Add4_e5 = rtb_Sqrt_b;
        }

        /* End of Signum: '<S66>/Sign5' */

        /* Signum: '<S66>/Sign3' */
        if (rtb_Subtract3_o < 0.0F) {
          rtb_Subtract3_o = -1.0F;
        } else {
          if (rtb_Subtract3_o > 0.0F) {
            rtb_Subtract3_o = 1.0F;
          }
        }

        /* End of Signum: '<S66>/Sign3' */

        /* Signum: '<S66>/Sign4' */
        if (rtb_Add3_c < 0.0F) {
          rtb_Add3_c = -1.0F;
        } else {
          if (rtb_Add3_c > 0.0F) {
            rtb_Add3_c = 1.0F;
          }
        }

        /* End of Signum: '<S66>/Sign4' */

        /* Update for DiscreteIntegrator: '<S64>/Integrator' incorporates:
         *  Constant: '<S66>/const'
         *  Gain: '<S66>/Gain3'
         *  Product: '<S66>/Divide'
         *  Product: '<S66>/Multiply5'
         *  Product: '<S66>/Multiply6'
         *  Sum: '<S66>/Subtract4'
         *  Sum: '<S66>/Subtract5'
         *  Sum: '<S66>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_bp += ((rtb_Sqrt_b / FMS_ConstB.d_e -
          rtb_MathFunction_f_idx_0) * FMS_ConstB.Gain4_d * ((rtb_Subtract3_o -
          rtb_Add3_c) * 0.5F) - rtb_Add4_e5 * 12.566371F) * 0.004F;

        /* End of Outputs for SubSystem: '<S35>/Altitude' */
        break;

       case 3:
        if (FMS_DW.SwitchCase_ActiveSubsystem_f != rtPrevAction) {
          /* SystemReset for IfAction SubSystem: '<S35>/Position' incorporates:
           *  ActionPort: '<S42>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S35>/Switch Case' incorporates:
           *  Chart: '<S110>/Motion State'
           *  Chart: '<S85>/Motion Status'
           *  Chart: '<S97>/Motion State'
           */
          FMS_DW.temporalCounter_i1_l = 0U;
          FMS_DW.is_active_c20_FMS = 0U;
          FMS_DW.is_c20_FMS = FMS_IN_NO_ACTIVE_CHILD_h;
          FMS_MotionState_j_Reset(&FMS_DW.sf_MotionState_j);
          FMS_DW.temporalCounter_i1_i = 0U;
          FMS_DW.is_active_c16_FMS = 0U;
          FMS_DW.is_c16_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

          /* End of SystemReset for SubSystem: '<S35>/Position' */
        }

        /* Outputs for IfAction SubSystem: '<S35>/Position' incorporates:
         *  ActionPort: '<S42>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* RelationalOperator: '<S94>/Compare' incorporates:
         *  Abs: '<S85>/Abs1'
         *  Constant: '<S94>/Constant'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtb_FixPtRelationalOperator_me = (fabsf(FMS_U.Pilot_Cmd.stick_throttle) >
          FMS_PARAM.THROTTLE_DZ);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Chart: '<S85>/Motion Status' incorporates:
         *  Abs: '<S85>/Abs'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        if (FMS_DW.temporalCounter_i1_l < 511U) {
          FMS_DW.temporalCounter_i1_l++;
        }

        if (FMS_DW.is_active_c20_FMS == 0U) {
          FMS_DW.is_active_c20_FMS = 1U;
          FMS_DW.is_c20_FMS = FMS_IN_Move_n;
          rtb_state_c = MotionState_Move;
        } else {
          switch (FMS_DW.is_c20_FMS) {
           case FMS_IN_Brake_o:
            rtb_state_c = MotionState_Brake;

            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            if ((fabsf(FMS_U.INS_Out.vd) <= 0.1) || (FMS_DW.temporalCounter_i1_l
                 >= 375U)) {
              FMS_DW.is_c20_FMS = FMS_IN_Hold_d;
              rtb_state_c = MotionState_Hold;
            } else {
              if (rtb_FixPtRelationalOperator_me) {
                FMS_DW.is_c20_FMS = FMS_IN_Move_n;
                rtb_state_c = MotionState_Move;
              }
            }

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            break;

           case FMS_IN_Hold_d:
            rtb_state_c = MotionState_Hold;
            if (rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c20_FMS = FMS_IN_Move_n;
              rtb_state_c = MotionState_Move;
            }
            break;

           default:
            rtb_state_c = MotionState_Move;
            if (!rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c20_FMS = FMS_IN_Brake_o;
              FMS_DW.temporalCounter_i1_l = 0U;
              rtb_state_c = MotionState_Brake;
            }
            break;
          }
        }

        /* End of Chart: '<S85>/Motion Status' */

        /* SwitchCase: '<S84>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_o;
        FMS_DW.SwitchCase_ActiveSubsystem_o = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_o = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_o = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_o = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_o) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_o != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S84>/Hold Control' incorporates:
             *  ActionPort: '<S87>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S84>/Switch Case' */
            FMS_HoldControl_Reset(&FMS_DW.HoldControl_p);

            /* End of SystemReset for SubSystem: '<S84>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S84>/Hold Control' incorporates:
           *  ActionPort: '<S87>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl(FMS_U.INS_Out.h_R, &FMS_B.Merge_k,
                          &FMS_DW.HoldControl_p);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S84>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S84>/Brake Control' incorporates:
           *  ActionPort: '<S86>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_k);

          /* End of Outputs for SubSystem: '<S84>/Brake Control' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S84>/Move Control' incorporates:
           *  ActionPort: '<S88>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_l(FMS_U.Pilot_Cmd.stick_throttle, FMS_U.INS_Out.h_AGL,
                            FMS_U.INS_Out.flag, &FMS_B.Merge_k);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S84>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S84>/Switch Case' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Chart: '<S97>/Motion State' incorporates:
         *  Abs: '<S97>/Abs'
         *  Abs: '<S97>/Abs1'
         *  Constant: '<S107>/Constant'
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  RelationalOperator: '<S107>/Compare'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        FMS_MotionState_e(fabsf(FMS_U.Pilot_Cmd.stick_yaw) > FMS_PARAM.YAW_DZ,
                          fabsf(FMS_U.INS_Out.r), &rtb_state_c,
                          &FMS_DW.sf_MotionState_j);

        /* Logic: '<S110>/Logical Operator' incorporates:
         *  Abs: '<S110>/Abs1'
         *  Abs: '<S110>/Abs2'
         *  Constant: '<S124>/Constant'
         *  Constant: '<S125>/Constant'
         *  Inport: '<Root>/Pilot_Cmd'
         *  RelationalOperator: '<S124>/Compare'
         *  RelationalOperator: '<S125>/Compare'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtb_FixPtRelationalOperator_me = ((fabsf(FMS_U.Pilot_Cmd.stick_pitch) >
          FMS_PARAM.PITCH_DZ) || (fabsf(FMS_U.Pilot_Cmd.stick_roll) >
          FMS_PARAM.ROLL_DZ));

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Chart: '<S110>/Motion State' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Math: '<S110>/Square'
         *  Math: '<S110>/Square1'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  Sqrt: '<S110>/Sqrt'
         *  Sum: '<S110>/Add'
         */
        if (FMS_DW.temporalCounter_i1_i < 1023U) {
          FMS_DW.temporalCounter_i1_i++;
        }

        if (FMS_DW.is_active_c16_FMS == 0U) {
          FMS_DW.is_active_c16_FMS = 1U;
          FMS_DW.is_c16_FMS = FMS_IN_Move_n;
          rtb_state_ki = MotionState_Move;
        } else {
          switch (FMS_DW.is_c16_FMS) {
           case FMS_IN_Brake_o:
            rtb_state_ki = MotionState_Brake;

            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            if ((sqrtf(FMS_U.INS_Out.vn * FMS_U.INS_Out.vn + FMS_U.INS_Out.ve *
                       FMS_U.INS_Out.ve) <= 0.2) || (FMS_DW.temporalCounter_i1_i
                 >= 625U)) {
              FMS_DW.is_c16_FMS = FMS_IN_Hold_d;
              rtb_state_ki = MotionState_Hold;
            } else {
              if (rtb_FixPtRelationalOperator_me) {
                FMS_DW.is_c16_FMS = FMS_IN_Move_n;
                rtb_state_ki = MotionState_Move;
              }
            }

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            break;

           case FMS_IN_Hold_d:
            rtb_state_ki = MotionState_Hold;
            if (rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c16_FMS = FMS_IN_Move_n;
              rtb_state_ki = MotionState_Move;
            }
            break;

           default:
            rtb_state_ki = MotionState_Move;
            if (!rtb_FixPtRelationalOperator_me) {
              FMS_DW.is_c16_FMS = FMS_IN_Brake_o;
              FMS_DW.temporalCounter_i1_i = 0U;
              rtb_state_ki = MotionState_Brake;
            }
            break;
          }
        }

        /* End of Chart: '<S110>/Motion State' */

        /* SwitchCase: '<S109>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S30>/Signal Copy1'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_l;
        FMS_DW.SwitchCase_ActiveSubsystem_l = -1;
        switch (rtb_state_ki) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_l = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_l = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_l = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_l) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_l != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S109>/Hold Control' incorporates:
             *  ActionPort: '<S112>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S109>/Switch Case' */
            FMS_HoldControl_k_Reset(&FMS_DW.HoldControl_at);

            /* End of SystemReset for SubSystem: '<S109>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S109>/Hold Control' incorporates:
           *  ActionPort: '<S112>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_m(FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                            FMS_U.INS_Out.psi, FMS_B.Merge,
                            &FMS_ConstB.HoldControl_at, &FMS_DW.HoldControl_at);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S109>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S109>/Brake Control' incorporates:
           *  ActionPort: '<S111>/Action Port'
           */
          FMS_BrakeControl_h(FMS_B.Merge);

          /* End of Outputs for SubSystem: '<S109>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_l != rtPrevAction) {
            /* InitializeConditions for IfAction SubSystem: '<S109>/Move Control' incorporates:
             *  ActionPort: '<S113>/Action Port'
             */
            /* InitializeConditions for SwitchCase: '<S109>/Switch Case' incorporates:
             *  DiscreteIntegrator: '<S120>/Integrator'
             *  DiscreteIntegrator: '<S120>/Integrator1'
             */
            FMS_DW.Integrator1_DSTATE[0] = 0.0F;
            FMS_DW.Integrator_DSTATE[0] = 0.0F;
            FMS_DW.Integrator1_DSTATE[1] = 0.0F;
            FMS_DW.Integrator_DSTATE[1] = 0.0F;

            /* End of InitializeConditions for SubSystem: '<S109>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S109>/Move Control' incorporates:
           *  ActionPort: '<S113>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* DeadZone: '<S118>/Dead Zone' incorporates:
           *  Inport: '<Root>/Pilot_Cmd'
           *  SignalConversion: '<S30>/Signal Copy2'
           */
          if (FMS_U.Pilot_Cmd.stick_pitch > FMS_PARAM.PITCH_DZ) {
            rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - FMS_PARAM.PITCH_DZ;
          } else if (FMS_U.Pilot_Cmd.stick_pitch >= -FMS_PARAM.PITCH_DZ) {
            rtb_a_l = 0.0F;
          } else {
            rtb_a_l = FMS_U.Pilot_Cmd.stick_pitch - (-FMS_PARAM.PITCH_DZ);
          }

          /* End of DeadZone: '<S118>/Dead Zone' */
          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Gain: '<S118>/Gain' */
          rtb_Add3_c = 1.0F / (1.0F - FMS_PARAM.PITCH_DZ) * rtb_a_l;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* DeadZone: '<S119>/Dead Zone' incorporates:
           *  Inport: '<Root>/Pilot_Cmd'
           *  SignalConversion: '<S30>/Signal Copy2'
           */
          if (FMS_U.Pilot_Cmd.stick_roll > FMS_PARAM.ROLL_DZ) {
            rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - FMS_PARAM.ROLL_DZ;
          } else if (FMS_U.Pilot_Cmd.stick_roll >= -FMS_PARAM.ROLL_DZ) {
            rtb_a_l = 0.0F;
          } else {
            rtb_a_l = FMS_U.Pilot_Cmd.stick_roll - (-FMS_PARAM.ROLL_DZ);
          }

          /* End of DeadZone: '<S119>/Dead Zone' */
          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Gain: '<S119>/Gain' */
          rtb_Add4_e5 = 1.0F / (1.0F - FMS_PARAM.ROLL_DZ) * rtb_a_l;

          /* Sum: '<S121>/Sum of Elements' incorporates:
           *  Math: '<S121>/Square'
           *  SignalConversion: '<S121>/TmpSignal ConversionAtSquareInport1'
           *  Sum: '<S122>/Sum of Elements'
           *  Switch: '<S113>/Switch'
           */
          rtb_a_l = rtb_Add3_c * rtb_Add3_c + rtb_Add4_e5 * rtb_Add4_e5;

          /* Switch: '<S113>/Switch' incorporates:
           *  Constant: '<S117>/Constant'
           *  Product: '<S122>/Divide'
           *  RelationalOperator: '<S117>/Compare'
           *  Sqrt: '<S121>/Sqrt'
           *  Sum: '<S121>/Sum of Elements'
           */
          if (sqrtf(rtb_a_l) > 1.0F) {
            /* Math: '<S122>/Math Function1'
             *
             * About '<S122>/Math Function1':
             *  Operator: sqrt
             */
            if (rtb_a_l < 0.0F) {
              rtb_Subtract3_o = -sqrtf(fabsf(rtb_a_l));
            } else {
              rtb_Subtract3_o = sqrtf(rtb_a_l);
            }

            /* End of Math: '<S122>/Math Function1' */

            /* Switch: '<S122>/Switch' incorporates:
             *  Constant: '<S122>/Constant'
             *  Product: '<S122>/Product'
             */
            if (rtb_Subtract3_o > 0.0F) {
              rtb_Switch_dw[0] = rtb_Add3_c;
              rtb_Switch_dw[1] = rtb_Add4_e5;
              rtb_Switch_dw[2] = rtb_Subtract3_o;
            } else {
              rtb_Switch_dw[0] = 0.0F;
              rtb_Switch_dw[1] = 0.0F;
              rtb_Switch_dw[2] = 1.0F;
            }

            /* End of Switch: '<S122>/Switch' */
            rtb_Add3_c = rtb_Switch_dw[0] / rtb_Switch_dw[2];
            rtb_Add4_e5 = rtb_Switch_dw[1] / rtb_Switch_dw[2];
          }

          /* Product: '<S123>/Multiply1' incorporates:
           *  Constant: '<S123>/const1'
           *  DiscreteIntegrator: '<S120>/Integrator'
           */
          rtb_MathFunction_f_idx_0 = FMS_DW.Integrator_DSTATE[0] * 0.05F;

          /* Sum: '<S123>/Add' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator1'
           *  Gain: '<S113>/Gain6'
           *  Sum: '<S120>/Subtract'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_DW.Integrator1_DSTATE[0] -
            FMS_PARAM.VEL_XY_LIM * rtb_Add3_c) + rtb_MathFunction_f_idx_0;

          /* Signum: '<S123>/Sign' */
          if (rtb_MathFunction_iq_idx_0 < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else if (rtb_MathFunction_iq_idx_0 > 0.0F) {
            rtb_Add3_c = 1.0F;
          } else {
            rtb_Add3_c = rtb_MathFunction_iq_idx_0;
          }

          /* Sum: '<S123>/Add2' incorporates:
           *  Abs: '<S123>/Abs'
           *  Gain: '<S123>/Gain'
           *  Gain: '<S123>/Gain1'
           *  Product: '<S123>/Multiply2'
           *  Product: '<S123>/Multiply3'
           *  Sqrt: '<S123>/Sqrt'
           *  Sum: '<S123>/Add1'
           *  Sum: '<S123>/Subtract'
           */
          rtb_Rem_p = (sqrtf((8.0F * fabsf(rtb_MathFunction_iq_idx_0) +
                              FMS_ConstB.d) * FMS_ConstB.d) - FMS_ConstB.d) *
            0.5F * rtb_Add3_c + rtb_MathFunction_f_idx_0;

          /* Sum: '<S123>/Add4' */
          rtb_MathFunction_f_idx_0 += rtb_MathFunction_iq_idx_0 - rtb_Rem_p;

          /* Sum: '<S123>/Add3' */
          rtb_Add3_c = rtb_MathFunction_iq_idx_0 + FMS_ConstB.d;

          /* Sum: '<S123>/Subtract1' */
          rtb_MathFunction_iq_idx_0 -= FMS_ConstB.d;

          /* Signum: '<S123>/Sign1' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else {
            if (rtb_Add3_c > 0.0F) {
              rtb_Add3_c = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign2' */
          if (rtb_MathFunction_iq_idx_0 < 0.0F) {
            rtb_MathFunction_iq_idx_0 = -1.0F;
          } else {
            if (rtb_MathFunction_iq_idx_0 > 0.0F) {
              rtb_MathFunction_iq_idx_0 = 1.0F;
            }
          }

          /* Sum: '<S123>/Add5' incorporates:
           *  Gain: '<S123>/Gain2'
           *  Product: '<S123>/Multiply4'
           *  Sum: '<S123>/Subtract2'
           */
          rtb_Rem_p += (rtb_Add3_c - rtb_MathFunction_iq_idx_0) * 0.5F *
            rtb_MathFunction_f_idx_0;

          /* SignalConversion: '<S113>/OutportBufferForuv_cmd_mPs' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator1'
           */
          FMS_B.Merge[0] = FMS_DW.Integrator1_DSTATE[0];

          /* Update for DiscreteIntegrator: '<S120>/Integrator1' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator'
           */
          FMS_DW.Integrator1_DSTATE[0] += 0.004F * FMS_DW.Integrator_DSTATE[0];

          /* Signum: '<S123>/Sign4' incorporates:
           *  Sum: '<S123>/Subtract3'
           */
          rtb_Add3_c = rtb_Rem_p - FMS_ConstB.d;

          /* Signum: '<S123>/Sign3' incorporates:
           *  Sum: '<S123>/Add6'
           */
          rtb_Subtract3_o = rtb_Rem_p + FMS_ConstB.d;

          /* Signum: '<S123>/Sign5' */
          if (rtb_Rem_p < 0.0F) {
            rtb_a_l = -1.0F;
          } else if (rtb_Rem_p > 0.0F) {
            rtb_a_l = 1.0F;
          } else {
            rtb_a_l = rtb_Rem_p;
          }

          /* Signum: '<S123>/Sign3' */
          if (rtb_Subtract3_o < 0.0F) {
            rtb_Subtract3_o = -1.0F;
          } else {
            if (rtb_Subtract3_o > 0.0F) {
              rtb_Subtract3_o = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign4' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else {
            if (rtb_Add3_c > 0.0F) {
              rtb_Add3_c = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign6' */
          if (rtb_Rem_p < 0.0F) {
            rtb_MathFunction_iq_idx_0 = -1.0F;
          } else if (rtb_Rem_p > 0.0F) {
            rtb_MathFunction_iq_idx_0 = 1.0F;
          } else {
            rtb_MathFunction_iq_idx_0 = rtb_Rem_p;
          }

          /* Update for DiscreteIntegrator: '<S120>/Integrator' incorporates:
           *  Constant: '<S123>/const'
           *  Gain: '<S123>/Gain3'
           *  Product: '<S123>/Divide'
           *  Product: '<S123>/Multiply5'
           *  Product: '<S123>/Multiply6'
           *  Sum: '<S123>/Subtract4'
           *  Sum: '<S123>/Subtract5'
           *  Sum: '<S123>/Subtract6'
           */
          FMS_DW.Integrator_DSTATE[0] += ((rtb_Rem_p / FMS_ConstB.d - rtb_a_l) *
            FMS_ConstB.Gain4 * ((rtb_Subtract3_o - rtb_Add3_c) * 0.5F) -
            rtb_MathFunction_iq_idx_0 * 58.836F) * 0.004F;

          /* Product: '<S123>/Multiply1' incorporates:
           *  Constant: '<S123>/const1'
           *  DiscreteIntegrator: '<S120>/Integrator'
           */
          rtb_MathFunction_f_idx_0 = FMS_DW.Integrator_DSTATE[1] * 0.05F;

          /* Sum: '<S123>/Add' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator1'
           *  Gain: '<S113>/Gain6'
           *  Sum: '<S120>/Subtract'
           */
          rtb_MathFunction_iq_idx_0 = (FMS_DW.Integrator1_DSTATE[1] -
            FMS_PARAM.VEL_XY_LIM * rtb_Add4_e5) + rtb_MathFunction_f_idx_0;

          /* Signum: '<S123>/Sign' */
          if (rtb_MathFunction_iq_idx_0 < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else if (rtb_MathFunction_iq_idx_0 > 0.0F) {
            rtb_Add3_c = 1.0F;
          } else {
            rtb_Add3_c = rtb_MathFunction_iq_idx_0;
          }

          /* Sum: '<S123>/Add2' incorporates:
           *  Abs: '<S123>/Abs'
           *  Gain: '<S123>/Gain'
           *  Gain: '<S123>/Gain1'
           *  Product: '<S123>/Multiply2'
           *  Product: '<S123>/Multiply3'
           *  Sqrt: '<S123>/Sqrt'
           *  Sum: '<S123>/Add1'
           *  Sum: '<S123>/Subtract'
           */
          rtb_Rem_p = (sqrtf((8.0F * fabsf(rtb_MathFunction_iq_idx_0) +
                              FMS_ConstB.d) * FMS_ConstB.d) - FMS_ConstB.d) *
            0.5F * rtb_Add3_c + rtb_MathFunction_f_idx_0;

          /* Sum: '<S123>/Add4' */
          rtb_MathFunction_f_idx_0 += rtb_MathFunction_iq_idx_0 - rtb_Rem_p;

          /* Sum: '<S123>/Add3' */
          rtb_Add3_c = rtb_MathFunction_iq_idx_0 + FMS_ConstB.d;

          /* Sum: '<S123>/Subtract1' */
          rtb_MathFunction_iq_idx_0 -= FMS_ConstB.d;

          /* Signum: '<S123>/Sign1' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else {
            if (rtb_Add3_c > 0.0F) {
              rtb_Add3_c = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign2' */
          if (rtb_MathFunction_iq_idx_0 < 0.0F) {
            rtb_MathFunction_iq_idx_0 = -1.0F;
          } else {
            if (rtb_MathFunction_iq_idx_0 > 0.0F) {
              rtb_MathFunction_iq_idx_0 = 1.0F;
            }
          }

          /* Sum: '<S123>/Add5' incorporates:
           *  Gain: '<S123>/Gain2'
           *  Product: '<S123>/Multiply4'
           *  Sum: '<S123>/Subtract2'
           */
          rtb_Rem_p += (rtb_Add3_c - rtb_MathFunction_iq_idx_0) * 0.5F *
            rtb_MathFunction_f_idx_0;

          /* SignalConversion: '<S113>/OutportBufferForuv_cmd_mPs' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator1'
           */
          FMS_B.Merge[1] = FMS_DW.Integrator1_DSTATE[1];

          /* Update for DiscreteIntegrator: '<S120>/Integrator1' incorporates:
           *  DiscreteIntegrator: '<S120>/Integrator'
           */
          FMS_DW.Integrator1_DSTATE[1] += 0.004F * FMS_DW.Integrator_DSTATE[1];

          /* Signum: '<S123>/Sign4' incorporates:
           *  Sum: '<S123>/Subtract3'
           */
          rtb_Add3_c = rtb_Rem_p - FMS_ConstB.d;

          /* Signum: '<S123>/Sign3' incorporates:
           *  Sum: '<S123>/Add6'
           */
          rtb_Subtract3_o = rtb_Rem_p + FMS_ConstB.d;

          /* Signum: '<S123>/Sign5' */
          if (rtb_Rem_p < 0.0F) {
            rtb_a_l = -1.0F;
          } else if (rtb_Rem_p > 0.0F) {
            rtb_a_l = 1.0F;
          } else {
            rtb_a_l = rtb_Rem_p;
          }

          /* Signum: '<S123>/Sign3' */
          if (rtb_Subtract3_o < 0.0F) {
            rtb_Subtract3_o = -1.0F;
          } else {
            if (rtb_Subtract3_o > 0.0F) {
              rtb_Subtract3_o = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign4' */
          if (rtb_Add3_c < 0.0F) {
            rtb_Add3_c = -1.0F;
          } else {
            if (rtb_Add3_c > 0.0F) {
              rtb_Add3_c = 1.0F;
            }
          }

          /* Signum: '<S123>/Sign6' */
          if (rtb_Rem_p < 0.0F) {
            rtb_MathFunction_iq_idx_0 = -1.0F;
          } else if (rtb_Rem_p > 0.0F) {
            rtb_MathFunction_iq_idx_0 = 1.0F;
          } else {
            rtb_MathFunction_iq_idx_0 = rtb_Rem_p;
          }

          /* Update for DiscreteIntegrator: '<S120>/Integrator' incorporates:
           *  Constant: '<S123>/const'
           *  Gain: '<S123>/Gain3'
           *  Product: '<S123>/Divide'
           *  Product: '<S123>/Multiply5'
           *  Product: '<S123>/Multiply6'
           *  Sum: '<S123>/Subtract4'
           *  Sum: '<S123>/Subtract5'
           *  Sum: '<S123>/Subtract6'
           */
          FMS_DW.Integrator_DSTATE[1] += ((rtb_Rem_p / FMS_ConstB.d - rtb_a_l) *
            FMS_ConstB.Gain4 * ((rtb_Subtract3_o - rtb_Add3_c) * 0.5F) -
            rtb_MathFunction_iq_idx_0 * 58.836F) * 0.004F;

          /* End of Outputs for SubSystem: '<S109>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S109>/Switch Case' */

        /* SwitchCase: '<S96>/Switch Case' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S30>/Signal Copy1'
         *  SignalConversion: '<S30>/Signal Copy2'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_j;
        FMS_DW.SwitchCase_ActiveSubsystem_j = -1;
        switch (rtb_state_c) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_j = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_j = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_j = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_j) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_j != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S96>/Hold Control' incorporates:
             *  ActionPort: '<S99>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S96>/Switch Case' */
            FMS_HoldControl_kp_Reset(&FMS_DW.HoldControl_e);

            /* End of SystemReset for SubSystem: '<S96>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S96>/Hold Control' incorporates:
           *  ActionPort: '<S99>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_HoldControl_k(FMS_U.INS_Out.psi, &FMS_B.Merge_d,
                            &FMS_DW.HoldControl_e);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S96>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S96>/Brake Control' incorporates:
           *  ActionPort: '<S98>/Action Port'
           */
          FMS_BrakeControl(&FMS_B.Merge_d);

          /* End of Outputs for SubSystem: '<S96>/Brake Control' */
          break;

         case 2:
          if (FMS_DW.SwitchCase_ActiveSubsystem_j != rtPrevAction) {
            /* SystemReset for IfAction SubSystem: '<S96>/Move Control' incorporates:
             *  ActionPort: '<S100>/Action Port'
             */
            /* SystemReset for SwitchCase: '<S96>/Switch Case' */
            FMS_MoveControl_l_Reset(&FMS_DW.MoveControl_mr);

            /* End of SystemReset for SubSystem: '<S96>/Move Control' */
          }

          /* Outputs for IfAction SubSystem: '<S96>/Move Control' incorporates:
           *  ActionPort: '<S100>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_MoveControl_b(FMS_U.Pilot_Cmd.stick_yaw, &FMS_B.Merge_d,
                            &FMS_ConstB.MoveControl_mr, &FMS_DW.MoveControl_mr);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          /* End of Outputs for SubSystem: '<S96>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S96>/Switch Case' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  BusAssignment: '<S42>/Bus Assignment'
         *  Constant: '<S42>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S42>/Bus Assignment' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_b;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_o;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2;

        /* Saturate: '<S96>/Saturation' */
        if (FMS_B.Merge_d > FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_PARAM.YAW_RATE_LIM;
        } else if (FMS_B.Merge_d < -FMS_PARAM.YAW_RATE_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -FMS_PARAM.YAW_RATE_LIM;
        } else {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_B.Merge_d;
        }

        /* End of Saturate: '<S96>/Saturation' */

        /* Saturate: '<S109>/Saturation1' */
        if (FMS_B.Merge[0] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (FMS_B.Merge[0] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_B.Merge[0];
        }

        if (FMS_B.Merge[1] > FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = FMS_PARAM.VEL_XY_LIM;
        } else if (FMS_B.Merge[1] < -FMS_PARAM.VEL_XY_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = -FMS_PARAM.VEL_XY_LIM;
        } else {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.v_cmd = FMS_B.Merge[1];
        }

        /* End of Saturate: '<S109>/Saturation1' */

        /* Saturate: '<S84>/Saturation1' */
        if (FMS_B.Merge_k > FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = FMS_PARAM.VEL_Z_LIM;
        } else if (FMS_B.Merge_k < -FMS_PARAM.VEL_Z_LIM) {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = -FMS_PARAM.VEL_Z_LIM;
        } else {
          /* BusAssignment: '<S42>/Bus Assignment' incorporates:
           *  BusAssignment: '<S32>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.w_cmd = FMS_B.Merge_k;
        }

        /* End of Saturate: '<S84>/Saturation1' */
        /* End of Outputs for SubSystem: '<S35>/Position' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S35>/Unknown' incorporates:
         *  ActionPort: '<S44>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S32>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_i);

        /* End of Outputs for SubSystem: '<S35>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S35>/Switch Case' */
      /* End of Outputs for SubSystem: '<S31>/Assist' */
      break;

     case 3:
      /* Outputs for IfAction SubSystem: '<S31>/Manual' incorporates:
       *  ActionPort: '<S37>/Action Port'
       */
      /* Outport: '<Root>/FMS_Out' incorporates:
       *  BusAssignment: '<S32>/Bus Assignment'
       *  BusAssignment: '<S37>/Bus Assignment'
       *  Constant: '<S37>/Constant'
       */
      FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

      /* BusAssignment: '<S37>/Bus Assignment' incorporates:
       *  BusAssignment: '<S32>/Bus Assignment'
       *  Constant: '<S37>/Constant2'
       *  Outport: '<Root>/FMS_Out'
       */
      FMS_Y.FMS_Out.reset = 1U;
      FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion;
      FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1;

      /* End of Outputs for SubSystem: '<S31>/Manual' */
      break;

     case 4:
      /* Outputs for IfAction SubSystem: '<S31>/Unknown' incorporates:
       *  ActionPort: '<S39>/Action Port'
       */
      /* Outport: '<Root>/FMS_Out' incorporates:
       *  BusAssignment: '<S32>/Bus Assignment'
       */
      FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown);

      /* End of Outputs for SubSystem: '<S31>/Unknown' */
      break;
    }

    /* End of SwitchCase: '<S31>/Switch Case' */
    /* End of Outputs for SubSystem: '<S29>/Arm' */
    break;
  }

  /* End of SwitchCase: '<S29>/Switch Case' */

  /* BusAssignment: '<S32>/Bus Assignment' incorporates:
   *  Constant: '<S32>/Constant'
   *  DataStoreRead: '<S32>/Data Store Read'
   *  DataTypeConversion: '<S32>/Data Type Conversion'
   *  DiscreteIntegrator: '<S475>/Discrete-Time Integrator'
   *  Outport: '<Root>/FMS_Out'
   *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy4Inport1'
   *  SignalConversion: '<S30>/TmpSignal ConversionAtSignal Copy5Inport1'
   *  Sum: '<S32>/Sum'
   */
  FMS_Y.FMS_Out.timestamp = FMS_DW.DiscreteTimeIntegrator_DSTATE_g;
  FMS_Y.FMS_Out.mode = (uint8_T)FMS_B.target_mode;

  /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
  FMS_Y.FMS_Out.wp_consume = FMS_B.wp_consume;
  FMS_Y.FMS_Out.wp_current = (uint8_T)(FMS_B.wp_index - 1);

  /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
  FMS_Y.FMS_Out.home[0] = FMS_DW.home[0];
  FMS_Y.FMS_Out.home[1] = FMS_DW.home[1];
  FMS_Y.FMS_Out.home[2] = FMS_DW.home[2];
  FMS_Y.FMS_Out.home[3] = FMS_DW.home[3];

  /* Update for DiscreteIntegrator: '<S475>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S475>/Constant'
   */
  FMS_DW.DiscreteTimeIntegrator_DSTATE_g += FMS_EXPORT.period;

  /* End of Outputs for SubSystem: '<Root>/FMS Commander' */

  /* RelationalOperator: '<S18>/Compare' incorporates:
   *  Constant: '<S18>/Constant'
   */
  rtb_FixPtRelationalOperator_me = (rtb_DataTypeConversion1_m == FMS_Cmd_SetHome);

  /* Outputs for Triggered SubSystem: '<S14>/SetHome' incorporates:
   *  TriggerPort: '<S21>/Trigger'
   */
  if (rtb_FixPtRelationalOperator_me && (FMS_PrevZCX.SetHome_Trig_ZCE !=
       POS_ZCSIG)) {
    /* DataStoreWrite: '<S21>/Data Store Write' incorporates:
     *  Inport: '<Root>/GCS_Cmd'
     */
    FMS_DW.home[0] = FMS_U.GCS_Cmd.param[0];
    FMS_DW.home[1] = FMS_U.GCS_Cmd.param[1];
    FMS_DW.home[2] = FMS_U.GCS_Cmd.param[2];
    FMS_DW.home[3] = FMS_U.GCS_Cmd.param[3];
  }

  FMS_PrevZCX.SetHome_Trig_ZCE = rtb_FixPtRelationalOperator_me;

  /* End of Outputs for SubSystem: '<S14>/SetHome' */

  /* Update for UnitDelay: '<S17>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S17>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE = FMS_U.Pilot_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator1' incorporates:
   *  Constant: '<S13>/Constant1'
   */
  rtb_a_l = (real32_T)FMS_DW.DiscreteTimeIntegrator1_DSTAT_b + (real32_T)
    FMS_EXPORT.period;
  if (rtb_a_l < 4.2949673E+9F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = (uint32_T)rtb_a_l;
  } else {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = MAX_uint32_T;
  }

  /* End of Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S24>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S24>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_f = FMS_U.GCS_Cmd.mode;

  /* Update for UnitDelay: '<S19>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S19>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_i = FMS_U.Pilot_Cmd.cmd_1;

  /* Update for UnitDelay: '<S20>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S20>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_p = FMS_U.GCS_Cmd.cmd_1;

  /* Update for UnitDelay: '<S25>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S25>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_d = FMS_U.GCS_Cmd.timestamp;

  /* Update for UnitDelay: '<S26>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S26>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_h = FMS_U.Pilot_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant'
   */
  FMS_DW.DiscreteTimeIntegrator_DSTATE += 0.004F;
  if (FMS_DW.DiscreteTimeIntegrator_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator_DSTATE = 0.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' incorporates:
   *  Constant: '<S3>/Constant'
   */
  FMS_DW.DiscreteTimeIntegrator1_DSTATE += 0.004F;
  if (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator1_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S10>/Delay Input1' incorporates:
   *  Inport: '<Root>/Auto_Cmd'
   *
   * Block description for '<S10>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_c = FMS_U.Auto_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S1>/Constant'
   */
  rtb_a_l = (real32_T)FMS_DW.DiscreteTimeIntegrator_DSTATE_b + (real32_T)
    FMS_EXPORT.period;
  if (rtb_a_l < 4.2949673E+9F) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_b = (uint32_T)rtb_a_l;
  } else {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_b = MAX_uint32_T;
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */

  /* Update for Delay: '<S11>/Delay' */
  FMS_DW.Delay_DSTATE_o = rtb_Switch_m;
}

/* Model initialize function */
void FMS_init(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(FMS_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &FMS_B), 0,
                sizeof(B_FMS_T));

  {
    FMS_B.state = VehicleState_None;
    FMS_B.target_mode = PilotMode_None;
    FMS_B.Switch1 = FMS_Cmd_None;
  }

  /* states (dwork) */
  (void) memset((void *)&FMS_DW, 0,
                sizeof(DW_FMS_T));

  /* external inputs */
  (void)memset(&FMS_U, 0, sizeof(ExtU_FMS_T));

  /* external outputs */
  FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

  /* Start for Atomic SubSystem: '<Root>/FMS Commander' */
  /* Start for SwitchCase: '<S29>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S29>/Arm' */
  /* Start for SwitchCase: '<S31>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_b = -1;

  /* Start for IfAction SubSystem: '<S31>/SubMode' */
  /* Start for SwitchCase: '<S38>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_at = -1;

  /* Start for IfAction SubSystem: '<S38>/Return' */
  /* Start for SwitchCase: '<S434>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_g = -1;

  /* Start for SwitchCase: '<S424>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_ld = -1;

  /* End of Start for SubSystem: '<S38>/Return' */

  /* Start for IfAction SubSystem: '<S38>/Hold' */
  /* Start for SwitchCase: '<S366>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_p = -1;

  /* Start for SwitchCase: '<S344>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_pp = -1;

  /* Start for SwitchCase: '<S354>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_bn = -1;

  /* End of Start for SubSystem: '<S38>/Hold' */
  /* End of Start for SubSystem: '<S31>/SubMode' */

  /* Start for IfAction SubSystem: '<S31>/Auto' */
  /* Start for SwitchCase: '<S36>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_i = -1;

  /* Start for IfAction SubSystem: '<S36>/Mission' */
  /* Start for Resettable SubSystem: '<S149>/Mission_SubSystem' */
  /* Start for SwitchCase: '<S200>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

  /* Start for SwitchCase: '<S190>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_e = -1;

  /* End of Start for SubSystem: '<S149>/Mission_SubSystem' */
  /* End of Start for SubSystem: '<S36>/Mission' */
  /* End of Start for SubSystem: '<S31>/Auto' */

  /* Start for IfAction SubSystem: '<S31>/Assist' */
  /* Start for SwitchCase: '<S35>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_f = -1;

  /* Start for IfAction SubSystem: '<S35>/Stabilize' */
  /* Start for SwitchCase: '<S136>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_fs = -1;

  /* End of Start for SubSystem: '<S35>/Stabilize' */

  /* Start for IfAction SubSystem: '<S35>/Altitude' */
  /* Start for SwitchCase: '<S50>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_m = -1;

  /* Start for SwitchCase: '<S68>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_h = -1;

  /* End of Start for SubSystem: '<S35>/Altitude' */

  /* Start for IfAction SubSystem: '<S35>/Position' */
  /* Start for SwitchCase: '<S84>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_o = -1;

  /* Start for SwitchCase: '<S109>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_l = -1;

  /* Start for SwitchCase: '<S96>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_j = -1;

  /* End of Start for SubSystem: '<S35>/Position' */
  /* End of Start for SubSystem: '<S31>/Assist' */
  /* End of Start for SubSystem: '<S29>/Arm' */
  /* End of Start for SubSystem: '<Root>/FMS Commander' */
  FMS_PrevZCX.SetHome_Trig_ZCE = POS_ZCSIG;
  FMS_PrevZCX.Mission_SubSystem_Reset_ZCE = POS_ZCSIG;

  /* SystemInitialize for Chart: '<Root>/SafeMode' */
  FMS_DW.is_active_c3_FMS = 0U;
  FMS_DW.is_c3_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

  /* SystemInitialize for Chart: '<Root>/FMS State Machine' */
  initialize_msg_local_queues_for();
  FMS_DW.sfEvent = -1;
  FMS_DW.is_active_Combo_Stick = 0U;
  FMS_DW.is_Combo_Stick = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_active_Command_Listener = 0U;
  FMS_DW.is_Command_Listener = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_active_Lost_Return = 0U;
  FMS_DW.is_Lost_Return = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_active_Vehicle = 0U;
  FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Offboard = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD_h;
  FMS_DW.temporalCounter_i1 = 0U;
  FMS_DW.is_active_c11_FMS = 0U;
  FMS_DW.M_msgReservedData = FMS_Cmd_None;
  FMS_DW.prep_takeoff = 0.0;
  FMS_DW.bl = false;
  FMS_DW.br = false;
  FMS_DW.prep_mission_takeoff = 0.0;
  FMS_B.wp_consume = 0U;
  FMS_B.wp_index = 1U;
  FMS_DW.chartAbsoluteTimeCounter = 0;

  /* SystemInitialize for Atomic SubSystem: '<Root>/FMS Commander' */
  /* SystemInitialize for IfAction SubSystem: '<S29>/Arm' */
  /* SystemInitialize for IfAction SubSystem: '<S31>/SubMode' */
  /* SystemInitialize for IfAction SubSystem: '<S38>/Takeoff' */
  /* InitializeConditions for Delay: '<S473>/cur_waypoint' */
  FMS_DW.icLoad_j1 = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S469>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_a = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S469>/Integrator' */
  FMS_DW.Integrator_DSTATE_m = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S38>/Takeoff' */

  /* SystemInitialize for IfAction SubSystem: '<S38>/Land' */
  /* InitializeConditions for DiscreteIntegrator: '<S382>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_j = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S382>/Integrator' */
  FMS_DW.Integrator_DSTATE_d = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S38>/Land' */

  /* SystemInitialize for IfAction SubSystem: '<S38>/Return' */
  /* InitializeConditions for Delay: '<S413>/Delay' */
  FMS_DW.icLoad_l = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S414>/Discrete-Time Integrator' */
  FMS_DW.DiscreteTimeIntegrator_DSTATE_m = 0U;

  /* InitializeConditions for DiscreteIntegrator: '<S409>/Acceleration_Speed' */
  FMS_DW.Acceleration_Speed_DSTATE_j = 0.0F;
  FMS_DW.Acceleration_Speed_PrevResetS_j = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S461>/Discrete-Time Integrator' */
  FMS_DW.l1_heading_e = 0.0F;

  /* InitializeConditions for Delay: '<S391>/Delay' */
  FMS_DW.icLoad_j = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S394>/Integrator1' */
  FMS_DW.Integrator1_IC_LOADING_j = 1U;

  /* InitializeConditions for Delay: '<S390>/Delay' */
  FMS_DW.icLoad_c = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S394>/Integrator' */
  FMS_DW.Integrator_DSTATE_bs = 0.0F;

  /* SystemInitialize for Chart: '<S435>/Motion State' */
  FMS_MotionState_Init(&FMS_DW.sf_MotionState);

  /* SystemInitialize for IfAction SubSystem: '<S434>/Hold Control' */
  FMS_HoldControl_c_Init(&FMS_DW.HoldControl_m);

  /* End of SystemInitialize for SubSystem: '<S434>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S434>/Move Control' */
  FMS_MoveControl_l_Init(&FMS_DW.MoveControl_j);

  /* End of SystemInitialize for SubSystem: '<S434>/Move Control' */

  /* SystemInitialize for Merge: '<S434>/Merge' */
  FMS_B.Merge_a[0] = 0.0F;
  FMS_B.Merge_a[1] = 0.0F;

  /* SystemInitialize for Chart: '<S425>/Motion Status' */
  FMS_MotionStatus_Init(&FMS_DW.sf_MotionStatus);

  /* SystemInitialize for IfAction SubSystem: '<S424>/Hold Control' */
  FMS_HoldControl_Init(&FMS_DW.HoldControl);

  /* End of SystemInitialize for SubSystem: '<S424>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S424>/Move Control' */
  FMS_MoveControl_Init(&FMS_DW.MoveControl);

  /* End of SystemInitialize for SubSystem: '<S424>/Move Control' */

  /* SystemInitialize for Merge: '<S424>/Merge' */
  FMS_B.Merge_jj = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S38>/Return' */

  /* SystemInitialize for IfAction SubSystem: '<S38>/Hold' */
  /* SystemInitialize for Chart: '<S345>/Motion Status' */
  FMS_MotionStatus_Init(&FMS_DW.sf_MotionStatus_j);

  /* SystemInitialize for Chart: '<S355>/Motion State' */
  FMS_DW.temporalCounter_i1_ai = 0U;
  FMS_DW.is_active_c15_FMS = 0U;
  FMS_DW.is_c15_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

  /* SystemInitialize for Chart: '<S367>/Motion State' */
  FMS_MotionState_Init(&FMS_DW.sf_MotionState_g);

  /* SystemInitialize for IfAction SubSystem: '<S366>/Hold Control' */
  FMS_HoldControl_c_Init(&FMS_DW.HoldControl_f);

  /* End of SystemInitialize for SubSystem: '<S366>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S366>/Move Control' */
  FMS_MoveControl_l_Init(&FMS_DW.MoveControl_i);

  /* End of SystemInitialize for SubSystem: '<S366>/Move Control' */

  /* SystemInitialize for Merge: '<S366>/Merge' */
  FMS_B.Merge_o[0] = 0.0F;
  FMS_B.Merge_o[1] = 0.0F;

  /* SystemInitialize for IfAction SubSystem: '<S344>/Hold Control' */
  FMS_HoldControl_Init(&FMS_DW.HoldControl_n);

  /* End of SystemInitialize for SubSystem: '<S344>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S344>/Move Control' */
  FMS_MoveControl_Init(&FMS_DW.MoveControl_n);

  /* End of SystemInitialize for SubSystem: '<S344>/Move Control' */

  /* SystemInitialize for Merge: '<S344>/Merge' */
  FMS_B.Merge_ey = 0.0F;

  /* SystemInitialize for IfAction SubSystem: '<S354>/Hold Control' */
  FMS_HoldControl_e_Init(&FMS_DW.HoldControl_k);

  /* End of SystemInitialize for SubSystem: '<S354>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S354>/Move Control' */
  FMS_MoveControl_j_Init(&FMS_DW.MoveControl_b);

  /* End of SystemInitialize for SubSystem: '<S354>/Move Control' */

  /* SystemInitialize for Merge: '<S354>/Merge' */
  FMS_B.Merge_n1 = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S38>/Hold' */
  /* End of SystemInitialize for SubSystem: '<S31>/SubMode' */

  /* SystemInitialize for IfAction SubSystem: '<S31>/Auto' */
  /* SystemInitialize for IfAction SubSystem: '<S36>/Mission' */
  /* InitializeConditions for UnitDelay: '<S152>/Delay Input1'
   *
   * Block description for '<S152>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_pe = 0U;

  /* SystemInitialize for Resettable SubSystem: '<S149>/Mission_SubSystem' */
  /* InitializeConditions for Delay: '<S179>/Delay' */
  FMS_DW.icLoad = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S180>/Discrete-Time Integrator' */
  FMS_DW.DiscreteTimeIntegrator_DSTATE_k = 0U;

  /* InitializeConditions for DiscreteIntegrator: '<S175>/Acceleration_Speed' */
  FMS_DW.Acceleration_Speed_DSTATE = 0.0F;
  FMS_DW.Acceleration_Speed_PrevResetSta = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S227>/Discrete-Time Integrator' */
  FMS_DW.l1_heading = 0.0F;

  /* InitializeConditions for Delay: '<S157>/Delay' */
  FMS_DW.icLoad_k = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S160>/Integrator1' */
  FMS_DW.Integrator1_IC_LOADING = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S160>/Integrator' */
  FMS_DW.Integrator_DSTATE_i = 0.0F;

  /* SystemInitialize for Chart: '<S201>/Motion State' */
  FMS_MotionState_Init(&FMS_DW.sf_MotionState_n);

  /* SystemInitialize for IfAction SubSystem: '<S200>/Hold Control' */
  FMS_HoldControl_c_Init(&FMS_DW.HoldControl_d);

  /* End of SystemInitialize for SubSystem: '<S200>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S200>/Move Control' */
  FMS_MoveControl_l_Init(&FMS_DW.MoveControl_c);

  /* End of SystemInitialize for SubSystem: '<S200>/Move Control' */

  /* SystemInitialize for Merge: '<S200>/Merge' */
  FMS_B.Merge_n[0] = 0.0F;
  FMS_B.Merge_n[1] = 0.0F;

  /* SystemInitialize for Chart: '<S191>/Motion Status' */
  FMS_MotionStatus_Init(&FMS_DW.sf_MotionStatus_jt);

  /* SystemInitialize for IfAction SubSystem: '<S190>/Hold Control' */
  FMS_HoldControl_Init(&FMS_DW.HoldControl_a);

  /* End of SystemInitialize for SubSystem: '<S190>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S190>/Move Control' */
  FMS_MoveControl_Init(&FMS_DW.MoveControl_m);

  /* End of SystemInitialize for SubSystem: '<S190>/Move Control' */

  /* SystemInitialize for Merge: '<S190>/Merge' */
  FMS_B.Merge_e = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S149>/Mission_SubSystem' */
  /* End of SystemInitialize for SubSystem: '<S36>/Mission' */
  /* End of SystemInitialize for SubSystem: '<S31>/Auto' */

  /* SystemInitialize for IfAction SubSystem: '<S31>/Assist' */
  /* SystemInitialize for IfAction SubSystem: '<S35>/Stabilize' */
  /* InitializeConditions for DiscreteIntegrator: '<S132>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_l = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S133>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_h = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S133>/Integrator' */
  FMS_DW.Integrator_DSTATE_a = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S132>/Integrator' */
  FMS_DW.Integrator_DSTATE_c = 0.0F;

  /* SystemInitialize for Chart: '<S137>/Motion State' */
  FMS_MotionState_l_Init(&FMS_DW.sf_MotionState_e);

  /* SystemInitialize for IfAction SubSystem: '<S136>/Hold Control' */
  FMS_HoldControl_e_Init(&FMS_DW.HoldControl_h);

  /* End of SystemInitialize for SubSystem: '<S136>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S136>/Move Control' */
  FMS_MoveControl_j_Init(&FMS_DW.MoveControl_k);

  /* End of SystemInitialize for SubSystem: '<S136>/Move Control' */

  /* SystemInitialize for Merge: '<S136>/Merge' */
  FMS_B.Merge_j = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S35>/Stabilize' */

  /* SystemInitialize for IfAction SubSystem: '<S35>/Altitude' */
  /* InitializeConditions for DiscreteIntegrator: '<S64>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_f = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S65>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_o = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S65>/Integrator' */
  FMS_DW.Integrator_DSTATE_b = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S64>/Integrator' */
  FMS_DW.Integrator_DSTATE_bp = 0.0F;

  /* SystemInitialize for Chart: '<S51>/Motion Status' */
  FMS_DW.temporalCounter_i1_a = 0U;
  FMS_DW.is_active_c17_FMS = 0U;
  FMS_DW.is_c17_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

  /* SystemInitialize for IfAction SubSystem: '<S50>/Hold Control' */
  FMS_HoldControl_Init(&FMS_DW.HoldControl_k2);

  /* End of SystemInitialize for SubSystem: '<S50>/Hold Control' */

  /* SystemInitialize for Merge: '<S50>/Merge' */
  FMS_B.Merge_l = 0.0F;

  /* SystemInitialize for Chart: '<S69>/Motion State' */
  FMS_MotionState_l_Init(&FMS_DW.sf_MotionState_k);

  /* SystemInitialize for IfAction SubSystem: '<S68>/Hold Control' */
  FMS_HoldControl_e_Init(&FMS_DW.HoldControl_o);

  /* End of SystemInitialize for SubSystem: '<S68>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S68>/Move Control' */
  FMS_MoveControl_j_Init(&FMS_DW.MoveControl_cr);

  /* End of SystemInitialize for SubSystem: '<S68>/Move Control' */

  /* SystemInitialize for Merge: '<S68>/Merge' */
  FMS_B.Merge_m = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S35>/Altitude' */

  /* SystemInitialize for IfAction SubSystem: '<S35>/Position' */
  /* SystemInitialize for Chart: '<S85>/Motion Status' */
  FMS_DW.temporalCounter_i1_l = 0U;
  FMS_DW.is_active_c20_FMS = 0U;
  FMS_DW.is_c20_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

  /* SystemInitialize for IfAction SubSystem: '<S84>/Hold Control' */
  FMS_HoldControl_Init(&FMS_DW.HoldControl_p);

  /* End of SystemInitialize for SubSystem: '<S84>/Hold Control' */

  /* SystemInitialize for Chart: '<S97>/Motion State' */
  FMS_MotionState_l_Init(&FMS_DW.sf_MotionState_j);

  /* SystemInitialize for Chart: '<S110>/Motion State' */
  FMS_DW.temporalCounter_i1_i = 0U;
  FMS_DW.is_active_c16_FMS = 0U;
  FMS_DW.is_c16_FMS = FMS_IN_NO_ACTIVE_CHILD_h;

  /* SystemInitialize for IfAction SubSystem: '<S109>/Hold Control' */
  FMS_HoldControl_c_Init(&FMS_DW.HoldControl_at);

  /* End of SystemInitialize for SubSystem: '<S109>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S109>/Move Control' */
  /* InitializeConditions for DiscreteIntegrator: '<S120>/Integrator1' */
  FMS_DW.Integrator1_DSTATE[0] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S120>/Integrator' */
  FMS_DW.Integrator_DSTATE[0] = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S109>/Move Control' */

  /* SystemInitialize for Merge: '<S109>/Merge' */
  FMS_B.Merge[0] = 0.0F;

  /* SystemInitialize for IfAction SubSystem: '<S109>/Move Control' */
  /* InitializeConditions for DiscreteIntegrator: '<S120>/Integrator1' */
  FMS_DW.Integrator1_DSTATE[1] = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S120>/Integrator' */
  FMS_DW.Integrator_DSTATE[1] = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S109>/Move Control' */

  /* SystemInitialize for Merge: '<S109>/Merge' */
  FMS_B.Merge[1] = 0.0F;

  /* SystemInitialize for Merge: '<S84>/Merge' */
  FMS_B.Merge_k = 0.0F;

  /* SystemInitialize for IfAction SubSystem: '<S96>/Hold Control' */
  FMS_HoldControl_e_Init(&FMS_DW.HoldControl_e);

  /* End of SystemInitialize for SubSystem: '<S96>/Hold Control' */

  /* SystemInitialize for IfAction SubSystem: '<S96>/Move Control' */
  FMS_MoveControl_j_Init(&FMS_DW.MoveControl_mr);

  /* End of SystemInitialize for SubSystem: '<S96>/Move Control' */

  /* SystemInitialize for Merge: '<S96>/Merge' */
  FMS_B.Merge_d = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S35>/Position' */
  /* End of SystemInitialize for SubSystem: '<S31>/Assist' */
  /* End of SystemInitialize for SubSystem: '<S29>/Arm' */
  /* End of SystemInitialize for SubSystem: '<Root>/FMS Commander' */
}

/* Model terminate function */
void FMS_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

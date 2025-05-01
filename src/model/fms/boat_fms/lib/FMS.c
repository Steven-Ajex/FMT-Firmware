/*
 * File: FMS.c
 *
 * Code generated for Simulink model 'FMS'.
 *
 * Model version                  : 1.2305
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Wed Apr 30 15:58:18 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "FMS.h"
#include "FMS_private.h"

/* Named constants for Chart: '<S80>/Motion State' */
#define FMS_IN_Brake                   ((uint8_T)1U)
#define FMS_IN_Hold                    ((uint8_T)2U)
#define FMS_IN_Move                    ((uint8_T)3U)
#define FMS_IN_NO_ACTIVE_CHILD         ((uint8_T)0U)

/* Named constants for Chart: '<S42>/Motion State' */
#define FMS_IN_Keep                    ((uint8_T)3U)
#define FMS_IN_Move_i                  ((uint8_T)4U)

/* Named constants for Chart: '<Root>/FMS State Machine' */
#define FMS_IN_Arm                     ((uint8_T)1U)
#define FMS_IN_Assist                  ((uint8_T)1U)
#define FMS_IN_Auto                    ((uint8_T)2U)
#define FMS_IN_Check                   ((uint8_T)1U)
#define FMS_IN_Connect                 ((uint8_T)1U)
#define FMS_IN_Disarm                  ((uint8_T)2U)
#define FMS_IN_Hold_h                  ((uint8_T)1U)
#define FMS_IN_Idle                    ((uint8_T)3U)
#define FMS_IN_InValidManualMode       ((uint8_T)1U)
#define FMS_IN_InvalidArmMode          ((uint8_T)3U)
#define FMS_IN_InvalidAssistMode       ((uint8_T)1U)
#define FMS_IN_InvalidAutoMode         ((uint8_T)1U)
#define FMS_IN_Listen                  ((uint8_T)2U)
#define FMS_IN_Loiter                  ((uint8_T)1U)
#define FMS_IN_Lost                    ((uint8_T)2U)
#define FMS_IN_Manual                  ((uint8_T)4U)
#define FMS_IN_Manual_g                ((uint8_T)2U)
#define FMS_IN_Mission                 ((uint8_T)2U)
#define FMS_IN_NextWP                  ((uint8_T)2U)
#define FMS_IN_Offboard                ((uint8_T)3U)
#define FMS_IN_Position                ((uint8_T)2U)
#define FMS_IN_Return                  ((uint8_T)2U)
#define FMS_IN_Return_h                ((uint8_T)3U)
#define FMS_IN_Run                     ((uint8_T)2U)
#define FMS_IN_Send                    ((uint8_T)3U)
#define FMS_IN_Stabilize               ((uint8_T)3U)
#define FMS_IN_Standby                 ((uint8_T)3U)
#define FMS_IN_SubMode                 ((uint8_T)5U)
#define FMS_IN_Waypoint                ((uint8_T)4U)
#define FMS_event_DisarmEvent          (0)

/* Named constants for Chart: '<Root>/SafeMode' */
#define FMS_IN_Manual_b                ((uint8_T)1U)
#define FMS_IN_Other                   ((uint8_T)4U)
#define FMS_IN_Position_h              ((uint8_T)5U)
#define FMS_IN_Stabilize_m             ((uint8_T)6U)

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
  ,                                    /* home */
  0.0F                                 /* local_psi */
} ;                                    /* FMS_Out_Bus ground */

/* Exported block parameters */
struct_TneGpl6isYNp9l2B8ROowD FMS_PARAM = {
  { 1500.0F, 1500.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F },

  { 1500.0F, 1500.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F },
  6.0F,
  5.0F,
  2.0F,
  5.0F,
  1.0F,
  1.0F,
  1.0F,
  10U,
  1U
} ;                                    /* Variable: FMS_PARAM
                                        * Referenced by:
                                        *   '<Root>/ACCEPT_R'
                                        *   '<S3>/Constant1'
                                        *   '<S23>/Constant'
                                        *   '<S29>/Constant6'
                                        *   '<S30>/Constant6'
                                        *   '<S154>/L1'
                                        *   '<S154>/vel'
                                        *   '<S40>/Gain'
                                        *   '<S94>/L1'
                                        *   '<S94>/vel'
                                        *   '<S159>/AY_P'
                                        *   '<S98>/AY_P'
                                        *   '<S44>/L1'
                                        *   '<S44>/AY_P'
                                        *   '<S45>/AY_P'
                                        *   '<S45>/AY_P1'
                                        *   '<S45>/VEL'
                                        *   '<S82>/Gain2'
                                        *   '<S163>/Gain2'
                                        *   '<S102>/Gain2'
                                        */

struct_TYt7YeNdxIDXfczXumtXXB FMS_EXPORT = {
  10U,

  { 66, 111, 97, 116, 32, 70, 77, 83, 32, 118, 49, 46, 48, 46, 48, 0 }
} ;                                    /* Variable: FMS_EXPORT
                                        * Referenced by:
                                        *   '<S1>/Constant'
                                        *   '<S11>/Constant1'
                                        *   '<S193>/Constant'
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
static void FMS_sf_msg_send_M(void);
static boolean_T FMS_CheckCmdValid(FMS_Cmd cmd_in, PilotMode mode_in, uint32_T
  ins_flag);
static boolean_T FMS_BottomRight(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle);
static boolean_T FMS_BottomLeft(real32_T pilot_cmd_stick_yaw, real32_T
  pilot_cmd_stick_throttle);
static boolean_T FMS_sf_msg_pop_M(void);
static real32_T FMS_norm(const real32_T x[2]);
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
 * Output and update for atomic system:
 *    '<S178>/NearbyRefWP'
 *    '<S119>/NearbyRefWP'
 */
void FMS_NearbyRefWP(const real32_T rtu_P2[2], real32_T rtu_P3, real32_T
                     rtu_P3_d, real32_T rtu_L1, real32_T rty_P[2], real32_T
                     *rty_d)
{
  real32_T P3P2_idx_0;
  real32_T P3P2_idx_1;

  /* SignalConversion: '<S182>/TmpSignal ConversionAt SFunction Inport2' */
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
 * Output and update for atomic system:
 *    '<S178>/OutRegionRegWP'
 *    '<S119>/OutRegionRegWP'
 */
void FMS_OutRegionRegWP(const real32_T rtu_P1[2], const real32_T rtu_P2[2],
  real32_T rtu_P3, real32_T rtu_P3_c, real32_T rty_P[2])
{
  real32_T P1P3_idx_0;
  real32_T P1P3_idx_1;
  rty_P[0] = rtu_P2[0] - rtu_P1[0];
  rty_P[1] = rtu_P2[1] - rtu_P1[1];

  /* SignalConversion: '<S183>/TmpSignal ConversionAt SFunction Inport3' */
  P1P3_idx_0 = rtu_P3 - rtu_P1[0];
  P1P3_idx_1 = rtu_P3_c - rtu_P1[1];
  P1P3_idx_0 = (P1P3_idx_0 * rty_P[0] + P1P3_idx_1 * rty_P[1]) / (rty_P[0] *
    rty_P[0] + rty_P[1] * rty_P[1]);
  if (P1P3_idx_0 <= 0.0F) {
    rty_P[0] = rtu_P1[0];
    rty_P[1] = rtu_P1[1];
  } else if (P1P3_idx_0 >= 1.0F) {
    rty_P[0] = rtu_P2[0];
    rty_P[1] = rtu_P2[1];
  } else {
    rty_P[0] = P1P3_idx_0 * rty_P[0] + rtu_P1[0];
    rty_P[1] = P1P3_idx_0 * rty_P[1] + rtu_P1[1];
  }
}

/*
 * Output and update for atomic system:
 *    '<S178>/SearchL1RefWP'
 *    '<S119>/SearchL1RefWP'
 */
void FMS_SearchL1RefWP(const real32_T rtu_P1[2], const real32_T rtu_P2[2],
  real32_T rtu_P3, real32_T rtu_P3_o, real32_T rtu_L1, real32_T rty_P[2],
  real32_T *rty_u)
{
  real32_T A;
  real32_T B;
  real32_T D;
  real32_T a;
  real32_T u1_tmp;
  boolean_T guard1 = false;
  a = rtu_P2[0] - rtu_P1[0];
  A = rtu_P2[1] - rtu_P1[1];
  A = a * a + A * A;

  /* SignalConversion: '<S184>/TmpSignal ConversionAt SFunction Inport3' */
  B = ((rtu_P2[0] - rtu_P1[0]) * (rtu_P1[0] - rtu_P3) + (rtu_P2[1] - rtu_P1[1]) *
       (rtu_P1[1] - rtu_P3_o)) * 2.0F;
  D = B * B - (((((rtu_P3 * rtu_P3 + rtu_P3_o * rtu_P3_o) + rtu_P1[0] * rtu_P1[0])
                 + rtu_P1[1] * rtu_P1[1]) - (rtu_P3 * rtu_P1[0] + rtu_P3_o *
    rtu_P1[1]) * 2.0F) - rtu_L1 * rtu_L1) * (4.0F * A);
  a = -1.0F;
  rty_P[0] = 0.0F;
  rty_P[1] = 0.0F;
  guard1 = false;
  if (D > 0.0F) {
    u1_tmp = sqrtf(D);
    D = (-B + u1_tmp) / (2.0F * A);
    A = (-B - u1_tmp) / (2.0F * A);
    if ((D >= 0.0F) && (D <= 1.0F) && (A >= 0.0F) && (A <= 1.0F)) {
      a = fmaxf(D, A);
      guard1 = true;
    } else if ((D >= 0.0F) && (D <= 1.0F)) {
      a = D;
      guard1 = true;
    } else {
      if ((A >= 0.0F) && (A <= 1.0F)) {
        a = A;
        guard1 = true;
      }
    }
  } else {
    if (D == 0.0F) {
      D = -B / (2.0F * A);
      if ((D >= 0.0F) && (D <= 1.0F)) {
        a = D;
        guard1 = true;
      }
    }
  }

  if (guard1) {
    rty_P[0] = (rtu_P2[0] - rtu_P1[0]) * a + rtu_P1[0];
    rty_P[1] = (rtu_P2[1] - rtu_P1[1]) * a + rtu_P1[1];
  }

  *rty_u = a;
}

/*
 * Output and update for action system:
 *    '<S34>/Unknown'
 *    '<S32>/Unknown'
 *    '<S31>/Unknown'
 *    '<S27>/Unknown'
 */
void FMS_Unknown(FMS_Out_Bus *rty_FMS_Out, const ConstB_Unknown_FMS_T *localC)
{
  int32_T i;

  /* BusAssignment: '<S155>/Bus Assignment' incorporates:
   *  Constant: '<S155>/Constant'
   *  Constant: '<S155>/Constant2'
   *  SignalConversion: '<S155>/TmpHiddenBufferAtBus AssignmentInport1'
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

  /* End of BusAssignment: '<S155>/Bus Assignment' */
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
  FMS_emplace(&FMS_DW.Queue_FMS_Cmd_g, &FMS_DW.M_msgReservedData);
}

/* Function for Chart: '<Root>/FMS State Machine' */
static boolean_T FMS_CheckCmdValid(FMS_Cmd cmd_in, PilotMode mode_in, uint32_T
  ins_flag)
{
  boolean_T valid;
  valid = false;
  if ((ins_flag & 1U) != 0U) {
    switch (cmd_in) {
     case FMS_Cmd_Return:
     case FMS_Cmd_Pause:
      if (((ins_flag & 8U) != 0U) && ((ins_flag & 16U) != 0U) && ((ins_flag &
            64U) != 0U)) {
        valid = true;
      }
      break;

     case FMS_Cmd_PreArm:
      switch (mode_in) {
       case PilotMode_Position:
       case PilotMode_Offboard:
        if (((ins_flag & 8U) != 0U) && ((ins_flag & 16U) != 0U) && ((ins_flag &
              64U) != 0U)) {
          valid = true;
        }
        break;

       case PilotMode_Mission:
        if (((ins_flag & 8U) != 0U) && ((ins_flag & 16U) != 0U) && ((ins_flag &
              32U) != 0U) && ((ins_flag & 64U) != 0U)) {
          valid = true;
        }
        break;

       case PilotMode_Stabilize:
        valid = true;
        break;

       case PilotMode_Manual:
        valid = true;
        break;
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
    FMS_DW.M_msgHandle = FMS_pop(&FMS_DW.Queue_FMS_Cmd_g, &FMS_DW.Msg_FMS_Cmd_a
      [0]) != 0 ? (void *)&FMS_DW.Msg_FMS_Cmd_a[0] : NULL;
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
    FMS_B.Cmd_In.cur_waypoint[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
    FMS_B.Cmd_In.cur_waypoint[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
    FMS_B.Cmd_In.cur_waypoint[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
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
     case PilotMode_Stabilize:
      FMS_DW.is_Assist = FMS_IN_Stabilize;
      FMS_B.state = VehicleState_Stabilize;
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
  /* RelationalOperator: '<S206>/Compare' incorporates:
   *  Abs: '<S195>/Abs'
   *  Constant: '<S206>/Constant'
   *  MinMax: '<S195>/Max'
   *  Sum: '<S195>/Sum'
   */
  FMS_B.Compare_k = (fmax(fmax(fmax(fabs(FMS_B.stick_val[0] -
    FMS_B.pilot_cmd.stick_yaw), fabs(FMS_B.stick_val[1] -
    FMS_B.pilot_cmd.stick_throttle)), fabs(FMS_B.stick_val[2] -
    FMS_B.pilot_cmd.stick_roll)), fabs(FMS_B.stick_val[3] -
    FMS_B.pilot_cmd.stick_pitch)) >= 0.1);

  /* End of Outputs for SubSystem: '<S5>/Vehicle.StickMoved' */
  if (FMS_B.Compare_k && (FMS_B.target_mode != PilotMode_None)) {
    tmp = FMS_getArmMode(FMS_B.target_mode);
    if (tmp == 3.0) {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Arm = FMS_IN_Auto;
      FMS_enter_internal_Auto();
    } else if (tmp == 2.0) {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Arm = FMS_IN_Assist;
      switch (FMS_B.target_mode) {
       case PilotMode_Stabilize:
        FMS_DW.is_Assist = FMS_IN_Stabilize;
        FMS_B.state = VehicleState_Stabilize;
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
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Arm = FMS_IN_Manual;
      if (FMS_B.target_mode == PilotMode_Manual) {
        FMS_DW.is_Manual = FMS_IN_Manual_g;
        FMS_B.state = VehicleState_Manual;
      } else {
        FMS_DW.is_Manual = FMS_IN_InValidManualMode;
      }
    } else {
      FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Arm = FMS_IN_InvalidArmMode;
    }
  } else {
    if (FMS_sf_msg_pop_M()) {
      b_sf_internal_predicateOutput = (FMS_DW.M_msgReservedData == FMS_Cmd_Pause);
    } else {
      b_sf_internal_predicateOutput = false;
    }

    if (b_sf_internal_predicateOutput) {
      FMS_DW.is_SubMode = FMS_IN_Hold_h;
      FMS_B.state = VehicleState_Hold;
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
          FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
          FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
          FMS_enter_internal_Arm();
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
          FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
          FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
          FMS_DW.is_Vehicle = FMS_IN_Disarm;
          FMS_B.state = VehicleState_Disarm;
        }

        /* End of Constant: '<Root>/ACCEPT_R' */
        break;
      }
    }
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_exit_internal_Arm(void)
{
  if (FMS_DW.is_Arm == FMS_IN_Auto) {
    if (FMS_DW.is_Auto == FMS_IN_Mission) {
      FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
    } else {
      FMS_DW.is_Offboard = FMS_IN_NO_ACTIVE_CHILD;
      FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
    }

    FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
  } else {
    FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD;
    FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD;
    FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
    FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
  }
}

/* Function for Chart: '<Root>/FMS State Machine' */
static void FMS_Arm(void)
{
  boolean_T b_sf_internal_predicateOutput;
  real_T rtb_Multiply2;
  real_T rtb_Sum2_m;
  real_T rtb_Gain;
  real_T rtb_Sum_d;
  real32_T tmp[2];
  real_T rtb_Sum_idx_0;
  uint32_T qY;
  int32_T lla_tmp;
  if ((FMS_DW.mode_prev != FMS_DW.mode_start) && (FMS_B.target_mode !=
       PilotMode_None)) {
    rtb_Sum_idx_0 = FMS_getArmMode(FMS_B.target_mode);
    if (rtb_Sum_idx_0 == 3.0) {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_Auto;
      FMS_enter_internal_Auto();
    } else if (rtb_Sum_idx_0 == 2.0) {
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_Assist;
      switch (FMS_B.target_mode) {
       case PilotMode_Stabilize:
        FMS_DW.is_Assist = FMS_IN_Stabilize;
        FMS_B.state = VehicleState_Stabilize;
        break;

       case PilotMode_Position:
        FMS_DW.is_Assist = FMS_IN_Position;
        FMS_B.state = VehicleState_Position;
        break;

       default:
        FMS_DW.is_Assist = FMS_IN_InvalidAssistMode;
        break;
      }
    } else if (rtb_Sum_idx_0 == 1.0) {
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
      b_sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
        FMS_Cmd_Return);
    } else {
      b_sf_internal_predicateOutput = false;
    }

    if (b_sf_internal_predicateOutput) {
      FMS_B.Cmd_In.sp_waypoint[0] = FMS_DW.home[0];
      FMS_B.Cmd_In.sp_waypoint[1] = FMS_DW.home[1];
      FMS_B.Cmd_In.sp_waypoint[2] = 0.0F;
      FMS_exit_internal_Arm();
      FMS_DW.is_Arm = FMS_IN_SubMode;
      FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
      FMS_DW.stick_val[1] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
      FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
      FMS_DW.stick_val[3] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
      FMS_DW.is_SubMode = FMS_IN_Return;
      FMS_B.state = VehicleState_Return;
    } else {
      switch (FMS_DW.is_Arm) {
       case FMS_IN_Assist:
        if (FMS_B.Compare) {
          FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD;
          FMS_DW.is_Vehicle = FMS_IN_Arm;
          FMS_B.Cmd_In.local_psi = FMS_B.BusConversion_InsertedFor_FMSSt.psi;
          FMS_DW.is_Arm = FMS_IN_SubMode;
          FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
          FMS_DW.stick_val[1] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
          FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
          FMS_DW.stick_val[3] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
          FMS_DW.is_SubMode = FMS_IN_Hold_h;
          FMS_B.state = VehicleState_Hold;
        } else {
          if (FMS_DW.is_Assist == FMS_IN_InvalidAssistMode) {
            FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
          }
        }
        break;

       case FMS_IN_Auto:
        if (FMS_sf_msg_pop_M()) {
          b_sf_internal_predicateOutput = (FMS_DW.M_msgReservedData ==
            FMS_Cmd_Pause);
        } else {
          b_sf_internal_predicateOutput = false;
        }

        if (b_sf_internal_predicateOutput) {
          if (FMS_DW.is_Auto == FMS_IN_Mission) {
            FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
          } else {
            FMS_DW.is_Offboard = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
          }

          FMS_DW.is_Arm = FMS_IN_SubMode;
          FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
          FMS_DW.stick_val[1] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
          FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
          FMS_DW.stick_val[3] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
          FMS_DW.is_SubMode = FMS_IN_Hold_h;
          FMS_B.state = VehicleState_Hold;
        } else {
          switch (FMS_DW.is_Auto) {
           case FMS_IN_InvalidAutoMode:
            FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
            break;

           case FMS_IN_Mission:
            if (FMS_DW.mission_timestamp_prev != FMS_DW.mission_timestamp_start)
            {
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
               case FMS_IN_Loiter:
                break;

               case FMS_IN_NextWP:
                if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Waypoint) {
                  FMS_DW.is_Mission = FMS_IN_Waypoint;
                  lla_tmp = FMS_B.wp_index - 1;

                  /* Inport: '<Root>/Mission_Data' */
                  FMS_B.lla[0] = (real_T)FMS_U.Mission_Data.x[lla_tmp] * 1.0E-7;
                  FMS_B.lla[1] = (real_T)FMS_U.Mission_Data.y[lla_tmp] * 1.0E-7;
                  FMS_B.lla[2] = -(FMS_U.Mission_Data.z[lla_tmp] + FMS_DW.home[2]);
                  FMS_B.href = 0.0;
                  FMS_B.psio = 0.0;
                  FMS_B.llo[0] = FMS_DW.llo[0];

                  /* Outputs for Function Call SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
                  /* Sum: '<S196>/Sum' */
                  rtb_Sum_idx_0 = FMS_B.lla[0] - FMS_B.llo[0];

                  /* End of Outputs for SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
                  FMS_B.llo[1] = FMS_DW.llo[1];

                  /* Outputs for Function Call SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
                  /* Gain: '<S196>/deg2rad' */
                  rtb_Multiply2 = 0.017453292519943295 * FMS_B.llo[0];

                  /* Trigonometry: '<S197>/Sin' */
                  rtb_Sum2_m = sin(rtb_Multiply2);

                  /* Math: '<S197>/Square1' */
                  rtb_Sum2_m *= rtb_Sum2_m;

                  /* Product: '<S197>/Multiply1' incorporates:
                   *  Product: '<S197>/Multiply'
                   */
                  rtb_Gain = FMS_ConstB.ff * rtb_Sum2_m;

                  /* Product: '<S197>/Divide' incorporates:
                   *  Constant: '<S197>/Constant'
                   *  Constant: '<S197>/R'
                   *  Sqrt: '<S197>/Sqrt'
                   *  Sum: '<S197>/Sum1'
                   */
                  rtb_Sum2_m = 6.378137E+6 / sqrt(1.0 - rtb_Gain);

                  /* Product: '<S197>/Product3' incorporates:
                   *  Constant: '<S197>/Constant1'
                   *  Product: '<S197>/Multiply1'
                   *  Sum: '<S197>/Sum2'
                   */
                  rtb_Gain = 1.0 / (1.0 - rtb_Gain) * FMS_ConstB.Sum4 *
                    rtb_Sum2_m;

                  /* Product: '<S197>/Multiply2' incorporates:
                   *  Trigonometry: '<S197>/Cos'
                   */
                  rtb_Sum2_m *= cos(rtb_Multiply2);

                  /* Abs: '<S201>/Abs' incorporates:
                   *  Abs: '<S204>/Abs1'
                   *  Switch: '<S201>/Switch1'
                   */
                  rtb_Multiply2 = fabs(rtb_Sum_idx_0);

                  /* Switch: '<S201>/Switch1' incorporates:
                   *  Abs: '<S201>/Abs'
                   *  Bias: '<S201>/Bias2'
                   *  Bias: '<S201>/Bias3'
                   *  Constant: '<S198>/Constant'
                   *  Constant: '<S198>/Constant1'
                   *  Constant: '<S203>/Constant'
                   *  Gain: '<S201>/Gain1'
                   *  Product: '<S201>/Multiply'
                   *  RelationalOperator: '<S203>/Compare'
                   *  Switch: '<S198>/Switch'
                   */
                  if (rtb_Multiply2 > 90.0) {
                    /* Switch: '<S204>/Switch1' incorporates:
                     *  Bias: '<S204>/Bias2'
                     *  Bias: '<S204>/Bias3'
                     *  Constant: '<S204>/Constant'
                     *  Constant: '<S205>/Constant'
                     *  Math: '<S204>/Math Function'
                     *  RelationalOperator: '<S205>/Compare'
                     */
                    if (rtb_Multiply2 > 180.0) {
                      rtb_Sum_idx_0 = rt_modd(rtb_Sum_idx_0 + 180.0, 360.0) +
                        -180.0;
                    }

                    /* End of Switch: '<S204>/Switch1' */

                    /* Signum: '<S201>/Sign' */
                    if (rtb_Sum_idx_0 < 0.0) {
                      rtb_Sum_idx_0 = -1.0;
                    } else {
                      if (rtb_Sum_idx_0 > 0.0) {
                        rtb_Sum_idx_0 = 1.0;
                      }
                    }

                    /* End of Signum: '<S201>/Sign' */
                    rtb_Multiply2 = (-(rtb_Multiply2 + -90.0) + 90.0) *
                      rtb_Sum_idx_0;
                    lla_tmp = 180;
                  } else {
                    rtb_Multiply2 = rtb_Sum_idx_0;
                    lla_tmp = 0;
                  }

                  /* Sum: '<S198>/Sum' incorporates:
                   *  Sum: '<S196>/Sum'
                   */
                  rtb_Sum_d = (FMS_B.lla[1] - FMS_B.llo[1]) + (real_T)lla_tmp;

                  /* Product: '<S196>/Multiply' incorporates:
                   *  Gain: '<S196>/deg2rad1'
                   */
                  rtb_Sum_idx_0 = 0.017453292519943295 * rtb_Multiply2 *
                    rtb_Gain;

                  /* Switch: '<S200>/Switch1' incorporates:
                   *  Abs: '<S200>/Abs1'
                   *  Bias: '<S200>/Bias2'
                   *  Bias: '<S200>/Bias3'
                   *  Constant: '<S200>/Constant'
                   *  Constant: '<S202>/Constant'
                   *  Math: '<S200>/Math Function'
                   *  RelationalOperator: '<S202>/Compare'
                   */
                  if (fabs(rtb_Sum_d) > 180.0) {
                    rtb_Sum_d = rt_modd(rtb_Sum_d + 180.0, 360.0) + -180.0;
                  }

                  /* End of Switch: '<S200>/Switch1' */

                  /* Product: '<S196>/Multiply' incorporates:
                   *  Gain: '<S196>/deg2rad1'
                   */
                  rtb_Multiply2 = 0.017453292519943295 * rtb_Sum_d * rtb_Sum2_m;

                  /* Gain: '<S196>/deg2rad2' */
                  rtb_Sum2_m = 0.017453292519943295 * FMS_B.psio;

                  /* Trigonometry: '<S199>/SinCos' */
                  rtb_Sum_d = sin(rtb_Sum2_m);
                  rtb_Gain = cos(rtb_Sum2_m);

                  /* Sum: '<S199>/Sum2' incorporates:
                   *  Product: '<S199>/Multiply1'
                   *  Product: '<S199>/Multiply2'
                   */
                  rtb_Sum2_m = rtb_Sum_idx_0 * rtb_Gain + rtb_Multiply2 *
                    rtb_Sum_d;

                  /* Product: '<S199>/Multiply3' */
                  rtb_Sum_d *= rtb_Sum_idx_0;

                  /* Product: '<S199>/Multiply4' */
                  rtb_Gain *= rtb_Multiply2;

                  /* Sum: '<S199>/Sum3' */
                  rtb_Sum_d = rtb_Gain - rtb_Sum_d;

                  /* DataTypeConversion: '<S194>/Data Type Conversion' incorporates:
                   *  Gain: '<S196>/Gain'
                   *  Sum: '<S196>/Sum1'
                   */
                  FMS_B.DataTypeConversion[0] = (real32_T)rtb_Sum2_m;
                  FMS_B.DataTypeConversion[1] = (real32_T)rtb_Sum_d;
                  FMS_B.DataTypeConversion[2] = (real32_T)-(FMS_B.lla[2] +
                    FMS_B.href);

                  /* End of Outputs for SubSystem: '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
                  FMS_B.Cmd_In.sp_waypoint[0] = FMS_B.DataTypeConversion[0];
                  FMS_B.Cmd_In.sp_waypoint[1] = FMS_B.DataTypeConversion[1];
                  FMS_B.Cmd_In.sp_waypoint[2] = FMS_B.DataTypeConversion[2];
                  FMS_B.state = VehicleState_Mission;
                } else if (FMS_DW.nav_cmd == (int32_T)NAV_Cmd_Return) {
                  FMS_DW.is_Mission = FMS_IN_Return_h;
                  FMS_B.Cmd_In.sp_waypoint[0] = FMS_DW.home[0];
                  FMS_B.Cmd_In.sp_waypoint[1] = FMS_DW.home[1];
                  FMS_B.Cmd_In.sp_waypoint[2] = FMS_DW.home[2];
                  FMS_B.state = VehicleState_Return;
                } else {
                  FMS_DW.is_Mission = FMS_IN_Loiter;
                  FMS_B.state = VehicleState_Hold;
                }
                break;

               case FMS_IN_Return_h:
                tmp[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R -
                  FMS_B.Cmd_In.sp_waypoint[0];
                tmp[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R -
                  FMS_B.Cmd_In.sp_waypoint[1];
                if (FMS_norm(tmp) < 0.5F) {
                  lla_tmp = (int32_T)(FMS_B.wp_index + 1U);
                  if ((uint32_T)lla_tmp > 255U) {
                    lla_tmp = 255;
                  }

                  FMS_B.wp_index = (uint8_T)lla_tmp;
                  FMS_DW.is_Mission = FMS_IN_NextWP;

                  /* Inport: '<Root>/Mission_Data' */
                  if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
                    FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index -
                      1];
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
                  FMS_B.Cmd_In.cur_waypoint[0] = FMS_B.Cmd_In.sp_waypoint[0];
                  FMS_B.Cmd_In.cur_waypoint[1] = FMS_B.Cmd_In.sp_waypoint[1];
                  FMS_B.Cmd_In.cur_waypoint[2] = FMS_B.Cmd_In.sp_waypoint[2];
                  lla_tmp = (int32_T)(FMS_B.wp_index + 1U);
                  if ((uint32_T)lla_tmp > 255U) {
                    lla_tmp = 255;
                  }

                  FMS_B.wp_index = (uint8_T)lla_tmp;
                  FMS_DW.is_Mission = FMS_IN_NextWP;

                  /* Inport: '<Root>/Mission_Data' */
                  if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
                    FMS_DW.nav_cmd = FMS_U.Mission_Data.command[FMS_B.wp_index -
                      1];
                  } else {
                    FMS_DW.nav_cmd = (uint16_T)NAV_Cmd_None;
                    qY = FMS_B.wp_index - /*MW:OvSatOk*/ 1U;
                    if (qY > FMS_B.wp_index) {
                      qY = 0U;
                    }

                    FMS_B.wp_consume = (uint8_T)qY;
                  }
                }

                /* End of Constant: '<Root>/ACCEPT_R' */
                break;
              }
            }
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
        }
        break;

       case FMS_IN_InvalidArmMode:
        FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
        FMS_DW.is_Vehicle = FMS_IN_Disarm;
        FMS_B.state = VehicleState_Disarm;
        break;

       case FMS_IN_Manual:
        if (FMS_B.Compare) {
          FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD;
          FMS_DW.is_Vehicle = FMS_IN_Arm;
          FMS_B.Cmd_In.local_psi = FMS_B.BusConversion_InsertedFor_FMSSt.psi;
          FMS_DW.is_Arm = FMS_IN_SubMode;
          FMS_DW.stick_val[0] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw;
          FMS_DW.stick_val[1] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle;
          FMS_DW.stick_val[2] = FMS_B.BusConversion_InsertedFor_FMS_f.stick_roll;
          FMS_DW.stick_val[3] =
            FMS_B.BusConversion_InsertedFor_FMS_f.stick_pitch;
          FMS_DW.is_SubMode = FMS_IN_Hold_h;
          FMS_B.state = VehicleState_Hold;
        } else {
          if (FMS_DW.is_Manual == FMS_IN_InValidManualMode) {
            FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
            FMS_DW.is_Vehicle = FMS_IN_Disarm;
            FMS_B.state = VehicleState_Disarm;
          }
        }
        break;

       case FMS_IN_SubMode:
        FMS_SubMode();
        break;
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
   case PilotMode_Stabilize:
   case PilotMode_Position:
    if (pilot_cmd_stick_throttle > 0.0F) {
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
  boolean_T guard1 = false;
  boolean_T guard2 = false;
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
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD;
      break;

     case FMS_IN_Standby:
      FMS_DW.prep_mission = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_mission == 1.0);
      if ((!sf_internal_predicateOutput) || (!FMS_DW.condWasTrueAtLastTimeStep_1))
      {
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1 = sf_internal_predicateOutput;
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD;
      break;

     default:
      FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD;
      break;
    }

    FMS_DW.is_Vehicle = FMS_IN_Disarm;
    FMS_B.state = VehicleState_Disarm;
  } else {
    guard1 = false;
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
        FMS_DW.condWasTrueAtLastTimeStep_1 = false;
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
        FMS_DW.is_Vehicle = FMS_IN_Standby;
        FMS_DW.temporalCounter_i1 = 0U;
        guard2 = false;
        if (FMS_B.target_mode == PilotMode_Mission) {
          if (FMS_B.wp_index <= FMS_U.Mission_Data.valid_items) {
            FMS_DW.prep_mission = 1.0;
            FMS_DW.condWasTrueAtLastTimeStep_1 = (FMS_DW.prep_mission == 1.0);
            guard2 = true;
          } else {
            b_previousEvent = FMS_DW.sfEvent;
            FMS_DW.sfEvent = FMS_event_DisarmEvent;

            /* Chart: '<Root>/FMS State Machine' */
            FMS_c11_FMS();
            FMS_DW.sfEvent = b_previousEvent;
            if (FMS_DW.is_Vehicle != FMS_IN_Standby) {
            } else {
              guard2 = true;
            }
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          FMS_DW.home[0] = FMS_B.BusConversion_InsertedFor_FMSSt.x_R;
          FMS_DW.home[1] = FMS_B.BusConversion_InsertedFor_FMSSt.y_R;
          FMS_DW.home[2] = FMS_B.BusConversion_InsertedFor_FMSSt.h_R;
          FMS_B.state = VehicleState_Standby;
        }

        if (FMS_DW.is_Vehicle == FMS_IN_Standby) {
          sf_internal_predicateOutput = (FMS_DW.prep_mission == 1.0);
          if ((!sf_internal_predicateOutput) ||
              (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
            FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
          }

          FMS_DW.condWasTrueAtLastTimeStep_1 = sf_internal_predicateOutput;
        }
      }
      break;

     case FMS_IN_Standby:
      if ((FMS_ManualArmEvent
           (FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle,
            FMS_B.BusConversion_InsertedFor_FMS_f.mode) == 1.0) &&
          (FMS_B.target_mode != PilotMode_None)) {
        guard1 = true;
      } else if ((FMS_DW.temporalCounter_i1 >= 1000U) || (FMS_DW.sfEvent ==
                  FMS_event_DisarmEvent)) {
        FMS_DW.prep_mission = 0.0;
        sf_internal_predicateOutput = (FMS_DW.prep_mission == 1.0);
        if ((!sf_internal_predicateOutput) ||
            (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
          FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1 = sf_internal_predicateOutput;
        FMS_DW.is_Vehicle = FMS_IN_Disarm;
        FMS_B.state = VehicleState_Disarm;
      } else {
        sf_internal_predicateOutput = (FMS_DW.prep_mission == 1.0);
        if ((!sf_internal_predicateOutput) ||
            (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
          FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1 = sf_internal_predicateOutput;
        if ((FMS_DW.chartAbsoluteTimeCounter -
             FMS_DW.durationLastReferenceTick_1 >= 200) || ((FMS_B.target_mode ==
              PilotMode_Offboard) && FMS_B.LogicalOperator)) {
          guard1 = true;
        }
      }
      break;
    }

    if (guard1) {
      FMS_DW.prep_mission = 0.0;
      sf_internal_predicateOutput = (FMS_DW.prep_mission == 1.0);
      if ((!sf_internal_predicateOutput) || (!FMS_DW.condWasTrueAtLastTimeStep_1))
      {
        FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
      }

      FMS_DW.condWasTrueAtLastTimeStep_1 = sf_internal_predicateOutput;
      FMS_DW.is_Vehicle = FMS_IN_Arm;
      FMS_B.Cmd_In.local_psi = FMS_B.BusConversion_InsertedFor_FMSSt.psi;
      FMS_enter_internal_Arm();
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
    FMS_DW.cmd_prev = FMS_B.Switch1;
    FMS_DW.cmd_start = FMS_B.Switch1;
    FMS_DW.mode_prev = FMS_B.target_mode;
    FMS_DW.mode_start = FMS_B.target_mode;
    FMS_DW.chartAbsoluteTimeCounter = 0;
    FMS_DW.is_active_c11_FMS = 1U;
    FMS_DW.is_active_Command_Listener = 1U;
    FMS_DW.is_Command_Listener = FMS_IN_Listen;
    FMS_DW.is_active_Combo_Stick = 1U;
    FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
    FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
    FMS_DW.is_Combo_Stick = FMS_IN_Idle;
    FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
    FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
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
          FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.is_Combo_Stick = FMS_IN_Idle;
          FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
          FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
        }
        break;

       case FMS_IN_Disarm:
        if (!FMS_BottomLeft(FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
                            FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle))
        {
          FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
          FMS_DW.is_Combo_Stick = FMS_IN_Idle;
          FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
          FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
        }
        break;

       case FMS_IN_Idle:
        if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_k)) {
          FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
        }

        FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
        if (FMS_DW.chartAbsoluteTimeCounter -
            FMS_DW.durationLastReferenceTick_1_k > 150) {
          FMS_DW.is_Combo_Stick = FMS_IN_Arm;
          FMS_DW.M_msgReservedData = FMS_Cmd_PreArm;
          FMS_sf_msg_send_M();
        } else {
          if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
            FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
          }

          FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
          if (FMS_DW.chartAbsoluteTimeCounter -
              FMS_DW.durationLastReferenceTick_2 > 150) {
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
            if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
              FMS_DW.durationLastReferenceTick_2 =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
            FMS_DW.br = FMS_BottomRight
              (FMS_B.BusConversion_InsertedFor_FMS_f.stick_yaw,
               FMS_B.BusConversion_InsertedFor_FMS_f.stick_throttle);
            if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_k)) {
              FMS_DW.durationLastReferenceTick_1_k =
                FMS_DW.chartAbsoluteTimeCounter;
            }

            FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
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
  FMS_initQueue((Queue_FMS_Cmd *)&FMS_DW.Queue_FMS_Cmd_g, MSG_FIFO_QUEUE, 10,
                (Msg_FMS_Cmd *)&FMS_DW.Msg_FMS_Cmd_a[1]);
}

/* Model step function */
void FMS_step(void)
{
  boolean_T rtb_FixPtRelationalOperator;
  int8_T rtPrevAction;
  real32_T rtb_TmpSignalConversionAtMat_co[2];
  real32_T rtb_MathFunction_i2[2];
  real32_T rtb_Subtract1_j;
  real32_T rtb_Sign5_d;
  real32_T rtb_Rem_kb;
  real32_T rtb_Sum_i;
  MotionState rtb_state_d;
  real32_T rtb_VectorConcatenate_f[9];
  boolean_T rtb_NOT_a;
  real32_T rtb_Switch_i[3];
  real32_T rtb_d_f;
  real32_T rtb_Switch_k1[3];
  real32_T rtb_sin_alpha_j;
  real32_T rtb_P_c[2];
  real32_T rtb_u_g;
  real32_T rtb_P_on[2];
  int32_T i;
  real32_T rtb_Switch_mc_idx_0;
  real32_T rtb_Switch_mc_idx_1;
  real32_T rtb_Subtract1_k_0;
  uint32_T rtb_Compare_iv_tmp;

  /* Outputs for Atomic SubSystem: '<Root>/CommandProcess' */
  /* DiscreteIntegrator: '<S11>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *  RelationalOperator: '<S15>/FixPt Relational Operator'
   *  UnitDelay: '<S15>/Delay Input1'
   *
   * Block description for '<S15>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Pilot_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE_ak) {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = 0U;
  }

  /* Switch: '<S13>/Switch' incorporates:
   *  Constant: '<S14>/Constant'
   *  Constant: '<S18>/Constant'
   *  Constant: '<S19>/Constant'
   *  DataTypeConversion: '<S13>/Data Type Conversion2'
   *  Delay: '<S13>/Delay'
   *  DiscreteIntegrator: '<S11>/Discrete-Time Integrator1'
   *  Inport: '<Root>/GCS_Cmd'
   *  Inport: '<Root>/Pilot_Cmd'
   *  Logic: '<S13>/Logical Operator'
   *  Logic: '<S13>/Logical Operator1'
   *  RelationalOperator: '<S14>/Compare'
   *  RelationalOperator: '<S18>/Compare'
   *  RelationalOperator: '<S19>/Compare'
   *  RelationalOperator: '<S20>/FixPt Relational Operator'
   *  Switch: '<S13>/Switch1'
   *  UnitDelay: '<S20>/Delay Input1'
   *
   * Block description for '<S20>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if ((FMS_U.Pilot_Cmd.mode != 0U) && (FMS_DW.DiscreteTimeIntegrator1_DSTAT_b <
       500U)) {
    FMS_DW.Delay_DSTATE_c = (PilotMode)FMS_U.Pilot_Cmd.mode;
  } else {
    if ((FMS_U.GCS_Cmd.mode != FMS_DW.DelayInput1_DSTATE_f) &&
        (FMS_U.GCS_Cmd.mode != 0U)) {
      /* Switch: '<S13>/Switch1' incorporates:
       *  DataTypeConversion: '<S13>/Data Type Conversion1'
       *  Delay: '<S13>/Delay'
       *  Inport: '<Root>/GCS_Cmd'
       */
      FMS_DW.Delay_DSTATE_c = (PilotMode)FMS_U.GCS_Cmd.mode;
    }
  }

  /* End of Switch: '<S13>/Switch' */

  /* Switch: '<S12>/Switch1' incorporates:
   *  DataTypeConversion: '<S12>/Data Type Conversion2'
   *  Inport: '<Root>/GCS_Cmd'
   *  Inport: '<Root>/Pilot_Cmd'
   *  RelationalOperator: '<S16>/FixPt Relational Operator'
   *  RelationalOperator: '<S17>/FixPt Relational Operator'
   *  Switch: '<S12>/Switch2'
   *  UnitDelay: '<S16>/Delay Input1'
   *  UnitDelay: '<S17>/Delay Input1'
   *
   * Block description for '<S16>/Delay Input1':
   *
   *  Store in Global RAM
   *
   * Block description for '<S17>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Pilot_Cmd.cmd_1 != FMS_DW.DelayInput1_DSTATE_i) {
    FMS_B.Switch1 = (FMS_Cmd)FMS_U.Pilot_Cmd.cmd_1;
  } else if (FMS_U.GCS_Cmd.cmd_1 != FMS_DW.DelayInput1_DSTATE_p) {
    /* Switch: '<S12>/Switch2' incorporates:
     *  DataTypeConversion: '<S12>/Data Type Conversion1'
     *  Inport: '<Root>/GCS_Cmd'
     */
    FMS_B.Switch1 = (FMS_Cmd)FMS_U.GCS_Cmd.cmd_1;
  } else {
    /* Switch: '<S12>/Switch2' incorporates:
     *  Constant: '<S12>/Constant1'
     */
    FMS_B.Switch1 = FMS_Cmd_None;
  }

  /* End of Switch: '<S12>/Switch1' */

  /* Update for UnitDelay: '<S15>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S15>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_ak = FMS_U.Pilot_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator1' incorporates:
   *  Constant: '<S11>/Constant1'
   */
  rtb_d_f = (real32_T)FMS_DW.DiscreteTimeIntegrator1_DSTAT_b + (real32_T)
    FMS_EXPORT.period;
  if (rtb_d_f < 4.2949673E+9F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = (uint32_T)rtb_d_f;
  } else {
    FMS_DW.DiscreteTimeIntegrator1_DSTAT_b = MAX_uint32_T;
  }

  /* End of Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S20>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S20>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_f = FMS_U.GCS_Cmd.mode;

  /* Update for UnitDelay: '<S17>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S17>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_p = FMS_U.GCS_Cmd.cmd_1;

  /* Update for UnitDelay: '<S16>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S16>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_i = FMS_U.Pilot_Cmd.cmd_1;

  /* End of Outputs for SubSystem: '<Root>/CommandProcess' */

  /* Chart: '<Root>/SafeMode' incorporates:
   *  Delay: '<S13>/Delay'
   *  Inport: '<Root>/INS_Out'
   */
  if (FMS_DW.is_active_c3_FMS == 0U) {
    FMS_DW.is_active_c3_FMS = 1U;
    switch (FMS_DW.Delay_DSTATE_c) {
     case PilotMode_Stabilize:
      FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
      break;

     case PilotMode_Position:
      FMS_DW.is_c3_FMS = FMS_IN_Position_h;
      break;

     case PilotMode_Mission:
      FMS_DW.is_c3_FMS = FMS_IN_Mission;
      break;

     case PilotMode_Offboard:
      FMS_DW.is_c3_FMS = FMS_IN_Offboard;
      break;

     default:
      FMS_DW.is_c3_FMS = FMS_IN_Other;
      break;
    }
  } else {
    switch (FMS_DW.is_c3_FMS) {
     case FMS_IN_Manual_b:
      FMS_B.target_mode = PilotMode_Manual;
      switch (FMS_DW.Delay_DSTATE_c) {
       case PilotMode_Stabilize:
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
        break;

       case PilotMode_Position:
        FMS_DW.is_c3_FMS = FMS_IN_Position_h;
        break;

       case PilotMode_Mission:
        FMS_DW.is_c3_FMS = FMS_IN_Mission;
        break;

       case PilotMode_Offboard:
        FMS_DW.is_c3_FMS = FMS_IN_Offboard;
        break;

       default:
        FMS_DW.is_c3_FMS = FMS_IN_Other;
        break;
      }
      break;

     case FMS_IN_Mission:
      if (((FMS_U.INS_Out.flag & 8U) != 0U) && ((FMS_U.INS_Out.flag & 32U) != 0U))
      {
        FMS_B.target_mode = PilotMode_Mission;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_h;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
      }
      break;

     case FMS_IN_Offboard:
      if (((FMS_U.INS_Out.flag & 8U) != 0U) && ((FMS_U.INS_Out.flag & 16U) != 0U))
      {
        FMS_B.target_mode = PilotMode_Offboard;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_h;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
      }
      break;

     case FMS_IN_Other:
      FMS_B.target_mode = FMS_DW.Delay_DSTATE_c;
      switch (FMS_DW.Delay_DSTATE_c) {
       case PilotMode_Stabilize:
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
        break;

       case PilotMode_Position:
        FMS_DW.is_c3_FMS = FMS_IN_Position_h;
        break;

       case PilotMode_Mission:
        FMS_DW.is_c3_FMS = FMS_IN_Mission;
        break;

       case PilotMode_Offboard:
        FMS_DW.is_c3_FMS = FMS_IN_Offboard;
        break;

       default:
        FMS_DW.is_c3_FMS = FMS_IN_Other;
        break;
      }
      break;

     case FMS_IN_Position_h:
      if (((FMS_U.INS_Out.flag & 8U) != 0U) && ((FMS_U.INS_Out.flag & 64U) != 0U))
      {
        FMS_B.target_mode = PilotMode_Position;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_h;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
      }
      break;

     default:
      if ((FMS_U.INS_Out.flag & 8U) != 0U) {
        FMS_B.target_mode = PilotMode_Stabilize;
        switch (FMS_DW.Delay_DSTATE_c) {
         case PilotMode_Stabilize:
          FMS_DW.is_c3_FMS = FMS_IN_Stabilize_m;
          break;

         case PilotMode_Position:
          FMS_DW.is_c3_FMS = FMS_IN_Position_h;
          break;

         case PilotMode_Mission:
          FMS_DW.is_c3_FMS = FMS_IN_Mission;
          break;

         case PilotMode_Offboard:
          FMS_DW.is_c3_FMS = FMS_IN_Offboard;
          break;

         default:
          FMS_DW.is_c3_FMS = FMS_IN_Other;
          break;
        }
      } else {
        FMS_DW.is_c3_FMS = FMS_IN_Manual_b;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/SafeMode' */

  /* BusCreator: '<Root>/BusConversion_InsertedFor_FMS State Machine_at_inport_2' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   */
  FMS_B.BusConversion_InsertedFor_FMS_f = FMS_U.Pilot_Cmd;

  /* RelationalOperator: '<S22>/FixPt Relational Operator' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *  UnitDelay: '<S22>/Delay Input1'
   *
   * Block description for '<S22>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_FixPtRelationalOperator = (FMS_U.Pilot_Cmd.timestamp !=
    FMS_DW.DelayInput1_DSTATE_d);

  /* DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Inport: '<Root>/GCS_Cmd'
   *  Logic: '<S3>/Logical Operator'
   *  Logic: '<S3>/Logical Operator1'
   *  RelationalOperator: '<S21>/FixPt Relational Operator'
   *  UnitDelay: '<S21>/Delay Input1'
   *
   * Block description for '<S21>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if ((FMS_U.GCS_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE_j) ||
      rtb_FixPtRelationalOperator || (FMS_PARAM.LOST_RETURN_EN == 0)) {
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
   *  Constant: '<S23>/Constant'
   *  Constant: '<S3>/Constant1'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   *  RelationalOperator: '<S23>/Compare'
   */
  FMS_B.LogicalOperator2 = ((FMS_DW.DiscreteTimeIntegrator_DSTATE >=
    FMS_PARAM.LOST_RETURN_TIME) && (FMS_PARAM.LOST_RETURN_EN != 0));

  /* DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' */
  if (rtb_FixPtRelationalOperator) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
  }

  if (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator1_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
    }
  }

  /* RelationalOperator: '<S24>/Compare' incorporates:
   *  Constant: '<S24>/Constant'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
   */
  FMS_B.Compare = (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 1.0F);

  /* BusCreator: '<Root>/BusConversion_InsertedFor_FMS State Machine_at_inport_5' incorporates:
   *  Inport: '<Root>/INS_Out'
   */
  FMS_B.BusConversion_InsertedFor_FMSSt = FMS_U.INS_Out;

  /* DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S9>/Constant1'
   *  Delay: '<S9>/Delay'
   *  Inport: '<Root>/Auto_Cmd'
   *  RelationalOperator: '<S8>/FixPt Relational Operator'
   *  Switch: '<S9>/Switch'
   *  UnitDelay: '<S8>/Delay Input1'
   *
   * Block description for '<S8>/Delay Input1':
   *
   *  Store in Global RAM
   */
  if (FMS_U.Auto_Cmd.timestamp != FMS_DW.DelayInput1_DSTATE_a) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_j = 0U;
    FMS_DW.Delay_DSTATE_p = 1U;
  }

  /* Logic: '<S1>/Logical Operator' incorporates:
   *  Constant: '<S10>/Upper Limit'
   *  Constant: '<S7>/Constant'
   *  Delay: '<S9>/Delay'
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   *  Inport: '<Root>/Auto_Cmd'
   *  RelationalOperator: '<S10>/Upper Test'
   *  RelationalOperator: '<S7>/Compare'
   */
  FMS_B.LogicalOperator = ((FMS_DW.DiscreteTimeIntegrator_DSTATE_j < 1000U) &&
    (FMS_DW.Delay_DSTATE_p != 0) && (FMS_U.Auto_Cmd.frame <= 2));

  /* Chart: '<Root>/FMS State Machine' incorporates:
   *  Inport: '<Root>/Mission_Data'
   */
  FMS_DW.chartAbsoluteTimeCounter++;
  rtb_FixPtRelationalOperator = (FMS_DW.prep_mission == 1.0);
  if ((!rtb_FixPtRelationalOperator) || (!FMS_DW.condWasTrueAtLastTimeStep_1)) {
    FMS_DW.durationLastReferenceTick_1 = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1 = rtb_FixPtRelationalOperator;
  if ((!FMS_DW.br) || (!FMS_DW.condWasTrueAtLastTimeStep_1_k)) {
    FMS_DW.durationLastReferenceTick_1_k = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_1_k = FMS_DW.br;
  if ((!FMS_DW.bl) || (!FMS_DW.condWasTrueAtLastTimeStep_2)) {
    FMS_DW.durationLastReferenceTick_2 = FMS_DW.chartAbsoluteTimeCounter;
  }

  FMS_DW.condWasTrueAtLastTimeStep_2 = FMS_DW.bl;
  if (FMS_DW.temporalCounter_i1 < 1023U) {
    FMS_DW.temporalCounter_i1++;
  }

  FMS_DW.sfEvent = -1;
  FMS_DW.mission_timestamp_prev = FMS_DW.mission_timestamp_start;
  FMS_DW.mission_timestamp_start = FMS_U.Mission_Data.timestamp;
  FMS_DW.cmd_prev = FMS_DW.cmd_start;
  FMS_DW.cmd_start = FMS_B.Switch1;
  FMS_DW.mode_prev = FMS_DW.mode_start;
  FMS_DW.mode_start = FMS_B.target_mode;
  FMS_DW.M_isValid = false;
  FMS_c11_FMS();
  FMS_sf_msg_discard_M();

  /* End of Chart: '<Root>/FMS State Machine' */

  /* Outputs for Atomic SubSystem: '<Root>/FMS Commander' */
  /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
  /* SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy3Inport1' */
  rtb_Switch_k1[0] = FMS_B.Cmd_In.sp_waypoint[0];
  rtb_Switch_i[0] = FMS_B.Cmd_In.cur_waypoint[0];
  rtb_Switch_k1[1] = FMS_B.Cmd_In.sp_waypoint[1];
  rtb_Switch_i[1] = FMS_B.Cmd_In.cur_waypoint[1];
  rtb_Switch_k1[2] = FMS_B.Cmd_In.sp_waypoint[2];
  rtb_Switch_i[2] = FMS_B.Cmd_In.cur_waypoint[2];

  /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

  /* SwitchCase: '<S25>/Switch Case' incorporates:
   *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy6Inport1'
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
    /* Disable for SwitchCase: '<S27>/Switch Case' */
    switch (FMS_DW.SwitchCase_ActiveSubsystem_b) {
     case 0:
      /* Disable for SwitchCase: '<S34>/Switch Case' */
      FMS_DW.SwitchCase_ActiveSubsystem_a = -1;
      break;

     case 1:
      /* Disable for SwitchCase: '<S32>/Switch Case' */
      FMS_DW.SwitchCase_ActiveSubsystem_i = -1;
      break;

     case 2:
      /* Disable for SwitchCase: '<S31>/Switch Case' */
      switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
       case 0:
        /* Disable for SwitchCase: '<S79>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_n = -1;
        break;

       case 1:
        /* Disable for SwitchCase: '<S41>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_d = -1;
        break;

       case 2:
        break;
      }

      FMS_DW.SwitchCase_ActiveSubsystem_f = -1;
      break;

     case 3:
     case 4:
      break;
    }

    FMS_DW.SwitchCase_ActiveSubsystem_b = -1;

    /* End of Disable for SwitchCase: '<S27>/Switch Case' */
  }

  switch (FMS_DW.SwitchCase_ActiveSubsystem) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S25>/Disarm' incorporates:
     *  ActionPort: '<S29>/Action Port'
     */
    /* Outport: '<Root>/FMS_Out' incorporates:
     *  BusAssignment: '<S28>/Bus Assignment'
     *  BusAssignment: '<S29>/Bus Assignment'
     *  Constant: '<S29>/Constant'
     *  SignalConversion: '<S29>/TmpHiddenBufferAtBus AssignmentInport1'
     */
    FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

    /* BusAssignment: '<S29>/Bus Assignment' incorporates:
     *  BusAssignment: '<S28>/Bus Assignment'
     *  Constant: '<S29>/Constant2'
     *  DataTypeConversion: '<S29>/Data Type Conversion2'
     *  Outport: '<Root>/FMS_Out'
     */
    FMS_Y.FMS_Out.reset = 1U;
    FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_m;
    FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_a;
    for (i = 0; i < 16; i++) {
      /* DataTypeConversion: '<S29>/Data Type Conversion2' incorporates:
       *  Constant: '<S29>/Constant6'
       */
      rtb_d_f = fmodf(floorf(FMS_PARAM.DISARM_OUT[i]), 65536.0F);
      FMS_Y.FMS_Out.actuator_cmd[i] = (uint16_T)(rtb_d_f < 0.0F ? (int32_T)
        (uint16_T)-(int16_T)(uint16_T)-rtb_d_f : (int32_T)(uint16_T)rtb_d_f);
    }

    /* End of Outputs for SubSystem: '<S25>/Disarm' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S25>/Standby' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    /* Outport: '<Root>/FMS_Out' incorporates:
     *  BusAssignment: '<S28>/Bus Assignment'
     *  BusAssignment: '<S30>/Bus Assignment'
     *  Constant: '<S30>/Constant'
     *  SignalConversion: '<S30>/TmpHiddenBufferAtBus AssignmentInport1'
     */
    FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

    /* BusAssignment: '<S30>/Bus Assignment' incorporates:
     *  BusAssignment: '<S28>/Bus Assignment'
     *  Constant: '<S30>/Constant2'
     *  DataTypeConversion: '<S30>/Data Type Conversion3'
     *  Outport: '<Root>/FMS_Out'
     */
    FMS_Y.FMS_Out.reset = 1U;
    FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion2_hd;
    FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_f;
    for (i = 0; i < 16; i++) {
      /* DataTypeConversion: '<S30>/Data Type Conversion3' incorporates:
       *  Constant: '<S30>/Constant6'
       */
      rtb_d_f = fmodf(floorf(FMS_PARAM.STANDBY_OUT[i]), 65536.0F);
      FMS_Y.FMS_Out.actuator_cmd[i] = (uint16_T)(rtb_d_f < 0.0F ? (int32_T)
        (uint16_T)-(int16_T)(uint16_T)-rtb_d_f : (int32_T)(uint16_T)rtb_d_f);
    }

    /* End of Outputs for SubSystem: '<S25>/Standby' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S25>/Arm' incorporates:
     *  ActionPort: '<S27>/Action Port'
     */
    /* SwitchCase: '<S27>/Switch Case' */
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
        /* Disable for SwitchCase: '<S34>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_a = -1;
        break;

       case 1:
        /* Disable for SwitchCase: '<S32>/Switch Case' */
        FMS_DW.SwitchCase_ActiveSubsystem_i = -1;
        break;

       case 2:
        /* Disable for SwitchCase: '<S31>/Switch Case' */
        switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
         case 0:
          /* Disable for SwitchCase: '<S79>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_n = -1;
          break;

         case 1:
          /* Disable for SwitchCase: '<S41>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_d = -1;
          break;

         case 2:
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
      /* Outputs for IfAction SubSystem: '<S27>/SubMode' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* SwitchCase: '<S34>/Switch Case' */
      rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_a;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      switch (FMS_B.state) {
       case VehicleState_Return:
        FMS_DW.SwitchCase_ActiveSubsystem_a = 0;
        break;

       case VehicleState_Hold:
        FMS_DW.SwitchCase_ActiveSubsystem_a = 1;
        break;

       default:
        FMS_DW.SwitchCase_ActiveSubsystem_a = 2;
        break;
      }

      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
      switch (FMS_DW.SwitchCase_ActiveSubsystem_a) {
       case 0:
        if (FMS_DW.SwitchCase_ActiveSubsystem_a != rtPrevAction) {
          /* InitializeConditions for IfAction SubSystem: '<S34>/Return' incorporates:
           *  ActionPort: '<S154>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S34>/Switch Case' incorporates:
           *  Delay: '<S161>/Delay'
           *  DiscreteIntegrator: '<S158>/Integrator'
           *  DiscreteIntegrator: '<S158>/Integrator1'
           *  DiscreteIntegrator: '<S164>/Integrator'
           *  DiscreteIntegrator: '<S164>/Integrator1'
           */
          FMS_DW.icLoad_k = 1U;
          FMS_DW.Integrator1_IC_LOADING_m = 1U;
          FMS_DW.Integrator1_DSTATE_g = 0.0F;
          FMS_DW.Integrator_DSTATE_g = 0.0F;
          FMS_DW.Integrator_DSTATE_i = 0.0F;

          /* End of InitializeConditions for SubSystem: '<S34>/Return' */
        }

        /* Outputs for IfAction SubSystem: '<S34>/Return' incorporates:
         *  ActionPort: '<S154>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S154>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_Switch_mc_idx_1 = FMS_B.Cmd_In.sp_waypoint[0] - FMS_U.INS_Out.x_R;
        rtb_P_c[0] = FMS_B.Cmd_In.sp_waypoint[0] - FMS_U.INS_Out.x_R;
        rtb_Switch_mc_idx_0 = FMS_B.Cmd_In.sp_waypoint[1] - FMS_U.INS_Out.y_R;
        rtb_P_c[1] = FMS_B.Cmd_In.sp_waypoint[1] - FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Math: '<S175>/Math Function' incorporates:
         *  Sum: '<S154>/Subtract'
         */
        rtb_MathFunction_i2[0] = rtb_Switch_mc_idx_0 * rtb_Switch_mc_idx_0;
        rtb_MathFunction_i2[1] = rtb_Switch_mc_idx_1 * rtb_Switch_mc_idx_1;

        /* Sum: '<S175>/Sum of Elements' */
        rtb_d_f = rtb_MathFunction_i2[0] + rtb_MathFunction_i2[1];

        /* Math: '<S175>/Math Function1' incorporates:
         *  Sum: '<S175>/Sum of Elements'
         *
         * About '<S175>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_d_f < 0.0F) {
          rtb_Sum_i = -sqrtf(fabsf(rtb_d_f));
        } else {
          rtb_Sum_i = sqrtf(rtb_d_f);
        }

        /* End of Math: '<S175>/Math Function1' */

        /* Switch: '<S175>/Switch' incorporates:
         *  Constant: '<S175>/Constant'
         *  Product: '<S175>/Product'
         */
        if (rtb_Sum_i <= 0.0F) {
          rtb_Switch_mc_idx_0 = 0.0F;
          rtb_Switch_mc_idx_1 = 0.0F;
          rtb_Sum_i = 1.0F;
        }

        /* End of Switch: '<S175>/Switch' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S178>/NearbyRefWP' incorporates:
         *  Constant: '<S154>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_NearbyRefWP(&rtb_Switch_k1[0], FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                        FMS_PARAM.L1, rtb_P_c, &rtb_d_f);

        /* MATLAB Function: '<S178>/SearchL1RefWP' incorporates:
         *  Constant: '<S154>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_SearchL1RefWP(&rtb_Switch_i[0], &rtb_Switch_k1[0], FMS_U.INS_Out.x_R,
                          FMS_U.INS_Out.y_R, FMS_PARAM.L1, rtb_MathFunction_i2,
                          &rtb_u_g);

        /* MATLAB Function: '<S178>/OutRegionRegWP' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_OutRegionRegWP(&rtb_Switch_i[0], &rtb_Switch_k1[0],
                           FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R, rtb_P_on);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Product: '<S175>/Divide' */
        rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_mc_idx_0 / rtb_Sum_i;
        rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_mc_idx_1 / rtb_Sum_i;

        /* Sum: '<S173>/Subtract' incorporates:
         *  Product: '<S173>/Multiply'
         *  Product: '<S173>/Multiply1'
         */
        rtb_Sum_i = rtb_TmpSignalConversionAtMat_co[0] * FMS_ConstB.Divide_h[1]
          - rtb_TmpSignalConversionAtMat_co[1] * FMS_ConstB.Divide_h[0];

        /* Signum: '<S162>/Sign1' */
        if (rtb_Sum_i < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else {
          if (rtb_Sum_i > 0.0F) {
            rtb_Sum_i = 1.0F;
          }
        }

        /* End of Signum: '<S162>/Sign1' */

        /* Switch: '<S162>/Switch2' incorporates:
         *  Constant: '<S162>/Constant4'
         */
        if (rtb_Sum_i == 0.0F) {
          rtb_Sum_i = 1.0F;
        }

        /* End of Switch: '<S162>/Switch2' */

        /* DotProduct: '<S162>/Dot Product' */
        rtb_Switch_mc_idx_0 = FMS_ConstB.Divide_h[0] *
          rtb_TmpSignalConversionAtMat_co[0] + FMS_ConstB.Divide_h[1] *
          rtb_TmpSignalConversionAtMat_co[1];

        /* Trigonometry: '<S162>/Acos' incorporates:
         *  DotProduct: '<S162>/Dot Product'
         */
        if (rtb_Switch_mc_idx_0 > 1.0F) {
          rtb_Switch_mc_idx_0 = 1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 < -1.0F) {
            rtb_Switch_mc_idx_0 = -1.0F;
          }
        }

        /* Product: '<S162>/Multiply' incorporates:
         *  Trigonometry: '<S162>/Acos'
         */
        rtb_Sum_i *= acosf(rtb_Switch_mc_idx_0);

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Math: '<S166>/Rem' incorporates:
         *  Constant: '<S166>/Constant1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         *  Sum: '<S161>/Sum1'
         */
        rtb_sin_alpha_j = rt_remf(rtb_Sum_i - FMS_U.INS_Out.psi, 6.28318548F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Switch: '<S166>/Switch' incorporates:
         *  Abs: '<S166>/Abs'
         *  Constant: '<S166>/Constant'
         *  Constant: '<S172>/Constant'
         *  Product: '<S166>/Multiply'
         *  RelationalOperator: '<S172>/Compare'
         *  Sum: '<S166>/Add'
         */
        if (fabsf(rtb_sin_alpha_j) > 3.14159274F) {
          /* Signum: '<S166>/Sign' */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_0 = -1.0F;
          } else if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          } else {
            rtb_Switch_mc_idx_0 = rtb_sin_alpha_j;
          }

          /* End of Signum: '<S166>/Sign' */
          rtb_sin_alpha_j -= 6.28318548F * rtb_Switch_mc_idx_0;
        }

        /* End of Switch: '<S166>/Switch' */

        /* Abs: '<S157>/Abs' */
        rtb_sin_alpha_j = fabsf(rtb_sin_alpha_j);

        /* RelationalOperator: '<S176>/Compare' incorporates:
         *  Constant: '<S176>/Constant'
         */
        rtb_FixPtRelationalOperator = (rtb_sin_alpha_j >= 0.17453292F);

        /* Delay: '<S161>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        if (FMS_DW.icLoad_k != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_k = FMS_U.INS_Out.psi;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* DiscreteIntegrator: '<S164>/Integrator1' incorporates:
         *  Delay: '<S161>/Delay'
         */
        if (FMS_DW.Integrator1_IC_LOADING_m != 0) {
          FMS_DW.Integrator1_DSTATE_f = FMS_DW.Delay_DSTATE_k;
        }

        /* Switch: '<S154>/Switch' incorporates:
         *  Constant: '<S154>/L1'
         *  Constant: '<S179>/Constant'
         *  Constant: '<S181>/Constant'
         *  Gain: '<S159>/AY_P'
         *  Gain: '<S179>/Gain'
         *  Math: '<S179>/Square'
         *  MinMax: '<S179>/Max'
         *  MinMax: '<S179>/Min'
         *  Product: '<S179>/Divide'
         *  Product: '<S179>/Multiply1'
         *  RelationalOperator: '<S181>/Compare'
         *  Sqrt: '<S187>/Sqrt'
         *  Switch: '<S178>/Switch1'
         */
        if (rtb_FixPtRelationalOperator) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Math: '<S168>/Rem' incorporates:
           *  Constant: '<S168>/Constant1'
           *  DiscreteIntegrator: '<S164>/Integrator1'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Sum: '<S163>/Sum'
           */
          rtb_sin_alpha_j = rt_remf(FMS_DW.Integrator1_DSTATE_f -
            FMS_U.INS_Out.psi, 6.28318548F);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Switch: '<S168>/Switch' incorporates:
           *  Abs: '<S168>/Abs'
           *  Constant: '<S168>/Constant'
           *  Constant: '<S169>/Constant'
           *  Product: '<S168>/Multiply'
           *  RelationalOperator: '<S169>/Compare'
           *  Sum: '<S168>/Add'
           */
          if (fabsf(rtb_sin_alpha_j) > 3.14159274F) {
            /* Signum: '<S168>/Sign' */
            if (rtb_sin_alpha_j < 0.0F) {
              rtb_d_f = -1.0F;
            } else if (rtb_sin_alpha_j > 0.0F) {
              rtb_d_f = 1.0F;
            } else {
              rtb_d_f = rtb_sin_alpha_j;
            }

            /* End of Signum: '<S168>/Sign' */
            rtb_sin_alpha_j -= 6.28318548F * rtb_d_f;
          }

          /* End of Switch: '<S168>/Switch' */

          /* Gain: '<S163>/Gain2' */
          rtb_sin_alpha_j *= FMS_PARAM.YAW_P;

          /* Saturate: '<S163>/Saturation' */
          if (rtb_sin_alpha_j > 1.04719758F) {
            rtb_sin_alpha_j = 1.04719758F;
          } else {
            if (rtb_sin_alpha_j < -1.04719758F) {
              rtb_sin_alpha_j = -1.04719758F;
            }
          }

          /* End of Saturate: '<S163>/Saturation' */
        } else {
          if (rtb_d_f <= 0.0F) {
            /* Switch: '<S178>/Switch' incorporates:
             *  Constant: '<S180>/Constant'
             *  RelationalOperator: '<S180>/Compare'
             *  Switch: '<S178>/Switch1'
             */
            if (rtb_u_g >= 0.0F) {
              rtb_P_c[0] = rtb_MathFunction_i2[0];
              rtb_P_c[1] = rtb_MathFunction_i2[1];
            } else {
              rtb_P_c[0] = rtb_P_on[0];
              rtb_P_c[1] = rtb_P_on[1];
            }

            /* End of Switch: '<S178>/Switch' */
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S179>/Subtract' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_P_on[0] = rtb_P_c[0] - FMS_U.INS_Out.x_R;
          rtb_P_on[1] = rtb_P_c[1] - FMS_U.INS_Out.y_R;

          /* Sum: '<S188>/Sum of Elements' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  Math: '<S188>/Math Function'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Sum: '<S186>/Sum of Elements'
           */
          rtb_d_f = FMS_U.INS_Out.vn * FMS_U.INS_Out.vn + FMS_U.INS_Out.ve *
            FMS_U.INS_Out.ve;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Math: '<S188>/Math Function1' incorporates:
           *  Sum: '<S188>/Sum of Elements'
           *
           * About '<S188>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_d_f < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_d_f));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_d_f);
          }

          /* End of Math: '<S188>/Math Function1' */

          /* Switch: '<S188>/Switch' incorporates:
           *  Constant: '<S188>/Constant'
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S188>/Product'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            rtb_Switch_k1[0] = FMS_U.INS_Out.vn;
            rtb_Switch_k1[1] = FMS_U.INS_Out.ve;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S188>/Switch' */

          /* Product: '<S188>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_k1[0] / rtb_Switch_k1
            [2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_k1[1] / rtb_Switch_k1
            [2];

          /* Sum: '<S189>/Sum of Elements' incorporates:
           *  Math: '<S189>/Math Function'
           *  Sum: '<S187>/Sum of Elements'
           */
          rtb_Switch_mc_idx_0 = rtb_P_on[0] * rtb_P_on[0] + rtb_P_on[1] *
            rtb_P_on[1];

          /* Math: '<S189>/Math Function1' incorporates:
           *  Sum: '<S189>/Sum of Elements'
           *
           * About '<S189>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_Switch_mc_idx_0 < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_Switch_mc_idx_0));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_Switch_mc_idx_0);
          }

          /* End of Math: '<S189>/Math Function1' */

          /* Switch: '<S189>/Switch' incorporates:
           *  Constant: '<S189>/Constant'
           *  Product: '<S189>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_P_on[0];
            rtb_Switch_k1[1] = rtb_P_on[1];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S189>/Switch' */

          /* Product: '<S189>/Divide' */
          rtb_P_on[0] = rtb_Switch_k1[0] / rtb_Switch_k1[2];
          rtb_P_on[1] = rtb_Switch_k1[1] / rtb_Switch_k1[2];

          /* Sum: '<S191>/Sum of Elements' incorporates:
           *  Math: '<S191>/Math Function'
           *  SignalConversion: '<S191>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[1] *
            rtb_TmpSignalConversionAtMat_co[1] +
            rtb_TmpSignalConversionAtMat_co[0] *
            rtb_TmpSignalConversionAtMat_co[0];

          /* Math: '<S191>/Math Function1' incorporates:
           *  Sum: '<S191>/Sum of Elements'
           *
           * About '<S191>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S191>/Math Function1' */

          /* Switch: '<S191>/Switch' incorporates:
           *  Constant: '<S191>/Constant'
           *  Product: '<S191>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_TmpSignalConversionAtMat_co[1];
            rtb_Switch_k1[1] = rtb_TmpSignalConversionAtMat_co[0];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S191>/Switch' */

          /* Product: '<S191>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_k1[0] / rtb_Switch_k1
            [2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_k1[1] / rtb_Switch_k1
            [2];

          /* Sum: '<S192>/Sum of Elements' incorporates:
           *  Math: '<S192>/Math Function'
           *  SignalConversion: '<S192>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_sin_alpha_j = rtb_P_on[1] * rtb_P_on[1] + rtb_P_on[0] * rtb_P_on[0];

          /* Math: '<S192>/Math Function1' incorporates:
           *  Sum: '<S192>/Sum of Elements'
           *
           * About '<S192>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S192>/Math Function1' */

          /* Switch: '<S192>/Switch' incorporates:
           *  Constant: '<S192>/Constant'
           *  Product: '<S192>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_P_on[1];
            rtb_Switch_k1[1] = rtb_P_on[0];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S192>/Switch' */

          /* Product: '<S192>/Divide' */
          rtb_P_on[0] = rtb_Switch_k1[0] / rtb_Switch_k1[2];
          rtb_P_on[1] = rtb_Switch_k1[1] / rtb_Switch_k1[2];

          /* Sum: '<S190>/Subtract' incorporates:
           *  Product: '<S190>/Multiply'
           *  Product: '<S190>/Multiply1'
           */
          rtb_Switch_mc_idx_1 = rtb_P_on[0] * rtb_TmpSignalConversionAtMat_co[1]
            - rtb_P_on[1] * rtb_TmpSignalConversionAtMat_co[0];

          /* Signum: '<S185>/Sign1' */
          if (rtb_Switch_mc_idx_1 < 0.0F) {
            rtb_Switch_mc_idx_1 = -1.0F;
          } else {
            if (rtb_Switch_mc_idx_1 > 0.0F) {
              rtb_Switch_mc_idx_1 = 1.0F;
            }
          }

          /* End of Signum: '<S185>/Sign1' */

          /* Switch: '<S185>/Switch2' incorporates:
           *  Constant: '<S185>/Constant4'
           */
          if (rtb_Switch_mc_idx_1 == 0.0F) {
            rtb_Switch_mc_idx_1 = 1.0F;
          }

          /* End of Switch: '<S185>/Switch2' */

          /* DotProduct: '<S185>/Dot Product' */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[0] * rtb_P_on[0] +
            rtb_TmpSignalConversionAtMat_co[1] * rtb_P_on[1];

          /* Trigonometry: '<S185>/Acos' incorporates:
           *  DotProduct: '<S185>/Dot Product'
           */
          if (rtb_sin_alpha_j > 1.0F) {
            rtb_sin_alpha_j = 1.0F;
          } else {
            if (rtb_sin_alpha_j < -1.0F) {
              rtb_sin_alpha_j = -1.0F;
            }
          }

          /* Product: '<S185>/Multiply' incorporates:
           *  Trigonometry: '<S185>/Acos'
           */
          rtb_Switch_mc_idx_1 *= acosf(rtb_sin_alpha_j);

          /* Saturate: '<S179>/Saturation' */
          if (rtb_Switch_mc_idx_1 > 1.57079637F) {
            rtb_Switch_mc_idx_1 = 1.57079637F;
          } else {
            if (rtb_Switch_mc_idx_1 < -1.57079637F) {
              rtb_Switch_mc_idx_1 = -1.57079637F;
            }
          }

          /* End of Saturate: '<S179>/Saturation' */

          /* Trigonometry: '<S179>/Sin' */
          rtb_sin_alpha_j = arm_sin_f32(rtb_Switch_mc_idx_1);

          /* Sqrt: '<S186>/Sqrt' */
          rtb_Switch_mc_idx_1 = sqrtf(rtb_d_f);
          rtb_sin_alpha_j = rtb_Switch_mc_idx_1 * rtb_Switch_mc_idx_1 * 2.0F *
            rtb_sin_alpha_j / fminf(FMS_PARAM.L1, fmaxf(sqrtf
            (rtb_Switch_mc_idx_0), 0.5F)) * FMS_PARAM.AY_P;
        }

        /* End of Switch: '<S154>/Switch' */

        /* Saturate: '<S154>/Saturation' */
        if (rtb_sin_alpha_j > 1.0F) {
          rtb_sin_alpha_j = 1.0F;
        } else {
          if (rtb_sin_alpha_j < -1.0F) {
            rtb_sin_alpha_j = -1.0F;
          }
        }

        /* End of Saturate: '<S154>/Saturation' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S154>/Bus Assignment1'
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Constant: '<S154>/Constant2'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S154>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_o;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_m0;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_b;

        /* Switch: '<S154>/Switch1' */
        if (rtb_FixPtRelationalOperator) {
          /* BusAssignment: '<S154>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S28>/Bus Assignment'
           *  Constant: '<S154>/Constant4'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = 0.0F;
        } else {
          /* BusAssignment: '<S154>/Bus Assignment1' incorporates:
           *  BusAssignment: '<S28>/Bus Assignment'
           *  DiscreteIntegrator: '<S158>/Integrator1'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.u_cmd = FMS_DW.Integrator1_DSTATE_g;
        }

        /* End of Switch: '<S154>/Switch1' */

        /* BusAssignment: '<S154>/Bus Assignment1' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.psi_rate_cmd = rtb_sin_alpha_j;

        /* Product: '<S177>/Multiply1' incorporates:
         *  Constant: '<S177>/const1'
         *  DiscreteIntegrator: '<S158>/Integrator'
         */
        rtb_Subtract1_j = FMS_DW.Integrator_DSTATE_g * 0.5F;

        /* Sum: '<S177>/Add' incorporates:
         *  Constant: '<S154>/vel'
         *  DiscreteIntegrator: '<S158>/Integrator1'
         *  Sum: '<S158>/Subtract'
         */
        rtb_sin_alpha_j = (FMS_DW.Integrator1_DSTATE_g - FMS_PARAM.CRUISE_SPEED)
          + rtb_Subtract1_j;

        /* Signum: '<S177>/Sign' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_Switch_mc_idx_1 = -1.0F;
        } else if (rtb_sin_alpha_j > 0.0F) {
          rtb_Switch_mc_idx_1 = 1.0F;
        } else {
          rtb_Switch_mc_idx_1 = rtb_sin_alpha_j;
        }

        /* End of Signum: '<S177>/Sign' */

        /* Sum: '<S177>/Add2' incorporates:
         *  Abs: '<S177>/Abs'
         *  Gain: '<S177>/Gain'
         *  Gain: '<S177>/Gain1'
         *  Product: '<S177>/Multiply2'
         *  Product: '<S177>/Multiply3'
         *  Sqrt: '<S177>/Sqrt'
         *  Sum: '<S177>/Add1'
         *  Sum: '<S177>/Subtract'
         */
        rtb_Rem_kb = (sqrtf((8.0F * fabsf(rtb_sin_alpha_j) + FMS_ConstB.d_o) *
                            FMS_ConstB.d_o) - FMS_ConstB.d_o) * 0.5F *
          rtb_Switch_mc_idx_1 + rtb_Subtract1_j;

        /* Sum: '<S177>/Add4' */
        rtb_Switch_mc_idx_0 = (rtb_sin_alpha_j - rtb_Rem_kb) + rtb_Subtract1_j;

        /* Sum: '<S177>/Add3' */
        rtb_d_f = rtb_sin_alpha_j + FMS_ConstB.d_o;

        /* Sum: '<S177>/Subtract1' */
        rtb_sin_alpha_j -= FMS_ConstB.d_o;

        /* Signum: '<S177>/Sign1' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S177>/Sign1' */

        /* Signum: '<S177>/Sign2' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_sin_alpha_j = -1.0F;
        } else {
          if (rtb_sin_alpha_j > 0.0F) {
            rtb_sin_alpha_j = 1.0F;
          }
        }

        /* End of Signum: '<S177>/Sign2' */

        /* Sum: '<S177>/Add5' incorporates:
         *  Gain: '<S177>/Gain2'
         *  Product: '<S177>/Multiply4'
         *  Sum: '<S177>/Subtract2'
         */
        rtb_Rem_kb += (rtb_d_f - rtb_sin_alpha_j) * 0.5F * rtb_Switch_mc_idx_0;

        /* Sum: '<S177>/Add6' */
        rtb_d_f = rtb_Rem_kb + FMS_ConstB.d_o;

        /* Sum: '<S177>/Subtract3' */
        rtb_Switch_mc_idx_0 = rtb_Rem_kb - FMS_ConstB.d_o;

        /* Product: '<S177>/Divide' */
        rtb_u_g = rtb_Rem_kb / FMS_ConstB.d_o;

        /* Signum: '<S177>/Sign5' incorporates:
         *  Signum: '<S177>/Sign6'
         */
        if (rtb_Rem_kb < 0.0F) {
          rtb_Sign5_d = -1.0F;
          rtb_Switch_mc_idx_1 = -1.0F;
        } else if (rtb_Rem_kb > 0.0F) {
          rtb_Sign5_d = 1.0F;
          rtb_Switch_mc_idx_1 = 1.0F;
        } else {
          rtb_Sign5_d = rtb_Rem_kb;
          rtb_Switch_mc_idx_1 = rtb_Rem_kb;
        }

        /* End of Signum: '<S177>/Sign5' */

        /* Sum: '<S161>/Sum2' incorporates:
         *  Delay: '<S161>/Delay'
         */
        rtb_Sum_i -= FMS_DW.Delay_DSTATE_k;

        /* Math: '<S165>/Rem' incorporates:
         *  Constant: '<S165>/Constant1'
         */
        rtb_Rem_kb = rt_remf(rtb_Sum_i, 6.28318548F);

        /* Switch: '<S165>/Switch' incorporates:
         *  Abs: '<S165>/Abs'
         *  Constant: '<S165>/Constant'
         *  Constant: '<S171>/Constant'
         *  Product: '<S165>/Multiply'
         *  RelationalOperator: '<S171>/Compare'
         *  Sum: '<S165>/Add'
         */
        if (fabsf(rtb_Rem_kb) > 3.14159274F) {
          /* Signum: '<S165>/Sign' */
          if (rtb_Rem_kb < 0.0F) {
            rtb_Sum_i = -1.0F;
          } else if (rtb_Rem_kb > 0.0F) {
            rtb_Sum_i = 1.0F;
          } else {
            rtb_Sum_i = rtb_Rem_kb;
          }

          /* End of Signum: '<S165>/Sign' */
          rtb_Rem_kb -= 6.28318548F * rtb_Sum_i;
        }

        /* End of Switch: '<S165>/Switch' */

        /* Sum: '<S161>/Sum' incorporates:
         *  Delay: '<S161>/Delay'
         */
        rtb_Sum_i = rtb_Rem_kb + FMS_DW.Delay_DSTATE_k;

        /* Product: '<S170>/Multiply1' incorporates:
         *  Constant: '<S170>/const1'
         *  DiscreteIntegrator: '<S164>/Integrator'
         */
        rtb_Rem_kb = FMS_DW.Integrator_DSTATE_i * 0.785398185F;

        /* Sum: '<S170>/Add' incorporates:
         *  DiscreteIntegrator: '<S164>/Integrator1'
         *  Sum: '<S164>/Subtract'
         */
        rtb_Subtract1_j = (FMS_DW.Integrator1_DSTATE_f - rtb_Sum_i) + rtb_Rem_kb;

        /* Signum: '<S170>/Sign' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else if (rtb_Subtract1_j > 0.0F) {
          rtb_Sum_i = 1.0F;
        } else {
          rtb_Sum_i = rtb_Subtract1_j;
        }

        /* End of Signum: '<S170>/Sign' */

        /* Sum: '<S170>/Add2' incorporates:
         *  Abs: '<S170>/Abs'
         *  Gain: '<S170>/Gain'
         *  Gain: '<S170>/Gain1'
         *  Product: '<S170>/Multiply2'
         *  Product: '<S170>/Multiply3'
         *  Sqrt: '<S170>/Sqrt'
         *  Sum: '<S170>/Add1'
         *  Sum: '<S170>/Subtract'
         */
        rtb_sin_alpha_j = (sqrtf((8.0F * fabsf(rtb_Subtract1_j) + FMS_ConstB.d_d)
          * FMS_ConstB.d_d) - FMS_ConstB.d_d) * 0.5F * rtb_Sum_i + rtb_Rem_kb;

        /* Sum: '<S170>/Add4' */
        rtb_Rem_kb += rtb_Subtract1_j - rtb_sin_alpha_j;

        /* Sum: '<S170>/Add3' */
        rtb_Sum_i = rtb_Subtract1_j + FMS_ConstB.d_d;

        /* Sum: '<S170>/Subtract1' */
        rtb_Subtract1_j -= FMS_ConstB.d_d;

        /* Signum: '<S170>/Sign1' */
        if (rtb_Sum_i < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else {
          if (rtb_Sum_i > 0.0F) {
            rtb_Sum_i = 1.0F;
          }
        }

        /* End of Signum: '<S170>/Sign1' */

        /* Signum: '<S170>/Sign2' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Subtract1_j = -1.0F;
        } else {
          if (rtb_Subtract1_j > 0.0F) {
            rtb_Subtract1_j = 1.0F;
          }
        }

        /* End of Signum: '<S170>/Sign2' */

        /* Sum: '<S170>/Add5' incorporates:
         *  Gain: '<S170>/Gain2'
         *  Product: '<S170>/Multiply4'
         *  Sum: '<S170>/Subtract2'
         */
        rtb_sin_alpha_j += (rtb_Sum_i - rtb_Subtract1_j) * 0.5F * rtb_Rem_kb;

        /* Update for Delay: '<S161>/Delay' */
        FMS_DW.icLoad_k = 0U;

        /* Update for DiscreteIntegrator: '<S164>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S164>/Integrator'
         */
        FMS_DW.Integrator1_IC_LOADING_m = 0U;
        FMS_DW.Integrator1_DSTATE_f += 0.01F * FMS_DW.Integrator_DSTATE_i;

        /* Update for DiscreteIntegrator: '<S158>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S158>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_g += 0.01F * FMS_DW.Integrator_DSTATE_g;

        /* Signum: '<S177>/Sign3' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S177>/Sign3' */

        /* Signum: '<S177>/Sign4' */
        if (rtb_Switch_mc_idx_0 < 0.0F) {
          rtb_Switch_mc_idx_0 = -1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          }
        }

        /* End of Signum: '<S177>/Sign4' */

        /* Update for DiscreteIntegrator: '<S158>/Integrator' incorporates:
         *  Constant: '<S177>/const'
         *  Gain: '<S177>/Gain3'
         *  Product: '<S177>/Multiply5'
         *  Product: '<S177>/Multiply6'
         *  Sum: '<S177>/Subtract4'
         *  Sum: '<S177>/Subtract5'
         *  Sum: '<S177>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_g += ((rtb_u_g - rtb_Sign5_d) *
          FMS_ConstB.Gain4_nx * ((rtb_d_f - rtb_Switch_mc_idx_0) * 0.5F) -
          rtb_Switch_mc_idx_1 * 19.612F) * 0.01F;

        /* Sum: '<S170>/Subtract3' */
        rtb_d_f = rtb_sin_alpha_j - FMS_ConstB.d_d;

        /* Sum: '<S170>/Add6' */
        rtb_Switch_mc_idx_0 = rtb_sin_alpha_j + FMS_ConstB.d_d;

        /* Signum: '<S170>/Sign5' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_Switch_mc_idx_1 = -1.0F;
        } else if (rtb_sin_alpha_j > 0.0F) {
          rtb_Switch_mc_idx_1 = 1.0F;
        } else {
          rtb_Switch_mc_idx_1 = rtb_sin_alpha_j;
        }

        /* End of Signum: '<S170>/Sign5' */

        /* Signum: '<S170>/Sign3' */
        if (rtb_Switch_mc_idx_0 < 0.0F) {
          rtb_Switch_mc_idx_0 = -1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          }
        }

        /* End of Signum: '<S170>/Sign3' */

        /* Signum: '<S170>/Sign4' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S170>/Sign4' */

        /* Signum: '<S170>/Sign6' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else if (rtb_sin_alpha_j > 0.0F) {
          rtb_Sum_i = 1.0F;
        } else {
          rtb_Sum_i = rtb_sin_alpha_j;
        }

        /* End of Signum: '<S170>/Sign6' */

        /* Update for DiscreteIntegrator: '<S164>/Integrator' incorporates:
         *  Constant: '<S170>/const'
         *  Gain: '<S170>/Gain3'
         *  Product: '<S170>/Divide'
         *  Product: '<S170>/Multiply5'
         *  Product: '<S170>/Multiply6'
         *  Sum: '<S170>/Subtract4'
         *  Sum: '<S170>/Subtract5'
         *  Sum: '<S170>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_i += ((rtb_sin_alpha_j / FMS_ConstB.d_d -
          rtb_Switch_mc_idx_1) * FMS_ConstB.Gain4_m * ((rtb_Switch_mc_idx_0 -
          rtb_d_f) * 0.5F) - rtb_Sum_i * 1.04719758F) * 0.01F;
        if (FMS_DW.Integrator_DSTATE_i >= 1.04719758F) {
          FMS_DW.Integrator_DSTATE_i = 1.04719758F;
        } else {
          if (FMS_DW.Integrator_DSTATE_i <= -1.04719758F) {
            FMS_DW.Integrator_DSTATE_i = -1.04719758F;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S164>/Integrator' */
        /* End of Outputs for SubSystem: '<S34>/Return' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S34>/Hold' incorporates:
         *  ActionPort: '<S153>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S153>/Bus Assignment'
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Constant: '<S153>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S153>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Constant: '<S153>/Constant3'
         *  Constant: '<S153>/Constant4'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_h;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_m;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_k;
        FMS_Y.FMS_Out.u_cmd = 0.0F;
        FMS_Y.FMS_Out.psi_rate_cmd = 0.0F;

        /* End of Outputs for SubSystem: '<S34>/Hold' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S34>/Unknown' incorporates:
         *  ActionPort: '<S155>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_g);

        /* End of Outputs for SubSystem: '<S34>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S34>/Switch Case' */
      /* End of Outputs for SubSystem: '<S27>/SubMode' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S27>/Auto' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* SwitchCase: '<S32>/Switch Case' */
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
      switch (FMS_DW.SwitchCase_ActiveSubsystem_i) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S32>/Offboard' incorporates:
         *  ActionPort: '<S92>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* S-Function (sfix_bitop): '<S143>/u_cmd valid' incorporates:
         *  Inport: '<Root>/Auto_Cmd'
         *  S-Function (sfix_bitop): '<S134>/u_cmd'
         *  SignalConversion: '<S26>/Signal Copy'
         */
        rtb_Compare_iv_tmp = FMS_U.Auto_Cmd.cmd_mask & 8192U;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* RelationalOperator: '<S146>/Compare' incorporates:
         *  Constant: '<S146>/Constant'
         *  S-Function (sfix_bitop): '<S143>/u_cmd valid'
         */
        rtb_FixPtRelationalOperator = (rtb_Compare_iv_tmp > 0U);

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MultiPortSwitch: '<S135>/Index Vector' incorporates:
         *  Inport: '<Root>/Auto_Cmd'
         *  Product: '<S135>/Multiply'
         *  Product: '<S144>/Multiply'
         *  Product: '<S145>/Multiply3'
         *  SignalConversion: '<S26>/Signal Copy'
         */
        switch (FMS_U.Auto_Cmd.frame) {
         case 0:
          /* SignalConversion: '<S148>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_f[6] = FMS_ConstB.VectorConcatenate3_c[0];
          rtb_VectorConcatenate_f[7] = FMS_ConstB.VectorConcatenate3_c[1];
          rtb_VectorConcatenate_f[8] = FMS_ConstB.VectorConcatenate3_c[2];

          /* SignalConversion: '<S148>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S148>/Constant4'
           */
          rtb_VectorConcatenate_f[5] = 0.0F;

          /* Trigonometry: '<S148>/Trigonometric Function3' incorporates:
           *  Gain: '<S147>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Trigonometry: '<S148>/Trigonometric Function1'
           */
          rtb_Sum_i = arm_cos_f32(-FMS_U.INS_Out.psi);
          rtb_VectorConcatenate_f[4] = rtb_Sum_i;

          /* Trigonometry: '<S148>/Trigonometric Function2' incorporates:
           *  Gain: '<S147>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Trigonometry: '<S148>/Trigonometric Function'
           */
          rtb_d_f = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Gain: '<S148>/Gain' incorporates:
           *  Trigonometry: '<S148>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_f[3] = -rtb_d_f;

          /* SignalConversion: '<S148>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S148>/Constant3'
           */
          rtb_VectorConcatenate_f[2] = 0.0F;

          /* Trigonometry: '<S148>/Trigonometric Function' */
          rtb_VectorConcatenate_f[1] = rtb_d_f;

          /* Trigonometry: '<S148>/Trigonometric Function1' */
          rtb_VectorConcatenate_f[0] = rtb_Sum_i;

          /* Product: '<S135>/Multiply' */
          rtb_Sum_i = rtb_FixPtRelationalOperator ? FMS_U.Auto_Cmd.u_cmd : 0.0F;
          for (i = 0; i < 3; i++) {
            rtb_Switch_k1[i] = rtb_VectorConcatenate_f[i] * rtb_Sum_i;
          }
          break;

         case 1:
          /* SignalConversion: '<S150>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_f[6] = FMS_ConstB.VectorConcatenate3_o[0];
          rtb_VectorConcatenate_f[7] = FMS_ConstB.VectorConcatenate3_o[1];
          rtb_VectorConcatenate_f[8] = FMS_ConstB.VectorConcatenate3_o[2];

          /* SignalConversion: '<S150>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S150>/Constant4'
           */
          rtb_VectorConcatenate_f[5] = 0.0F;

          /* Gain: '<S149>/Gain' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy3Inport1'
           *  Sum: '<S145>/Subtract'
           */
          rtb_d_f = -(FMS_U.INS_Out.psi - FMS_B.Cmd_In.local_psi);

          /* Trigonometry: '<S150>/Trigonometric Function3' incorporates:
           *  Trigonometry: '<S150>/Trigonometric Function1'
           */
          rtb_Sum_i = arm_cos_f32(rtb_d_f);
          rtb_VectorConcatenate_f[4] = rtb_Sum_i;

          /* Trigonometry: '<S150>/Trigonometric Function2' incorporates:
           *  Trigonometry: '<S150>/Trigonometric Function'
           */
          rtb_d_f = arm_sin_f32(rtb_d_f);

          /* Gain: '<S150>/Gain' incorporates:
           *  Trigonometry: '<S150>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_f[3] = -rtb_d_f;

          /* SignalConversion: '<S150>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S150>/Constant3'
           */
          rtb_VectorConcatenate_f[2] = 0.0F;

          /* Trigonometry: '<S150>/Trigonometric Function' */
          rtb_VectorConcatenate_f[1] = rtb_d_f;

          /* Trigonometry: '<S150>/Trigonometric Function1' */
          rtb_VectorConcatenate_f[0] = rtb_Sum_i;

          /* Product: '<S135>/Multiply' */
          rtb_Sum_i = rtb_FixPtRelationalOperator ? FMS_U.Auto_Cmd.u_cmd : 0.0F;
          for (i = 0; i < 3; i++) {
            rtb_Switch_k1[i] = rtb_VectorConcatenate_f[i] * rtb_Sum_i;
          }
          break;

         default:
          rtb_Switch_k1[0] = rtb_FixPtRelationalOperator ? FMS_U.Auto_Cmd.u_cmd :
            0.0F;
          break;
        }

        /* End of MultiPortSwitch: '<S135>/Index Vector' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  BusAssignment: '<S92>/Bus Assignment'
         *  Constant: '<S92>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S92>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Constant: '<S136>/Constant'
         *  Constant: '<S137>/Constant'
         *  Inport: '<Root>/Auto_Cmd'
         *  MATLAB Function: '<S138>/bit_shift'
         *  MATLAB Function: '<S139>/bit_shift'
         *  Outport: '<Root>/FMS_Out'
         *  RelationalOperator: '<S136>/Compare'
         *  RelationalOperator: '<S137>/Compare'
         *  S-Function (sfix_bitop): '<S134>/psi_rate_cmd'
         *  SignalConversion: '<S26>/Signal Copy'
         *  Sum: '<S134>/Add'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_d;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_c;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_f;
        FMS_Y.FMS_Out.u_cmd = rtb_Switch_k1[0];

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        FMS_Y.FMS_Out.psi_rate_cmd = FMS_U.Auto_Cmd.psi_rate_cmd;

        /* Outputs for Atomic SubSystem: '<S134>/psi_rate_cmd_valid' */
        /* Outputs for Atomic SubSystem: '<S134>/u_cmd_valid' */
        FMS_Y.FMS_Out.cmd_mask = (uint16_T)((uint32_T)(((FMS_U.Auto_Cmd.cmd_mask
          & 64U) > 0U) << 5) + ((rtb_Compare_iv_tmp > 0U) << 6));

        /* End of Outputs for SubSystem: '<S134>/u_cmd_valid' */
        /* End of Outputs for SubSystem: '<S134>/psi_rate_cmd_valid' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        /* End of Outputs for SubSystem: '<S32>/Offboard' */
        break;

       case 1:
        if (FMS_DW.SwitchCase_ActiveSubsystem_i != rtPrevAction) {
          /* SystemReset for IfAction SubSystem: '<S32>/Mission' incorporates:
           *  ActionPort: '<S91>/Action Port'
           */
          /* SystemReset for Atomic SubSystem: '<S91>/Mission_SubSystem' */
          /* SystemReset for SwitchCase: '<S32>/Switch Case' incorporates:
           *  Delay: '<S100>/Delay'
           *  DiscreteIntegrator: '<S103>/Integrator'
           *  DiscreteIntegrator: '<S103>/Integrator1'
           *  DiscreteIntegrator: '<S116>/Discrete-Time Integrator'
           *  DiscreteIntegrator: '<S97>/Integrator'
           *  DiscreteIntegrator: '<S97>/Integrator1'
           */
          FMS_DW.icLoad_j = 1U;
          FMS_DW.Integrator1_IC_LOADING = 1U;
          FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
          FMS_DW.DiscreteTimeIntegrator_PrevRese = 2;
          FMS_DW.Integrator1_DSTATE_m = 0.0F;
          FMS_DW.Integrator_DSTATE = 0.0F;
          FMS_DW.Integrator_DSTATE_b = 0.0F;

          /* End of SystemReset for SubSystem: '<S91>/Mission_SubSystem' */
          /* End of SystemReset for SubSystem: '<S32>/Mission' */
        }

        /* Outputs for IfAction SubSystem: '<S32>/Mission' incorporates:
         *  ActionPort: '<S91>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S91>/Mission_SubSystem' */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Sum: '<S94>/Subtract' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy3Inport1'
         */
        rtb_Switch_mc_idx_1 = FMS_B.Cmd_In.sp_waypoint[0] - FMS_U.INS_Out.x_R;
        rtb_Switch_mc_idx_0 = FMS_B.Cmd_In.sp_waypoint[1] - FMS_U.INS_Out.y_R;

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* SignalConversion: '<S114>/TmpSignal ConversionAtMath FunctionInport1' incorporates:
         *  Sum: '<S94>/Subtract'
         */
        rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_mc_idx_0;
        rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_mc_idx_1;

        /* Math: '<S114>/Math Function' incorporates:
         *  Sum: '<S94>/Subtract'
         */
        rtb_MathFunction_i2[0] = rtb_Switch_mc_idx_0 * rtb_Switch_mc_idx_0;
        rtb_MathFunction_i2[1] = rtb_Switch_mc_idx_1 * rtb_Switch_mc_idx_1;

        /* Sum: '<S114>/Sum of Elements' */
        rtb_d_f = rtb_MathFunction_i2[0] + rtb_MathFunction_i2[1];

        /* Math: '<S114>/Math Function1' incorporates:
         *  Sum: '<S114>/Sum of Elements'
         *
         * About '<S114>/Math Function1':
         *  Operator: sqrt
         */
        if (rtb_d_f < 0.0F) {
          rtb_Sum_i = -sqrtf(fabsf(rtb_d_f));
        } else {
          rtb_Sum_i = sqrtf(rtb_d_f);
        }

        /* End of Math: '<S114>/Math Function1' */

        /* Switch: '<S114>/Switch' incorporates:
         *  Constant: '<S114>/Constant'
         *  Product: '<S114>/Product'
         */
        if (rtb_Sum_i <= 0.0F) {
          rtb_Switch_mc_idx_0 = 0.0F;
          rtb_Switch_mc_idx_1 = 0.0F;
          rtb_Sum_i = 1.0F;
        }

        /* End of Switch: '<S114>/Switch' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* MATLAB Function: '<S119>/NearbyRefWP' incorporates:
         *  Constant: '<S94>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_NearbyRefWP(&rtb_Switch_k1[0], FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R,
                        FMS_PARAM.L1, rtb_TmpSignalConversionAtMat_co, &rtb_d_f);

        /* MATLAB Function: '<S119>/SearchL1RefWP' incorporates:
         *  Constant: '<S94>/L1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_SearchL1RefWP(&rtb_Switch_i[0], &rtb_Switch_k1[0], FMS_U.INS_Out.x_R,
                          FMS_U.INS_Out.y_R, FMS_PARAM.L1, rtb_MathFunction_i2,
                          &rtb_u_g);

        /* MATLAB Function: '<S119>/OutRegionRegWP' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        FMS_OutRegionRegWP(&rtb_Switch_i[0], &rtb_Switch_k1[0],
                           FMS_U.INS_Out.x_R, FMS_U.INS_Out.y_R, rtb_P_on);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Product: '<S114>/Divide' */
        rtb_P_c[0] = rtb_Switch_mc_idx_0 / rtb_Sum_i;
        rtb_P_c[1] = rtb_Switch_mc_idx_1 / rtb_Sum_i;

        /* Delay: '<S100>/Delay' incorporates:
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         */
        if (FMS_DW.icLoad_j != 0) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          FMS_DW.Delay_DSTATE_l = FMS_U.INS_Out.psi;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
        }

        /* DiscreteIntegrator: '<S103>/Integrator1' incorporates:
         *  Delay: '<S100>/Delay'
         */
        if (FMS_DW.Integrator1_IC_LOADING != 0) {
          FMS_DW.Integrator1_DSTATE = FMS_DW.Delay_DSTATE_l;
        }

        /* Sum: '<S112>/Subtract' incorporates:
         *  Product: '<S112>/Multiply'
         *  Product: '<S112>/Multiply1'
         */
        rtb_Sum_i = rtb_P_c[0] * FMS_ConstB.Divide[1] - rtb_P_c[1] *
          FMS_ConstB.Divide[0];

        /* Signum: '<S101>/Sign1' */
        if (rtb_Sum_i < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else {
          if (rtb_Sum_i > 0.0F) {
            rtb_Sum_i = 1.0F;
          }
        }

        /* End of Signum: '<S101>/Sign1' */

        /* Switch: '<S101>/Switch2' incorporates:
         *  Constant: '<S101>/Constant4'
         */
        if (rtb_Sum_i == 0.0F) {
          rtb_Sum_i = 1.0F;
        }

        /* End of Switch: '<S101>/Switch2' */

        /* DotProduct: '<S101>/Dot Product' */
        rtb_Switch_mc_idx_0 = FMS_ConstB.Divide[0] * rtb_P_c[0] +
          FMS_ConstB.Divide[1] * rtb_P_c[1];

        /* Trigonometry: '<S101>/Acos' incorporates:
         *  DotProduct: '<S101>/Dot Product'
         */
        if (rtb_Switch_mc_idx_0 > 1.0F) {
          rtb_Switch_mc_idx_0 = 1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 < -1.0F) {
            rtb_Switch_mc_idx_0 = -1.0F;
          }
        }

        /* Product: '<S101>/Multiply' incorporates:
         *  Trigonometry: '<S101>/Acos'
         */
        rtb_Sum_i *= acosf(rtb_Switch_mc_idx_0);

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Math: '<S105>/Rem' incorporates:
         *  Constant: '<S105>/Constant1'
         *  Inport: '<Root>/INS_Out'
         *  SignalConversion: '<S26>/Signal Copy1'
         *  Sum: '<S100>/Sum1'
         */
        rtb_sin_alpha_j = rt_remf(rtb_Sum_i - FMS_U.INS_Out.psi, 6.28318548F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Switch: '<S105>/Switch' incorporates:
         *  Abs: '<S105>/Abs'
         *  Constant: '<S105>/Constant'
         *  Constant: '<S111>/Constant'
         *  Product: '<S105>/Multiply'
         *  RelationalOperator: '<S111>/Compare'
         *  Sum: '<S105>/Add'
         */
        if (fabsf(rtb_sin_alpha_j) > 3.14159274F) {
          /* Signum: '<S105>/Sign' */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_0 = -1.0F;
          } else if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          } else {
            rtb_Switch_mc_idx_0 = rtb_sin_alpha_j;
          }

          /* End of Signum: '<S105>/Sign' */
          rtb_sin_alpha_j -= 6.28318548F * rtb_Switch_mc_idx_0;
        }

        /* End of Switch: '<S105>/Switch' */

        /* Abs: '<S96>/Abs' */
        rtb_sin_alpha_j = fabsf(rtb_sin_alpha_j);

        /* RelationalOperator: '<S115>/Compare' incorporates:
         *  Constant: '<S115>/Constant'
         */
        rtb_FixPtRelationalOperator = (rtb_sin_alpha_j <= 0.17453292F);

        /* DiscreteIntegrator: '<S116>/Discrete-Time Integrator' */
        if ((!rtb_FixPtRelationalOperator) &&
            (FMS_DW.DiscreteTimeIntegrator_PrevRese == 1)) {
          FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
        }

        if (FMS_DW.DiscreteTimeIntegrator_DSTATE_c >= 100) {
          FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 100U;
        } else {
          if (FMS_DW.DiscreteTimeIntegrator_DSTATE_c <= 0) {
            FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
          }
        }

        /* Logic: '<S94>/NOT' incorporates:
         *  Constant: '<S117>/Constant'
         *  DiscreteIntegrator: '<S116>/Discrete-Time Integrator'
         *  RelationalOperator: '<S117>/Compare'
         */
        rtb_NOT_a = (FMS_DW.DiscreteTimeIntegrator_DSTATE_c <= 3);

        /* Switch: '<S94>/Switch' incorporates:
         *  Constant: '<S120>/Constant'
         *  Constant: '<S122>/Constant'
         *  Constant: '<S94>/L1'
         *  Gain: '<S120>/Gain'
         *  Gain: '<S98>/AY_P'
         *  Math: '<S120>/Square'
         *  MinMax: '<S120>/Max'
         *  MinMax: '<S120>/Min'
         *  Product: '<S120>/Divide'
         *  Product: '<S120>/Multiply1'
         *  RelationalOperator: '<S122>/Compare'
         *  Sqrt: '<S128>/Sqrt'
         *  Switch: '<S119>/Switch1'
         */
        if (rtb_NOT_a) {
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Math: '<S107>/Rem' incorporates:
           *  Constant: '<S107>/Constant1'
           *  DiscreteIntegrator: '<S103>/Integrator1'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Sum: '<S102>/Sum'
           */
          rtb_sin_alpha_j = rt_remf(FMS_DW.Integrator1_DSTATE -
            FMS_U.INS_Out.psi, 6.28318548F);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Switch: '<S107>/Switch' incorporates:
           *  Abs: '<S107>/Abs'
           *  Constant: '<S107>/Constant'
           *  Constant: '<S108>/Constant'
           *  Product: '<S107>/Multiply'
           *  RelationalOperator: '<S108>/Compare'
           *  Sum: '<S107>/Add'
           */
          if (fabsf(rtb_sin_alpha_j) > 3.14159274F) {
            /* Signum: '<S107>/Sign' */
            if (rtb_sin_alpha_j < 0.0F) {
              rtb_d_f = -1.0F;
            } else if (rtb_sin_alpha_j > 0.0F) {
              rtb_d_f = 1.0F;
            } else {
              rtb_d_f = rtb_sin_alpha_j;
            }

            /* End of Signum: '<S107>/Sign' */
            rtb_sin_alpha_j -= 6.28318548F * rtb_d_f;
          }

          /* End of Switch: '<S107>/Switch' */

          /* Gain: '<S102>/Gain2' */
          rtb_sin_alpha_j *= FMS_PARAM.YAW_P;

          /* Saturate: '<S102>/Saturation' */
          if (rtb_sin_alpha_j > 1.04719758F) {
            rtb_sin_alpha_j = 1.04719758F;
          } else {
            if (rtb_sin_alpha_j < -1.04719758F) {
              rtb_sin_alpha_j = -1.04719758F;
            }
          }

          /* End of Saturate: '<S102>/Saturation' */
        } else {
          if (rtb_d_f <= 0.0F) {
            /* Switch: '<S119>/Switch' incorporates:
             *  Constant: '<S121>/Constant'
             *  RelationalOperator: '<S121>/Compare'
             *  Switch: '<S119>/Switch1'
             */
            if (rtb_u_g >= 0.0F) {
              rtb_TmpSignalConversionAtMat_co[0] = rtb_MathFunction_i2[0];
              rtb_TmpSignalConversionAtMat_co[1] = rtb_MathFunction_i2[1];
            } else {
              rtb_TmpSignalConversionAtMat_co[0] = rtb_P_on[0];
              rtb_TmpSignalConversionAtMat_co[1] = rtb_P_on[1];
            }

            /* End of Switch: '<S119>/Switch' */
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S120>/Subtract' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_P_on[0] = rtb_TmpSignalConversionAtMat_co[0] - FMS_U.INS_Out.x_R;
          rtb_P_on[1] = rtb_TmpSignalConversionAtMat_co[1] - FMS_U.INS_Out.y_R;

          /* Sum: '<S129>/Sum of Elements' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  Math: '<S129>/Math Function'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Sum: '<S127>/Sum of Elements'
           */
          rtb_d_f = FMS_U.INS_Out.vn * FMS_U.INS_Out.vn + FMS_U.INS_Out.ve *
            FMS_U.INS_Out.ve;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Math: '<S129>/Math Function1' incorporates:
           *  Sum: '<S129>/Sum of Elements'
           *
           * About '<S129>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_d_f < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_d_f));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_d_f);
          }

          /* End of Math: '<S129>/Math Function1' */

          /* Switch: '<S129>/Switch' incorporates:
           *  Constant: '<S129>/Constant'
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S129>/Product'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            rtb_Switch_k1[0] = FMS_U.INS_Out.vn;
            rtb_Switch_k1[1] = FMS_U.INS_Out.ve;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S129>/Switch' */

          /* Product: '<S129>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_k1[0] / rtb_Switch_k1
            [2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_k1[1] / rtb_Switch_k1
            [2];

          /* Sum: '<S130>/Sum of Elements' incorporates:
           *  Math: '<S130>/Math Function'
           *  Sum: '<S128>/Sum of Elements'
           */
          rtb_Switch_mc_idx_0 = rtb_P_on[0] * rtb_P_on[0] + rtb_P_on[1] *
            rtb_P_on[1];

          /* Math: '<S130>/Math Function1' incorporates:
           *  Sum: '<S130>/Sum of Elements'
           *
           * About '<S130>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_Switch_mc_idx_0 < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_Switch_mc_idx_0));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_Switch_mc_idx_0);
          }

          /* End of Math: '<S130>/Math Function1' */

          /* Switch: '<S130>/Switch' incorporates:
           *  Constant: '<S130>/Constant'
           *  Product: '<S130>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_P_on[0];
            rtb_Switch_k1[1] = rtb_P_on[1];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S130>/Switch' */

          /* Product: '<S130>/Divide' */
          rtb_P_on[0] = rtb_Switch_k1[0] / rtb_Switch_k1[2];
          rtb_P_on[1] = rtb_Switch_k1[1] / rtb_Switch_k1[2];

          /* Sum: '<S132>/Sum of Elements' incorporates:
           *  Math: '<S132>/Math Function'
           *  SignalConversion: '<S132>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[1] *
            rtb_TmpSignalConversionAtMat_co[1] +
            rtb_TmpSignalConversionAtMat_co[0] *
            rtb_TmpSignalConversionAtMat_co[0];

          /* Math: '<S132>/Math Function1' incorporates:
           *  Sum: '<S132>/Sum of Elements'
           *
           * About '<S132>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S132>/Math Function1' */

          /* Switch: '<S132>/Switch' incorporates:
           *  Constant: '<S132>/Constant'
           *  Product: '<S132>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_TmpSignalConversionAtMat_co[1];
            rtb_Switch_k1[1] = rtb_TmpSignalConversionAtMat_co[0];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S132>/Switch' */

          /* Product: '<S132>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_k1[0] / rtb_Switch_k1
            [2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_k1[1] / rtb_Switch_k1
            [2];

          /* Sum: '<S133>/Sum of Elements' incorporates:
           *  Math: '<S133>/Math Function'
           *  SignalConversion: '<S133>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_sin_alpha_j = rtb_P_on[1] * rtb_P_on[1] + rtb_P_on[0] * rtb_P_on[0];

          /* Math: '<S133>/Math Function1' incorporates:
           *  Sum: '<S133>/Sum of Elements'
           *
           * About '<S133>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_Switch_mc_idx_1 = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_Switch_mc_idx_1 = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S133>/Math Function1' */

          /* Switch: '<S133>/Switch' incorporates:
           *  Constant: '<S133>/Constant'
           *  Product: '<S133>/Product'
           */
          if (rtb_Switch_mc_idx_1 > 0.0F) {
            rtb_Switch_k1[0] = rtb_P_on[1];
            rtb_Switch_k1[1] = rtb_P_on[0];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_1;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S133>/Switch' */

          /* Product: '<S133>/Divide' */
          rtb_P_on[0] = rtb_Switch_k1[0] / rtb_Switch_k1[2];
          rtb_P_on[1] = rtb_Switch_k1[1] / rtb_Switch_k1[2];

          /* Sum: '<S131>/Subtract' incorporates:
           *  Product: '<S131>/Multiply'
           *  Product: '<S131>/Multiply1'
           */
          rtb_Switch_mc_idx_1 = rtb_P_on[0] * rtb_TmpSignalConversionAtMat_co[1]
            - rtb_P_on[1] * rtb_TmpSignalConversionAtMat_co[0];

          /* Signum: '<S126>/Sign1' */
          if (rtb_Switch_mc_idx_1 < 0.0F) {
            rtb_Switch_mc_idx_1 = -1.0F;
          } else {
            if (rtb_Switch_mc_idx_1 > 0.0F) {
              rtb_Switch_mc_idx_1 = 1.0F;
            }
          }

          /* End of Signum: '<S126>/Sign1' */

          /* Switch: '<S126>/Switch2' incorporates:
           *  Constant: '<S126>/Constant4'
           */
          if (rtb_Switch_mc_idx_1 == 0.0F) {
            rtb_Switch_mc_idx_1 = 1.0F;
          }

          /* End of Switch: '<S126>/Switch2' */

          /* DotProduct: '<S126>/Dot Product' */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[0] * rtb_P_on[0] +
            rtb_TmpSignalConversionAtMat_co[1] * rtb_P_on[1];

          /* Trigonometry: '<S126>/Acos' incorporates:
           *  DotProduct: '<S126>/Dot Product'
           */
          if (rtb_sin_alpha_j > 1.0F) {
            rtb_sin_alpha_j = 1.0F;
          } else {
            if (rtb_sin_alpha_j < -1.0F) {
              rtb_sin_alpha_j = -1.0F;
            }
          }

          /* Product: '<S126>/Multiply' incorporates:
           *  Trigonometry: '<S126>/Acos'
           */
          rtb_Switch_mc_idx_1 *= acosf(rtb_sin_alpha_j);

          /* Saturate: '<S120>/Saturation' */
          if (rtb_Switch_mc_idx_1 > 1.57079637F) {
            rtb_Switch_mc_idx_1 = 1.57079637F;
          } else {
            if (rtb_Switch_mc_idx_1 < -1.57079637F) {
              rtb_Switch_mc_idx_1 = -1.57079637F;
            }
          }

          /* End of Saturate: '<S120>/Saturation' */

          /* Trigonometry: '<S120>/Sin' */
          rtb_sin_alpha_j = arm_sin_f32(rtb_Switch_mc_idx_1);

          /* Sqrt: '<S127>/Sqrt' */
          rtb_Switch_mc_idx_1 = sqrtf(rtb_d_f);
          rtb_sin_alpha_j = rtb_Switch_mc_idx_1 * rtb_Switch_mc_idx_1 * 2.0F *
            rtb_sin_alpha_j / fminf(FMS_PARAM.L1, fmaxf(sqrtf
            (rtb_Switch_mc_idx_0), 0.5F)) * FMS_PARAM.AY_P;
        }

        /* End of Switch: '<S94>/Switch' */

        /* Saturate: '<S94>/Saturation' */
        if (rtb_sin_alpha_j > 1.0F) {
          rtb_sin_alpha_j = 1.0F;
        } else {
          if (rtb_sin_alpha_j < -1.0F) {
            rtb_sin_alpha_j = -1.0F;
          }
        }

        /* End of Saturate: '<S94>/Saturation' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  BusAssignment: '<S94>/Bus Assignment'
         *  Constant: '<S94>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S94>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  DiscreteIntegrator: '<S97>/Integrator1'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_l;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_b;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_h;
        FMS_Y.FMS_Out.u_cmd = FMS_DW.Integrator1_DSTATE_m;
        FMS_Y.FMS_Out.psi_rate_cmd = rtb_sin_alpha_j;

        /* Product: '<S118>/Multiply1' incorporates:
         *  Constant: '<S118>/const1'
         *  DiscreteIntegrator: '<S97>/Integrator'
         */
        rtb_sin_alpha_j = FMS_DW.Integrator_DSTATE * 0.5F;

        /* Switch: '<S94>/Switch1' incorporates:
         *  Constant: '<S94>/Constant4'
         *  Constant: '<S94>/vel'
         */
        if (rtb_NOT_a) {
          rtb_d_f = 0.0F;
        } else {
          rtb_d_f = FMS_PARAM.CRUISE_SPEED;
        }

        /* End of Switch: '<S94>/Switch1' */

        /* Sum: '<S118>/Add' incorporates:
         *  DiscreteIntegrator: '<S97>/Integrator1'
         *  Sum: '<S97>/Subtract'
         */
        rtb_Subtract1_j = (FMS_DW.Integrator1_DSTATE_m - rtb_d_f) +
          rtb_sin_alpha_j;

        /* Signum: '<S118>/Sign' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Subtract1_k_0 = -1.0F;
        } else if (rtb_Subtract1_j > 0.0F) {
          rtb_Subtract1_k_0 = 1.0F;
        } else {
          rtb_Subtract1_k_0 = rtb_Subtract1_j;
        }

        /* End of Signum: '<S118>/Sign' */

        /* Sum: '<S118>/Add2' incorporates:
         *  Abs: '<S118>/Abs'
         *  Gain: '<S118>/Gain'
         *  Gain: '<S118>/Gain1'
         *  Product: '<S118>/Multiply2'
         *  Product: '<S118>/Multiply3'
         *  Sqrt: '<S118>/Sqrt'
         *  Sum: '<S118>/Add1'
         *  Sum: '<S118>/Subtract'
         */
        rtb_Rem_kb = (sqrtf((8.0F * fabsf(rtb_Subtract1_j) + FMS_ConstB.d) *
                            FMS_ConstB.d) - FMS_ConstB.d) * 0.5F *
          rtb_Subtract1_k_0 + rtb_sin_alpha_j;

        /* Sum: '<S118>/Add4' */
        rtb_Switch_mc_idx_0 = (rtb_Subtract1_j - rtb_Rem_kb) + rtb_sin_alpha_j;

        /* Sum: '<S118>/Add3' */
        rtb_d_f = rtb_Subtract1_j + FMS_ConstB.d;

        /* Sum: '<S118>/Subtract1' */
        rtb_Subtract1_j -= FMS_ConstB.d;

        /* Signum: '<S118>/Sign1' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S118>/Sign1' */

        /* Signum: '<S118>/Sign2' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Subtract1_j = -1.0F;
        } else {
          if (rtb_Subtract1_j > 0.0F) {
            rtb_Subtract1_j = 1.0F;
          }
        }

        /* End of Signum: '<S118>/Sign2' */

        /* Sum: '<S118>/Add5' incorporates:
         *  Gain: '<S118>/Gain2'
         *  Product: '<S118>/Multiply4'
         *  Sum: '<S118>/Subtract2'
         */
        rtb_Rem_kb += (rtb_d_f - rtb_Subtract1_j) * 0.5F * rtb_Switch_mc_idx_0;

        /* Sum: '<S118>/Add6' */
        rtb_d_f = rtb_Rem_kb + FMS_ConstB.d;

        /* Sum: '<S118>/Subtract3' */
        rtb_Switch_mc_idx_0 = rtb_Rem_kb - FMS_ConstB.d;

        /* Product: '<S118>/Divide' */
        rtb_u_g = rtb_Rem_kb / FMS_ConstB.d;

        /* Signum: '<S118>/Sign5' incorporates:
         *  Signum: '<S118>/Sign6'
         */
        if (rtb_Rem_kb < 0.0F) {
          rtb_Sign5_d = -1.0F;
          rtb_Switch_mc_idx_1 = -1.0F;
        } else if (rtb_Rem_kb > 0.0F) {
          rtb_Sign5_d = 1.0F;
          rtb_Switch_mc_idx_1 = 1.0F;
        } else {
          rtb_Sign5_d = rtb_Rem_kb;
          rtb_Switch_mc_idx_1 = rtb_Rem_kb;
        }

        /* End of Signum: '<S118>/Sign5' */

        /* Sum: '<S100>/Sum2' incorporates:
         *  Delay: '<S100>/Delay'
         */
        rtb_Sum_i -= FMS_DW.Delay_DSTATE_l;

        /* Math: '<S104>/Rem' incorporates:
         *  Constant: '<S104>/Constant1'
         */
        rtb_Rem_kb = rt_remf(rtb_Sum_i, 6.28318548F);

        /* Switch: '<S104>/Switch' incorporates:
         *  Abs: '<S104>/Abs'
         *  Constant: '<S104>/Constant'
         *  Constant: '<S110>/Constant'
         *  Product: '<S104>/Multiply'
         *  RelationalOperator: '<S110>/Compare'
         *  Sum: '<S104>/Add'
         */
        if (fabsf(rtb_Rem_kb) > 3.14159274F) {
          /* Signum: '<S104>/Sign' */
          if (rtb_Rem_kb < 0.0F) {
            rtb_Sum_i = -1.0F;
          } else if (rtb_Rem_kb > 0.0F) {
            rtb_Sum_i = 1.0F;
          } else {
            rtb_Sum_i = rtb_Rem_kb;
          }

          /* End of Signum: '<S104>/Sign' */
          rtb_Rem_kb -= 6.28318548F * rtb_Sum_i;
        }

        /* End of Switch: '<S104>/Switch' */

        /* Sum: '<S100>/Sum' incorporates:
         *  Delay: '<S100>/Delay'
         */
        rtb_Sum_i = rtb_Rem_kb + FMS_DW.Delay_DSTATE_l;

        /* Product: '<S109>/Multiply1' incorporates:
         *  Constant: '<S109>/const1'
         *  DiscreteIntegrator: '<S103>/Integrator'
         */
        rtb_Rem_kb = FMS_DW.Integrator_DSTATE_b * 0.785398185F;

        /* Sum: '<S109>/Add' incorporates:
         *  DiscreteIntegrator: '<S103>/Integrator1'
         *  Sum: '<S103>/Subtract'
         */
        rtb_Subtract1_j = (FMS_DW.Integrator1_DSTATE - rtb_Sum_i) + rtb_Rem_kb;

        /* Signum: '<S109>/Sign' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Subtract1_k_0 = -1.0F;
        } else if (rtb_Subtract1_j > 0.0F) {
          rtb_Subtract1_k_0 = 1.0F;
        } else {
          rtb_Subtract1_k_0 = rtb_Subtract1_j;
        }

        /* End of Signum: '<S109>/Sign' */

        /* Sum: '<S109>/Add2' incorporates:
         *  Abs: '<S109>/Abs'
         *  Gain: '<S109>/Gain'
         *  Gain: '<S109>/Gain1'
         *  Product: '<S109>/Multiply2'
         *  Product: '<S109>/Multiply3'
         *  Sqrt: '<S109>/Sqrt'
         *  Sum: '<S109>/Add1'
         *  Sum: '<S109>/Subtract'
         */
        rtb_sin_alpha_j = (sqrtf((8.0F * fabsf(rtb_Subtract1_j) + FMS_ConstB.d_b)
          * FMS_ConstB.d_b) - FMS_ConstB.d_b) * 0.5F * rtb_Subtract1_k_0 +
          rtb_Rem_kb;

        /* Sum: '<S109>/Add4' */
        rtb_Rem_kb += rtb_Subtract1_j - rtb_sin_alpha_j;

        /* Sum: '<S109>/Add3' */
        rtb_Sum_i = rtb_Subtract1_j + FMS_ConstB.d_b;

        /* Sum: '<S109>/Subtract1' */
        rtb_Subtract1_j -= FMS_ConstB.d_b;

        /* Signum: '<S109>/Sign1' */
        if (rtb_Sum_i < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else {
          if (rtb_Sum_i > 0.0F) {
            rtb_Sum_i = 1.0F;
          }
        }

        /* End of Signum: '<S109>/Sign1' */

        /* Signum: '<S109>/Sign2' */
        if (rtb_Subtract1_j < 0.0F) {
          rtb_Subtract1_j = -1.0F;
        } else {
          if (rtb_Subtract1_j > 0.0F) {
            rtb_Subtract1_j = 1.0F;
          }
        }

        /* End of Signum: '<S109>/Sign2' */

        /* Sum: '<S109>/Add5' incorporates:
         *  Gain: '<S109>/Gain2'
         *  Product: '<S109>/Multiply4'
         *  Sum: '<S109>/Subtract2'
         */
        rtb_sin_alpha_j += (rtb_Sum_i - rtb_Subtract1_j) * 0.5F * rtb_Rem_kb;

        /* Update for Delay: '<S100>/Delay' */
        FMS_DW.icLoad_j = 0U;

        /* Update for DiscreteIntegrator: '<S103>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S103>/Integrator'
         */
        FMS_DW.Integrator1_IC_LOADING = 0U;
        FMS_DW.Integrator1_DSTATE += 0.01F * FMS_DW.Integrator_DSTATE_b;

        /* Update for DiscreteIntegrator: '<S116>/Discrete-Time Integrator' */
        FMS_DW.DiscreteTimeIntegrator_DSTATE_c = (uint8_T)((uint32_T)
          FMS_DW.DiscreteTimeIntegrator_DSTATE_c + rtb_FixPtRelationalOperator);
        if (FMS_DW.DiscreteTimeIntegrator_DSTATE_c >= 100) {
          FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 100U;
        } else {
          if (FMS_DW.DiscreteTimeIntegrator_DSTATE_c <= 0) {
            FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
          }
        }

        FMS_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)
          rtb_FixPtRelationalOperator;

        /* End of Update for DiscreteIntegrator: '<S116>/Discrete-Time Integrator' */

        /* Update for DiscreteIntegrator: '<S97>/Integrator1' incorporates:
         *  DiscreteIntegrator: '<S97>/Integrator'
         */
        FMS_DW.Integrator1_DSTATE_m += 0.01F * FMS_DW.Integrator_DSTATE;

        /* Signum: '<S118>/Sign3' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S118>/Sign3' */

        /* Signum: '<S118>/Sign4' */
        if (rtb_Switch_mc_idx_0 < 0.0F) {
          rtb_Switch_mc_idx_0 = -1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          }
        }

        /* End of Signum: '<S118>/Sign4' */

        /* Update for DiscreteIntegrator: '<S97>/Integrator' incorporates:
         *  Constant: '<S118>/const'
         *  Gain: '<S118>/Gain3'
         *  Product: '<S118>/Multiply5'
         *  Product: '<S118>/Multiply6'
         *  Sum: '<S118>/Subtract4'
         *  Sum: '<S118>/Subtract5'
         *  Sum: '<S118>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE += ((rtb_u_g - rtb_Sign5_d) * FMS_ConstB.Gain4 *
          ((rtb_d_f - rtb_Switch_mc_idx_0) * 0.5F) - rtb_Switch_mc_idx_1 *
          19.612F) * 0.01F;

        /* Sum: '<S109>/Subtract3' */
        rtb_d_f = rtb_sin_alpha_j - FMS_ConstB.d_b;

        /* Sum: '<S109>/Add6' */
        rtb_Switch_mc_idx_0 = rtb_sin_alpha_j + FMS_ConstB.d_b;

        /* Signum: '<S109>/Sign5' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_Sum_i = -1.0F;
        } else if (rtb_sin_alpha_j > 0.0F) {
          rtb_Sum_i = 1.0F;
        } else {
          rtb_Sum_i = rtb_sin_alpha_j;
        }

        /* End of Signum: '<S109>/Sign5' */

        /* Signum: '<S109>/Sign3' */
        if (rtb_Switch_mc_idx_0 < 0.0F) {
          rtb_Switch_mc_idx_0 = -1.0F;
        } else {
          if (rtb_Switch_mc_idx_0 > 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          }
        }

        /* End of Signum: '<S109>/Sign3' */

        /* Signum: '<S109>/Sign4' */
        if (rtb_d_f < 0.0F) {
          rtb_d_f = -1.0F;
        } else {
          if (rtb_d_f > 0.0F) {
            rtb_d_f = 1.0F;
          }
        }

        /* End of Signum: '<S109>/Sign4' */

        /* Signum: '<S109>/Sign6' */
        if (rtb_sin_alpha_j < 0.0F) {
          rtb_Switch_mc_idx_1 = -1.0F;
        } else if (rtb_sin_alpha_j > 0.0F) {
          rtb_Switch_mc_idx_1 = 1.0F;
        } else {
          rtb_Switch_mc_idx_1 = rtb_sin_alpha_j;
        }

        /* End of Signum: '<S109>/Sign6' */

        /* Update for DiscreteIntegrator: '<S103>/Integrator' incorporates:
         *  Constant: '<S109>/const'
         *  Gain: '<S109>/Gain3'
         *  Product: '<S109>/Divide'
         *  Product: '<S109>/Multiply5'
         *  Product: '<S109>/Multiply6'
         *  Sum: '<S109>/Subtract4'
         *  Sum: '<S109>/Subtract5'
         *  Sum: '<S109>/Subtract6'
         */
        FMS_DW.Integrator_DSTATE_b += ((rtb_sin_alpha_j / FMS_ConstB.d_b -
          rtb_Sum_i) * FMS_ConstB.Gain4_n * ((rtb_Switch_mc_idx_0 - rtb_d_f) *
          0.5F) - rtb_Switch_mc_idx_1 * 1.04719758F) * 0.01F;
        if (FMS_DW.Integrator_DSTATE_b >= 1.04719758F) {
          FMS_DW.Integrator_DSTATE_b = 1.04719758F;
        } else {
          if (FMS_DW.Integrator_DSTATE_b <= -1.04719758F) {
            FMS_DW.Integrator_DSTATE_b = -1.04719758F;
          }
        }

        /* End of Update for DiscreteIntegrator: '<S103>/Integrator' */
        /* End of Outputs for SubSystem: '<S91>/Mission_SubSystem' */
        /* End of Outputs for SubSystem: '<S32>/Mission' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S32>/Unknown' incorporates:
         *  ActionPort: '<S93>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_d);

        /* End of Outputs for SubSystem: '<S32>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S32>/Switch Case' */
      /* End of Outputs for SubSystem: '<S27>/Auto' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S27>/Assist' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      /* SwitchCase: '<S31>/Switch Case' */
      rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_f;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      switch (FMS_B.state) {
       case VehicleState_Stabilize:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 0;
        break;

       case VehicleState_Position:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 1;
        break;

       default:
        FMS_DW.SwitchCase_ActiveSubsystem_f = 2;
        break;
      }

      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
      if (rtPrevAction != FMS_DW.SwitchCase_ActiveSubsystem_f) {
        switch (rtPrevAction) {
         case 0:
          /* Disable for SwitchCase: '<S79>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_n = -1;
          break;

         case 1:
          /* Disable for SwitchCase: '<S41>/Switch Case' */
          FMS_DW.SwitchCase_ActiveSubsystem_d = -1;
          break;

         case 2:
          break;
        }
      }

      switch (FMS_DW.SwitchCase_ActiveSubsystem_f) {
       case 0:
        if (FMS_DW.SwitchCase_ActiveSubsystem_f != rtPrevAction) {
          /* SystemReset for IfAction SubSystem: '<S31>/Stabilize' incorporates:
           *  ActionPort: '<S37>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S31>/Switch Case' incorporates:
           *  Chart: '<S80>/Motion State'
           */
          FMS_DW.temporalCounter_i1_b = 0U;
          FMS_DW.is_active_c10_FMS = 0U;
          FMS_DW.is_c10_FMS = FMS_IN_NO_ACTIVE_CHILD;

          /* End of SystemReset for SubSystem: '<S31>/Stabilize' */
        }

        /* Outputs for IfAction SubSystem: '<S31>/Stabilize' incorporates:
         *  ActionPort: '<S37>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S76>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S26>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_throttle > 0.05F) {
          rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - 0.05F;
        } else if (FMS_U.Pilot_Cmd.stick_throttle >= -0.05F) {
          rtb_d_f = 0.0F;
        } else {
          rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - -0.05F;
        }

        /* End of DeadZone: '<S76>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Gain: '<S76>/Gain' */
        rtb_Sum_i = 1.05263162F * rtb_d_f;

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Logic: '<S80>/Logical Operator' incorporates:
         *  Abs: '<S80>/Abs1'
         *  Abs: '<S80>/Abs2'
         *  Constant: '<S88>/Constant'
         *  Constant: '<S89>/Constant'
         *  Inport: '<Root>/Pilot_Cmd'
         *  RelationalOperator: '<S88>/Compare'
         *  RelationalOperator: '<S89>/Compare'
         *  SignalConversion: '<S26>/Signal Copy2'
         */
        rtb_FixPtRelationalOperator = ((fabsf(FMS_U.Pilot_Cmd.stick_roll) >
          0.05F) || (fabsf(FMS_U.Pilot_Cmd.stick_throttle) <= 0.05F));

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Chart: '<S80>/Motion State' */
        if (FMS_DW.temporalCounter_i1_b < 255U) {
          FMS_DW.temporalCounter_i1_b++;
        }

        if (FMS_DW.is_active_c10_FMS == 0U) {
          FMS_DW.is_active_c10_FMS = 1U;
          FMS_DW.is_c10_FMS = FMS_IN_Move;
          rtb_state_d = MotionState_Move;
        } else {
          switch (FMS_DW.is_c10_FMS) {
           case FMS_IN_Brake:
            rtb_state_d = MotionState_Brake;
            if (rtb_FixPtRelationalOperator) {
              FMS_DW.is_c10_FMS = FMS_IN_Move;
              rtb_state_d = MotionState_Move;
            } else {
              if (FMS_DW.temporalCounter_i1_b >= 150U) {
                FMS_DW.is_c10_FMS = FMS_IN_Hold;
                rtb_state_d = MotionState_Hold;
              }
            }
            break;

           case FMS_IN_Hold:
            rtb_state_d = MotionState_Hold;
            if (rtb_FixPtRelationalOperator) {
              FMS_DW.is_c10_FMS = FMS_IN_Move;
              rtb_state_d = MotionState_Move;
            }
            break;

           default:
            rtb_state_d = MotionState_Move;
            if (!rtb_FixPtRelationalOperator) {
              FMS_DW.is_c10_FMS = FMS_IN_Brake;
              FMS_DW.temporalCounter_i1_b = 0U;
              rtb_state_d = MotionState_Brake;
            }
            break;
          }
        }

        /* End of Chart: '<S80>/Motion State' */

        /* SwitchCase: '<S79>/Switch Case' */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_n;
        FMS_DW.SwitchCase_ActiveSubsystem_n = -1;
        switch (rtb_state_d) {
         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_n = 0;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_n = 1;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_n = 2;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_n) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_n != rtPrevAction) {
            /* InitializeConditions for IfAction SubSystem: '<S79>/Hold Control' incorporates:
             *  ActionPort: '<S82>/Action Port'
             */
            /* InitializeConditions for SwitchCase: '<S79>/Switch Case' incorporates:
             *  Delay: '<S82>/Delay'
             */
            FMS_DW.icLoad_nm = 1U;

            /* End of InitializeConditions for SubSystem: '<S79>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S79>/Hold Control' incorporates:
           *  ActionPort: '<S82>/Action Port'
           */
          /* Delay: '<S82>/Delay' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (FMS_DW.icLoad_nm != 0) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            FMS_DW.Delay_DSTATE = FMS_U.INS_Out.psi;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S82>/Sum' incorporates:
           *  Delay: '<S82>/Delay'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_d_f = FMS_DW.Delay_DSTATE - FMS_U.INS_Out.psi;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Abs: '<S85>/Abs' */
          rtb_Switch_mc_idx_0 = fabsf(rtb_d_f);

          /* Switch: '<S85>/Switch' incorporates:
           *  Constant: '<S85>/Constant'
           *  Constant: '<S86>/Constant'
           *  Product: '<S85>/Multiply'
           *  RelationalOperator: '<S86>/Compare'
           *  Sum: '<S85>/Subtract'
           */
          if (rtb_Switch_mc_idx_0 > 3.14159274F) {
            /* Signum: '<S85>/Sign' */
            if (rtb_d_f < 0.0F) {
              rtb_d_f = -1.0F;
            } else {
              if (rtb_d_f > 0.0F) {
                rtb_d_f = 1.0F;
              }
            }

            /* End of Signum: '<S85>/Sign' */
            rtb_d_f *= rtb_Switch_mc_idx_0 - 6.28318548F;
          }

          /* End of Switch: '<S85>/Switch' */

          /* Gain: '<S82>/Gain2' */
          FMS_B.Merge_h = FMS_PARAM.YAW_P * rtb_d_f;

          /* Update for Delay: '<S82>/Delay' */
          FMS_DW.icLoad_nm = 0U;

          /* End of Outputs for SubSystem: '<S79>/Hold Control' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S79>/Brake Control' incorporates:
           *  ActionPort: '<S81>/Action Port'
           */
          /* SignalConversion: '<S81>/OutportBuffer_InsertedFor_psi_rate_cmd_radPs_at_inport_0' incorporates:
           *  Constant: '<S81>/Constant'
           */
          FMS_B.Merge_h = 0.0F;

          /* End of Outputs for SubSystem: '<S79>/Brake Control' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S79>/Move Control' incorporates:
           *  ActionPort: '<S83>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* DeadZone: '<S87>/Dead Zone' incorporates:
           *  Inport: '<Root>/Pilot_Cmd'
           *  SignalConversion: '<S26>/Signal Copy2'
           */
          if (FMS_U.Pilot_Cmd.stick_roll > 0.05F) {
            rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - 0.05F;
          } else if (FMS_U.Pilot_Cmd.stick_roll >= -0.05F) {
            rtb_d_f = 0.0F;
          } else {
            rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - -0.05F;
          }

          /* End of DeadZone: '<S87>/Dead Zone' */
          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Gain: '<S87>/Gain' */
          FMS_B.Merge_h = 1.05263162F * rtb_d_f;

          /* End of Outputs for SubSystem: '<S79>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S79>/Switch Case' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  BusAssignment: '<S37>/Bus Assignment'
         *  Constant: '<S37>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S37>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_ba;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_k;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_o;
        FMS_Y.FMS_Out.u_cmd = rtb_Sum_i;

        /* Switch: '<S78>/Switch' incorporates:
         *  Constant: '<S78>/Constant'
         *  Constant: '<S78>/Constant1'
         */
        if (rtb_Sum_i >= 0.0F) {
          i = 1;
        } else {
          i = -1;
        }

        /* End of Switch: '<S78>/Switch' */

        /* Saturate: '<S77>/Saturation' */
        if (FMS_B.Merge_h > 1.0F) {
          rtb_d_f = 1.0F;
        } else if (FMS_B.Merge_h < -1.0F) {
          rtb_d_f = -1.0F;
        } else {
          rtb_d_f = FMS_B.Merge_h;
        }

        /* End of Saturate: '<S77>/Saturation' */

        /* BusAssignment: '<S37>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         *  Product: '<S37>/Multiply'
         */
        FMS_Y.FMS_Out.psi_rate_cmd = (real32_T)i * rtb_d_f;

        /* End of Outputs for SubSystem: '<S31>/Stabilize' */
        break;

       case 1:
        if (FMS_DW.SwitchCase_ActiveSubsystem_f != rtPrevAction) {
          /* SystemReset for IfAction SubSystem: '<S31>/Position' incorporates:
           *  ActionPort: '<S36>/Action Port'
           */
          /* SystemReset for SwitchCase: '<S31>/Switch Case' incorporates:
           *  Chart: '<S42>/Motion State'
           */
          FMS_DW.temporalCounter_i1_o = 0U;
          FMS_DW.is_active_c16_FMS = 0U;
          FMS_DW.is_c16_FMS = FMS_IN_NO_ACTIVE_CHILD;

          /* End of SystemReset for SubSystem: '<S31>/Position' */
        }

        /* Outputs for IfAction SubSystem: '<S31>/Position' incorporates:
         *  ActionPort: '<S36>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* Abs: '<S42>/Abs1' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S26>/Signal Copy2'
         */
        rtb_Sum_i = fabsf(FMS_U.Pilot_Cmd.stick_roll);

        /* RelationalOperator: '<S72>/Compare' incorporates:
         *  Abs: '<S42>/Abs2'
         *  Constant: '<S72>/Constant'
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S26>/Signal Copy2'
         */
        rtb_NOT_a = (fabsf(FMS_U.Pilot_Cmd.stick_throttle) <= 0.05F);

        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Logic: '<S42>/AND' incorporates:
         *  Abs: '<S42>/Abs1'
         *  Constant: '<S73>/Constant'
         *  RelationalOperator: '<S73>/Compare'
         */
        rtb_FixPtRelationalOperator = ((rtb_Sum_i <= 0.05F) && rtb_NOT_a);

        /* Logic: '<S42>/AND1' incorporates:
         *  Abs: '<S42>/Abs1'
         *  Constant: '<S71>/Constant'
         *  Logic: '<S42>/Logical Operator'
         *  Logic: '<S42>/NOT'
         *  RelationalOperator: '<S71>/Compare'
         */
        rtb_NOT_a = ((!rtb_FixPtRelationalOperator) && ((rtb_Sum_i > 0.05F) ||
          rtb_NOT_a));

        /* Chart: '<S42>/Motion State' */
        if (FMS_DW.temporalCounter_i1_o < 255U) {
          FMS_DW.temporalCounter_i1_o++;
        }

        if (FMS_DW.is_active_c16_FMS == 0U) {
          FMS_DW.is_active_c16_FMS = 1U;
          FMS_DW.is_c16_FMS = FMS_IN_Move_i;
          rtb_state_d = MotionState_Move;
        } else {
          switch (FMS_DW.is_c16_FMS) {
           case FMS_IN_Brake:
            rtb_state_d = MotionState_Brake;
            if (rtb_NOT_a) {
              FMS_DW.is_c16_FMS = FMS_IN_Move_i;
              rtb_state_d = MotionState_Move;
            } else if (FMS_DW.temporalCounter_i1_o >= 150U) {
              FMS_DW.is_c16_FMS = FMS_IN_Hold;
              rtb_state_d = MotionState_Hold;
            } else {
              if (rtb_FixPtRelationalOperator) {
                FMS_DW.is_c16_FMS = FMS_IN_Keep;
                rtb_state_d = MotionState_Keep;
              }
            }
            break;

           case FMS_IN_Hold:
            rtb_state_d = MotionState_Hold;
            if (rtb_NOT_a) {
              FMS_DW.is_c16_FMS = FMS_IN_Move_i;
              rtb_state_d = MotionState_Move;
            } else {
              if (rtb_FixPtRelationalOperator) {
                FMS_DW.is_c16_FMS = FMS_IN_Keep;
                rtb_state_d = MotionState_Keep;
              }
            }
            break;

           case FMS_IN_Keep:
            rtb_state_d = MotionState_Keep;
            if (!rtb_FixPtRelationalOperator) {
              FMS_DW.is_c16_FMS = FMS_IN_Move_i;
              rtb_state_d = MotionState_Move;
            }
            break;

           default:
            rtb_state_d = MotionState_Move;
            if ((!rtb_NOT_a) && (!rtb_FixPtRelationalOperator)) {
              FMS_DW.is_c16_FMS = FMS_IN_Brake;
              FMS_DW.temporalCounter_i1_o = 0U;
              rtb_state_d = MotionState_Brake;
            } else {
              if (rtb_FixPtRelationalOperator) {
                FMS_DW.is_c16_FMS = FMS_IN_Keep;
                rtb_state_d = MotionState_Keep;
              }
            }
            break;
          }
        }

        /* End of Chart: '<S42>/Motion State' */

        /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
        /* DeadZone: '<S75>/Dead Zone' incorporates:
         *  Inport: '<Root>/Pilot_Cmd'
         *  SignalConversion: '<S26>/Signal Copy2'
         */
        if (FMS_U.Pilot_Cmd.stick_throttle > 0.05F) {
          rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - 0.05F;
        } else if (FMS_U.Pilot_Cmd.stick_throttle >= -0.05F) {
          rtb_d_f = 0.0F;
        } else {
          rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - -0.05F;
        }

        /* End of DeadZone: '<S75>/Dead Zone' */
        /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

        /* Gain: '<S40>/Gain' incorporates:
         *  Gain: '<S75>/Gain'
         */
        rtb_Sum_i = 1.05263162F * rtb_d_f * FMS_PARAM.MAX_VEL;

        /* SwitchCase: '<S41>/Switch Case' incorporates:
         *  Inport: '<S46>/u_cmd'
         *  Math: '<S57>/Square'
         *  Product: '<S64>/Divide'
         *  Sum: '<S48>/Subtract'
         *  Switch: '<S47>/Switch'
         */
        rtPrevAction = FMS_DW.SwitchCase_ActiveSubsystem_d;
        FMS_DW.SwitchCase_ActiveSubsystem_d = -1;
        switch (rtb_state_d) {
         case MotionState_Keep:
          FMS_DW.SwitchCase_ActiveSubsystem_d = 0;
          break;

         case MotionState_Hold:
          FMS_DW.SwitchCase_ActiveSubsystem_d = 1;
          break;

         case MotionState_Brake:
          FMS_DW.SwitchCase_ActiveSubsystem_d = 2;
          break;

         case MotionState_Move:
          FMS_DW.SwitchCase_ActiveSubsystem_d = 3;
          break;
        }

        switch (FMS_DW.SwitchCase_ActiveSubsystem_d) {
         case 0:
          if (FMS_DW.SwitchCase_ActiveSubsystem_d != rtPrevAction) {
            /* InitializeConditions for IfAction SubSystem: '<S41>/Keep Control' incorporates:
             *  ActionPort: '<S45>/Action Port'
             */
            /* InitializeConditions for SwitchCase: '<S41>/Switch Case' incorporates:
             *  Delay: '<S66>/start_wp'
             *  Delay: '<S66>/start_wp1'
             */
            FMS_DW.icLoad_n = 1U;
            FMS_DW.icLoad_g = 1U;

            /* End of InitializeConditions for SubSystem: '<S41>/Keep Control' */
          }

          /* Outputs for IfAction SubSystem: '<S41>/Keep Control' incorporates:
           *  ActionPort: '<S45>/Action Port'
           */
          /* Delay: '<S66>/start_wp' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (FMS_DW.icLoad_n != 0) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            FMS_DW.start_wp_DSTATE_j[0] = FMS_U.INS_Out.x_R;
            FMS_DW.start_wp_DSTATE_j[1] = FMS_U.INS_Out.y_R;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Trigonometry: '<S69>/Trigonometric Function1' incorporates:
           *  Gain: '<S67>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_VectorConcatenate_f[0] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S69>/Trigonometric Function' incorporates:
           *  Gain: '<S67>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_VectorConcatenate_f[1] = arm_sin_f32(-FMS_U.INS_Out.psi);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* SignalConversion: '<S69>/ConcatBufferAtVector Concatenate1In3' incorporates:
           *  Constant: '<S69>/Constant3'
           */
          rtb_VectorConcatenate_f[2] = 0.0F;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Gain: '<S69>/Gain' incorporates:
           *  Gain: '<S67>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Trigonometry: '<S69>/Trigonometric Function2'
           */
          rtb_VectorConcatenate_f[3] = -arm_sin_f32(-FMS_U.INS_Out.psi);

          /* Trigonometry: '<S69>/Trigonometric Function3' incorporates:
           *  Gain: '<S67>/Gain'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_VectorConcatenate_f[4] = arm_cos_f32(-FMS_U.INS_Out.psi);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* SignalConversion: '<S69>/ConcatBufferAtVector Concatenate2In3' incorporates:
           *  Constant: '<S69>/Constant4'
           */
          rtb_VectorConcatenate_f[5] = 0.0F;

          /* SignalConversion: '<S69>/ConcatBufferAtVector ConcatenateIn3' */
          rtb_VectorConcatenate_f[6] = FMS_ConstB.VectorConcatenate3[0];
          rtb_VectorConcatenate_f[7] = FMS_ConstB.VectorConcatenate3[1];
          rtb_VectorConcatenate_f[8] = FMS_ConstB.VectorConcatenate3[2];

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* SignalConversion: '<S45>/TmpSignal ConversionAtMultiplyInport2' incorporates:
           *  Delay: '<S66>/start_wp'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Sum: '<S45>/Subtract'
           */
          rtb_Sum_i = FMS_DW.start_wp_DSTATE_j[0] - FMS_U.INS_Out.x_R;
          rtb_d_f = FMS_DW.start_wp_DSTATE_j[1] - FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Product: '<S45>/Multiply' incorporates:
           *  SignalConversion: '<S45>/TmpSignal ConversionAtMultiplyInport2'
           */
          for (i = 0; i < 3; i++) {
            rtb_Switch_k1[i] = rtb_VectorConcatenate_f[i + 3] * rtb_d_f +
              rtb_VectorConcatenate_f[i] * rtb_Sum_i;
          }

          /* End of Product: '<S45>/Multiply' */

          /* Delay: '<S66>/start_wp1' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (FMS_DW.icLoad_g != 0) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            FMS_DW.start_wp1_DSTATE = FMS_U.INS_Out.psi;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          }

          /* Switch: '<S45>/Switch' incorporates:
           *  Constant: '<S45>/Constant1'
           *  Gain: '<S45>/AY_P'
           *  Gain: '<S45>/AY_P1'
           *  Gain: '<S45>/VEL'
           *  Math: '<S45>/Square'
           *  Math: '<S45>/Square1'
           *  Product: '<S45>/Multiply1'
           *  Sqrt: '<S45>/Sqrt'
           *  Sum: '<S45>/Add'
           *  Trigonometry: '<S45>/Atan2'
           *  Trigonometry: '<S45>/Cos'
           *  Trigonometry: '<S45>/Sin'
           */
          if (sqrtf(rtb_Switch_k1[0] * rtb_Switch_k1[0] + rtb_Switch_k1[1] *
                    rtb_Switch_k1[1]) > 0.5F) {
            FMS_B.Merge[0] = FMS_PARAM.MAX_VEL * rtb_Switch_k1[0];

            /* Switch: '<S68>/Switch' incorporates:
             *  Constant: '<S68>/Constant'
             *  Constant: '<S68>/Constant1'
             *  Gain: '<S45>/VEL'
             */
            if (rtb_Switch_k1[0] >= 0.0F) {
              i = 1;
            } else {
              i = -1;
            }

            /* End of Switch: '<S68>/Switch' */
            FMS_B.Merge[1] = FMS_PARAM.AY_P * rtb_Switch_k1[1] * (real32_T)i;
          } else {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            /* Sum: '<S45>/Subtract1' incorporates:
             *  Delay: '<S66>/start_wp1'
             *  Inport: '<Root>/INS_Out'
             *  SignalConversion: '<S26>/Signal Copy1'
             */
            rtb_Sum_i = FMS_DW.start_wp1_DSTATE - FMS_U.INS_Out.psi;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
            FMS_B.Merge[0] = 0.0F;
            FMS_B.Merge[1] = FMS_PARAM.AY_P * atan2f(arm_sin_f32(rtb_Sum_i),
              arm_cos_f32(rtb_Sum_i));
          }

          /* End of Switch: '<S45>/Switch' */

          /* Update for Delay: '<S66>/start_wp' */
          FMS_DW.icLoad_n = 0U;

          /* Update for Delay: '<S66>/start_wp1' */
          FMS_DW.icLoad_g = 0U;

          /* End of Outputs for SubSystem: '<S41>/Keep Control' */
          break;

         case 1:
          if (FMS_DW.SwitchCase_ActiveSubsystem_d != rtPrevAction) {
            /* InitializeConditions for IfAction SubSystem: '<S41>/Hold Control' incorporates:
             *  ActionPort: '<S44>/Action Port'
             */
            /* InitializeConditions for SwitchCase: '<S41>/Switch Case' incorporates:
             *  Delay: '<S49>/start_vel'
             *  Delay: '<S49>/start_wp'
             *  UnitDelay: '<S63>/Delay Input1'
             *
             * Block description for '<S63>/Delay Input1':
             *
             *  Store in Global RAM
             */
            FMS_DW.DelayInput1_DSTATE = 0.0F;
            FMS_DW.icLoad = 1U;
            FMS_DW.icLoad_p = 1U;

            /* End of InitializeConditions for SubSystem: '<S41>/Hold Control' */
          }

          /* Outputs for IfAction SubSystem: '<S41>/Hold Control' incorporates:
           *  ActionPort: '<S44>/Action Port'
           */
          /* Switch: '<S50>/Switch' incorporates:
           *  Constant: '<S50>/Constant'
           *  Constant: '<S50>/Constant1'
           *  Inport: '<S44>/u_cmd'
           */
          if (rtb_Sum_i >= 0.0F) {
            rtb_Switch_mc_idx_0 = 1.0F;
          } else {
            rtb_Switch_mc_idx_0 = -1.0F;
          }

          /* End of Switch: '<S50>/Switch' */

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Product: '<S44>/Multiply' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_P_c[0] = rtb_Switch_mc_idx_0 * FMS_U.INS_Out.vn;
          rtb_P_c[1] = rtb_Switch_mc_idx_0 * FMS_U.INS_Out.ve;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Math: '<S58>/Math Function' incorporates:
           *  Math: '<S56>/Square'
           */
          rtb_d_f = rtb_P_c[0] * rtb_P_c[0];

          /* Sum: '<S58>/Sum of Elements' incorporates:
           *  Math: '<S58>/Math Function'
           */
          rtb_Switch_mc_idx_0 = rtb_d_f + rtb_P_c[1] * rtb_P_c[1];

          /* Math: '<S58>/Math Function1' incorporates:
           *  Sum: '<S58>/Sum of Elements'
           *
           * About '<S58>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_Switch_mc_idx_0 < 0.0F) {
            rtb_Switch_mc_idx_0 = -sqrtf(fabsf(rtb_Switch_mc_idx_0));
          } else {
            rtb_Switch_mc_idx_0 = sqrtf(rtb_Switch_mc_idx_0);
          }

          /* End of Math: '<S58>/Math Function1' */

          /* Switch: '<S58>/Switch' incorporates:
           *  Constant: '<S58>/Constant'
           *  Product: '<S58>/Product'
           */
          if (rtb_Switch_mc_idx_0 > 0.0F) {
            rtb_Switch_k1[0] = rtb_P_c[0];
            rtb_Switch_k1[1] = rtb_P_c[1];
            rtb_Switch_k1[2] = rtb_Switch_mc_idx_0;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S58>/Switch' */

          /* Switch: '<S65>/Switch' incorporates:
           *  Constant: '<S65>/Constant'
           *  Constant: '<S65>/Constant1'
           *  Inport: '<S44>/u_cmd'
           */
          if (rtb_Sum_i >= 0.0F) {
            i = 1;
          } else {
            i = -1;
          }

          /* End of Switch: '<S65>/Switch' */

          /* RelationalOperator: '<S63>/FixPt Relational Operator' incorporates:
           *  UnitDelay: '<S63>/Delay Input1'
           *
           * Block description for '<S63>/Delay Input1':
           *
           *  Store in Global RAM
           */
          rtb_FixPtRelationalOperator = (i != FMS_DW.DelayInput1_DSTATE);

          /* Delay: '<S49>/start_vel' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  Product: '<S49>/Multiply'
           *  SignalConversion: '<S26>/Signal Copy1'
           *  Trigonometry: '<S49>/Cos'
           *  Trigonometry: '<S49>/Cos1'
           */
          if (rtb_FixPtRelationalOperator && (FMS_PrevZCX.start_vel_Reset_ZCE !=
               POS_ZCSIG)) {
            FMS_DW.icLoad = 1U;
          }

          FMS_PrevZCX.start_vel_Reset_ZCE = rtb_FixPtRelationalOperator;
          if (FMS_DW.icLoad != 0) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            FMS_DW.start_vel_DSTATE[0] = arm_cos_f32(FMS_U.INS_Out.psi) *
              (real32_T)i;
            FMS_DW.start_vel_DSTATE[1] = arm_sin_f32(FMS_U.INS_Out.psi) *
              (real32_T)i;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          }

          /* Sum: '<S64>/Sum of Elements' incorporates:
           *  Delay: '<S49>/start_vel'
           *  Math: '<S64>/Math Function'
           */
          rtb_Switch_mc_idx_0 = FMS_DW.start_vel_DSTATE[0] *
            FMS_DW.start_vel_DSTATE[0] + FMS_DW.start_vel_DSTATE[1] *
            FMS_DW.start_vel_DSTATE[1];

          /* Math: '<S64>/Math Function1' incorporates:
           *  Sum: '<S64>/Sum of Elements'
           *
           * About '<S64>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_Switch_mc_idx_0 < 0.0F) {
            rtb_sin_alpha_j = -sqrtf(fabsf(rtb_Switch_mc_idx_0));
          } else {
            rtb_sin_alpha_j = sqrtf(rtb_Switch_mc_idx_0);
          }

          /* End of Math: '<S64>/Math Function1' */

          /* Switch: '<S64>/Switch' incorporates:
           *  Constant: '<S64>/Constant'
           *  Delay: '<S49>/start_vel'
           *  Product: '<S64>/Product'
           */
          if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_i[0] = FMS_DW.start_vel_DSTATE[0];
            rtb_Switch_i[1] = FMS_DW.start_vel_DSTATE[1];
            rtb_Switch_i[2] = rtb_sin_alpha_j;
          } else {
            rtb_Switch_i[0] = 0.0F;
            rtb_Switch_i[1] = 0.0F;
            rtb_Switch_i[2] = 1.0F;
          }

          /* End of Switch: '<S64>/Switch' */

          /* Product: '<S58>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_k1[0] / rtb_Switch_k1
            [2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_k1[1] / rtb_Switch_k1
            [2];

          /* Sum: '<S61>/Sum of Elements' incorporates:
           *  Math: '<S61>/Math Function'
           *  SignalConversion: '<S61>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_Switch_mc_idx_0 = rtb_TmpSignalConversionAtMat_co[1] *
            rtb_TmpSignalConversionAtMat_co[1] +
            rtb_TmpSignalConversionAtMat_co[0] *
            rtb_TmpSignalConversionAtMat_co[0];

          /* Math: '<S61>/Math Function1' incorporates:
           *  Sum: '<S61>/Sum of Elements'
           *
           * About '<S61>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_Switch_mc_idx_0 < 0.0F) {
            rtb_sin_alpha_j = -sqrtf(fabsf(rtb_Switch_mc_idx_0));
          } else {
            rtb_sin_alpha_j = sqrtf(rtb_Switch_mc_idx_0);
          }

          /* End of Math: '<S61>/Math Function1' */

          /* Switch: '<S61>/Switch' incorporates:
           *  Constant: '<S61>/Constant'
           *  Product: '<S61>/Product'
           */
          if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_k1[0] = rtb_TmpSignalConversionAtMat_co[1];
            rtb_Switch_k1[1] = rtb_TmpSignalConversionAtMat_co[0];
            rtb_Switch_k1[2] = rtb_sin_alpha_j;
          } else {
            rtb_Switch_k1[0] = 0.0F;
            rtb_Switch_k1[1] = 0.0F;
            rtb_Switch_k1[2] = 1.0F;
          }

          /* End of Switch: '<S61>/Switch' */

          /* Delay: '<S49>/start_wp' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          if (rtb_FixPtRelationalOperator && (FMS_PrevZCX.start_wp_Reset_ZCE !=
               POS_ZCSIG)) {
            FMS_DW.icLoad_p = 1U;
          }

          FMS_PrevZCX.start_wp_Reset_ZCE = rtb_FixPtRelationalOperator;
          if (FMS_DW.icLoad_p != 0) {
            /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
            FMS_DW.start_wp_DSTATE[0] = FMS_U.INS_Out.x_R;
            FMS_DW.start_wp_DSTATE[1] = FMS_U.INS_Out.y_R;

            /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* MATLAB Function: '<S47>/SearchL1RefWP' incorporates:
           *  Delay: '<S49>/start_wp'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_TmpSignalConversionAtMat_co[0] = FMS_DW.start_wp_DSTATE[0] -
            FMS_U.INS_Out.x_R;
          rtb_TmpSignalConversionAtMat_co[1] = FMS_DW.start_wp_DSTATE[1] -
            FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Product: '<S64>/Divide' */
          rtb_Switch_mc_idx_0 = rtb_Switch_i[0] / rtb_Switch_i[2];

          /* MATLAB Function: '<S47>/SearchL1RefWP' */
          rtb_sin_alpha_j = rtb_Switch_mc_idx_0 *
            rtb_TmpSignalConversionAtMat_co[0];
          rtb_Switch_mc_idx_1 = rtb_Switch_mc_idx_0;

          /* Product: '<S64>/Divide' */
          rtb_Switch_mc_idx_0 = rtb_Switch_i[1] / rtb_Switch_i[2];

          /* MATLAB Function: '<S47>/SearchL1RefWP' incorporates:
           *  Constant: '<S44>/L1'
           *  Delay: '<S49>/start_wp'
           */
          rtb_sin_alpha_j += rtb_Switch_mc_idx_0 *
            rtb_TmpSignalConversionAtMat_co[1];
          rtb_u_g = 2.0F * rtb_sin_alpha_j;
          rtb_MathFunction_i2[0] = 0.0F;
          rtb_MathFunction_i2[1] = 0.0F;
          rtb_Sign5_d = rtb_u_g * rtb_u_g - ((rtb_TmpSignalConversionAtMat_co[0]
            * rtb_TmpSignalConversionAtMat_co[0] +
            rtb_TmpSignalConversionAtMat_co[1] *
            rtb_TmpSignalConversionAtMat_co[1]) - FMS_PARAM.L1 * FMS_PARAM.L1) *
            4.0F;
          rtb_sin_alpha_j = -1.0F;
          if (rtb_Sign5_d > 0.0F) {
            rtb_sin_alpha_j = sqrtf(rtb_Sign5_d);
            rtb_sin_alpha_j = fmaxf((-rtb_u_g + rtb_sin_alpha_j) / 2.0F,
              (-rtb_u_g - rtb_sin_alpha_j) / 2.0F);
            rtb_MathFunction_i2[0] = rtb_sin_alpha_j * rtb_Switch_mc_idx_1 +
              FMS_DW.start_wp_DSTATE[0];
            rtb_MathFunction_i2[1] = rtb_sin_alpha_j * rtb_Switch_mc_idx_0 +
              FMS_DW.start_wp_DSTATE[1];
          } else {
            if (rtb_Sign5_d == 0.0F) {
              rtb_sin_alpha_j = -rtb_u_g / 2.0F;
              rtb_MathFunction_i2[0] = rtb_sin_alpha_j * rtb_Switch_mc_idx_1 +
                FMS_DW.start_wp_DSTATE[0];
              rtb_MathFunction_i2[1] = rtb_sin_alpha_j * rtb_Switch_mc_idx_0 +
                FMS_DW.start_wp_DSTATE[1];
            }
          }

          /* RelationalOperator: '<S51>/Compare' incorporates:
           *  Constant: '<S51>/Constant'
           *  MATLAB Function: '<S47>/SearchL1RefWP'
           */
          rtb_FixPtRelationalOperator = (rtb_sin_alpha_j > 0.0F);

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* MATLAB Function: '<S47>/OutRegionRefWP' incorporates:
           *  Delay: '<S49>/start_wp'
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_sin_alpha_j = (FMS_U.INS_Out.x_R - FMS_DW.start_wp_DSTATE[0]) *
            rtb_Switch_mc_idx_1 + (FMS_U.INS_Out.y_R - FMS_DW.start_wp_DSTATE[1])
            * rtb_Switch_mc_idx_0;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          rtb_u_g = 1.29246971E-26F;

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_Sign5_d = fabsf((rtb_sin_alpha_j * rtb_Switch_mc_idx_1 +
                               FMS_DW.start_wp_DSTATE[0]) - FMS_U.INS_Out.x_R);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          if (rtb_Sign5_d > 1.29246971E-26F) {
            rtb_Rem_kb = 1.0F;
            rtb_u_g = rtb_Sign5_d;
          } else {
            rtb_Subtract1_j = rtb_Sign5_d / 1.29246971E-26F;
            rtb_Rem_kb = rtb_Subtract1_j * rtb_Subtract1_j;
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          rtb_Sign5_d = fabsf((rtb_sin_alpha_j * rtb_Switch_mc_idx_0 +
                               FMS_DW.start_wp_DSTATE[1]) - FMS_U.INS_Out.y_R);

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */
          if (rtb_Sign5_d > rtb_u_g) {
            rtb_Subtract1_j = rtb_u_g / rtb_Sign5_d;
            rtb_Rem_kb = rtb_Rem_kb * rtb_Subtract1_j * rtb_Subtract1_j + 1.0F;
            rtb_u_g = rtb_Sign5_d;
          } else {
            rtb_Subtract1_j = rtb_Sign5_d / rtb_u_g;
            rtb_Rem_kb += rtb_Subtract1_j * rtb_Subtract1_j;
          }

          rtb_Rem_kb = rtb_u_g * sqrtf(rtb_Rem_kb);
          rtb_sin_alpha_j += rtb_Rem_kb * 0.577350259F;

          /* Switch: '<S47>/Switch' incorporates:
           *  Delay: '<S49>/start_wp'
           *  MATLAB Function: '<S47>/OutRegionRefWP'
           */
          if (rtb_FixPtRelationalOperator) {
            rtb_Switch_mc_idx_1 = rtb_MathFunction_i2[0];
          } else {
            rtb_Switch_mc_idx_1 = rtb_sin_alpha_j * rtb_Switch_mc_idx_1 +
              FMS_DW.start_wp_DSTATE[0];
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S48>/Subtract' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_Switch_mc_idx_1 -= FMS_U.INS_Out.x_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Math: '<S59>/Math Function' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_mc_idx_1 *
            rtb_Switch_mc_idx_1;
          rtb_MathFunction_i2[0] = rtb_Switch_mc_idx_1;

          /* Switch: '<S47>/Switch' incorporates:
           *  Delay: '<S49>/start_wp'
           *  MATLAB Function: '<S47>/OutRegionRefWP'
           *  Sum: '<S48>/Subtract'
           */
          if (rtb_FixPtRelationalOperator) {
            rtb_Switch_mc_idx_1 = rtb_MathFunction_i2[1];
          } else {
            rtb_Switch_mc_idx_1 = rtb_sin_alpha_j * rtb_Switch_mc_idx_0 +
              FMS_DW.start_wp_DSTATE[1];
          }

          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* Sum: '<S48>/Subtract' incorporates:
           *  Inport: '<Root>/INS_Out'
           *  SignalConversion: '<S26>/Signal Copy1'
           */
          rtb_Switch_mc_idx_1 -= FMS_U.INS_Out.y_R;

          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Math: '<S59>/Math Function' incorporates:
           *  Math: '<S57>/Square'
           */
          rtb_Switch_mc_idx_0 = rtb_Switch_mc_idx_1 * rtb_Switch_mc_idx_1;

          /* Sum: '<S59>/Sum of Elements' incorporates:
           *  Math: '<S59>/Math Function'
           */
          rtb_sin_alpha_j = rtb_Switch_mc_idx_0 +
            rtb_TmpSignalConversionAtMat_co[0];

          /* Math: '<S59>/Math Function1' incorporates:
           *  Sum: '<S59>/Sum of Elements'
           *
           * About '<S59>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_sin_alpha_j = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_sin_alpha_j = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S59>/Math Function1' */

          /* Switch: '<S59>/Switch' incorporates:
           *  Constant: '<S59>/Constant'
           *  Product: '<S59>/Product'
           */
          if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_i[0] = rtb_MathFunction_i2[0];
            rtb_Switch_i[1] = rtb_Switch_mc_idx_1;
            rtb_Switch_i[2] = rtb_sin_alpha_j;
          } else {
            rtb_Switch_i[0] = 0.0F;
            rtb_Switch_i[1] = 0.0F;
            rtb_Switch_i[2] = 1.0F;
          }

          /* End of Switch: '<S59>/Switch' */

          /* Product: '<S59>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_i[0] / rtb_Switch_i[2];
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_i[1] / rtb_Switch_i[2];

          /* Sum: '<S62>/Sum of Elements' incorporates:
           *  Math: '<S62>/Math Function'
           *  SignalConversion: '<S62>/TmpSignal ConversionAtMath FunctionInport1'
           */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[1] *
            rtb_TmpSignalConversionAtMat_co[1] +
            rtb_TmpSignalConversionAtMat_co[0] *
            rtb_TmpSignalConversionAtMat_co[0];

          /* Math: '<S62>/Math Function1' incorporates:
           *  Sum: '<S62>/Sum of Elements'
           *
           * About '<S62>/Math Function1':
           *  Operator: sqrt
           */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_sin_alpha_j = -sqrtf(fabsf(rtb_sin_alpha_j));
          } else {
            rtb_sin_alpha_j = sqrtf(rtb_sin_alpha_j);
          }

          /* End of Math: '<S62>/Math Function1' */

          /* Switch: '<S62>/Switch' incorporates:
           *  Constant: '<S62>/Constant'
           *  Product: '<S62>/Product'
           */
          if (rtb_sin_alpha_j > 0.0F) {
            rtb_Switch_i[0] = rtb_TmpSignalConversionAtMat_co[1];
            rtb_Switch_i[1] = rtb_TmpSignalConversionAtMat_co[0];
            rtb_Switch_i[2] = rtb_sin_alpha_j;
          } else {
            rtb_Switch_i[0] = 0.0F;
            rtb_Switch_i[1] = 0.0F;
            rtb_Switch_i[2] = 1.0F;
          }

          /* End of Switch: '<S62>/Switch' */

          /* Product: '<S62>/Divide' */
          rtb_TmpSignalConversionAtMat_co[0] = rtb_Switch_i[0] / rtb_Switch_i[2];

          /* Product: '<S61>/Divide' */
          rtb_P_on[0] = rtb_Switch_k1[0] / rtb_Switch_k1[2];
          rtb_MathFunction_i2[0] *= rtb_MathFunction_i2[0];

          /* Product: '<S62>/Divide' incorporates:
           *  Math: '<S57>/Square'
           */
          rtb_TmpSignalConversionAtMat_co[1] = rtb_Switch_i[1] / rtb_Switch_i[2];

          /* Product: '<S61>/Divide' */
          rtb_P_on[1] = rtb_Switch_k1[1] / rtb_Switch_k1[2];

          /* Sqrt: '<S56>/Sqrt' incorporates:
           *  Math: '<S56>/Square'
           *  Sum: '<S56>/Sum of Elements'
           */
          rtb_sin_alpha_j = sqrtf(rtb_P_c[1] * rtb_P_c[1] + rtb_d_f);

          /* Gain: '<S54>/Gain' incorporates:
           *  Math: '<S54>/Square'
           */
          rtb_d_f = rtb_sin_alpha_j * rtb_sin_alpha_j * 2.0F;

          /* Sum: '<S60>/Subtract' incorporates:
           *  Product: '<S60>/Multiply'
           *  Product: '<S60>/Multiply1'
           */
          rtb_sin_alpha_j = rtb_TmpSignalConversionAtMat_co[0] * rtb_P_on[1] -
            rtb_TmpSignalConversionAtMat_co[1] * rtb_P_on[0];

          /* Signum: '<S55>/Sign1' */
          if (rtb_sin_alpha_j < 0.0F) {
            rtb_sin_alpha_j = -1.0F;
          } else {
            if (rtb_sin_alpha_j > 0.0F) {
              rtb_sin_alpha_j = 1.0F;
            }
          }

          /* End of Signum: '<S55>/Sign1' */

          /* Switch: '<S55>/Switch2' incorporates:
           *  Constant: '<S55>/Constant4'
           */
          if (rtb_sin_alpha_j == 0.0F) {
            rtb_sin_alpha_j = 1.0F;
          }

          /* End of Switch: '<S55>/Switch2' */

          /* DotProduct: '<S55>/Dot Product' */
          rtb_Switch_mc_idx_1 = rtb_P_on[0] * rtb_TmpSignalConversionAtMat_co[0]
            + rtb_P_on[1] * rtb_TmpSignalConversionAtMat_co[1];

          /* Trigonometry: '<S55>/Acos' incorporates:
           *  DotProduct: '<S55>/Dot Product'
           */
          if (rtb_Switch_mc_idx_1 > 1.0F) {
            rtb_Switch_mc_idx_1 = 1.0F;
          } else {
            if (rtb_Switch_mc_idx_1 < -1.0F) {
              rtb_Switch_mc_idx_1 = -1.0F;
            }
          }

          /* Product: '<S55>/Multiply' incorporates:
           *  Trigonometry: '<S55>/Acos'
           */
          rtb_sin_alpha_j *= acosf(rtb_Switch_mc_idx_1);

          /* Saturate: '<S54>/Saturation' */
          if (rtb_sin_alpha_j > 1.57079637F) {
            rtb_sin_alpha_j = 1.57079637F;
          } else {
            if (rtb_sin_alpha_j < -1.57079637F) {
              rtb_sin_alpha_j = -1.57079637F;
            }
          }

          /* End of Saturate: '<S54>/Saturation' */

          /* SignalConversion: '<S44>/OutportBufferForu_psi_cmd' incorporates:
           *  Constant: '<S44>/L1'
           *  Constant: '<S54>/Constant'
           *  Gain: '<S44>/AY_P'
           *  Inport: '<S44>/u_cmd'
           *  MinMax: '<S54>/Max'
           *  MinMax: '<S54>/Min'
           *  Product: '<S54>/Divide'
           *  Product: '<S54>/Multiply1'
           *  Sqrt: '<S57>/Sqrt'
           *  Sum: '<S57>/Sum of Elements'
           *  Trigonometry: '<S54>/Sin'
           */
          FMS_B.Merge[0] = rtb_Sum_i;
          FMS_B.Merge[1] = arm_sin_f32(rtb_sin_alpha_j) * rtb_d_f / fminf
            (FMS_PARAM.L1, fmaxf(sqrtf(rtb_Switch_mc_idx_0 +
               rtb_MathFunction_i2[0]), 0.5F)) * FMS_PARAM.AY_P;

          /* Update for UnitDelay: '<S63>/Delay Input1'
           *
           * Block description for '<S63>/Delay Input1':
           *
           *  Store in Global RAM
           */
          FMS_DW.DelayInput1_DSTATE = (real32_T)i;

          /* Update for Delay: '<S49>/start_vel' */
          FMS_DW.icLoad = 0U;

          /* Update for Delay: '<S49>/start_wp' */
          FMS_DW.icLoad_p = 0U;

          /* End of Outputs for SubSystem: '<S41>/Hold Control' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S41>/Brake Control' incorporates:
           *  ActionPort: '<S43>/Action Port'
           */
          /* SignalConversion: '<S43>/OutportBufferForu_psi_cmd' incorporates:
           *  Constant: '<S43>/Constant'
           */
          FMS_B.Merge[0] = 0.0F;
          FMS_B.Merge[1] = 0.0F;

          /* End of Outputs for SubSystem: '<S41>/Brake Control' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S41>/Move Control' incorporates:
           *  ActionPort: '<S46>/Action Port'
           */
          /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
          /* DeadZone: '<S70>/Dead Zone' incorporates:
           *  Inport: '<Root>/Pilot_Cmd'
           *  SignalConversion: '<S26>/Signal Copy2'
           */
          if (FMS_U.Pilot_Cmd.stick_roll > 0.05F) {
            rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - 0.05F;
          } else if (FMS_U.Pilot_Cmd.stick_roll >= -0.05F) {
            rtb_d_f = 0.0F;
          } else {
            rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - -0.05F;
          }

          /* End of DeadZone: '<S70>/Dead Zone' */
          /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

          /* Gain: '<S70>/Gain' */
          FMS_B.Merge[1] = 1.05263162F * rtb_d_f;
          FMS_B.Merge[0] = rtb_Sum_i;

          /* End of Outputs for SubSystem: '<S41>/Move Control' */
          break;
        }

        /* End of SwitchCase: '<S41>/Switch Case' */

        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  BusAssignment: '<S36>/Bus Assignment'
         *  Constant: '<S36>/Constant'
         */
        FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

        /* BusAssignment: '<S36>/Bus Assignment' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         *  Outport: '<Root>/FMS_Out'
         */
        FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion_b;
        FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1_o;
        FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2_m;
        FMS_Y.FMS_Out.u_cmd = FMS_B.Merge[0];

        /* Saturate: '<S39>/Saturation' */
        if (FMS_B.Merge[1] > 1.0F) {
          /* BusAssignment: '<S36>/Bus Assignment' incorporates:
           *  BusAssignment: '<S28>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = 1.0F;
        } else if (FMS_B.Merge[1] < -1.0F) {
          /* BusAssignment: '<S36>/Bus Assignment' incorporates:
           *  BusAssignment: '<S28>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = -1.0F;
        } else {
          /* BusAssignment: '<S36>/Bus Assignment' incorporates:
           *  BusAssignment: '<S28>/Bus Assignment'
           *  Outport: '<Root>/FMS_Out'
           */
          FMS_Y.FMS_Out.psi_rate_cmd = FMS_B.Merge[1];
        }

        /* End of Saturate: '<S39>/Saturation' */
        /* End of Outputs for SubSystem: '<S31>/Position' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S31>/Unknown' incorporates:
         *  ActionPort: '<S38>/Action Port'
         */
        /* Outport: '<Root>/FMS_Out' incorporates:
         *  BusAssignment: '<S28>/Bus Assignment'
         */
        FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown_i);

        /* End of Outputs for SubSystem: '<S31>/Unknown' */
        break;
      }

      /* End of SwitchCase: '<S31>/Switch Case' */
      /* End of Outputs for SubSystem: '<S27>/Assist' */
      break;

     case 3:
      /* Outputs for IfAction SubSystem: '<S27>/Manual' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
      /* Outport: '<Root>/FMS_Out' incorporates:
       *  BusAssignment: '<S28>/Bus Assignment'
       *  BusAssignment: '<S33>/Bus Assignment'
       *  Constant: '<S33>/Constant'
       */
      FMS_Y.FMS_Out = FMS_rtZFMS_Out_Bus;

      /* BusAssignment: '<S33>/Bus Assignment' incorporates:
       *  BusAssignment: '<S28>/Bus Assignment'
       *  Constant: '<S33>/Constant2'
       *  Outport: '<Root>/FMS_Out'
       */
      FMS_Y.FMS_Out.reset = 1U;
      FMS_Y.FMS_Out.status = FMS_ConstB.DataTypeConversion;
      FMS_Y.FMS_Out.state = FMS_ConstB.DataTypeConversion1;
      FMS_Y.FMS_Out.ctrl_mode = FMS_ConstB.DataTypeConversion2;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      /* DeadZone: '<S151>/Dead Zone' incorporates:
       *  Inport: '<Root>/Pilot_Cmd'
       *  SignalConversion: '<S26>/Signal Copy2'
       */
      if (FMS_U.Pilot_Cmd.stick_throttle > 0.05F) {
        rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - 0.05F;
      } else if (FMS_U.Pilot_Cmd.stick_throttle >= -0.05F) {
        rtb_d_f = 0.0F;
      } else {
        rtb_d_f = FMS_U.Pilot_Cmd.stick_throttle - -0.05F;
      }

      /* End of DeadZone: '<S151>/Dead Zone' */
      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

      /* BusAssignment: '<S33>/Bus Assignment' incorporates:
       *  BusAssignment: '<S28>/Bus Assignment'
       *  Gain: '<S151>/Gain'
       *  Outport: '<Root>/FMS_Out'
       */
      FMS_Y.FMS_Out.u_cmd = 1.05263162F * rtb_d_f;

      /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
      /* DeadZone: '<S152>/Dead Zone' incorporates:
       *  Inport: '<Root>/Pilot_Cmd'
       *  SignalConversion: '<S26>/Signal Copy2'
       */
      if (FMS_U.Pilot_Cmd.stick_roll > 0.05F) {
        rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - 0.05F;
      } else if (FMS_U.Pilot_Cmd.stick_roll >= -0.05F) {
        rtb_d_f = 0.0F;
      } else {
        rtb_d_f = FMS_U.Pilot_Cmd.stick_roll - -0.05F;
      }

      /* End of DeadZone: '<S152>/Dead Zone' */
      /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

      /* BusAssignment: '<S33>/Bus Assignment' incorporates:
       *  BusAssignment: '<S28>/Bus Assignment'
       *  Gain: '<S152>/Gain'
       *  Outport: '<Root>/FMS_Out'
       */
      FMS_Y.FMS_Out.psi_rate_cmd = 1.05263162F * rtb_d_f;

      /* End of Outputs for SubSystem: '<S27>/Manual' */
      break;

     case 4:
      /* Outputs for IfAction SubSystem: '<S27>/Unknown' incorporates:
       *  ActionPort: '<S35>/Action Port'
       */
      /* Outport: '<Root>/FMS_Out' incorporates:
       *  BusAssignment: '<S28>/Bus Assignment'
       */
      FMS_Unknown(&FMS_Y.FMS_Out, &FMS_ConstB.Unknown);

      /* End of Outputs for SubSystem: '<S27>/Unknown' */
      break;
    }

    /* End of SwitchCase: '<S27>/Switch Case' */
    /* End of Outputs for SubSystem: '<S25>/Arm' */
    break;
  }

  /* End of SwitchCase: '<S25>/Switch Case' */

  /* BusAssignment: '<S28>/Bus Assignment' incorporates:
   *  Constant: '<S28>/Constant'
   *  DataTypeConversion: '<S28>/Data Type Conversion'
   *  DiscreteIntegrator: '<S193>/Discrete-Time Integrator'
   *  Outport: '<Root>/FMS_Out'
   *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy3Inport1'
   *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy4Inport1'
   *  SignalConversion: '<S26>/TmpSignal ConversionAtSignal Copy5Inport1'
   *  Sum: '<S28>/Sum'
   */
  FMS_Y.FMS_Out.timestamp = FMS_DW.DiscreteTimeIntegrator_DSTATE_g;
  FMS_Y.FMS_Out.mode = (uint8_T)FMS_B.target_mode;

  /* Outputs for Atomic SubSystem: '<S4>/FMS_Input' */
  FMS_Y.FMS_Out.wp_consume = FMS_B.wp_consume;
  FMS_Y.FMS_Out.wp_current = (uint8_T)(FMS_B.wp_index - 1);
  FMS_Y.FMS_Out.local_psi = FMS_B.Cmd_In.local_psi;

  /* End of Outputs for SubSystem: '<S4>/FMS_Input' */

  /* Update for DiscreteIntegrator: '<S193>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S193>/Constant'
   */
  FMS_DW.DiscreteTimeIntegrator_DSTATE_g += FMS_EXPORT.period;

  /* End of Outputs for SubSystem: '<Root>/FMS Commander' */

  /* Update for UnitDelay: '<S21>/Delay Input1' incorporates:
   *  Inport: '<Root>/GCS_Cmd'
   *
   * Block description for '<S21>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_j = FMS_U.GCS_Cmd.timestamp;

  /* Update for UnitDelay: '<S22>/Delay Input1' incorporates:
   *  Inport: '<Root>/Pilot_Cmd'
   *
   * Block description for '<S22>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_d = FMS_U.Pilot_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S3>/Constant'
   */
  FMS_DW.DiscreteTimeIntegrator_DSTATE += 0.01F;
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
  FMS_DW.DiscreteTimeIntegrator1_DSTATE += 0.01F;
  if (FMS_DW.DiscreteTimeIntegrator1_DSTATE >= 65535.0F) {
    FMS_DW.DiscreteTimeIntegrator1_DSTATE = 65535.0F;
  } else {
    if (FMS_DW.DiscreteTimeIntegrator1_DSTATE <= 0.0F) {
      FMS_DW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S8>/Delay Input1' incorporates:
   *  Inport: '<Root>/Auto_Cmd'
   *
   * Block description for '<S8>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE_a = FMS_U.Auto_Cmd.timestamp;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S1>/Constant'
   */
  rtb_d_f = (real32_T)FMS_DW.DiscreteTimeIntegrator_DSTATE_j + (real32_T)
    FMS_EXPORT.period;
  if (rtb_d_f < 4.2949673E+9F) {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_j = (uint32_T)rtb_d_f;
  } else {
    FMS_DW.DiscreteTimeIntegrator_DSTATE_j = MAX_uint32_T;
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
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
  /* Start for SwitchCase: '<S25>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S25>/Arm' */
  /* Start for SwitchCase: '<S27>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_b = -1;

  /* Start for IfAction SubSystem: '<S27>/SubMode' */
  /* Start for SwitchCase: '<S34>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_a = -1;

  /* End of Start for SubSystem: '<S27>/SubMode' */

  /* Start for IfAction SubSystem: '<S27>/Auto' */
  /* Start for SwitchCase: '<S32>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_i = -1;

  /* End of Start for SubSystem: '<S27>/Auto' */

  /* Start for IfAction SubSystem: '<S27>/Assist' */
  /* Start for SwitchCase: '<S31>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_f = -1;

  /* Start for IfAction SubSystem: '<S31>/Stabilize' */
  /* Start for SwitchCase: '<S79>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_n = -1;

  /* End of Start for SubSystem: '<S31>/Stabilize' */

  /* Start for IfAction SubSystem: '<S31>/Position' */
  /* Start for SwitchCase: '<S41>/Switch Case' */
  FMS_DW.SwitchCase_ActiveSubsystem_d = -1;

  /* End of Start for SubSystem: '<S31>/Position' */
  /* End of Start for SubSystem: '<S27>/Assist' */
  /* End of Start for SubSystem: '<S25>/Arm' */
  /* End of Start for SubSystem: '<Root>/FMS Commander' */
  FMS_PrevZCX.start_vel_Reset_ZCE = POS_ZCSIG;
  FMS_PrevZCX.start_wp_Reset_ZCE = POS_ZCSIG;

  /* SystemInitialize for Chart: '<Root>/SafeMode' */
  FMS_DW.is_active_c3_FMS = 0U;
  FMS_DW.is_c3_FMS = FMS_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for Chart: '<Root>/FMS State Machine' */
  initialize_msg_local_queues_for();
  FMS_DW.sfEvent = -1;
  FMS_DW.is_active_Combo_Stick = 0U;
  FMS_DW.is_Combo_Stick = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_active_Command_Listener = 0U;
  FMS_DW.is_Command_Listener = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_active_Lost_Return = 0U;
  FMS_DW.is_Lost_Return = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_active_Vehicle = 0U;
  FMS_DW.is_Vehicle = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Arm = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Assist = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Auto = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Mission = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Offboard = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_Manual = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.is_SubMode = FMS_IN_NO_ACTIVE_CHILD;
  FMS_DW.temporalCounter_i1 = 0U;
  FMS_DW.is_active_c11_FMS = 0U;
  FMS_DW.M_msgReservedData = FMS_Cmd_None;
  FMS_DW.bl = false;
  FMS_DW.br = false;
  FMS_DW.prep_mission = 0.0;
  FMS_B.wp_consume = 0U;
  FMS_B.wp_index = 1U;
  FMS_DW.chartAbsoluteTimeCounter = 0;

  /* SystemInitialize for Atomic SubSystem: '<Root>/FMS Commander' */
  /* SystemInitialize for IfAction SubSystem: '<S25>/Arm' */
  /* SystemInitialize for IfAction SubSystem: '<S27>/SubMode' */
  /* SystemInitialize for IfAction SubSystem: '<S34>/Return' */
  /* InitializeConditions for Delay: '<S161>/Delay' */
  FMS_DW.icLoad_k = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S164>/Integrator1' */
  FMS_DW.Integrator1_IC_LOADING_m = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S158>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_g = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S158>/Integrator' */
  FMS_DW.Integrator_DSTATE_g = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S164>/Integrator' */
  FMS_DW.Integrator_DSTATE_i = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S34>/Return' */
  /* End of SystemInitialize for SubSystem: '<S27>/SubMode' */

  /* SystemInitialize for IfAction SubSystem: '<S27>/Auto' */
  /* SystemInitialize for IfAction SubSystem: '<S32>/Mission' */
  /* SystemInitialize for Atomic SubSystem: '<S91>/Mission_SubSystem' */
  /* InitializeConditions for Delay: '<S100>/Delay' */
  FMS_DW.icLoad_j = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S103>/Integrator1' */
  FMS_DW.Integrator1_IC_LOADING = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S116>/Discrete-Time Integrator' */
  FMS_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
  FMS_DW.DiscreteTimeIntegrator_PrevRese = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S97>/Integrator1' */
  FMS_DW.Integrator1_DSTATE_m = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S97>/Integrator' */
  FMS_DW.Integrator_DSTATE = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S103>/Integrator' */
  FMS_DW.Integrator_DSTATE_b = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S91>/Mission_SubSystem' */
  /* End of SystemInitialize for SubSystem: '<S32>/Mission' */
  /* End of SystemInitialize for SubSystem: '<S27>/Auto' */

  /* SystemInitialize for IfAction SubSystem: '<S27>/Assist' */
  /* SystemInitialize for IfAction SubSystem: '<S31>/Stabilize' */
  /* SystemInitialize for Chart: '<S80>/Motion State' */
  FMS_DW.temporalCounter_i1_b = 0U;
  FMS_DW.is_active_c10_FMS = 0U;
  FMS_DW.is_c10_FMS = FMS_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for IfAction SubSystem: '<S79>/Hold Control' */
  /* InitializeConditions for Delay: '<S82>/Delay' */
  FMS_DW.icLoad_nm = 1U;

  /* End of SystemInitialize for SubSystem: '<S79>/Hold Control' */

  /* SystemInitialize for Merge: '<S79>/Merge' */
  FMS_B.Merge_h = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S31>/Stabilize' */

  /* SystemInitialize for IfAction SubSystem: '<S31>/Position' */
  /* SystemInitialize for Chart: '<S42>/Motion State' */
  FMS_DW.temporalCounter_i1_o = 0U;
  FMS_DW.is_active_c16_FMS = 0U;
  FMS_DW.is_c16_FMS = FMS_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for IfAction SubSystem: '<S41>/Keep Control' */
  /* InitializeConditions for Delay: '<S66>/start_wp' */
  FMS_DW.icLoad_n = 1U;

  /* InitializeConditions for Delay: '<S66>/start_wp1' */
  FMS_DW.icLoad_g = 1U;

  /* End of SystemInitialize for SubSystem: '<S41>/Keep Control' */

  /* SystemInitialize for IfAction SubSystem: '<S41>/Hold Control' */
  /* InitializeConditions for UnitDelay: '<S63>/Delay Input1'
   *
   * Block description for '<S63>/Delay Input1':
   *
   *  Store in Global RAM
   */
  FMS_DW.DelayInput1_DSTATE = 0.0F;

  /* InitializeConditions for Delay: '<S49>/start_vel' */
  FMS_DW.icLoad = 1U;

  /* InitializeConditions for Delay: '<S49>/start_wp' */
  FMS_DW.icLoad_p = 1U;

  /* End of SystemInitialize for SubSystem: '<S41>/Hold Control' */

  /* SystemInitialize for Merge: '<S41>/Merge' */
  FMS_B.Merge[0] = 0.0F;
  FMS_B.Merge[1] = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S31>/Position' */
  /* End of SystemInitialize for SubSystem: '<S27>/Assist' */
  /* End of SystemInitialize for SubSystem: '<S25>/Arm' */
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

/*
 * File: Controller.h
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

#ifndef RTW_HEADER_Controller_h_
#define RTW_HEADER_Controller_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef Controller_COMMON_INCLUDES_
# define Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Controller_COMMON_INCLUDES_ */

#include "Controller_types.h"
#include "arm_math.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<S4>/Quadcopter_x' */
#if AIRFRAME == 1

typedef struct {
  real32_T Delay_DSTATE[8];            /* '<S14>/Delay' */
} DW_Quadcopter_x_Controller_T;

#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator1_DSTATE[2];      /* '<S99>/Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE[2];/* '<S104>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE[2];/* '<S106>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator5_DSTATE[3];/* '<S73>/Discrete-Time Integrator5' */
  real32_T DiscreteTimeIntegrator_DSTATE_o[3];/* '<S76>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_o[3];/* '<S78>/Discrete-Time Integrator1' */
  real32_T Integrator1_DSTATE_p;       /* '<S116>/Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_m;/* '<S119>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_h;/* '<S121>/Discrete-Time Integrator1' */
  real32_T Integrator_DSTATE[2];       /* '<S99>/Integrator' */
  real32_T Integrator_DSTATE_p;        /* '<S116>/Integrator' */
  uint32_T DiscreteTimeIntegrator_DSTATE_n;/* '<S3>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S104>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S106>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_g;/* '<S76>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_i;/* '<S78>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S119>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_Prev_iy;/* '<S121>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S106>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator5_IC_LOAD;/* '<S73>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_l;/* '<S78>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_k;/* '<S121>/Discrete-Time Integrator1' */

#if AIRFRAME == 1

  DW_Quadcopter_x_Controller_T Quadcopter_x;/* '<S4>/Quadcopter_x' */

#define DW_CONTROLLER_T_VARIANT_EXISTS
#endif

} DW_Controller_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T VectorConcatenate3[3];/* '<S101>/Vector Concatenate3' */
  const real32_T Constant;             /* '<S104>/Constant' */
  const real32_T Gain;                 /* '<S88>/Gain' */
  const real32_T Constant_n[3];        /* '<S76>/Constant' */
  const real32_T Square;               /* '<S102>/Square' */
  const real32_T d;                    /* '<S102>/Multiply' */
  const real32_T Gain4;                /* '<S102>/Gain4' */
  const real32_T Square_g;             /* '<S117>/Square' */
  const real32_T d_n;                  /* '<S117>/Multiply' */
  const real32_T Gain4_k;              /* '<S117>/Gain4' */
} ConstB_Controller_T;

/* Constant parameters (default storage) */
typedef struct {

#if AIRFRAME == 1

  /* Expression: [2; 2; 1; 5]
   * Referenced by: '<S14>/Constant7'
   */
  real_T Constant7_Value[4];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 6

  /* Computed Parameter: Effective_Matrix_Value
   * Referenced by: '<S7>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value[36];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 3

  /* Computed Parameter: Effective_Matrix_Value_j
   * Referenced by: '<S8>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_j[24];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 5

  /* Computed Parameter: Effective_Matrix_Value_je
   * Referenced by: '<S9>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_je[18];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 4

  /* Computed Parameter: Effective_Matrix_Value_f
   * Referenced by: '<S10>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_f[18];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 8

  /* Computed Parameter: Effective_Matrix_Value_m
   * Referenced by: '<S11>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_m[24];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 7

  /* Computed Parameter: Effective_Matrix_Value_k
   * Referenced by: '<S12>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_k[24];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#if AIRFRAME == 2

  /* Computed Parameter: Effective_Matrix_Value_fr
   * Referenced by: '<S13>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value_fr[12];

#define CONSTP_CONTROLLER_T_VARIANT_EXISTS
#endif

#ifndef CONSTP_CONTROLLER_T_VARIANT_EXISTS

  char _rt_unused;

#endif

} ConstP_Controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  FMS_Out_Bus FMS_Out;                 /* '<Root>/FMS_Out' */
  INS_Out_Bus INS_Out;                 /* '<Root>/INS_Out' */
} ExtU_Controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  Control_Out_Bus Control_Out;         /* '<Root>/Control_Out' */
} ExtY_Controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_Controller_T {
  const char_T *errorStatus;
};

/* Block states (default storage) */
extern DW_Controller_T Controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Controller_T Controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Controller_T Controller_Y;

/* External data declarations for dependent source files */
extern const Control_Out_Bus Controller_rtZControl_Out_Bus;/* Control_Out_Bus ground */
extern const ConstB_Controller_T Controller_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Controller_T Controller_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T att_cmd_B_rad[2];      /* '<S84>/Switch' */
extern real32_T att_est_B_rad[2];      /* '<S87>/Signal Copy' */
extern real32_T rate_cmd_B_radPs[3];   /* '<S68>/Switch' */
extern real32_T rate_est_B_radPs[3];   /* '<S72>/Signal Copy' */
extern real32_T w_cmd_C_mPs;           /* '<S116>/Integrator1' */
extern real32_T w_est_C_mPs;           /* '<S113>/Signal Copy1' */

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern struct_oqmoiGpNYV6eEvDoym9FPF CONTROL_PARAM;/* Variable: CONTROL_PARAM
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
extern struct_ny3PY9hontv4J5WqwlFzJB CONTROL_EXPORT;/* Variable: CONTROL_EXPORT
                                                     * Referenced by: '<S3>/Constant'
                                                     */

/* Model entry point functions */
extern void Controller_init(void);
extern void Controller_step(void);
extern void Controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Controller_T *const Controller_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S89>/Data Type Duplicate' : Unused code path elimination
 * Block '<S89>/Data Type Propagation' : Unused code path elimination
 * Block '<S3>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Controller'
 * '<S1>'   : 'Controller/Bus_Constructor'
 * '<S2>'   : 'Controller/Controller'
 * '<S3>'   : 'Controller/Bus_Constructor/timestamp'
 * '<S4>'   : 'Controller/Controller/Control_Allocation'
 * '<S5>'   : 'Controller/Controller/Horizontal_Control'
 * '<S6>'   : 'Controller/Controller/Vertical_Control'
 * '<S7>'   : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x'
 * '<S8>'   : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x'
 * '<S9>'   : 'Controller/Controller/Control_Allocation/Hexacopter_+'
 * '<S10>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x'
 * '<S11>'  : 'Controller/Controller/Control_Allocation/Octocopter_+'
 * '<S12>'  : 'Controller/Controller/Control_Allocation/Octocopter_x'
 * '<S13>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+'
 * '<S14>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x'
 * '<S15>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/Signal_Select'
 * '<S16>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/actuator_cmd_routing'
 * '<S17>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/throttle_mapping'
 * '<S18>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/Signal_Select/Compare To Constant'
 * '<S19>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/Signal_Select/Compare To Constant1'
 * '<S20>'  : 'Controller/Controller/Control_Allocation/Coxial_Hexacopter_x/Signal_Select/Offboard_Signal_Select'
 * '<S21>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/Signal_Select'
 * '<S22>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/actuator_cmd_routing'
 * '<S23>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/throttle_mapping'
 * '<S24>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/Signal_Select/Compare To Constant'
 * '<S25>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/Signal_Select/Compare To Constant1'
 * '<S26>'  : 'Controller/Controller/Control_Allocation/Coxial_Quadcopter_x/Signal_Select/Offboard_Signal_Select'
 * '<S27>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/Signal_Select'
 * '<S28>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/actuator_cmd_routing'
 * '<S29>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/throttle_mapping'
 * '<S30>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/Signal_Select/Compare To Constant'
 * '<S31>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/Signal_Select/Compare To Constant1'
 * '<S32>'  : 'Controller/Controller/Control_Allocation/Hexacopter_+/Signal_Select/Offboard_Signal_Select'
 * '<S33>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/Signal_Select'
 * '<S34>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/actuator_cmd_routing'
 * '<S35>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/throttle_mapping'
 * '<S36>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/Signal_Select/Compare To Constant'
 * '<S37>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/Signal_Select/Compare To Constant1'
 * '<S38>'  : 'Controller/Controller/Control_Allocation/Hexacopter_x/Signal_Select/Offboard_Signal_Select'
 * '<S39>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/Signal_Select'
 * '<S40>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/actuator_cmd_routing'
 * '<S41>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/throttle_mapping'
 * '<S42>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/Signal_Select/Compare To Constant'
 * '<S43>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/Signal_Select/Compare To Constant1'
 * '<S44>'  : 'Controller/Controller/Control_Allocation/Octocopter_+/Signal_Select/Offboard_Signal_Select'
 * '<S45>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/Signal_Select'
 * '<S46>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/actuator_cmd_routing'
 * '<S47>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/throttle_mapping'
 * '<S48>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/Signal_Select/Compare To Constant'
 * '<S49>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/Signal_Select/Compare To Constant1'
 * '<S50>'  : 'Controller/Controller/Control_Allocation/Octocopter_x/Signal_Select/Offboard_Signal_Select'
 * '<S51>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/Signal_Select'
 * '<S52>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/actuator_cmd_routing'
 * '<S53>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/throttle_mapping'
 * '<S54>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/Signal_Select/Compare To Constant'
 * '<S55>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/Signal_Select/Compare To Constant1'
 * '<S56>'  : 'Controller/Controller/Control_Allocation/Quadcopter_+/Signal_Select/Offboard_Signal_Select'
 * '<S57>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x/MATLAB Function'
 * '<S58>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x/MATLAB Function1'
 * '<S59>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x/MATLAB Function2'
 * '<S60>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x/MATLAB Function3'
 * '<S61>'  : 'Controller/Controller/Control_Allocation/Quadcopter_x/actuator_cmd_routing'
 * '<S62>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller'
 * '<S63>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller'
 * '<S64>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop'
 * '<S65>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop'
 * '<S66>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller'
 * '<S67>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Convert'
 * '<S68>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select'
 * '<S69>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error'
 * '<S70>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Feedforward_Controller'
 * '<S71>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller'
 * '<S72>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error/Bus_Select'
 * '<S73>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error/First Order LPF'
 * '<S74>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Feedforward_Controller/DeadZone'
 * '<S75>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/D_Control'
 * '<S76>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/I_Control'
 * '<S77>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/P_Control'
 * '<S78>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/D_Control/DT Filter'
 * '<S79>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Convert/Euler To Angle Rate'
 * '<S80>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Compare To Constant'
 * '<S81>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Compare To Constant1'
 * '<S82>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Offboard_Signal_Select'
 * '<S83>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller'
 * '<S84>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select'
 * '<S85>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Error'
 * '<S86>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller'
 * '<S87>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Error/Bus_Select'
 * '<S88>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller/Sqrt_Root_Control'
 * '<S89>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller/Sqrt_Root_Control/Saturation Dynamic1'
 * '<S90>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Compare To Constant'
 * '<S91>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Compare To Constant1'
 * '<S92>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Offboard_Signal_Select'
 * '<S93>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Accel_to_Attitude_CMD'
 * '<S94>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error'
 * '<S95>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller'
 * '<S96>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select'
 * '<S97>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select'
 * '<S98>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Compare To Constant'
 * '<S99>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/TD'
 * '<S100>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select/Psi To DCM'
 * '<S101>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select/Psi To DCM/Rotation Matrix Z'
 * '<S102>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/TD/fhan '
 * '<S103>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/D_Control'
 * '<S104>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/I_Control'
 * '<S105>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/P_Control'
 * '<S106>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/D_Control/DT Filter'
 * '<S107>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard'
 * '<S108>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select'
 * '<S109>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select/Compare To Zero'
 * '<S110>' : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select/Compare To Zero2'
 * '<S111>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller'
 * '<S112>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/Boosted_Throttle'
 * '<S113>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/Error'
 * '<S114>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/PID_Controller'
 * '<S115>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/Error/Compare To Constant'
 * '<S116>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/Error/TD'
 * '<S117>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/Error/TD/fhan '
 * '<S118>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/PID_Controller/D_Control'
 * '<S119>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/PID_Controller/I_Control'
 * '<S120>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/PID_Controller/P_Control'
 * '<S121>' : 'Controller/Controller/Vertical_Control/Veloxity_Z_Controller/PID_Controller/D_Control/DT Filter'
 */
#endif                                 /* RTW_HEADER_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * File: Controller.h
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

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator1_DSTATE[2];      /* '<S50>/Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE[2];/* '<S55>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE[2];/* '<S57>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator5_DSTATE[3];/* '<S26>/Discrete-Time Integrator5' */
  real32_T DiscreteTimeIntegrator_DSTATE_o[3];/* '<S28>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_o[3];/* '<S30>/Discrete-Time Integrator1' */
  real32_T Integrator1_DSTATE_p;       /* '<S66>/Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_m;/* '<S69>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_h;/* '<S71>/Discrete-Time Integrator1' */
  real32_T Integrator_DSTATE[2];       /* '<S50>/Integrator' */
  real32_T Integrator_DSTATE_p;        /* '<S66>/Integrator' */
  uint32_T DiscreteTimeIntegrator_DSTATE_n;/* '<S3>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S55>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S57>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_g;/* '<S28>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_i;/* '<S30>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S69>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_Prev_iy;/* '<S71>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S57>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator5_IC_LOAD;/* '<S26>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_l;/* '<S30>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_k;/* '<S71>/Discrete-Time Integrator1' */
} DW_Controller_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T VectorConcatenate3[3];/* '<S52>/Vector Concatenate3' */
  const real32_T Constant;             /* '<S55>/Constant' */
  const real32_T Gain;                 /* '<S40>/Gain' */
  const real32_T Constant_n[3];        /* '<S28>/Constant' */
  const real32_T Square;               /* '<S53>/Square' */
  const real32_T d;                    /* '<S53>/Multiply' */
  const real32_T Gain4;                /* '<S53>/Gain4' */
  const real32_T Square_g;             /* '<S67>/Square' */
  const real32_T d_n;                  /* '<S67>/Multiply' */
  const real32_T Gain4_k;              /* '<S67>/Gain4' */
} ConstB_Controller_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Effective_Matrix_Value
   * Referenced by: '<S7>/Effective_Matrix'
   */
  real32_T Effective_Matrix_Value[12];
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
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern struct_3zLBZv1DafkAI5Ov6R3JHD CONTROL_PARAM;/* Variable: CONTROL_PARAM
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
extern struct_AtpMUMXHQ4EpFICCggTZNH CONTROL_EXPORT;/* Variable: CONTROL_EXPORT
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
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S3>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S7>/Reshape' : Reshape block reduction
 * Block '<S9>/Reshape1' : Reshape block reduction
 * Block '<S26>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S64>/Signal Copy1' : Eliminate redundant signal conversion block
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
 * '<S7>'   : 'Controller/Controller/Control_Allocation/Ducted_Rocket'
 * '<S8>'   : 'Controller/Controller/Control_Allocation/Ducted_Rocket/Signal_Select'
 * '<S9>'   : 'Controller/Controller/Control_Allocation/Ducted_Rocket/actuator_cmd_routing'
 * '<S10>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/throttle_mapping'
 * '<S11>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/Signal_Select/Compare To Constant'
 * '<S12>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/Signal_Select/Compare To Constant1'
 * '<S13>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/Signal_Select/Offboard_Signal_Select'
 * '<S14>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/actuator_cmd_routing/Disarm_Output'
 * '<S15>'  : 'Controller/Controller/Control_Allocation/Ducted_Rocket/actuator_cmd_routing/Standby_Output'
 * '<S16>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller'
 * '<S17>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller'
 * '<S18>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop'
 * '<S19>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop'
 * '<S20>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller'
 * '<S21>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Convert'
 * '<S22>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select'
 * '<S23>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error'
 * '<S24>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller'
 * '<S25>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error/Bus_Select'
 * '<S26>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/Error/First Order LPF'
 * '<S27>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/D_Control'
 * '<S28>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/I_Control'
 * '<S29>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/P_Control'
 * '<S30>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Rate_Controller/PID_Controller/D_Control/DT Filter'
 * '<S31>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Convert/Euler To Angle Rate'
 * '<S32>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Compare To Constant'
 * '<S33>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Compare To Constant1'
 * '<S34>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Inner_Loop/Signal_Select/Offboard_Signal_Select'
 * '<S35>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller'
 * '<S36>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select'
 * '<S37>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Error'
 * '<S38>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller'
 * '<S39>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Error/Bus_Select'
 * '<S40>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller/Sqrt_Root_Control'
 * '<S41>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Attitude_Controller/Sqrt_Root_Controller/Sqrt_Root_Control/Saturation Dynamic1'
 * '<S42>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Compare To Constant'
 * '<S43>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Compare To Constant1'
 * '<S44>'  : 'Controller/Controller/Horizontal_Control/Attitude_Controller/Outter_Loop/Signal_Select/Offboard_Signal_Select'
 * '<S45>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Accel_to_Attitude_CMD'
 * '<S46>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error'
 * '<S47>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller'
 * '<S48>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select'
 * '<S49>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select'
 * '<S50>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/TD'
 * '<S51>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select/Psi To DCM'
 * '<S52>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/Bus_Select/Psi To DCM/Rotation Matrix Z'
 * '<S53>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Error/TD/fhan '
 * '<S54>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/D_Control'
 * '<S55>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/I_Control'
 * '<S56>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/P_Control'
 * '<S57>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/PID_Controller/D_Control/DT Filter'
 * '<S58>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard'
 * '<S59>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select'
 * '<S60>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select/Compare To Zero'
 * '<S61>'  : 'Controller/Controller/Horizontal_Control/Velocity_Controller/Signal_Select/Offboard_Signal_Select/Compare To Zero2'
 * '<S62>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller'
 * '<S63>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/Boosted_Throttle'
 * '<S64>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/Error'
 * '<S65>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/PID_Controller'
 * '<S66>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/Error/TD'
 * '<S67>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/Error/TD/fhan '
 * '<S68>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/PID_Controller/D_Control'
 * '<S69>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/PID_Controller/I_Control'
 * '<S70>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/PID_Controller/P_Control'
 * '<S71>'  : 'Controller/Controller/Vertical_Control/Velocity_Z_Controller/PID_Controller/D_Control/DT Filter'
 */
#endif                                 /* RTW_HEADER_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * File: FMS_data.c
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

/* Invariant block signals (default storage) */
const ConstB_FMS_T FMS_ConstB = {
  1.9966471893352524,                  /* '<S318>/Sum' */
  0.0066943799901413165,               /* '<S318>/Multiply3' */
  0.99330562000985867,                 /* '<S318>/Sum4' */
  0.0,                                 /* '<S317>/deg2rad2' */
  0.0,                                 /* '<S320>/SinCos' */
  1.0,                                 /* '<S320>/SinCos' */
  0.00250000018F,                      /* '<S123>/Square' */
  0.14709F,                            /* '<S123>/Multiply' */
  -58.836F,                            /* '<S123>/Gain4' */
  0.0016F,                             /* '<S67>/Square' */
  0.0201061927F,                       /* '<S67>/Multiply' */
  -12.566371F,                         /* '<S67>/Gain4' */
  0.0016F,                             /* '<S66>/Square' */
  0.0201061927F,                       /* '<S66>/Multiply' */
  -12.566371F,                         /* '<S66>/Gain4' */
  0.0016F,                             /* '<S135>/Square' */
  0.0201061927F,                       /* '<S135>/Multiply' */
  -12.566371F,                         /* '<S135>/Gain4' */
  0.0016F,                             /* '<S134>/Square' */
  0.0201061927F,                       /* '<S134>/Multiply' */
  -12.566371F,                         /* '<S134>/Gain4' */

  { 0.0F, 0.0F, 1.0F },                /* '<S230>/Vector Concatenate3' */

  { 0.0F, 1.0F },                      /* synthesized block */

  { 0.0F, 1.0F },                      /* '<S170>/Math Function' */
  1.0F,                                /* '<S170>/Sum of Elements' */
  1.0F,                                /* '<S170>/Math Function1' */

  { 0.0F, 0.0F },                      /* '<S170>/Product' */

  { 0.0F, 1.0F, 1.0F },                /* '<S170>/Switch' */

  { 0.0F, 1.0F },                      /* '<S170>/Divide' */
  0.616850317F,                        /* '<S166>/Square' */
  0.645964146F,                        /* '<S166>/Multiply' */
  -1.04719758F,                        /* '<S166>/Gain4' */

  { 0.0F, 0.0F, 1.0F },                /* '<S311>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S312>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S308>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S304>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S242>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S244>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S333>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S335>/Vector Concatenate3' */

  { 0.0F, 0.0F, 1.0F },                /* '<S464>/Vector Concatenate3' */

  { 0.0F, 1.0F },                      /* synthesized block */

  { 0.0F, 1.0F },                      /* '<S404>/Math Function' */
  1.0F,                                /* '<S404>/Sum of Elements' */
  1.0F,                                /* '<S404>/Math Function1' */

  { 0.0F, 0.0F },                      /* '<S404>/Product' */

  { 0.0F, 1.0F, 1.0F },                /* '<S404>/Switch' */

  { 0.0F, 1.0F },                      /* '<S404>/Divide' */
  0.616850317F,                        /* '<S400>/Square' */
  0.645964146F,                        /* '<S400>/Multiply' */
  -1.04719758F,                        /* '<S400>/Gain4' */

  { 0.0F, 0.0F, 1.0F },                /* '<S387>/Vector Concatenate3' */
  0.122499995F,                        /* '<S383>/Square' */
  1.20123494F,                         /* '<S383>/Multiply' */
  -9.806F,                             /* '<S383>/Gain4' */

  { 0.0F, 0.0F, 1.0F },                /* '<S474>/Vector Concatenate3' */
  0.122499995F,                        /* '<S470>/Square' */
  1.20123494F,                         /* '<S470>/Multiply' */
  -9.806F,                             /* '<S470>/Gain4' */
  1U,                                  /* '<S37>/Data Type Conversion' */
  12U,                                 /* '<S37>/Data Type Conversion1' */
  3U,                                  /* '<S42>/Data Type Conversion' */
  10U,                                 /* '<S42>/Data Type Conversion1' */
  5U,                                  /* '<S42>/Data Type Conversion2' */
  3U,                                  /* '<S41>/Data Type Conversion1' */
  9U,                                  /* '<S41>/Data Type Conversion2' */
  4U,                                  /* '<S41>/Data Type Conversion3' */
  3U,                                  /* '<S43>/Data Type Conversion' */
  8U,                                  /* '<S43>/Data Type Conversion1' */
  3U,                                  /* '<S43>/Data Type Conversion2' */
  3U,                                  /* '<S40>/Data Type Conversion' */
  7U,                                  /* '<S40>/Data Type Conversion1' */
  2U,                                  /* '<S40>/Data Type Conversion2' */
  3U,                                  /* '<S153>/Data Type Conversion' */
  4U,                                  /* '<S153>/Data Type Conversion1' */
  5U,                                  /* '<S153>/Data Type Conversion2' */
  3U,                                  /* '<S150>/Data Type Conversion' */
  3U,                                  /* '<S150>/Data Type Conversion1' */
  6U,                                  /* '<S150>/Data Type Conversion2' */
  3U,                                  /* '<S336>/Data Type Conversion' */
  6U,                                  /* '<S336>/Data Type Conversion1' */
  5U,                                  /* '<S336>/Data Type Conversion2' */
  3U,                                  /* '<S338>/Data Type Conversion' */
  16U,                                 /* '<S338>/Data Type Conversion1' */
  5U,                                  /* '<S338>/Data Type Conversion2' */
  3U,                                  /* '<S337>/Data Type Conversion' */
  15U,                                 /* '<S337>/Data Type Conversion1' */
  5U,                                  /* '<S337>/Data Type Conversion2' */
  3U,                                  /* '<S339>/Data Type Conversion' */
  17U,                                 /* '<S339>/Data Type Conversion1' */
  5U,                                  /* '<S339>/Data Type Conversion2' */
  2U,                                  /* '<S34>/Data Type Conversion2' */
  2U,                                  /* '<S34>/Data Type Conversion1' */
  1U,                                  /* '<S33>/Data Type Conversion' */
  1U,                                  /* '<S33>/Data Type Conversion1' */

  /* Start of '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */
  {
    1.9966471893352524,                /* '<S480>/Sum' */
    0.0066943799901413165,             /* '<S480>/Multiply3' */
    0.99330562000985867                /* '<S480>/Sum4' */
  }
  ,

  /* End of '<S5>/Vehicle.Arm.Auto.Mission.LLA2FLAT' */

  /* Start of '<S31>/Unknown' */
  {
    1U,                                /* '<S39>/Data Type Conversion' */
    1U                                 /* '<S39>/Data Type Conversion1' */
  }
  ,

  /* End of '<S31>/Unknown' */

  /* Start of '<S35>/Unknown' */
  {
    1U,                                /* '<S44>/Data Type Conversion' */
    1U                                 /* '<S44>/Data Type Conversion1' */
  }
  ,

  /* End of '<S35>/Unknown' */

  /* Start of '<S109>/Hold Control' */
  {
    { 0.0F, 0.0F, 1.0F }               /* '<S116>/Vector Concatenate3' */
  }
  ,

  /* End of '<S109>/Hold Control' */

  /* Start of '<S96>/Move Control' */
  {
    0.0004F,                           /* '<S106>/Square' */
    0.00628318498F,                    /* '<S106>/Multiply' */
    -15.707963F                        /* '<S106>/Gain4' */
  }
  ,

  /* End of '<S96>/Move Control' */

  /* Start of '<S68>/Move Control' */
  {
    0.0004F,                           /* '<S78>/Square' */
    0.00628318498F,                    /* '<S78>/Multiply' */
    -15.707963F                        /* '<S78>/Gain4' */
  }
  ,

  /* End of '<S68>/Move Control' */

  /* Start of '<S136>/Move Control' */
  {
    0.0004F,                           /* '<S146>/Square' */
    0.00628318498F,                    /* '<S146>/Multiply' */
    -15.707963F                        /* '<S146>/Gain4' */
  }
  ,

  /* End of '<S136>/Move Control' */

  /* Start of '<S36>/Unknown' */
  {
    1U,                                /* '<S151>/Data Type Conversion' */
    1U                                 /* '<S151>/Data Type Conversion1' */
  }
  ,

  /* End of '<S36>/Unknown' */

  /* Start of '<S200>/Move Control' */
  {
    0.00250000018F,                    /* '<S211>/Square' */
    0.14709F,                          /* '<S211>/Multiply' */
    -58.836F                           /* '<S211>/Gain4' */
  }
  ,

  /* End of '<S200>/Move Control' */

  /* Start of '<S200>/Hold Control' */
  {
    { 0.0F, 0.0F, 1.0F }               /* '<S207>/Vector Concatenate3' */
  }
  ,

  /* End of '<S200>/Hold Control' */

  /* Start of '<S190>/Move Control' */
  {
    0.00250000018F,                    /* '<S198>/Square' */
    0.196120009F,                      /* '<S198>/Multiply' */
    -78.448F                           /* '<S198>/Gain4' */
  }
  ,

  /* End of '<S190>/Move Control' */

  /* Start of '<S38>/Unknown' */
  {
    1U,                                /* '<S340>/Data Type Conversion' */
    1U                                 /* '<S340>/Data Type Conversion1' */
  }
  ,

  /* End of '<S38>/Unknown' */

  /* Start of '<S366>/Move Control' */
  {
    0.00250000018F,                    /* '<S377>/Square' */
    0.14709F,                          /* '<S377>/Multiply' */
    -58.836F                           /* '<S377>/Gain4' */
  }
  ,

  /* End of '<S366>/Move Control' */

  /* Start of '<S366>/Hold Control' */
  {
    { 0.0F, 0.0F, 1.0F }               /* '<S373>/Vector Concatenate3' */
  }
  ,

  /* End of '<S366>/Hold Control' */

  /* Start of '<S354>/Move Control' */
  {
    0.0004F,                           /* '<S364>/Square' */
    0.00628318498F,                    /* '<S364>/Multiply' */
    -15.707963F                        /* '<S364>/Gain4' */
  }
  ,

  /* End of '<S354>/Move Control' */

  /* Start of '<S344>/Move Control' */
  {
    0.00250000018F,                    /* '<S352>/Square' */
    0.196120009F,                      /* '<S352>/Multiply' */
    -78.448F                           /* '<S352>/Gain4' */
  }
  ,

  /* End of '<S344>/Move Control' */

  /* Start of '<S434>/Move Control' */
  {
    0.00250000018F,                    /* '<S445>/Square' */
    0.14709F,                          /* '<S445>/Multiply' */
    -58.836F                           /* '<S445>/Gain4' */
  }
  ,

  /* End of '<S434>/Move Control' */

  /* Start of '<S434>/Hold Control' */
  {
    { 0.0F, 0.0F, 1.0F }               /* '<S441>/Vector Concatenate3' */
  }
  ,

  /* End of '<S434>/Hold Control' */

  /* Start of '<S424>/Move Control' */
  {
    0.00250000018F,                    /* '<S432>/Square' */
    0.196120009F,                      /* '<S432>/Multiply' */
    -78.448F                           /* '<S432>/Gain4' */
  }
  /* End of '<S424>/Move Control' */
};

/* Constant parameters (default storage) */
const ConstP_FMS_T FMS_ConstP = {
  /* Pooled Parameter (Expression: [4,4,2])
   * Referenced by:
   *   '<S297>/Saturation1'
   *   '<S299>/Saturation'
   *   '<S300>/Saturation1'
   */
  { 4.0F, 4.0F, 2.0F },

  /* Pooled Parameter (Expression: [-4,-4,-2])
   * Referenced by:
   *   '<S297>/Saturation1'
   *   '<S299>/Saturation'
   *   '<S300>/Saturation1'
   */
  { -4.0F, -4.0F, -2.0F }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

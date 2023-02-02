/*
 * LaneKeeping_data.c
 *
 * Code generation for model "LaneKeeping".
 *
 * Model version              : 1.53
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Thu Feb  2 10:52:01 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LaneKeeping.h"

/* Block parameters (default storage) */
P_LaneKeeping_T LaneKeeping_P = {
  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S1>/Constant'
   */
  1.0,

  /* Mask Parameter: DetIncAct_vinit
   * Referenced by: '<S4>/Delay Input1'
   */
  0U,

  /* Expression: 0
   * Referenced by: '<S2>/Data Store Memory1'
   */
  0.0,

  /* Expression: 1.5
   * Referenced by: '<Root>/UpTorque'
   */
  1.5,

  /* Expression: 0.1
   * Referenced by: '<Root>/UpDuration'
   */
  0.1,

  /* Expression: 0.375
   * Referenced by: '<Root>/DownTorque'
   */
  0.375,

  /* Expression: 0.1
   * Referenced by: '<Root>/DownDuration'
   */
  0.1,

  /* Expression: 0
   * Referenced by: '<Root>/ShockCount'
   */
  0.0,

  /* Computed Parameter: SwitchAct_Threshold
   * Referenced by: '<S2>/SwitchAct'
   */
  1U
};

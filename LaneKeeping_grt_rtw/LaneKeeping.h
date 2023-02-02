/*
 * LaneKeeping.h
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

#ifndef RTW_HEADER_LaneKeeping_h_
#define RTW_HEADER_LaneKeeping_h_
#ifndef LaneKeeping_COMMON_INCLUDES_
#define LaneKeeping_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* LaneKeeping_COMMON_INCLUDES_ */

#include "LaneKeeping_types.h"
#include "rtGetInf.h"
#include <float.h>
#include <string.h>
#include <stddef.h>
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  uint8_T Compare;                     /* '<S1>/Compare' */
  uint8_T Uk1;                         /* '<S4>/Delay Input1' */
} B_LaneKeeping_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Time;                         /* '<S2>/Data Store Memory1' */
  uint8_T DelayInput1_DSTATE;          /* '<S4>/Delay Input1' */
} DW_LaneKeeping_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T LateralShift;                 /* '<Root>/Lateral Shift' */
} ExtU_LaneKeeping_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T AdditinalTorque;              /* '<Root>/Additinal Torque' */
} ExtY_LaneKeeping_T;

/* Parameters (default storage) */
struct P_LaneKeeping_T_ {
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S1>/Constant'
                                       */
  uint8_T DetIncAct_vinit;             /* Mask Parameter: DetIncAct_vinit
                                        * Referenced by: '<S4>/Delay Input1'
                                        */
  real_T DataStoreMemory1_InitialValue;/* Expression: 0
                                        * Referenced by: '<S2>/Data Store Memory1'
                                        */
  real_T UpTorque_Value;               /* Expression: 1.5
                                        * Referenced by: '<Root>/UpTorque'
                                        */
  real_T UpDuration_Value;             /* Expression: 0.1
                                        * Referenced by: '<Root>/UpDuration'
                                        */
  real_T DownTorque_Value;             /* Expression: 0.375
                                        * Referenced by: '<Root>/DownTorque'
                                        */
  real_T DownDuration_Value;           /* Expression: 0.1
                                        * Referenced by: '<Root>/DownDuration'
                                        */
  real_T ShockCount_Value;             /* Expression: 0
                                        * Referenced by: '<Root>/ShockCount'
                                        */
  uint8_T SwitchAct_Threshold;        /* Computed Parameter: SwitchAct_Threshold
                                       * Referenced by: '<S2>/SwitchAct'
                                       */
};

/* Real-time Model Data Structure */
struct tag_RTM_LaneKeeping_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_LaneKeeping_T LaneKeeping_P;

/* Block signals (default storage) */
extern B_LaneKeeping_T LaneKeeping_B;

/* Block states (default storage) */
extern DW_LaneKeeping_T LaneKeeping_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_LaneKeeping_T LaneKeeping_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LaneKeeping_T LaneKeeping_Y;

/* Model entry point functions */
extern void LaneKeeping_initialize(void);
extern void LaneKeeping_step(void);
extern void LaneKeeping_terminate(void);

/* Real-time Model object */
extern RT_MODEL_LaneKeeping_T *const LaneKeeping_M;

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
 * '<Root>' : 'LaneKeeping'
 * '<S1>'   : 'LaneKeeping/Compare To Constant'
 * '<S2>'   : 'LaneKeeping/Compute activation time'
 * '<S3>'   : 'LaneKeeping/Embedded MATLAB Function'
 * '<S4>'   : 'LaneKeeping/Compute activation time/DetIncAct'
 */
#endif                                 /* RTW_HEADER_LaneKeeping_h_ */

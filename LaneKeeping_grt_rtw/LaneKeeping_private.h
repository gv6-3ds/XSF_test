/*
 * LaneKeeping_private.h
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

#ifndef RTW_HEADER_LaneKeeping_private_h_
#define RTW_HEADER_LaneKeeping_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "LaneKeeping_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_remd_snf(real_T u0, real_T u1);

#endif                                 /* RTW_HEADER_LaneKeeping_private_h_ */

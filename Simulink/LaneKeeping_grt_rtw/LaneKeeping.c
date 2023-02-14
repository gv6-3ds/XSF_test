/*
 * LaneKeeping.c
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
#include "rtwtypes.h"
#include <math.h>
#include "LaneKeeping_private.h"
#include "rt_nonfinite.h"
#include <float.h>
#include <string.h>

/* Block signals (default storage) */
B_LaneKeeping_T LaneKeeping_B;

/* Block states (default storage) */
DW_LaneKeeping_T LaneKeeping_DW;

/* External inputs (root inport signals with default storage) */
ExtU_LaneKeeping_T LaneKeeping_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LaneKeeping_T LaneKeeping_Y;

/* Real-time model */
static RT_MODEL_LaneKeeping_T LaneKeeping_M_;
RT_MODEL_LaneKeeping_T *const LaneKeeping_M = &LaneKeeping_M_;
real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T q;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else {
    if (u1 < 0.0) {
      q = ceil(u1);
    } else {
      q = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != q)) {
      q = fabs(u0 / u1);
      if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void LaneKeeping_step(void)
//Covered Requirements : #Block_ABS_001, #Block_CAT_002//
{
  real_T NumCreneau;
  real_T PosDansCreneau;
  real_T rtb_Clock;
  real_T rtb_SwitchAct;
  real_T tmp;
  int32_T sens;

  /* Clock: '<Root>/Clock' */
  //Covered Requirements : #Block_MATLAB_FUNCTION_003//
  rtb_Clock = LaneKeeping_M->Timing.t[0];

  /* RelationalOperator: '<S1>/Compare' incorporates:
   *  Abs: '<Root>/Abs'
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/Lateral Shift'
   */
  LaneKeeping_B.Compare = (uint8_T)(fabs(LaneKeeping_U.LateralShift) >
    LaneKeeping_P.CompareToConstant_const);

  /* UnitDelay: '<S4>/Delay Input1' */
  LaneKeeping_B.Uk1 = LaneKeeping_DW.DelayInput1_DSTATE;

  /* Switch: '<S2>/SwitchAct' incorporates:
   *  DataStoreRead: '<S2>/ReadTime'
   *  RelationalOperator: '<S4>/FixPt Relational Operator'
   */
  if ((LaneKeeping_B.Compare > LaneKeeping_B.Uk1) >=
      LaneKeeping_P.SwitchAct_Threshold) {
    rtb_SwitchAct = rtb_Clock;
  } else {
    rtb_SwitchAct = LaneKeeping_DW.Time;
  }

  /* End of Switch: '<S2>/SwitchAct' */

  /* DataStoreWrite: '<S2>/WriteTime' */
  LaneKeeping_DW.Time = rtb_SwitchAct;

  /* MATLAB Function: '<Root>/Embedded MATLAB Function' incorporates:
   *  Constant: '<Root>/DownDuration'
   *  Constant: '<Root>/ShockCount'
   *  Constant: '<Root>/UpDuration'
   *  Inport: '<Root>/Lateral Shift'
   */
  /* MATLAB Function 'Embedded MATLAB Function': '<S3>:1' */
  /* '<S3>:1:5' */
  NumCreneau = 0.0;

  /* '<S3>:1:6' */
  PosDansCreneau = 0.0;
  tmp = LaneKeeping_P.UpDuration_Value + LaneKeeping_P.DownDuration_Value;
  if (tmp > 0.0) {
    /* '<S3>:1:7' */
    /* '<S3>:1:8' */
    rtb_Clock -= rtb_SwitchAct;
    NumCreneau = floor(rtb_Clock / tmp) * 2.0;

    /* '<S3>:1:9' */
    PosDansCreneau = rt_remd_snf(rtb_Clock, tmp);
  }

  if (PosDansCreneau >= LaneKeeping_P.UpDuration_Value) {
    /* '<S3>:1:12' */
    /* '<S3>:1:13' */
    NumCreneau++;
  }

  if (LaneKeeping_U.LateralShift > 0.0) {
    /* '<S3>:1:16' */
    /* '<S3>:1:17' */
    sens = 1;
  } else {
    /* '<S3>:1:19' */
    sens = -1;
  }

  if ((LaneKeeping_B.Compare == 1) && ((LaneKeeping_P.ShockCount_Value == 0.0) ||
       (NumCreneau < LaneKeeping_P.ShockCount_Value))) {
    /* '<S3>:1:22' */
    if (PosDansCreneau < LaneKeeping_P.UpDuration_Value) {
      /* Outport: '<Root>/Additinal Torque' incorporates:
       *  Constant: '<Root>/UpTorque'
       */
      /* '<S3>:1:23' */
      /* '<S3>:1:24' */
      LaneKeeping_Y.AdditinalTorque = (real_T)sens *
        LaneKeeping_P.UpTorque_Value;
    } else {
      /* Outport: '<Root>/Additinal Torque' incorporates:
       *  Constant: '<Root>/DownTorque'
       */
      /* '<S3>:1:26' */
      LaneKeeping_Y.AdditinalTorque = -(real_T)sens *
        LaneKeeping_P.DownTorque_Value;
    }
  } else {
    /* Outport: '<Root>/Additinal Torque' */
    /* '<S3>:1:29' */
    LaneKeeping_Y.AdditinalTorque = 0.0;
  }

  /* End of MATLAB Function: '<Root>/Embedded MATLAB Function' */

  /* Matfile logging */
  rt_UpdateTXYLogVars(LaneKeeping_M->rtwLogInfo, (LaneKeeping_M->Timing.t));

  /* Update for UnitDelay: '<S4>/Delay Input1' */
  LaneKeeping_DW.DelayInput1_DSTATE = LaneKeeping_B.Compare;

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(LaneKeeping_M)!=-1) &&
        !((rtmGetTFinal(LaneKeeping_M)-LaneKeeping_M->Timing.t[0]) >
          LaneKeeping_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(LaneKeeping_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++LaneKeeping_M->Timing.clockTick0)) {
    ++LaneKeeping_M->Timing.clockTickH0;
  }

  LaneKeeping_M->Timing.t[0] = LaneKeeping_M->Timing.clockTick0 *
    LaneKeeping_M->Timing.stepSize0 + LaneKeeping_M->Timing.clockTickH0 *
    LaneKeeping_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.2s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.2, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    LaneKeeping_M->Timing.clockTick1++;
    if (!LaneKeeping_M->Timing.clockTick1) {
      LaneKeeping_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void LaneKeeping_initialize(void)
//Covered Requirements : #Input_1, #Input_2//
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)LaneKeeping_M, 0,
                sizeof(RT_MODEL_LaneKeeping_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&LaneKeeping_M->solverInfo,
                          &LaneKeeping_M->Timing.simTimeStep);
    rtsiSetTPtr(&LaneKeeping_M->solverInfo, &rtmGetTPtr(LaneKeeping_M));
    rtsiSetStepSizePtr(&LaneKeeping_M->solverInfo,
                       &LaneKeeping_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&LaneKeeping_M->solverInfo, (&rtmGetErrorStatus
      (LaneKeeping_M)));
    rtsiSetRTModelPtr(&LaneKeeping_M->solverInfo, LaneKeeping_M);
  }

  rtsiSetSimTimeStep(&LaneKeeping_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&LaneKeeping_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(LaneKeeping_M, &LaneKeeping_M->Timing.tArray[0]);
  rtmSetTFinal(LaneKeeping_M, -1);
  LaneKeeping_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    LaneKeeping_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(LaneKeeping_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(LaneKeeping_M->rtwLogInfo, (NULL));
    rtliSetLogT(LaneKeeping_M->rtwLogInfo, "tout");
    rtliSetLogX(LaneKeeping_M->rtwLogInfo, "");
    rtliSetLogXFinal(LaneKeeping_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(LaneKeeping_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(LaneKeeping_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(LaneKeeping_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(LaneKeeping_M->rtwLogInfo, 1);

    /*
     * Set pointers to the data and signal info for each output
     */
    {
      static void * rt_LoggedOutputSignalPtrs[] = {
        &LaneKeeping_Y.AdditinalTorque
      };

      rtliSetLogYSignalPtrs(LaneKeeping_M->rtwLogInfo, ((LogSignalPtrsType)
        rt_LoggedOutputSignalPtrs));
    }

    {
      static int_T rt_LoggedOutputWidths[] = {
        1
      };

      static int_T rt_LoggedOutputNumDimensions[] = {
        1
      };

      static int_T rt_LoggedOutputDimensions[] = {
        1
      };

      static boolean_T rt_LoggedOutputIsVarDims[] = {
        0
      };

      static void* rt_LoggedCurrentSignalDimensions[] = {
        (NULL)
      };

      static int_T rt_LoggedCurrentSignalDimensionsSize[] = {
        4
      };

      static BuiltInDTypeId rt_LoggedOutputDataTypeIds[] = {
        SS_DOUBLE
      };

      static int_T rt_LoggedOutputComplexSignals[] = {
        0
      };

      static RTWPreprocessingFcnPtr rt_LoggingPreprocessingFcnPtrs[] = {
        (NULL)
      };

      static const char_T *rt_LoggedOutputLabels[] = {
        "" };

      static const char_T *rt_LoggedOutputBlockNames[] = {
        "LaneKeeping/Additinal Torque" };

      static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert[] = {
        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 }
      };

      static RTWLogSignalInfo rt_LoggedOutputSignalInfo[] = {
        {
          1,
          rt_LoggedOutputWidths,
          rt_LoggedOutputNumDimensions,
          rt_LoggedOutputDimensions,
          rt_LoggedOutputIsVarDims,
          rt_LoggedCurrentSignalDimensions,
          rt_LoggedCurrentSignalDimensionsSize,
          rt_LoggedOutputDataTypeIds,
          rt_LoggedOutputComplexSignals,
          (NULL),
          rt_LoggingPreprocessingFcnPtrs,

          { rt_LoggedOutputLabels },
          (NULL),
          (NULL),
          (NULL),

          { rt_LoggedOutputBlockNames },

          { (NULL) },
          (NULL),
          rt_RTWLogDataTypeConvert
        }
      };

      rtliSetLogYSignalInfo(LaneKeeping_M->rtwLogInfo, rt_LoggedOutputSignalInfo);

      /* set currSigDims field */
      rt_LoggedCurrentSignalDimensions[0] = &rt_LoggedOutputWidths[0];
    }

    rtliSetLogY(LaneKeeping_M->rtwLogInfo, "yout");
  }

  /* block I/O */
  (void) memset(((void *) &LaneKeeping_B), 0,
                sizeof(B_LaneKeeping_T));

  /* states (dwork) */
  (void) memset((void *)&LaneKeeping_DW, 0,
                sizeof(DW_LaneKeeping_T));
  LaneKeeping_DW.Time = 0.0;

  /* external inputs */
  LaneKeeping_U.LateralShift = 0.0;

  /* external outputs */
  LaneKeeping_Y.AdditinalTorque = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(LaneKeeping_M->rtwLogInfo, 0.0, rtmGetTFinal
    (LaneKeeping_M), LaneKeeping_M->Timing.stepSize0, (&rtmGetErrorStatus
    (LaneKeeping_M)));

  /* Start for DataStoreMemory: '<S2>/Data Store Memory1' */
  LaneKeeping_DW.Time = LaneKeeping_P.DataStoreMemory1_InitialValue;

  /* InitializeConditions for UnitDelay: '<S4>/Delay Input1' */
  LaneKeeping_DW.DelayInput1_DSTATE = LaneKeeping_P.DetIncAct_vinit;
}

/* Model terminate function */
void LaneKeeping_terminate(void)
{
  /* (no terminate code required) */
}

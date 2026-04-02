/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: tv_controller_2_1.c
 *
 * Code generated for Simulink model 'tv_controller_2_1'.
 *
 * Model version                  : 1.279
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Wed Apr  1 20:38:13 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "tv_controller_2_1.h"
#include <math.h>
#include "rtwtypes.h"
#include "tv_controller_2_1_private.h"

/* Block states (default storage) */
DW_tv_controller_2_1_T tv_controller_2_1_DW;

/* External inputs (root inport signals with default storage) */
ExtU_tv_controller_2_1_T tv_controller_2_1_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_tv_controller_2_1_T tv_controller_2_1_Y;

/* Real-time model */
static RT_MODEL_tv_controller_2_1_T tv_controller_2_1_M_;
RT_MODEL_tv_controller_2_1_T *const tv_controller_2_1_M = &tv_controller_2_1_M_;

/* Lookup Binary Search Utility BINARYSEARCH_real_T */
void BINARYSEARCH_real_T(uint32_T *piLeft, uint32_T *piRght, real_T u, const
  real_T *pData, uint32_T iHi)
{
  /* Find the location of current input value in the data table. */
  *piLeft = 0U;
  *piRght = iHi;
  if (u <= pData[0] ) {
    /* Less than or equal to the smallest point in the table. */
    *piRght = 0U;
  } else if (u >= pData[iHi] ) {
    /* Greater than or equal to the largest point in the table. */
    *piLeft = iHi;
  } else {
    uint32_T i;

    /* Do a binary search. */
    while (( *piRght - *piLeft ) > 1U ) {
      /* Get the average of the left and right indices using to Floor rounding. */
      i = (*piLeft + *piRght) >> 1;

      /* Move either the right index or the left index so that */
      /*  LeftDataPoint <= CurrentValue < RightDataPoint */
      if (u < pData[i] ) {
        *piRght = i;
      } else {
        *piLeft = i;
      }
    }
  }
}

/* Lookup Utility LookUp_real_T_real_T */
void LookUp_real_T_real_T(real_T *pY, const real_T *pYData, real_T u, const
  real_T *pUData, uint32_T iHi)
{
  uint32_T iLeft;
  uint32_T iRght;
  BINARYSEARCH_real_T( &(iLeft), &(iRght), u, pUData, iHi);

  {
    real_T lambda;
    if (pUData[iRght] > pUData[iLeft] ) {
      real_T num;
      real_T den;
      den = pUData[iRght];
      den -= pUData[iLeft];
      num = u;
      num -= pUData[iLeft];
      lambda = num / den;
    } else {
      lambda = 0.0;
    }

    {
      real_T yLeftCast;
      real_T yRghtCast;
      yLeftCast = pYData[iLeft];
      yRghtCast = pYData[iRght];
      yLeftCast += lambda * ( yRghtCast - yLeftCast );
      (*pY) = yLeftCast;
    }
  }
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/* Model step function */
void tv_controller_2_1_step(void)
{
  real_T rtb_Integrator;
  real_T rtb_PProdOut;
  real_T rtb_PProdOut_a;
  real_T rtb_Reset;
  real_T rtb_Sum;
  int32_T j;

  /* Lookup_n-D: '<Root>/thrust (lbf) to pom (psi)' incorporates:
   *  Inport: '<Root>/thrust_lbf_set_inport'
   */
  tv_controller_2_1_Y.pom_psi_set_outport = look1_binlxpw
    (tv_controller_2_1_U.thrust_lbf_set_inport,
     tv_controller_2_1_ConstP.thrustlbftopompsi_bp01Data,
     tv_controller_2_1_ConstP.thrustlbftopompsi_tableData, 3U);

  /* DiscreteFir: '<Root>/Discrete FIR Filter1' incorporates:
   *  Inport: '<Root>/pom_psi_inport'
   */
  rtb_PProdOut_a = tv_controller_2_1_U.pom_psi_inport * 0.1;
  for (j = tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf; j < 9; j++) {
    rtb_PProdOut_a += tv_controller_2_1_DW.DiscreteFIRFilter1_states[j] * 0.1;
  }

  for (j = 0; j < tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf; j++) {
    rtb_PProdOut_a += tv_controller_2_1_DW.DiscreteFIRFilter1_states[j] * 0.1;
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  DiscreteFir: '<Root>/Discrete FIR Filter1'
   */
  rtb_Sum = tv_controller_2_1_Y.pom_psi_set_outport - rtb_PProdOut_a;

  /* Switch: '<S108>/Reset' incorporates:
   *  Inport: '<Root>/thrust_lbf_set_inport'
   *  MinMax: '<S5>/MinMax'
   */
  rtb_Reset = fmax(tv_controller_2_1_U.thrust_lbf_set_inport,
                   tv_controller_2_1_DW.Memory_PreviousInput);

  /* S-Function (sfix_look1_dyn): '<S120>/Gain Dynamic Lookup' incorporates:
   *  Constant: '<S7>/Internal breakpoints'
   *  DataStoreRead: '<S120>/Data Store Read'
   */
  /* Dynamic Look-Up Table Block: '<S120>/Gain Dynamic Lookup'
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Input2  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_real_T_real_T( &(tv_controller_2_1_DW.Memory_PreviousInput),
                       &tv_controller_2_1_DW.Kp_LOX[0], rtb_Reset,
                       tv_controller_2_1_ConstP.pooled1, 3U);

  /* Product: '<S96>/PProd Out' */
  rtb_PProdOut = rtb_Sum * tv_controller_2_1_DW.Memory_PreviousInput;

  /* DiscreteIntegrator: '<S91>/Integrator' */
  tv_controller_2_1_DW.Memory_PreviousInput =
    tv_controller_2_1_DW.Integrator_DSTATE;

  /* Product: '<Root>/O//F MR' incorporates:
   *  Sum: '<S100>/Sum'
   */
  tv_controller_2_1_Y.mr_outport = rtb_PProdOut +
    tv_controller_2_1_DW.Memory_PreviousInput;

  /* Saturate: '<S98>/Saturation' */
  if (tv_controller_2_1_Y.mr_outport > 90.0) {
    /* Product: '<Root>/O//F MR' */
    tv_controller_2_1_Y.mr_outport = 90.0;
  } else if (tv_controller_2_1_Y.mr_outport < 0.0) {
    /* Product: '<Root>/O//F MR' */
    tv_controller_2_1_Y.mr_outport = 0.0;
  }

  /* End of Saturate: '<S98>/Saturation' */

  /* RateLimiter: '<Root>/rate limiter' */
  rtb_PProdOut = tv_controller_2_1_Y.mr_outport - tv_controller_2_1_DW.PrevY;
  if (rtb_PProdOut > 0.3529411764705882) {
    /* Product: '<Root>/O//F MR' */
    tv_controller_2_1_Y.mr_outport = tv_controller_2_1_DW.PrevY +
      0.3529411764705882;
  } else if (rtb_PProdOut < -0.3529411764705882) {
    /* Product: '<Root>/O//F MR' */
    tv_controller_2_1_Y.mr_outport = tv_controller_2_1_DW.PrevY -
      0.3529411764705882;
  }

  tv_controller_2_1_DW.PrevY = tv_controller_2_1_Y.mr_outport;

  /* End of RateLimiter: '<Root>/rate limiter' */

  /* Outport: '<Root>/lox_deg_outport' */
  tv_controller_2_1_Y.lox_deg_outport = tv_controller_2_1_Y.mr_outport;

  /* DiscreteFir: '<Root>/Discrete FIR Filter2' incorporates:
   *  Inport: '<Root>/pc_psi_inport'
   */
  rtb_PProdOut = tv_controller_2_1_U.pc_psi_inport * 0.1;
  for (j = tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf; j < 9; j++) {
    rtb_PProdOut += tv_controller_2_1_DW.DiscreteFIRFilter2_states[j] * 0.1;
  }

  for (j = 0; j < tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf; j++) {
    rtb_PProdOut += tv_controller_2_1_DW.DiscreteFIRFilter2_states[j] * 0.1;
  }

  /* MATLAB Function: '<Root>/LOX_mdot' incorporates:
   *  DiscreteFir: '<Root>/Discrete FIR Filter1'
   *  DiscreteFir: '<Root>/Discrete FIR Filter2'
   */
  tv_controller_2_1_Y.lox_mdot_kgs_outport = sqrt((rtb_PProdOut_a - rtb_PProdOut)
    * 1.504292293924E+7) * 2.58E-5;

  /* DiscreteFir: '<Root>/Discrete FIR Filter' incorporates:
   *  Inport: '<Root>/pfm_psi_inport'
   */
  rtb_PProdOut_a = tv_controller_2_1_U.pfm_psi_inport * 0.1;
  for (j = tv_controller_2_1_DW.DiscreteFIRFilter_circBuf; j < 9; j++) {
    rtb_PProdOut_a += tv_controller_2_1_DW.DiscreteFIRFilter_states[j] * 0.1;
  }

  for (j = 0; j < tv_controller_2_1_DW.DiscreteFIRFilter_circBuf; j++) {
    rtb_PProdOut_a += tv_controller_2_1_DW.DiscreteFIRFilter_states[j] * 0.1;
  }

  /* MATLAB Function: '<Root>/IPA_mdot' incorporates:
   *  DiscreteFir: '<Root>/Discrete FIR Filter'
   *  DiscreteFir: '<Root>/Discrete FIR Filter2'
   */
  tv_controller_2_1_Y.ipa_mdot_kgs_outport = sqrt((rtb_PProdOut_a - rtb_PProdOut)
    * 1.08260448331E+7) * 2.53E-5;

  /* Product: '<Root>/O//F MR' */
  tv_controller_2_1_Y.mr_outport = tv_controller_2_1_Y.lox_mdot_kgs_outport /
    tv_controller_2_1_Y.ipa_mdot_kgs_outport;

  /* S-Function (sfix_look1_dyn): '<S114>/Gain Dynamic Lookup' incorporates:
   *  Constant: '<S6>/Internal breakpoints'
   *  DataStoreRead: '<S114>/Data Store Read'
   */
  /* Dynamic Look-Up Table Block: '<S114>/Gain Dynamic Lookup'
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Input2  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_real_T_real_T( &(rtb_Integrator), &tv_controller_2_1_DW.Kp_IPA[0],
                       rtb_Reset, tv_controller_2_1_ConstP.pooled1, 3U);

  /* Product: '<S46>/PProd Out' incorporates:
   *  Constant: '<Root>/overall MR set'
   *  Sum: '<Root>/Sum3'
   */
  rtb_PProdOut_a = (1.2 - tv_controller_2_1_Y.mr_outport) * rtb_Integrator;

  /* DiscreteIntegrator: '<S41>/Integrator' */
  rtb_Integrator = tv_controller_2_1_DW.Integrator_DSTATE_c;

  /* Sum: '<S50>/Sum' */
  rtb_Integrator += rtb_PProdOut_a;

  /* Saturate: '<S48>/Saturation' */
  if (rtb_Integrator > 90.0) {
    rtb_Integrator = 90.0;
  } else if (rtb_Integrator < 0.0) {
    rtb_Integrator = 0.0;
  }

  /* End of Saturate: '<S48>/Saturation' */

  /* RateLimiter: '<Root>/rate limiter IPA' */
  rtb_PProdOut = rtb_Integrator - tv_controller_2_1_DW.PrevY_j;
  if (rtb_PProdOut > 0.3529411764705882) {
    /* RateLimiter: '<Root>/rate limiter IPA' */
    tv_controller_2_1_DW.PrevY_j += 0.3529411764705882;
  } else if (rtb_PProdOut < -0.3529411764705882) {
    /* RateLimiter: '<Root>/rate limiter IPA' */
    tv_controller_2_1_DW.PrevY_j -= 0.3529411764705882;
  } else {
    /* RateLimiter: '<Root>/rate limiter IPA' */
    tv_controller_2_1_DW.PrevY_j = rtb_Integrator;
  }

  /* End of RateLimiter: '<Root>/rate limiter IPA' */

  /* Outport: '<Root>/ipa_deg_outport' */
  tv_controller_2_1_Y.ipa_deg_outport = tv_controller_2_1_DW.PrevY_j;

  /* Outport: '<Root>/thrust_lbf_est_outport' incorporates:
   *  Gain: '<Root>/Gain7'
   *  Gain: '<Root>/Gain8'
   *  Sum: '<Root>/Sum4'
   */
  tv_controller_2_1_Y.thrust_lbf_est_outport =
    (tv_controller_2_1_Y.ipa_mdot_kgs_outport +
     tv_controller_2_1_Y.lox_mdot_kgs_outport) * 1752.461343 *
    0.22482014388489208;

  /* S-Function (sfix_look1_dyn): '<S113>/Gain Dynamic Lookup' incorporates:
   *  Constant: '<S6>/Internal breakpoints'
   *  DataStoreRead: '<S113>/Data Store Read'
   */
  /* Dynamic Look-Up Table Block: '<S113>/Gain Dynamic Lookup'
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Input2  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_real_T_real_T( &(rtb_Integrator), &tv_controller_2_1_DW.Ki_IPA[0],
                       rtb_Reset, tv_controller_2_1_ConstP.pooled1, 3U);

  /* Product: '<S38>/IProd Out' incorporates:
   *  Constant: '<Root>/overall MR set'
   *  Sum: '<Root>/Sum3'
   */
  rtb_PProdOut_a = (1.2 - tv_controller_2_1_Y.mr_outport) * rtb_Integrator;

  /* S-Function (sfix_look1_dyn): '<S119>/Gain Dynamic Lookup' incorporates:
   *  Constant: '<S7>/Internal breakpoints'
   *  DataStoreRead: '<S119>/Data Store Read'
   */
  /* Dynamic Look-Up Table Block: '<S119>/Gain Dynamic Lookup'
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Input2  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_real_T_real_T( &(rtb_Integrator), &tv_controller_2_1_DW.Ki_LOX[0],
                       rtb_Reset, tv_controller_2_1_ConstP.pooled1, 3U);

  /* Update for DiscreteFir: '<Root>/Discrete FIR Filter1' incorporates:
   *  Inport: '<Root>/pom_psi_inport'
   */
  /* Update circular buffer index */
  tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf--;
  if (tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf < 0) {
    tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf = 8;
  }

  /* Update circular buffer */
  tv_controller_2_1_DW.DiscreteFIRFilter1_states[tv_controller_2_1_DW.DiscreteFIRFilter1_circBuf]
    = tv_controller_2_1_U.pom_psi_inport;

  /* End of Update for DiscreteFir: '<Root>/Discrete FIR Filter1' */

  /* Update for DiscreteIntegrator: '<S91>/Integrator' incorporates:
   *  Memory: '<S108>/Memory'
   */
  tv_controller_2_1_DW.Memory_PreviousInput = rtb_Reset;

  /* Update for DiscreteIntegrator: '<S91>/Integrator' incorporates:
   *  Product: '<S88>/IProd Out'
   */
  tv_controller_2_1_DW.Integrator_DSTATE += rtb_Sum * rtb_Integrator *
    0.0058823529411764705;

  /* Update for DiscreteFir: '<Root>/Discrete FIR Filter2' incorporates:
   *  Inport: '<Root>/pc_psi_inport'
   */
  /* Update circular buffer index */
  tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf--;
  if (tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf < 0) {
    tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf = 8;
  }

  /* Update circular buffer */
  tv_controller_2_1_DW.DiscreteFIRFilter2_states[tv_controller_2_1_DW.DiscreteFIRFilter2_circBuf]
    = tv_controller_2_1_U.pc_psi_inport;

  /* End of Update for DiscreteFir: '<Root>/Discrete FIR Filter2' */

  /* Update for DiscreteFir: '<Root>/Discrete FIR Filter' incorporates:
   *  Inport: '<Root>/pfm_psi_inport'
   */
  /* Update circular buffer index */
  tv_controller_2_1_DW.DiscreteFIRFilter_circBuf--;
  if (tv_controller_2_1_DW.DiscreteFIRFilter_circBuf < 0) {
    tv_controller_2_1_DW.DiscreteFIRFilter_circBuf = 8;
  }

  /* Update circular buffer */
  tv_controller_2_1_DW.DiscreteFIRFilter_states[tv_controller_2_1_DW.DiscreteFIRFilter_circBuf]
    = tv_controller_2_1_U.pfm_psi_inport;

  /* End of Update for DiscreteFir: '<Root>/Discrete FIR Filter' */

  /* Update for DiscreteIntegrator: '<S41>/Integrator' */
  tv_controller_2_1_DW.Integrator_DSTATE_c += 0.0058823529411764705 *
    rtb_PProdOut_a;
}

/* Model initialize function */
void tv_controller_2_1_initialize(void)
{
  /* Start for DataStoreMemory generated from: '<S113>/Data Store Read' */
  tv_controller_2_1_DW.Ki_IPA[0] = -119.81414;

  /* Start for DataStoreMemory generated from: '<S119>/Data Store Read' */
  tv_controller_2_1_DW.Ki_LOX[0] = 0.2814616224;

  /* Start for DataStoreMemory generated from: '<S114>/Data Store Read' */
  tv_controller_2_1_DW.Kp_IPA[0] = -73.38819058;

  /* Start for DataStoreMemory generated from: '<S120>/Data Store Read' */
  tv_controller_2_1_DW.Kp_LOX[0] = 0.1761712538;

  /* Start for DataStoreMemory generated from: '<S113>/Data Store Read' */
  tv_controller_2_1_DW.Ki_IPA[1] = -60.19250076;

  /* Start for DataStoreMemory generated from: '<S119>/Data Store Read' */
  tv_controller_2_1_DW.Ki_LOX[1] = 0.315;

  /* Start for DataStoreMemory generated from: '<S114>/Data Store Read' */
  tv_controller_2_1_DW.Kp_IPA[1] = -36.86892648;

  /* Start for DataStoreMemory generated from: '<S120>/Data Store Read' */
  tv_controller_2_1_DW.Kp_LOX[1] = 0.1964983295;

  /* Start for DataStoreMemory generated from: '<S113>/Data Store Read' */
  tv_controller_2_1_DW.Ki_IPA[2] = -119.81414;

  /* Start for DataStoreMemory generated from: '<S119>/Data Store Read' */
  tv_controller_2_1_DW.Ki_LOX[2] = 0.757;

  /* Start for DataStoreMemory generated from: '<S114>/Data Store Read' */
  tv_controller_2_1_DW.Kp_IPA[2] = -73.38819058;

  /* Start for DataStoreMemory generated from: '<S120>/Data Store Read' */
  tv_controller_2_1_DW.Kp_LOX[2] = 0.4571218056;

  /* Start for DataStoreMemory generated from: '<S113>/Data Store Read' */
  tv_controller_2_1_DW.Ki_IPA[3] = -446.115062;

  /* Start for DataStoreMemory generated from: '<S119>/Data Store Read' */
  tv_controller_2_1_DW.Ki_LOX[3] = 3.44;

  /* Start for DataStoreMemory generated from: '<S114>/Data Store Read' */
  tv_controller_2_1_DW.Kp_IPA[3] = -273.2530334;

  /* Start for DataStoreMemory generated from: '<S120>/Data Store Read' */
  tv_controller_2_1_DW.Kp_LOX[3] = 2.025919193;

  /* InitializeConditions for DiscreteIntegrator: '<S91>/Integrator' */
  tv_controller_2_1_DW.Integrator_DSTATE = 41.06017851;

  /* InitializeConditions for RateLimiter: '<Root>/rate limiter' */
  tv_controller_2_1_DW.PrevY = 41.06017851;

  /* InitializeConditions for DiscreteIntegrator: '<S41>/Integrator' */
  tv_controller_2_1_DW.Integrator_DSTATE_c = 40.67697413;

  /* InitializeConditions for RateLimiter: '<Root>/rate limiter IPA' */
  tv_controller_2_1_DW.PrevY_j = 40.67697413;
}

/* Model terminate function */
void tv_controller_2_1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

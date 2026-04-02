/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: tv_controller_2_1_private.h
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

#ifndef tv_controller_2_1_private_h_
#define tv_controller_2_1_private_h_
#include "rtwtypes.h"
#include "tv_controller_2_1_types.h"
#include "tv_controller_2_1.h"

void BINARYSEARCH_real_T(uint32_T *piLeft, uint32_T *piRght, real_T u, const
  real_T *pData, uint32_T iHi);
void LookUp_real_T_real_T(real_T *pY, const real_T *pYData, real_T u, const
  real_T *pUData, uint32_T iHi);
extern real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);

#endif                                 /* tv_controller_2_1_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

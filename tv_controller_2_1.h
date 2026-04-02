/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: tv_controller_2_1.h
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

#ifndef tv_controller_2_1_h_
#define tv_controller_2_1_h_
#ifndef tv_controller_2_1_COMMON_INCLUDES_
#define tv_controller_2_1_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* tv_controller_2_1_COMMON_INCLUDES_ */

#include "tv_controller_2_1_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteFIRFilter1_states[9]; /* '<Root>/Discrete FIR Filter1' */
  real_T Integrator_DSTATE;            /* '<S91>/Integrator' */
  real_T DiscreteFIRFilter2_states[9]; /* '<Root>/Discrete FIR Filter2' */
  real_T DiscreteFIRFilter_states[9];  /* '<Root>/Discrete FIR Filter' */
  real_T Integrator_DSTATE_c;          /* '<S41>/Integrator' */
  real_T DiscreteFIRFilter1_simContextBu[18];/* '<Root>/Discrete FIR Filter1' */
  real_T DiscreteFIRFilter1_simRevCoeff[10];/* '<Root>/Discrete FIR Filter1' */
  real_T Memory_PreviousInput;         /* '<S108>/Memory' */
  real_T PrevY;                        /* '<Root>/rate limiter' */
  real_T DiscreteFIRFilter2_simContextBu[18];/* '<Root>/Discrete FIR Filter2' */
  real_T DiscreteFIRFilter2_simRevCoeff[10];/* '<Root>/Discrete FIR Filter2' */
  real_T DiscreteFIRFilter_simContextBuf[18];/* '<Root>/Discrete FIR Filter' */
  real_T DiscreteFIRFilter_simRevCoeff[10];/* '<Root>/Discrete FIR Filter' */
  real_T PrevY_j;                      /* '<Root>/rate limiter IPA' */
  real_T Ki_IPA[4];                    /* synthesized block */
  real_T Ki_LOX[4];                    /* synthesized block */
  real_T Kp_IPA[4];                    /* synthesized block */
  real_T Kp_LOX[4];                    /* synthesized block */
  int32_T DiscreteFIRFilter1_circBuf;  /* '<Root>/Discrete FIR Filter1' */
  int32_T DiscreteFIRFilter2_circBuf;  /* '<Root>/Discrete FIR Filter2' */
  int32_T DiscreteFIRFilter_circBuf;   /* '<Root>/Discrete FIR Filter' */
} DW_tv_controller_2_1_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: GainArrayBreakpoints)
   * Referenced by:
   *   '<S6>/Internal breakpoints'
   *   '<S7>/Internal breakpoints'
   */
  real_T pooled1[4];

  /* Expression: [0;333.6294;408.8458;487.1441]
   * Referenced by: '<Root>/thrust (lbf) to pom (psi)'
   */
  real_T thrustlbftopompsi_tableData[4];

  /* Expression: [0;450;550;650]
   * Referenced by: '<Root>/thrust (lbf) to pom (psi)'
   */
  real_T thrustlbftopompsi_bp01Data[4];
} ConstP_tv_controller_2_1_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T thrust_lbf_set_inport;        /* '<Root>/thrust_lbf_set_inport' */
  real_T pom_psi_inport;               /* '<Root>/pom_psi_inport' */
  real_T pc_psi_inport;                /* '<Root>/pc_psi_inport' */
  real_T pfm_psi_inport;               /* '<Root>/pfm_psi_inport' */
} ExtU_tv_controller_2_1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T lox_deg_outport;              /* '<Root>/lox_deg_outport' */
  real_T ipa_deg_outport;              /* '<Root>/ipa_deg_outport' */
  real_T lox_mdot_kgs_outport;         /* '<Root>/lox_mdot_kgs_outport' */
  real_T ipa_mdot_kgs_outport;         /* '<Root>/ipa_mdot_kgs_outport' */
  real_T mr_outport;                   /* '<Root>/mr_outport' */
  real_T thrust_lbf_est_outport;       /* '<Root>/thrust_lbf_est_outport' */
  real_T pom_psi_set_outport;          /* '<Root>/pom_psi_set_outport' */
} ExtY_tv_controller_2_1_T;

/* Real-time Model Data Structure */
struct tag_RTM_tv_controller_2_1_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_tv_controller_2_1_T tv_controller_2_1_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_tv_controller_2_1_T tv_controller_2_1_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_tv_controller_2_1_T tv_controller_2_1_Y;

/* Constant parameters (default storage) */
extern const ConstP_tv_controller_2_1_T tv_controller_2_1_ConstP;

/* Model entry point functions */
extern void tv_controller_2_1_initialize(void);
extern void tv_controller_2_1_step(void);
extern void tv_controller_2_1_terminate(void);

/* Real-time Model object */
extern RT_MODEL_tv_controller_2_1_T *const tv_controller_2_1_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/MR' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S108>/FixPt Data Type Duplicate2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope9' : Unused code path elimination
 * Block '<Root>/resultant thrust [lbf]' : Unused code path elimination
 * Block '<Root>/Constant' : Unused code path elimination
 * Block '<S108>/Initial Condition' : Unused code path elimination
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
 * '<Root>' : 'tv_controller_2_1'
 * '<S1>'   : 'tv_controller_2_1/IPA PI controller'
 * '<S2>'   : 'tv_controller_2_1/IPA_mdot'
 * '<S3>'   : 'tv_controller_2_1/LOX PI controller'
 * '<S4>'   : 'tv_controller_2_1/LOX_mdot'
 * '<S5>'   : 'tv_controller_2_1/MinMax Running Resettable'
 * '<S6>'   : 'tv_controller_2_1/PID Gain Scheduler'
 * '<S7>'   : 'tv_controller_2_1/PID Gain Scheduler1'
 * '<S8>'   : 'tv_controller_2_1/IPA PI controller/Anti-windup'
 * '<S9>'   : 'tv_controller_2_1/IPA PI controller/D Gain'
 * '<S10>'  : 'tv_controller_2_1/IPA PI controller/External Derivative'
 * '<S11>'  : 'tv_controller_2_1/IPA PI controller/Filter'
 * '<S12>'  : 'tv_controller_2_1/IPA PI controller/Filter ICs'
 * '<S13>'  : 'tv_controller_2_1/IPA PI controller/I Gain'
 * '<S14>'  : 'tv_controller_2_1/IPA PI controller/Ideal P Gain'
 * '<S15>'  : 'tv_controller_2_1/IPA PI controller/Ideal P Gain Fdbk'
 * '<S16>'  : 'tv_controller_2_1/IPA PI controller/Integrator'
 * '<S17>'  : 'tv_controller_2_1/IPA PI controller/Integrator ICs'
 * '<S18>'  : 'tv_controller_2_1/IPA PI controller/N Copy'
 * '<S19>'  : 'tv_controller_2_1/IPA PI controller/N Gain'
 * '<S20>'  : 'tv_controller_2_1/IPA PI controller/P Copy'
 * '<S21>'  : 'tv_controller_2_1/IPA PI controller/Parallel P Gain'
 * '<S22>'  : 'tv_controller_2_1/IPA PI controller/Reset Signal'
 * '<S23>'  : 'tv_controller_2_1/IPA PI controller/Saturation'
 * '<S24>'  : 'tv_controller_2_1/IPA PI controller/Saturation Fdbk'
 * '<S25>'  : 'tv_controller_2_1/IPA PI controller/Sum'
 * '<S26>'  : 'tv_controller_2_1/IPA PI controller/Sum Fdbk'
 * '<S27>'  : 'tv_controller_2_1/IPA PI controller/Tracking Mode'
 * '<S28>'  : 'tv_controller_2_1/IPA PI controller/Tracking Mode Sum'
 * '<S29>'  : 'tv_controller_2_1/IPA PI controller/Tsamp - Integral'
 * '<S30>'  : 'tv_controller_2_1/IPA PI controller/Tsamp - Ngain'
 * '<S31>'  : 'tv_controller_2_1/IPA PI controller/postSat Signal'
 * '<S32>'  : 'tv_controller_2_1/IPA PI controller/preSat Signal'
 * '<S33>'  : 'tv_controller_2_1/IPA PI controller/Anti-windup/Passthrough'
 * '<S34>'  : 'tv_controller_2_1/IPA PI controller/D Gain/Disabled'
 * '<S35>'  : 'tv_controller_2_1/IPA PI controller/External Derivative/Disabled'
 * '<S36>'  : 'tv_controller_2_1/IPA PI controller/Filter/Disabled'
 * '<S37>'  : 'tv_controller_2_1/IPA PI controller/Filter ICs/Disabled'
 * '<S38>'  : 'tv_controller_2_1/IPA PI controller/I Gain/External Parameters'
 * '<S39>'  : 'tv_controller_2_1/IPA PI controller/Ideal P Gain/Passthrough'
 * '<S40>'  : 'tv_controller_2_1/IPA PI controller/Ideal P Gain Fdbk/Disabled'
 * '<S41>'  : 'tv_controller_2_1/IPA PI controller/Integrator/Discrete'
 * '<S42>'  : 'tv_controller_2_1/IPA PI controller/Integrator ICs/Internal IC'
 * '<S43>'  : 'tv_controller_2_1/IPA PI controller/N Copy/Disabled wSignal Specification'
 * '<S44>'  : 'tv_controller_2_1/IPA PI controller/N Gain/Disabled'
 * '<S45>'  : 'tv_controller_2_1/IPA PI controller/P Copy/Disabled'
 * '<S46>'  : 'tv_controller_2_1/IPA PI controller/Parallel P Gain/External Parameters'
 * '<S47>'  : 'tv_controller_2_1/IPA PI controller/Reset Signal/Disabled'
 * '<S48>'  : 'tv_controller_2_1/IPA PI controller/Saturation/Enabled'
 * '<S49>'  : 'tv_controller_2_1/IPA PI controller/Saturation Fdbk/Disabled'
 * '<S50>'  : 'tv_controller_2_1/IPA PI controller/Sum/Sum_PI'
 * '<S51>'  : 'tv_controller_2_1/IPA PI controller/Sum Fdbk/Disabled'
 * '<S52>'  : 'tv_controller_2_1/IPA PI controller/Tracking Mode/Disabled'
 * '<S53>'  : 'tv_controller_2_1/IPA PI controller/Tracking Mode Sum/Passthrough'
 * '<S54>'  : 'tv_controller_2_1/IPA PI controller/Tsamp - Integral/TsSignalSpecification'
 * '<S55>'  : 'tv_controller_2_1/IPA PI controller/Tsamp - Ngain/Passthrough'
 * '<S56>'  : 'tv_controller_2_1/IPA PI controller/postSat Signal/Forward_Path'
 * '<S57>'  : 'tv_controller_2_1/IPA PI controller/preSat Signal/Forward_Path'
 * '<S58>'  : 'tv_controller_2_1/LOX PI controller/Anti-windup'
 * '<S59>'  : 'tv_controller_2_1/LOX PI controller/D Gain'
 * '<S60>'  : 'tv_controller_2_1/LOX PI controller/External Derivative'
 * '<S61>'  : 'tv_controller_2_1/LOX PI controller/Filter'
 * '<S62>'  : 'tv_controller_2_1/LOX PI controller/Filter ICs'
 * '<S63>'  : 'tv_controller_2_1/LOX PI controller/I Gain'
 * '<S64>'  : 'tv_controller_2_1/LOX PI controller/Ideal P Gain'
 * '<S65>'  : 'tv_controller_2_1/LOX PI controller/Ideal P Gain Fdbk'
 * '<S66>'  : 'tv_controller_2_1/LOX PI controller/Integrator'
 * '<S67>'  : 'tv_controller_2_1/LOX PI controller/Integrator ICs'
 * '<S68>'  : 'tv_controller_2_1/LOX PI controller/N Copy'
 * '<S69>'  : 'tv_controller_2_1/LOX PI controller/N Gain'
 * '<S70>'  : 'tv_controller_2_1/LOX PI controller/P Copy'
 * '<S71>'  : 'tv_controller_2_1/LOX PI controller/Parallel P Gain'
 * '<S72>'  : 'tv_controller_2_1/LOX PI controller/Reset Signal'
 * '<S73>'  : 'tv_controller_2_1/LOX PI controller/Saturation'
 * '<S74>'  : 'tv_controller_2_1/LOX PI controller/Saturation Fdbk'
 * '<S75>'  : 'tv_controller_2_1/LOX PI controller/Sum'
 * '<S76>'  : 'tv_controller_2_1/LOX PI controller/Sum Fdbk'
 * '<S77>'  : 'tv_controller_2_1/LOX PI controller/Tracking Mode'
 * '<S78>'  : 'tv_controller_2_1/LOX PI controller/Tracking Mode Sum'
 * '<S79>'  : 'tv_controller_2_1/LOX PI controller/Tsamp - Integral'
 * '<S80>'  : 'tv_controller_2_1/LOX PI controller/Tsamp - Ngain'
 * '<S81>'  : 'tv_controller_2_1/LOX PI controller/postSat Signal'
 * '<S82>'  : 'tv_controller_2_1/LOX PI controller/preSat Signal'
 * '<S83>'  : 'tv_controller_2_1/LOX PI controller/Anti-windup/Passthrough'
 * '<S84>'  : 'tv_controller_2_1/LOX PI controller/D Gain/Disabled'
 * '<S85>'  : 'tv_controller_2_1/LOX PI controller/External Derivative/Disabled'
 * '<S86>'  : 'tv_controller_2_1/LOX PI controller/Filter/Disabled'
 * '<S87>'  : 'tv_controller_2_1/LOX PI controller/Filter ICs/Disabled'
 * '<S88>'  : 'tv_controller_2_1/LOX PI controller/I Gain/External Parameters'
 * '<S89>'  : 'tv_controller_2_1/LOX PI controller/Ideal P Gain/Passthrough'
 * '<S90>'  : 'tv_controller_2_1/LOX PI controller/Ideal P Gain Fdbk/Disabled'
 * '<S91>'  : 'tv_controller_2_1/LOX PI controller/Integrator/Discrete'
 * '<S92>'  : 'tv_controller_2_1/LOX PI controller/Integrator ICs/Internal IC'
 * '<S93>'  : 'tv_controller_2_1/LOX PI controller/N Copy/Disabled wSignal Specification'
 * '<S94>'  : 'tv_controller_2_1/LOX PI controller/N Gain/Disabled'
 * '<S95>'  : 'tv_controller_2_1/LOX PI controller/P Copy/Disabled'
 * '<S96>'  : 'tv_controller_2_1/LOX PI controller/Parallel P Gain/External Parameters'
 * '<S97>'  : 'tv_controller_2_1/LOX PI controller/Reset Signal/Disabled'
 * '<S98>'  : 'tv_controller_2_1/LOX PI controller/Saturation/Enabled'
 * '<S99>'  : 'tv_controller_2_1/LOX PI controller/Saturation Fdbk/Disabled'
 * '<S100>' : 'tv_controller_2_1/LOX PI controller/Sum/Sum_PI'
 * '<S101>' : 'tv_controller_2_1/LOX PI controller/Sum Fdbk/Disabled'
 * '<S102>' : 'tv_controller_2_1/LOX PI controller/Tracking Mode/Disabled'
 * '<S103>' : 'tv_controller_2_1/LOX PI controller/Tracking Mode Sum/Passthrough'
 * '<S104>' : 'tv_controller_2_1/LOX PI controller/Tsamp - Integral/TsSignalSpecification'
 * '<S105>' : 'tv_controller_2_1/LOX PI controller/Tsamp - Ngain/Passthrough'
 * '<S106>' : 'tv_controller_2_1/LOX PI controller/postSat Signal/Forward_Path'
 * '<S107>' : 'tv_controller_2_1/LOX PI controller/preSat Signal/Forward_Path'
 * '<S108>' : 'tv_controller_2_1/MinMax Running Resettable/Subsystem'
 * '<S109>' : 'tv_controller_2_1/PID Gain Scheduler/DGain Lookup'
 * '<S110>' : 'tv_controller_2_1/PID Gain Scheduler/IGain Lookup'
 * '<S111>' : 'tv_controller_2_1/PID Gain Scheduler/NGain Lookup'
 * '<S112>' : 'tv_controller_2_1/PID Gain Scheduler/PGain Lookup'
 * '<S113>' : 'tv_controller_2_1/PID Gain Scheduler/IGain Lookup/Enabled'
 * '<S114>' : 'tv_controller_2_1/PID Gain Scheduler/PGain Lookup/Enabled'
 * '<S115>' : 'tv_controller_2_1/PID Gain Scheduler1/DGain Lookup'
 * '<S116>' : 'tv_controller_2_1/PID Gain Scheduler1/IGain Lookup'
 * '<S117>' : 'tv_controller_2_1/PID Gain Scheduler1/NGain Lookup'
 * '<S118>' : 'tv_controller_2_1/PID Gain Scheduler1/PGain Lookup'
 * '<S119>' : 'tv_controller_2_1/PID Gain Scheduler1/IGain Lookup/Enabled'
 * '<S120>' : 'tv_controller_2_1/PID Gain Scheduler1/PGain Lookup/Enabled'
 */
#endif                                 /* tv_controller_2_1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

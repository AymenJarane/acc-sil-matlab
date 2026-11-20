/*
 * File: acc_ctrl_ref.c
 *
 * Code generated for Simulink model 'acc_ctrl_ref'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Nov 20 15:28:17 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "acc_ctrl_ref.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_acc_ctrl_ref_T acc_ctrl_ref_DW;

/* External inputs (root inport signals with default storage) */
ExtU_acc_ctrl_ref_T acc_ctrl_ref_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_acc_ctrl_ref_T acc_ctrl_ref_Y;

/* Real-time model */
static RT_MODEL_acc_ctrl_ref_T acc_ctrl_ref_M_;
RT_MODEL_acc_ctrl_ref_T *const acc_ctrl_ref_M = &acc_ctrl_ref_M_;

/* Model step function */
void acc_ctrl_ref_step(void)
{
  real_T a_cruise;
  real_T a_follow;
  int32_T acc_mode;

  /* MATLAB Function: '<Root>/acc_logic' incorporates:
   *  Inport: '<Root>/d'
   *  Inport: '<Root>/v_ego'
   *  Inport: '<Root>/v_rel'
   */
  a_cruise = (30.0 - acc_ctrl_ref_U.v_ego) * 0.5 + acc_ctrl_ref_DW.integ;
  a_follow = (acc_ctrl_ref_U.d - (1.8 * fmax(acc_ctrl_ref_U.v_ego, 0.0) + 5.0)) *
    0.25 + 0.8 * acc_ctrl_ref_U.v_rel;
  if ((acc_ctrl_ref_U.d < 150.0) && (a_follow < a_cruise)) {
    acc_mode = 2;
    a_cruise = a_follow;
  } else {
    acc_mode = 1;
    if (((!(a_cruise > 3.0)) || (!(30.0 - acc_ctrl_ref_U.v_ego > 0.0))) &&
        ((!(a_cruise < -3.0)) || (!(30.0 - acc_ctrl_ref_U.v_ego < 0.0)))) {
      acc_ctrl_ref_DW.integ = fmin(fmax((30.0 - acc_ctrl_ref_U.v_ego) * 0.1 *
        0.01 + acc_ctrl_ref_DW.integ, -3.0), 3.0);
    }
  }

  /* Outport: '<Root>/a_cmd' incorporates:
   *  MATLAB Function: '<Root>/acc_logic'
   */
  acc_ctrl_ref_Y.a_cmd = fmin(fmax(a_cruise, -3.0), 3.0);

  /* Outport: '<Root>/acc_mode' incorporates:
   *  MATLAB Function: '<Root>/acc_logic'
   */
  acc_ctrl_ref_Y.acc_mode = acc_mode;
}

/* Model initialize function */
void acc_ctrl_ref_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void acc_ctrl_ref_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

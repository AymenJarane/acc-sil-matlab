/*
 * File: acc_ctrl_ref.h
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

#ifndef acc_ctrl_ref_h_
#define acc_ctrl_ref_h_
#ifndef acc_ctrl_ref_COMMON_INCLUDES_
#define acc_ctrl_ref_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* acc_ctrl_ref_COMMON_INCLUDES_ */

#include "acc_ctrl_ref_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T integ;                        /* '<Root>/acc_logic' */
} DW_acc_ctrl_ref_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T v_ego;                        /* '<Root>/v_ego' */
  real_T d;                            /* '<Root>/d' */
  real_T v_rel;                        /* '<Root>/v_rel' */
} ExtU_acc_ctrl_ref_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T a_cmd;                        /* '<Root>/a_cmd' */
  real_T acc_mode;                     /* '<Root>/acc_mode' */
} ExtY_acc_ctrl_ref_T;

/* Real-time Model Data Structure */
struct tag_RTM_acc_ctrl_ref_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_acc_ctrl_ref_T acc_ctrl_ref_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_acc_ctrl_ref_T acc_ctrl_ref_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_acc_ctrl_ref_T acc_ctrl_ref_Y;

/* Model entry point functions */
extern void acc_ctrl_ref_initialize(void);
extern void acc_ctrl_ref_step(void);
extern void acc_ctrl_ref_terminate(void);

/* Real-time Model object */
extern RT_MODEL_acc_ctrl_ref_T *const acc_ctrl_ref_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/zoh_d' : Eliminated since input and output rates are identical
 * Block '<Root>/zoh_v' : Eliminated since input and output rates are identical
 * Block '<Root>/zoh_vrel' : Eliminated since input and output rates are identical
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
 * '<Root>' : 'acc_ctrl_ref'
 * '<S1>'   : 'acc_ctrl_ref/acc_logic'
 */
#endif                                 /* acc_ctrl_ref_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

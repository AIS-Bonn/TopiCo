/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control_lib.h
 *
 * Code generation for function 'opt_control_lib'
 *
 */

#ifndef OPT_CONTROL_LIB_H
#define OPT_CONTROL_LIB_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void opt_control_lib(const emxArray_real_T *State_start_in, const
  emxArray_real_T *Waypoints_in, const emxArray_real_T *V_max_in, const
  emxArray_real_T *V_min_in, const emxArray_real_T *A_max_in, const
  emxArray_real_T *A_min_in, const emxArray_real_T *J_max_in, const
  emxArray_real_T *J_min_in, const emxArray_real_T *A_global_in, bool
  b_comp_global_in, const emxArray_boolean_T *b_sync_V_in, const
  emxArray_boolean_T *b_sync_A_in, const emxArray_boolean_T *b_sync_J_in, const
  emxArray_boolean_T *b_sync_W_in, const emxArray_boolean_T *b_rotate_in, const
  emxArray_boolean_T *b_best_solution_in, const emxArray_boolean_T
  *b_hard_vel_limit_in, const emxArray_boolean_T *b_catch_up_in, const
  emxArray_int16_T *solution_in, double ts_rollout_in, emxArray_struct0_T
  *J_setp_struct, emxArray_int16_T *solution_out, emxArray_real_T *T_waypoints,
  emxArray_real_T *P_rollout, emxArray_real_T *V_rollout, emxArray_real_T
  *A_rollout, emxArray_real_T *J_rollout, emxArray_real_T *t_rollout);

#endif

/* End of code generation (opt_control_lib.h) */

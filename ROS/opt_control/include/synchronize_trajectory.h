/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * synchronize_trajectory.h
 *
 * Code generation for function 'synchronize_trajectory'
 *
 */

#ifndef SYNCHRONIZE_TRAJECTORY_H
#define SYNCHRONIZE_TRAJECTORY_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void synchronize_trajectory(const emxArray_real_T *P_init, const
  emxArray_real_T *V_init, const emxArray_real_T *A_init, const emxArray_real_T *
  P_wayp, const emxArray_real_T *V_wayp, const emxArray_real_T *A_wayp, const
  emxArray_real_T *V_max, const emxArray_real_T *V_min, const emxArray_real_T
  *A_max, const emxArray_real_T *A_min, const emxArray_real_T *J_max, const
  emxArray_real_T *J_min, const emxArray_boolean_T *b_sync_V, const
  emxArray_boolean_T *b_sync_A, const emxArray_boolean_T *b_sync_J, const
  emxArray_boolean_T *b_sync_W, const emxArray_boolean_T *b_best_solution, const
  emxArray_boolean_T *b_hard_vel_limit, const emxArray_real_T *T_catch_up, const
  emxArray_int16_T *solution_in, emxArray_cell_wrap_0 *t, emxArray_cell_wrap_0
  *J, emxArray_int16_T *solution_out);

#endif

/* End of code generation (synchronize_trajectory.h) */

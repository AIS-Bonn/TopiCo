/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * calculate_score.h
 *
 * Code generation for function 'calculate_score'
 *
 */

#ifndef CALCULATE_SCORE_H
#define CALCULATE_SCORE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern double calculate_score(double P_init_opt, double V_init_opt, const
  emxArray_real_T *J_setp_struct_opt_time, const emxArray_real_T
  *c_J_setp_struct_opt_signals_val, double P_init_test, double V_init_test,
  double A_init_test, const double J_setp_struct_test_time_data[], const int
  J_setp_struct_test_time_size[1], const double c_J_setp_struct_test_signals_va[],
  const int d_J_setp_struct_test_signals_va[2]);

#endif

/* End of code generation (calculate_score.h) */

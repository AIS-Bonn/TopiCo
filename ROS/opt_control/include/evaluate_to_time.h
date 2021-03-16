/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * evaluate_to_time.h
 *
 * Code generation for function 'evaluate_to_time'
 *
 */

#ifndef EVALUATE_TO_TIME_H
#define EVALUATE_TO_TIME_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_evaluate_to_time(double P_init, double V_init, double A_init,
  const emxArray_real_T *J_setp_struct_time, const emxArray_real_T
  *J_setp_struct_signals_values, double *P, double *V, double *A);
extern void c_evaluate_to_time(double P_init, double V_init, double A_init,
  emxArray_real_T *J_setp_struct_time, emxArray_real_T
  *J_setp_struct_signals_values, double T_in, double *P, double *V, double *A);
extern void d_evaluate_to_time(double P_init, double V_init, emxArray_real_T
  *J_setp_struct_time, emxArray_real_T *J_setp_struct_signals_values, double
  T_in, double *P, double *V, double *A, double *J);
extern void e_evaluate_to_time(const emxArray_real_T *P_init, const
  emxArray_real_T *V_init, const emxArray_real_T *A_init, emxArray_struct0_T
  *J_setp_struct, const emxArray_real_T *T_in, emxArray_real_T *P,
  emxArray_real_T *V, emxArray_real_T *A, emxArray_real_T *J);
extern void evaluate_to_time(double P_init, double V_init, double A_init, double
  J_setp_struct_time_data[], int J_setp_struct_time_size[1], double
  c_J_setp_struct_signals_values_[], int d_J_setp_struct_signals_values_[2],
  double T_in, double *P, double *V, double *A);

#endif

/* End of code generation (evaluate_to_time.h) */

/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * construct_setp_struct.h
 *
 * Code generation for function 'construct_setp_struct'
 *
 */

#ifndef CONSTRUCT_SETP_STRUCT_H
#define CONSTRUCT_SETP_STRUCT_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_construct_setp_struct(const cell_wrap_11 t_in2[1], const
  cell_wrap_11 J_in2[1], emxArray_real_T *J_setp_struct_time, emxArray_real_T
  *J_setp_struct_signals_values);
extern void c_construct_setp_struct(const double t_in2[1], emxArray_real_T
  *J_setp_struct_time, emxArray_real_T *J_setp_struct_signals_values);
extern void construct_setp_struct(const double t_in2[1], const double J_in2[1],
  double J_setp_struct_time_data[], int J_setp_struct_time_size[1], double
  c_J_setp_struct_signals_values_[], int d_J_setp_struct_signals_values_[2]);
extern void d_construct_setp_struct(const cell_wrap_16 t_in2[1], const
  cell_wrap_16 J_in2[1], double J_setp_struct_time_data[], int
  J_setp_struct_time_size[1], double c_J_setp_struct_signals_values_[], int
  d_J_setp_struct_signals_values_[2]);
extern void e_construct_setp_struct(const emxArray_cell_wrap_0 *t_in2, const
  emxArray_cell_wrap_0 *J_in2, emxArray_struct0_T *J_setp_struct);

#endif

/* End of code generation (construct_setp_struct.h) */

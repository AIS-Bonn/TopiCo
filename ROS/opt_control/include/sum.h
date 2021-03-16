/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * sum.h
 *
 * Code generation for function 'sum'
 *
 */

#ifndef SUM_H
#define SUM_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_sum(const emxArray_real_T *x, emxArray_real_T *y);
extern double c_sum(const double x[3]);
extern void d_sum(const creal_T x_data[], const int x_size[2], creal_T y_data[],
                  int y_size[1]);
extern void e_sum(const bool x_data[], const int x_size[2], double y[11]);
extern double f_sum(const double x[11]);
extern void g_sum(const double x_data[], const int x_size[2], double y_data[],
                  int y_size[1]);
extern double h_sum(const emxArray_real_T *x);
extern void i_sum(const creal_T x_data[], const int x_size[2], creal_T y_data[],
                  int y_size[1]);
extern void sum(const emxArray_boolean_T *x, emxArray_real_T *y);

#endif

/* End of code generation (sum.h) */

/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * power.h
 *
 * Code generation for function 'power'
 *
 */

#ifndef POWER_H
#define POWER_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_power(const emxArray_real_T *a, emxArray_real_T *y);
extern creal_T c_power(const creal_T a);
extern creal_T d_power(const creal_T a);
extern void e_power(const creal_T a[2], creal_T y[2]);
extern creal_T f_power(const creal_T a);
extern creal_T g_power(const creal_T a);
extern void h_power(const creal_T a_data[], const int a_size[1], creal_T y_data[],
                    int y_size[1]);
extern void i_power(const creal_T a[2], creal_T y[2]);
extern void j_power(const creal_T a[4], creal_T y[4]);
extern void power(const emxArray_real_T *a, emxArray_real_T *y);

#endif

/* End of code generation (power.h) */

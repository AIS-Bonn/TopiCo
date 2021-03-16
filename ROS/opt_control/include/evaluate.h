/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * evaluate.h
 *
 * Code generation for function 'evaluate'
 *
 */

#ifndef EVALUATE_H
#define EVALUATE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_evaluate(const double t_data[], const int t_size[2], const double
  J[7], double P_init, double V_init, double A_init, double P_data[], int
  P_size[2], double V_data[], int V_size[2], double A_data[], int A_size[2]);
extern void evaluate(const emxArray_real_T *t, const emxArray_real_T *J, double
                     P_init, double V_init, double A_init, emxArray_real_T *P,
                     emxArray_real_T *V, emxArray_real_T *A);

#endif

/* End of code generation (evaluate.h) */

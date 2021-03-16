/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * unaryMinOrMax.h
 *
 * Code generation for function 'unaryMinOrMax'
 *
 */

#ifndef UNARYMINORMAX_H
#define UNARYMINORMAX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern int findFirst(const emxArray_real_T *x);
extern void minOrMaxRealFloatVectorKernel(const emxArray_real_T *x, int first,
  int last, double *ex, int *idx);

#endif

/* End of code generation (unaryMinOrMax.h) */

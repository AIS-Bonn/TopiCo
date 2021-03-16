/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * compensate_global.h
 *
 * Code generation for function 'compensate_global'
 *
 */

#ifndef COMPENSATE_GLOBAL_H
#define COMPENSATE_GLOBAL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void compensate_global(const emxArray_real_T *State_in, const
  emxArray_real_T *A_global, const emxArray_real_T *A_max_in, const
  emxArray_real_T *A_min_in, emxArray_real_T *State_out, emxArray_real_T
  *A_max_out, emxArray_real_T *A_min_out);

#endif

/* End of code generation (compensate_global.h) */

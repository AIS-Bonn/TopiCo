/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * predict_state.h
 *
 * Code generation for function 'predict_state'
 *
 */

#ifndef PREDICT_STATE_H
#define PREDICT_STATE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void predict_state(emxArray_real_T *V_init, emxArray_real_T *A_init,
  emxArray_real_T *V_wayp, emxArray_real_T *A_wayp, const emxArray_real_T
  *VP_wayp, const emxArray_real_T *AP_wayp, emxArray_real_T *V_max,
  emxArray_real_T *V_min, emxArray_real_T *A_max, emxArray_real_T *A_min);

#endif

/* End of code generation (predict_state.h) */

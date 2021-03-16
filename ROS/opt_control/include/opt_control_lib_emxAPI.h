/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control_lib_emxAPI.h
 *
 * Code generation for function 'opt_control_lib_emxAPI'
 *
 */

#ifndef OPT_CONTROL_LIB_EMXAPI_H
#define OPT_CONTROL_LIB_EMXAPI_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern emxArray_boolean_T *emxCreateND_boolean_T(int numDimensions, int *size);
extern emxArray_int16_T *emxCreateND_int16_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_struct0_T *emxCreateND_struct0_T(int numDimensions, int *size);
extern emxArray_boolean_T *emxCreateWrapperND_boolean_T(bool *data, int
  numDimensions, int *size);
extern emxArray_int16_T *emxCreateWrapperND_int16_T(short *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_struct0_T *emxCreateWrapperND_struct0_T(struct0_T *data, int
  numDimensions, int *size);
extern emxArray_boolean_T *emxCreateWrapper_boolean_T(bool *data, int rows, int
  cols);
extern emxArray_int16_T *emxCreateWrapper_int16_T(short *data, int rows, int
  cols);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_struct0_T *emxCreateWrapper_struct0_T(struct0_T *data, int rows,
  int cols);
extern emxArray_boolean_T *emxCreate_boolean_T(int rows, int cols);
extern emxArray_int16_T *emxCreate_int16_T(int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern emxArray_struct0_T *emxCreate_struct0_T(int rows, int cols);
extern void emxDestroyArray_boolean_T(emxArray_boolean_T *emxArray);
extern void emxDestroyArray_int16_T(emxArray_int16_T *emxArray);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxDestroyArray_struct0_T(emxArray_struct0_T *emxArray);
extern void emxInitArray_boolean_T(emxArray_boolean_T **pEmxArray, int
  numDimensions);
extern void emxInitArray_int16_T(emxArray_int16_T **pEmxArray, int numDimensions);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInitArray_struct0_T(emxArray_struct0_T **pEmxArray, int
  numDimensions);

#endif

/* End of code generation (opt_control_lib_emxAPI.h) */

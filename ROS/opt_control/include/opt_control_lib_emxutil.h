/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control_lib_emxutil.h
 *
 * Code generation for function 'opt_control_lib_emxutil'
 *
 */

#ifndef OPT_CONTROL_LIB_EMXUTIL_H
#define OPT_CONTROL_LIB_EMXUTIL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void emxCopyStruct_cell_wrap_0(cell_wrap_0 *dst, const cell_wrap_0 *src);
extern void emxCopyStruct_struct0_T(struct0_T *dst, const struct0_T *src);
extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_boolean_T1(emxArray_boolean_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_cell_wrap_0(emxArray_cell_wrap_0 *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_cell_wrap_01(emxArray_cell_wrap_0 *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int16_T1(emxArray_int16_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T1(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T1(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T2(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_struct0_T(emxArray_struct0_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_struct_T(emxArray_struct_T *emxArray, int oldNumel);
extern void emxFreeStruct_struct0_T(struct0_T *pStruct);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_cell_wrap_0(emxArray_cell_wrap_0 **pEmxArray);
extern void emxFree_int16_T(emxArray_int16_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxFree_struct0_T(emxArray_struct0_T **pEmxArray);
extern void emxFree_struct_T(emxArray_struct_T **pEmxArray);
extern void emxInitStruct_struct0_T(struct0_T *pStruct);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_boolean_T1(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_cell_wrap_0(emxArray_cell_wrap_0 **pEmxArray, int
  numDimensions);
extern void emxInit_cell_wrap_01(emxArray_cell_wrap_0 **pEmxArray, int
  numDimensions);
extern void emxInit_int16_T(emxArray_int16_T **pEmxArray, int numDimensions);
extern void emxInit_int16_T1(emxArray_int16_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T1(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T2(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_struct0_T(emxArray_struct0_T **pEmxArray, int numDimensions);
extern void emxInit_struct_T(emxArray_struct_T **pEmxArray, int numDimensions);

#endif

/* End of code generation (opt_control_lib_emxutil.h) */

/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control_lib_types.h
 *
 * Code generation for function 'opt_control_lib'
 *
 */

#ifndef OPT_CONTROL_LIB_TYPES_H
#define OPT_CONTROL_LIB_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  double values;
} struct_T;

typedef struct {
  double time;
  struct_T signals;
} b_struct_T;

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

typedef struct {
  emxArray_real_T *f1;
} cell_wrap_0;

typedef struct {
  double f1[3];
} cell_wrap_11;

typedef struct {
  double f1[11];
} cell_wrap_16;

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  bool *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_boolean_T*/

typedef struct {
  cell_wrap_0 *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
} emxArray_cell_wrap_0;

#ifndef struct_emxArray_int16_T
#define struct_emxArray_int16_T

struct emxArray_int16_T
{
  short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_int16_T*/

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_emxArray_int32_T*/

#ifndef struct_sI9qUddGj0NnlWswvJOiECB_tag
#define struct_sI9qUddGj0NnlWswvJOiECB_tag

struct sI9qUddGj0NnlWswvJOiECB_tag
{
  emxArray_real_T *values;
};

#endif                                 /*struct_sI9qUddGj0NnlWswvJOiECB_tag*/

typedef sI9qUddGj0NnlWswvJOiECB_tag struct1_T;

#ifndef struct_ssRBF2u2K6q7pPUZWB4x74_tag
#define struct_ssRBF2u2K6q7pPUZWB4x74_tag

struct ssRBF2u2K6q7pPUZWB4x74_tag
{
  emxArray_real_T *time;
  struct1_T signals;
};

#endif                                 /*struct_ssRBF2u2K6q7pPUZWB4x74_tag*/

typedef ssRBF2u2K6q7pPUZWB4x74_tag struct0_T;

#ifndef struct_c_emxArray_ssRBF2u2K6q7pPUZWB4x
#define struct_c_emxArray_ssRBF2u2K6q7pPUZWB4x

struct c_emxArray_ssRBF2u2K6q7pPUZWB4x
{
  struct0_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};

#endif                                 /*struct_c_emxArray_ssRBF2u2K6q7pPUZWB4x*/

typedef c_emxArray_ssRBF2u2K6q7pPUZWB4x emxArray_struct0_T;
typedef struct {
  b_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
} emxArray_struct_T;

typedef struct {
  int iFirst;
  int iLast;
  int lineNo;
  int colNo;
  const char * aName;
  const char * fName;
  const char * pName;
  int checkKind;
} rtBoundsCheckInfo;

typedef struct {
  int lineNo;
  int colNo;
  const char * fName;
  const char * pName;
  int checkKind;
} rtDoubleCheckInfo;

typedef struct {
  int nDims;
  int lineNo;
  int colNo;
  const char * fName;
  const char * pName;
} rtEqualityCheckInfo;

typedef struct {
  int lineNo;
  int colNo;
  const char * fName;
  const char * pName;
} rtRunTimeErrorInfo;

#endif

/* End of code generation (opt_control_lib_types.h) */

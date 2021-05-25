//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// topico_wrapper_internal_types.h
//
// Code generation for function 'topico_wrapper'
//

#ifndef TOPICO_WRAPPER_INTERNAL_TYPES_H
#define TOPICO_WRAPPER_INTERNAL_TYPES_H

// Include files
#include "rtwtypes.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "coder_bounded_array.h"

// Type Definitions
struct rtDoubleCheckInfo {
  int lineNo;
  int colNo;
  const char *fName;
  const char *pName;
  int checkKind;
};

struct cell_wrap_22 {
  coder::bounded_array<int, 24U, 2U> f1;
};

struct cell_wrap_0 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_17 {
  coder::array<double, 2U> f1;
};

#endif
// End of code generation (topico_wrapper_internal_types.h)

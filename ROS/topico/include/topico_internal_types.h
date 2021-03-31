//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// topico_internal_types.h
//
// Code generation for function 'topico'
//

#ifndef TOPICO_INTERNAL_TYPES_H
#define TOPICO_INTERNAL_TYPES_H

// Include files
#include "rtwtypes.h"
#include "topico_types.h"
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

struct cell_wrap_21 {
  coder::bounded_array<int, 24U, 2U> f1;
};

struct cell_wrap_0 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_15 {
  coder::array<double, 2U> f1;
};

#endif
// End of code generation (topico_internal_types.h)

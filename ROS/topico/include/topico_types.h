//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// topico_types.h
//
// Code generation for function 'topico'
//

#ifndef TOPICO_TYPES_H
#define TOPICO_TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct rtBoundsCheckInfo {
  int iFirst;
  int iLast;
  int lineNo;
  int colNo;
  const char *aName;
  const char *fName;
  const char *pName;
  int checkKind;
};

struct rtEqualityCheckInfo {
  int nDims;
  int lineNo;
  int colNo;
  const char *fName;
  const char *pName;
};

struct rtRunTimeErrorInfo {
  int lineNo;
  int colNo;
  const char *fName;
  const char *pName;
};

struct struct1_T {
  coder::array<double, 2U> values;
};

struct struct0_T {
  coder::array<double, 2U> time;
  struct1_T signals;
};

#endif
// End of code generation (topico_types.h)

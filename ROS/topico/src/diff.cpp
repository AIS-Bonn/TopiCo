//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// diff.cpp
//
// Code generation for function 'diff'
//

// Include files
#include "diff.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Variable Definitions
static rtRunTimeErrorInfo j_emlrtRTEI = {
    51,                                                              // lineNo
    19,                                                              // colNo
    "diff",                                                          // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/datafun/diff.m" // pName
};

// Function Declarations
static void f_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void f_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "The dimension to operate along was selected automatically, is "
         "variable-length, and has length 1 at run time. This is not support"
         "ed. Manually select the dimension to operate along by supplying the "
         "DIM argument.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void diff(const ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &y)
{
  int dimSize;
  int u0;
  dimSize = x.size(0);
  u0 = x.size(0) - 1;
  if (u0 >= 1) {
    u0 = 1;
  }
  if (u0 < 1) {
    y.set_size(0);
  } else {
    if (x.size(0) == 1) {
      f_rtErrorWithMessageID(j_emlrtRTEI.fName, j_emlrtRTEI.lineNo);
    }
    y.set_size(x.size(0) - 1);
    if (x.size(0) - 1 != 0) {
      double work_data;
      work_data = x[0];
      if ((2 <= x.size(0)) && (x.size(0) > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (u0 = 2; u0 <= dimSize; u0++) {
        double d;
        double tmp1;
        tmp1 = x[u0 - 1];
        d = tmp1;
        tmp1 -= work_data;
        work_data = d;
        y[u0 - 2] = tmp1;
      }
    }
  }
}

void diff(const ::coder::array<double, 2U> &x, ::coder::array<double, 2U> &y)
{
  int dimSize;
  dimSize = x.size(1);
  if (x.size(1) == 0) {
    y.set_size(1, 0);
  } else {
    int u0;
    u0 = x.size(1) - 1;
    if (u0 >= 1) {
      u0 = 1;
    }
    if (u0 < 1) {
      y.set_size(1, 0);
    } else {
      double work_data;
      if (x.size(1) == 1) {
        f_rtErrorWithMessageID(j_emlrtRTEI.fName, j_emlrtRTEI.lineNo);
      }
      y.set_size(1, x.size(1) - 1);
      work_data = x[0];
      if (x.size(1) > 2147483646) {
        check_forloop_overflow_error();
      }
      for (u0 = 2; u0 <= dimSize; u0++) {
        double d;
        double tmp1;
        tmp1 = x[u0 - 1];
        d = tmp1;
        tmp1 -= work_data;
        work_data = d;
        y[u0 - 2] = tmp1;
      }
    }
  }
}

} // namespace coder

// End of code generation (diff.cpp)

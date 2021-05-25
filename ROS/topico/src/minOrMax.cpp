//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// minOrMax.cpp
//
// Code generation for function 'minOrMax'
//

// Include files
#include "minOrMax.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
static void e_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void e_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "If the working dimension of MAX or MIN is variable in length, "
               "it must not have zero length at runtime.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
namespace internal {
void maximum(const ::coder::array<double, 1U> &x, double *ex, int *idx)
{
  int last;
  last = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      *ex = x[0];
      *idx = 1;
    } else if ((x[0] < x[x.size(0) - 1]) ||
               (rtIsNaN(x[0]) && (!rtIsNaN(x[x.size(0) - 1])))) {
      *ex = x[x.size(0) - 1];
      *idx = x.size(0);
    } else {
      *ex = x[0];
      *idx = 1;
    }
  } else {
    int k;
    if (!rtIsNaN(x[0])) {
      *idx = 1;
    } else {
      bool exitg1;
      *idx = 0;
      if (x.size(0) > 2147483646) {
        check_forloop_overflow_error();
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (*idx == 0) {
      *ex = x[0];
      *idx = 1;
    } else {
      int a;
      *ex = x[*idx - 1];
      a = *idx + 1;
      if ((*idx + 1 <= x.size(0)) && (x.size(0) > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (k = a; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (*ex < d) {
          *ex = d;
          *idx = k;
        }
      }
    }
  }
}

double maximum(const ::coder::array<double, 1U> &x)
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      124,             // lineNo
      27,              // colNo
      "unaryMinOrMax", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
      "unaryMinOrMax.m" // pName
  };
  double ex;
  int last;
  if (x.size(0) < 1) {
    e_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  last = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      ex = x[0];
    } else if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
      ex = x[1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!rtIsNaN(x[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      if (x.size(0) > 2147483646) {
        check_forloop_overflow_error();
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x[0];
    } else {
      int a;
      ex = x[idx - 1];
      a = idx + 1;
      if ((idx + 1 <= x.size(0)) && (x.size(0) > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (k = a; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (ex < d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

} // namespace internal
} // namespace coder

// End of code generation (minOrMax.cpp)

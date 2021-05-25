//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// repmat.cpp
//
// Code generation for function 'repmat'
//

// Include files
#include "repmat.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
static void rtErrorWithMessageID(const int b, const int c, const char *aFcnName,
                                 int aLineNum);

// Function Definitions
static void rtErrorWithMessageID(const int b, const int c, const char *aFcnName,
                                 int aLineNum)
{
  std::stringstream outStream;
  ((((outStream << "Size argument must be an integer in the range: ") << b)
    << " to ")
   << c)
      << ".";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void repmat(const ::coder::array<double, 1U> &a, double varargin_2,
            ::coder::array<double, 2U> &b)
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      58,                   // lineNo
      23,                   // colNo
      "assertValidSizeArg", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
      "assertValidSizeArg.m" // pName
  };
  int i;
  int nrows;
  if (varargin_2 != varargin_2) {
    rtErrorWithMessageID(MIN_int32_T, MAX_int32_T, s_emlrtRTEI.fName,
                         s_emlrtRTEI.lineNo);
  }
  i = static_cast<int>(varargin_2);
  b.set_size(a.size(0), i);
  nrows = a.size(0);
  if ((1 <= static_cast<int>(varargin_2)) &&
      (static_cast<int>(varargin_2) > 2147483646)) {
    check_forloop_overflow_error();
  }
  for (int jtilecol = 0; jtilecol < i; jtilecol++) {
    int ibtile;
    ibtile = jtilecol * nrows;
    if ((1 <= nrows) && (nrows > 2147483646)) {
      check_forloop_overflow_error();
    }
    for (int k = 0; k < nrows; k++) {
      b[ibtile + k] = a[k];
    }
  }
}

} // namespace coder

// End of code generation (repmat.cpp)

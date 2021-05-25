//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eml_int_forloop_overflow_check.cpp
//
// Code generation for function 'eml_int_forloop_overflow_check'
//

// Include files
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
static void e_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

// Function Definitions
static void e_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "The loop variable of class ") << b)
      << " might overflow on the last iteration of the for loop. This could "
         "lead to an infinite loop.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void check_forloop_overflow_error()
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      88,                             // lineNo
      9,                              // colNo
      "check_forloop_overflow_error", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/eml/"
      "eml_int_forloop_overflow_check.m" // pName
  };
  e_rtErrorWithMessageID("int32", s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
}

} // namespace coder

// End of code generation (eml_int_forloop_overflow_check.cpp)

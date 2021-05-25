//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// useConstantDim.cpp
//
// Code generation for function 'useConstantDim'
//

// Include files
#include "useConstantDim.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
namespace internal {
void useConstantDim(::coder::array<double, 2U> &varargin_2)
{
  if ((varargin_2.size(1) != 0) && (varargin_2.size(1) != 1)) {
    int b;
    b = varargin_2.size(1);
    if ((1 <= varargin_2.size(1) - 1) &&
        (varargin_2.size(1) - 1 > 2147483646)) {
      check_forloop_overflow_error();
    }
    for (int k = 0; k <= b - 2; k++) {
      varargin_2[k + 1] = varargin_2[k] + varargin_2[k + 1];
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (useConstantDim.cpp)

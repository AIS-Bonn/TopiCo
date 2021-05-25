//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// any1.cpp
//
// Code generation for function 'any1'
//

// Include files
#include "any1.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
bool any(const ::coder::array<bool, 1U> &x)
{
  int ix;
  bool exitg1;
  bool y;
  y = false;
  if ((1 <= x.size(0)) && (x.size(0) > 2147483646)) {
    check_forloop_overflow_error();
  }
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x.size(0))) {
    if (!x[ix - 1]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }
  return y;
}

} // namespace coder

// End of code generation (any1.cpp)

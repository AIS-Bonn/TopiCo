//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// all.cpp
//
// Code generation for function 'all'
//

// Include files
#include "all.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
bool all(const bool x_data[], const int x_size[2])
{
  int ix;
  bool exitg1;
  bool y;
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size[1])) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

bool b_all(const ::coder::array<bool, 1U> &x)
{
  int ix;
  bool exitg1;
  bool y;
  y = true;
  if ((1 <= x.size(0)) && (x.size(0) > 2147483646)) {
    check_forloop_overflow_error();
  }
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x.size(0))) {
    if (!x[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

} // namespace coder

// End of code generation (all.cpp)

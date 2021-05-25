//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// vAllOrAny.cpp
//
// Code generation for function 'vAllOrAny'
//

// Include files
#include "vAllOrAny.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
bool flatVectorAllOrAny(const double x_data[], const int x_size[2])
{
  int nx;
  bool p;
  nx = x_size[1];
  p = true;
  for (int k = 0; k < nx; k++) {
    if (p) {
      double x;
      x = x_data[k];
      if (rtIsInf(x) || rtIsNaN(x)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  return p;
}

} // namespace internal
} // namespace coder

// End of code generation (vAllOrAny.cpp)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// anyNonFinite.cpp
//
// Code generation for function 'anyNonFinite'
//

// Include files
#include "anyNonFinite.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
bool anyNonFinite(const double x_data[], const int x_size[2])
{
  int nx;
  bool p;
  nx = x_size[0] * x_size[1];
  p = true;
  for (int k = 0; k < nx; k++) {
    if ((!p) || (rtIsInf(x_data[k]) || rtIsNaN(x_data[k]))) {
      p = false;
    }
  }
  return !p;
}

} // namespace internal
} // namespace coder

// End of code generation (anyNonFinite.cpp)

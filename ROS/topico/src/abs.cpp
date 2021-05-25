//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abs.cpp
//
// Code generation for function 'abs'
//

// Include files
#include "abs.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void b_abs(const double x_data[], const int x_size[2], double y_data[],
           int y_size[2])
{
  int nx;
  nx = x_size[1];
  y_size[0] = 1;
  y_size[1] = static_cast<signed char>(x_size[1]);
  for (int k = 0; k < nx; k++) {
    y_data[k] = std::abs(x_data[k]);
  }
}

} // namespace coder

// End of code generation (abs.cpp)

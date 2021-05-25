//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xrot.cpp
//
// Code generation for function 'xrot'
//

// Include files
#include "xrot.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xrot(int n, double x_data[], int ix0, int incx, int iy0, int incy,
          double c, double s)
{
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error();
    }
    for (int k = 0; k < n; k++) {
      double temp;
      int b_temp_tmp;
      int temp_tmp;
      temp_tmp = (iy0 + k * incx) - 1;
      b_temp_tmp = (ix0 + k * incy) - 1;
      temp = c * x_data[b_temp_tmp] + s * x_data[temp_tmp];
      x_data[temp_tmp] = c * x_data[temp_tmp] - s * x_data[b_temp_tmp];
      x_data[b_temp_tmp] = temp;
    }
  }
}

void xrot(int n, double x_data[], int ix0, int iy0, double c, double s)
{
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error();
    }
    for (int k = 0; k < n; k++) {
      double temp;
      int b_temp_tmp;
      int temp_tmp;
      temp_tmp = (iy0 + k) - 1;
      b_temp_tmp = (ix0 + k) - 1;
      temp = c * x_data[b_temp_tmp] + s * x_data[temp_tmp];
      x_data[temp_tmp] = c * x_data[temp_tmp] - s * x_data[b_temp_tmp];
      x_data[b_temp_tmp] = temp;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xrot.cpp)

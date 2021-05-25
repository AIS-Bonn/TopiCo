//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xnrm2.cpp
//
// Code generation for function 'xnrm2'
//

// Include files
#include "xnrm2.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
double xnrm2(int n, const double x_data[], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      if ((ix0 <= kend) && (kend > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (int k = ix0; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

double xnrm2(int n, const double x[3])
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[1]);
    } else {
      double absxk;
      double scale;
      double t;
      scale = 3.3121686421112381E-170;
      absxk = std::abs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
      }
      absxk = std::abs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xnrm2.cpp)

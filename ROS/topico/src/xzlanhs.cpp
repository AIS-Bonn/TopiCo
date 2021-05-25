//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlanhs.cpp
//
// Code generation for function 'xzlanhs'
//

// Include files
#include "xzlanhs.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xzlanhs(const creal_T A_data[], const int A_size[2], int ilo, int ihi)
{
  double f;
  f = 0.0;
  if (ilo <= ihi) {
    double scale;
    double ssq;
    int nm1;
    scale = 3.3121686421112381E-170;
    ssq = 0.0;
    nm1 = ihi - ilo;
    if ((1 <= nm1 + 1) && (nm1 + 1 > 2147483646)) {
      check_forloop_overflow_error();
    }
    for (int j = 0; j <= nm1; j++) {
      double absxk;
      double colscale;
      double colssq;
      int col;
      int u0;
      colscale = 3.3121686421112381E-170;
      colssq = 0.0;
      col = (ilo + j) - 1;
      u0 = j + 1;
      if (u0 >= nm1) {
        u0 = nm1;
      }
      u0 += ilo;
      if ((ilo <= u0) && (u0 > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (int row = ilo; row <= u0; row++) {
        double t;
        int absxk_tmp;
        absxk_tmp = (row + A_size[0] * col) - 1;
        absxk = std::abs(A_data[absxk_tmp].re);
        if (absxk > colscale) {
          t = colscale / absxk;
          colssq = colssq * t * t + 1.0;
          colscale = absxk;
        } else {
          t = absxk / colscale;
          colssq += t * t;
        }
        absxk = std::abs(A_data[absxk_tmp].im);
        if (absxk > colscale) {
          t = colscale / absxk;
          colssq = colssq * t * t + 1.0;
          colscale = absxk;
        } else {
          t = absxk / colscale;
          colssq += t * t;
        }
      }
      if (scale >= colscale) {
        absxk = colscale / scale;
        ssq += absxk * absxk * colssq;
      } else {
        absxk = scale / colscale;
        ssq = colssq + absxk * absxk * ssq;
        scale = colscale;
      }
    }
    f = scale * std::sqrt(ssq);
  }
  return f;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlanhs.cpp)

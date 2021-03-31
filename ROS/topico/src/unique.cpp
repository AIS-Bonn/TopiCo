//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// unique.cpp
//
// Code generation for function 'unique'
//

// Include files
#include "unique.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_rtwutil.h"
#include "topico_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>

// Function Definitions
namespace coder {
void unique_vector(const ::coder::array<double, 2U> &a,
                   ::coder::array<double, 2U> &b)
{
  static rtRunTimeErrorInfo t_emlrtRTEI = {
      236,                                                           // lineNo
      1,                                                             // colNo
      "unique_vector",                                               // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/ops/unique.m" // pName
  };
  array<int, 2U> idx;
  array<int, 1U> iwork;
  double absx;
  int b_i;
  int exponent;
  int i;
  int i2;
  int j;
  int k;
  int n;
  int na;
  int p;
  int pEnd;
  int q;
  int qEnd;
  bool exitg1;
  na = a.size(1);
  n = a.size(1) + 1;
  idx.set_size(1, a.size(1));
  i = a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    idx[b_i] = 0;
  }
  if (a.size(1) != 0) {
    iwork.set_size(a.size(1));
    i = a.size(1) - 1;
    if ((1 <= a.size(1) - 1) && (a.size(1) - 1 > 2147483645)) {
      check_forloop_overflow_error();
    }
    for (k = 1; k <= i; k += 2) {
      absx = a[k];
      if ((a[k - 1] <= absx) || rtIsNaN(absx)) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((a.size(1) & 1) != 0) {
      idx[a.size(1) - 1] = a.size(1);
    }
    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int kEnd;
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          absx = a[idx[q] - 1];
          b_i = idx[p - 1];
          if ((a[b_i - 1] <= absx) || rtIsNaN(absx)) {
            iwork[k] = b_i;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork[k] = idx[q];
                q++;
              }
            }
          } else {
            iwork[k] = idx[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                k++;
                iwork[k] = idx[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      i = i2;
    }
  }
  b.set_size(1, a.size(1));
  if ((1 <= a.size(1)) && (a.size(1) > 2147483646)) {
    check_forloop_overflow_error();
  }
  for (k = 0; k < na; k++) {
    b[k] = a[idx[k] - 1];
  }
  k = 0;
  while ((k + 1 <= na) && rtIsInf(b[k]) && (b[k] < 0.0)) {
    k++;
  }
  pEnd = k;
  k = a.size(1);
  while ((k >= 1) && rtIsNaN(b[k - 1])) {
    k--;
  }
  p = a.size(1) - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    absx = b[k - 1];
    if (rtIsInf(absx) && (absx > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  i = (a.size(1) - k) - p;
  q = 0;
  if (pEnd > 0) {
    q = 1;
    if (pEnd > 2147483646) {
      check_forloop_overflow_error();
    }
  }
  while (pEnd + 1 <= k) {
    double x;
    x = b[pEnd];
    i2 = pEnd;
    int exitg2;
    do {
      exitg2 = 0;
      pEnd++;
      if (pEnd + 1 > k) {
        exitg2 = 1;
      } else {
        absx = std::abs(x / 2.0);
        if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
          if (absx <= 2.2250738585072014E-308) {
            absx = 4.94065645841247E-324;
          } else {
            frexp(absx, &exponent);
            absx = std::ldexp(1.0, exponent - 53);
          }
        } else {
          absx = rtNaN;
        }
        if ((!(std::abs(x - b[pEnd]) < absx)) &&
            ((!rtIsInf(b[pEnd])) || (!rtIsInf(x)) ||
             ((b[pEnd] > 0.0) != (x > 0.0)))) {
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);
    q++;
    b[q - 1] = x;
    if ((i2 + 1 <= pEnd) && (pEnd > 2147483646)) {
      check_forloop_overflow_error();
    }
  }
  if (i > 0) {
    q++;
    b[q - 1] = b[k];
    if (i > 2147483646) {
      check_forloop_overflow_error();
    }
  }
  pEnd = k + i;
  for (j = 0; j < p; j++) {
    b[q + j] = b[pEnd + j];
  }
  q += p;
  if (q > a.size(1)) {
    i_rtErrorWithMessageID(t_emlrtRTEI.fName, t_emlrtRTEI.lineNo);
  }
  if (1 > q) {
    q = 0;
  }
  b.set_size(b.size(0), q);
}

} // namespace coder

// End of code generation (unique.cpp)

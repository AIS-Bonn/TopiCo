//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlarf.cpp
//
// Code generation for function 'xzlarf'
//

// Include files
#include "xzlarf.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0,
            int ldc, double work_data[])
{
  int colbottom;
  int i;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m - 1;
    i = iv0 + m;
    while ((lastv + 1 > 0) && (C_data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int exitg1;
      i = ic0 + (lastc - 1) * ldc;
      colbottom = i + lastv;
      if ((i <= colbottom) && (colbottom > 2147483646)) {
        check_forloop_overflow_error();
      }
      do {
        exitg1 = 0;
        if (i <= colbottom) {
          if (C_data[i - 1] != 0.0) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = -1;
    lastc = 0;
  }
  if (lastv + 1 > 0) {
    double c;
    int b;
    int iac;
    if (lastc != 0) {
      if ((1 <= lastc) && (lastc > 2147483646)) {
        check_forloop_overflow_error();
      }
      if (0 <= lastc - 1) {
        std::memset(&work_data[0], 0, lastc * sizeof(double));
      }
      i = 0;
      colbottom = ic0 + ldc * (lastc - 1);
      for (iac = ic0; ldc < 0 ? iac >= colbottom : iac <= colbottom;
           iac += ldc) {
        c = 0.0;
        b = iac + lastv;
        if ((iac <= b) && (b > 2147483646)) {
          check_forloop_overflow_error();
        }
        for (int ia = iac; ia <= b; ia++) {
          c += C_data[ia - 1] * C_data[((iv0 + ia) - iac) - 1];
        }
        work_data[i] += c;
        i++;
      }
    }
    if (!(-tau == 0.0)) {
      i = ic0;
      for (colbottom = 0; colbottom < lastc; colbottom++) {
        if (work_data[colbottom] != 0.0) {
          c = work_data[colbottom] * -tau;
          b = lastv + i;
          if ((i <= b) && (b > 2147483646)) {
            check_forloop_overflow_error();
          }
          for (iac = i; iac <= b; iac++) {
            C_data[iac - 1] += C_data[((iv0 + iac) - i) - 1] * c;
          }
        }
        i += ldc;
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlarf.cpp)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// find.cpp
//
// Code generation for function 'find'
//

// Include files
#include "find.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"

// Variable Definitions
static rtRunTimeErrorInfo k_emlrtRTEI = {
    392,                                                           // lineNo
    1,                                                             // colNo
    "find_first_indices",                                          // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/elmat/find.m" // pName
};

// Function Definitions
namespace coder {
void b_eml_find(const ::coder::array<bool, 2U> &x, int i_data[], int i_size[2])
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      81,                                                            // lineNo
      1,                                                             // colNo
      "eml_find",                                                    // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/elmat/find.m" // pName
  };
  array<int, 2U> i;
  int idx;
  int ii;
  int k;
  bool exitg1;
  k = (1 <= x.size(1));
  if (k > x.size(1)) {
    i_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  idx = 0;
  i.set_size(1, k);
  if ((1 <= x.size(1)) && (x.size(1) > 2147483646)) {
    check_forloop_overflow_error();
  }
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x.size(1) - 1)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= k) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx > k) {
    i_rtErrorWithMessageID(k_emlrtRTEI.fName, k_emlrtRTEI.lineNo);
  }
  if (k == 1) {
    if (idx == 0) {
      i.set_size(1, 0);
    }
  } else {
    i.set_size(i.size(0), static_cast<int>(1 <= idx));
  }
  i_size[0] = 1;
  i_size[1] = i.size(1);
  k = i.size(1);
  if (0 <= k - 1) {
    i_data[0] = i[0];
  }
}

void eml_find(const ::coder::array<bool, 2U> &x, ::coder::array<int, 2U> &i)
{
  int idx;
  int ii;
  int nx;
  bool exitg1;
  nx = x.size(1);
  idx = 0;
  i.set_size(1, x.size(1));
  if ((1 <= x.size(1)) && (x.size(1) > 2147483646)) {
    check_forloop_overflow_error();
  }
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx > x.size(1)) {
    i_rtErrorWithMessageID(k_emlrtRTEI.fName, k_emlrtRTEI.lineNo);
  }
  if (x.size(1) == 1) {
    if (idx == 0) {
      i.set_size(1, 0);
    }
  } else {
    if (1 > idx) {
      idx = 0;
    }
    i.set_size(i.size(0), idx);
  }
}

} // namespace coder

// End of code generation (find.cpp)

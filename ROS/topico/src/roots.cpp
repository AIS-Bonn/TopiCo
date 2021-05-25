//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// roots.cpp
//
// Code generation for function 'roots'
//

// Include files
#include "roots.h"
#include "eig.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "vAllOrAny.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

// Variable Definitions
static rtRunTimeErrorInfo q_emlrtRTEI = {
    24,                                                               // lineNo
    5,                                                                // colNo
    "roots",                                                          // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/polyfun/roots.m" // pName
};

static rtRunTimeErrorInfo r_emlrtRTEI = {
    99,                                                               // lineNo
    5,                                                                // colNo
    "roots",                                                          // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/polyfun/roots.m" // pName
};

// Function Declarations
static void n_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void n_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Input to ROOTS must not contain NaN or Inf.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void b_roots(const double c[7], creal_T r_data[], int *r_size)
{
  static const int iv[2] = {1, 7};
  creal_T eiga_data[6];
  double a_data[36];
  double ctmp[7];
  int a_size[2];
  int k1;
  int k2;
  int nTrailingZeros;
  if (!internal::flatVectorAllOrAny(c, iv)) {
    n_rtErrorWithMessageID(q_emlrtRTEI.fName, q_emlrtRTEI.lineNo);
  }
  std::memset(&r_data[0], 0, 6U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 7) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }
  k2 = 7;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }
  nTrailingZeros = 6 - k2;
  if (k1 < k2) {
    int companDim;
    int j;
    bool exitg1;
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      bool exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(std::abs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }
      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }
    if (companDim < 1) {
      if (1 > 7 - k2) {
        *r_size = 0;
      } else {
        *r_size = 7 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim;
      if (0 <= k1 - 1) {
        std::memset(&a_data[0], 0, k1 * sizeof(double));
      }
      for (k1 = 0; k1 <= companDim - 2; k1++) {
        j = companDim * k1;
        a_data[j] = -ctmp[k1];
        a_data[(k1 + j) + 1] = 1.0;
      }
      a_data[companDim * (companDim - 1)] = -ctmp[companDim - 1];
      if (0 <= nTrailingZeros) {
        std::memset(&r_data[0], 0, (nTrailingZeros + 1) * sizeof(creal_T));
      }
      eig(a_data, a_size, eiga_data, &k1);
      for (k1 = 0; k1 < companDim; k1++) {
        r_data[(k1 - k2) + 7] = eiga_data[k1];
      }
      *r_size = (companDim - k2) + 7;
      if (*r_size > 7) {
        i_rtErrorWithMessageID(r_emlrtRTEI.fName, r_emlrtRTEI.lineNo);
      }
    }
  } else if (1 > 7 - k2) {
    *r_size = 0;
  } else {
    *r_size = 7 - k2;
  }
}

void roots(const double c[6], creal_T r_data[], int *r_size)
{
  static const int iv[2] = {1, 6};
  creal_T eiga_data[6];
  double a_data[25];
  double ctmp[6];
  int a_size[2];
  int k1;
  int k2;
  int nTrailingZeros;
  if (!internal::flatVectorAllOrAny(c, iv)) {
    n_rtErrorWithMessageID(q_emlrtRTEI.fName, q_emlrtRTEI.lineNo);
  }
  std::memset(&r_data[0], 0, 5U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 6) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }
  k2 = 6;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }
  nTrailingZeros = 5 - k2;
  if (k1 < k2) {
    int companDim;
    int j;
    bool exitg1;
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      bool exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(std::abs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }
      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }
    if (companDim < 1) {
      if (1 > 6 - k2) {
        *r_size = 0;
      } else {
        *r_size = 6 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim;
      if (0 <= k1 - 1) {
        std::memset(&a_data[0], 0, k1 * sizeof(double));
      }
      for (k1 = 0; k1 <= companDim - 2; k1++) {
        j = companDim * k1;
        a_data[j] = -ctmp[k1];
        a_data[(k1 + j) + 1] = 1.0;
      }
      a_data[companDim * (companDim - 1)] = -ctmp[companDim - 1];
      if (0 <= nTrailingZeros) {
        std::memset(&r_data[0], 0, (nTrailingZeros + 1) * sizeof(creal_T));
      }
      eig(a_data, a_size, eiga_data, &k1);
      for (k1 = 0; k1 < companDim; k1++) {
        r_data[(k1 - k2) + 6] = eiga_data[k1];
      }
      *r_size = (companDim - k2) + 6;
      if (*r_size > 6) {
        i_rtErrorWithMessageID(r_emlrtRTEI.fName, r_emlrtRTEI.lineNo);
      }
    }
  } else if (1 > 6 - k2) {
    *r_size = 0;
  } else {
    *r_size = 6 - k2;
  }
}

} // namespace coder

// End of code generation (roots.cpp)

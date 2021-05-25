//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eigHermitianStandard.cpp
//
// Code generation for function 'eigHermitianStandard'
//

// Include files
#include "eigHermitianStandard.h"
#include "anyNonFinite.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "xdhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
static void p_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void p_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Matrix must be square.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void eigHermitianStandard(const double A_data[], const int A_size[2],
                          double V_data[], int *V_size)
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      18,                                                              // lineNo
      15,                                                              // colNo
      "schur",                                                         // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/matfun/schur.m" // pName
  };
  double T_data[36];
  double work_data[6];
  double tau_data[5];
  int T_size[2];
  int i;
  int k;
  int n;
  T_size[0] = A_size[0];
  T_size[1] = A_size[1];
  i = A_size[0] * A_size[1];
  if (0 <= i - 1) {
    std::copy(&A_data[0], &A_data[i], &T_data[0]);
  }
  if (A_size[0] != A_size[1]) {
    p_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (internal::anyNonFinite(A_data, A_size)) {
    T_size[0] = static_cast<signed char>(A_size[0]);
    i = static_cast<signed char>(A_size[0]) *
        static_cast<signed char>(A_size[1]);
    for (int n_tmp = 0; n_tmp < i; n_tmp++) {
      T_data[n_tmp] = rtNaN;
    }
    i = static_cast<signed char>(A_size[0]);
    if (1 < static_cast<signed char>(A_size[0])) {
      int knt;
      int rowleft;
      knt = 2;
      if (static_cast<signed char>(A_size[0]) - 2 <
          static_cast<signed char>(A_size[1]) - 1) {
        rowleft = static_cast<signed char>(A_size[0]) - 1;
      } else {
        rowleft = static_cast<signed char>(A_size[1]);
      }
      for (k = 0; k < rowleft; k++) {
        for (int b_i = knt; b_i <= i; b_i++) {
          T_data[(b_i + T_size[0] * k) - 1] = 0.0;
        }
        knt++;
      }
    }
  } else {
    int b;
    int b_i;
    int knt;
    int rowleft;
    n = A_size[0];
    i = static_cast<signed char>(A_size[0]);
    if (0 <= i - 1) {
      std::memset(&work_data[0], 0, i * sizeof(double));
    }
    b = A_size[0];
    for (b_i = 0; b_i <= b - 2; b_i++) {
      double alpha1;
      double xnorm;
      int alpha1_tmp;
      int b_b;
      int ic0;
      int im1n_tmp;
      int in;
      int iv0;
      int lastc;
      int lastv;
      int n_tmp;
      im1n_tmp = b_i * n;
      in = (b_i + 1) * n;
      alpha1_tmp = (b_i + T_size[0] * b_i) + 1;
      alpha1 = T_data[alpha1_tmp];
      i = b_i + 3;
      if (i >= n) {
        i = n;
      }
      i += im1n_tmp;
      n_tmp = (n - b_i) - 3;
      tau_data[b_i] = 0.0;
      if (n_tmp + 2 > 0) {
        xnorm = internal::blas::xnrm2(n_tmp + 1, T_data, i);
        if (xnorm != 0.0) {
          double beta1;
          beta1 = rt_hypotd_snf(alpha1, xnorm);
          if (alpha1 >= 0.0) {
            beta1 = -beta1;
          }
          if (std::abs(beta1) < 1.0020841800044864E-292) {
            knt = 0;
            rowleft = i + n_tmp;
            do {
              knt++;
              for (k = i; k <= rowleft; k++) {
                T_data[k - 1] *= 9.9792015476736E+291;
              }
              beta1 *= 9.9792015476736E+291;
              alpha1 *= 9.9792015476736E+291;
            } while (!(std::abs(beta1) >= 1.0020841800044864E-292));
            xnorm = internal::blas::xnrm2(n_tmp + 1, T_data, i);
            beta1 = rt_hypotd_snf(alpha1, xnorm);
            if (alpha1 >= 0.0) {
              beta1 = -beta1;
            }
            tau_data[b_i] = (beta1 - alpha1) / beta1;
            xnorm = 1.0 / (alpha1 - beta1);
            for (k = i; k <= rowleft; k++) {
              T_data[k - 1] *= xnorm;
            }
            if ((1 <= knt) && (knt > 2147483646)) {
              check_forloop_overflow_error();
            }
            for (k = 0; k < knt; k++) {
              beta1 *= 1.0020841800044864E-292;
            }
            alpha1 = beta1;
          } else {
            tau_data[b_i] = (beta1 - alpha1) / beta1;
            xnorm = 1.0 / (alpha1 - beta1);
            b_b = i + n_tmp;
            for (k = i; k <= b_b; k++) {
              T_data[k - 1] *= xnorm;
            }
            alpha1 = beta1;
          }
        }
      }
      T_data[alpha1_tmp] = 1.0;
      iv0 = (b_i + im1n_tmp) + 1;
      ic0 = in + 1;
      if (tau_data[b_i] != 0.0) {
        bool exitg2;
        lastv = n_tmp + 1;
        i = iv0 + n_tmp;
        while ((lastv + 1 > 0) && (T_data[i + 1] == 0.0)) {
          lastv--;
          i--;
        }
        lastc = n;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int exitg1;
          rowleft = in + lastc;
          k = rowleft;
          do {
            exitg1 = 0;
            if ((n > 0) && (k <= rowleft + lastv * n)) {
              if (T_data[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k += n;
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
        if (lastc != 0) {
          if (0 <= lastc - 1) {
            std::memset(&work_data[0], 0, lastc * sizeof(double));
          }
          i = iv0;
          n_tmp = (in + n * lastv) + 1;
          for (rowleft = ic0; n < 0 ? rowleft >= n_tmp : rowleft <= n_tmp;
               rowleft += n) {
            b_b = (rowleft + lastc) - 1;
            for (k = rowleft; k <= b_b; k++) {
              knt = k - rowleft;
              work_data[knt] += T_data[k - 1] * T_data[i];
            }
            i++;
          }
        }
        if (!(-tau_data[b_i] == 0.0)) {
          knt = in;
          for (k = 0; k <= lastv; k++) {
            n_tmp = iv0 + k;
            if (T_data[n_tmp] != 0.0) {
              xnorm = T_data[n_tmp] * -tau_data[b_i];
              i = knt + 1;
              b_b = lastc + knt;
              if ((knt + 1 <= b_b) && (b_b > 2147483646)) {
                check_forloop_overflow_error();
              }
              for (rowleft = i; rowleft <= b_b; rowleft++) {
                T_data[rowleft - 1] += work_data[(rowleft - knt) - 1] * xnorm;
              }
            }
            knt += n;
          }
        }
      }
      internal::reflapack::xzlarf((n - b_i) - 1, (n - b_i) - 1,
                                  (b_i + im1n_tmp) + 2, tau_data[b_i], T_data,
                                  (b_i + in) + 2, n, work_data);
      T_data[alpha1_tmp] = alpha1;
    }
    internal::reflapack::eml_dlahqr(T_data, T_size);
    i = T_size[0];
    if (3 < T_size[0]) {
      knt = 4;
      if (T_size[0] - 4 < T_size[1] - 1) {
        rowleft = T_size[0] - 3;
      } else {
        rowleft = T_size[1];
      }
      for (k = 0; k < rowleft; k++) {
        for (b_i = knt; b_i <= i; b_i++) {
          T_data[(b_i + T_size[0] * k) - 1] = 0.0;
        }
        knt++;
      }
    }
  }
  n = T_size[0];
  *V_size = T_size[0];
  for (k = 0; k < n; k++) {
    V_data[k] = T_data[k + T_size[0] * k];
  }
}

} // namespace coder

// End of code generation (eigHermitianStandard.cpp)

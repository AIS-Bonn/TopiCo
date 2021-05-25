//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eig.cpp
//
// Code generation for function 'eig'
//

// Include files
#include "eig.h"
#include "anyNonFinite.h"
#include "eigHermitianStandard.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "xzggbal.h"
#include "xzhgeqz.h"
#include "xzlartg.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
static void o_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void o_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Input matrix must be square.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void eig(const double A_data[], const int A_size[2], creal_T V_data[],
         int *V_size)
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      62,                                                            // lineNo
      27,                                                            // colNo
      "eig",                                                         // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/matfun/eig.m" // pName
  };
  creal_T At_data[36];
  creal_T beta1_data[6];
  creal_T s;
  double tmp_data[6];
  double absxk;
  int rscale_data[6];
  int At_size[2];
  int b_i;
  int ihi;
  int ilo;
  int j;
  if (A_size[0] != A_size[1]) {
    o_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (internal::anyNonFinite(A_data, A_size)) {
    *V_size = A_size[0];
    j = A_size[0];
    for (int i = 0; i < j; i++) {
      V_data[i].re = rtNaN;
      V_data[i].im = 0.0;
    }
  } else {
    bool exitg2;
    bool ilascl;
    ilascl = (A_size[0] == A_size[1]);
    if (ilascl) {
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j <= A_size[1] - 1)) {
        int exitg1;
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            if (!(A_data[b_i + A_size[0] * j] == A_data[j + A_size[0] * b_i])) {
              ilascl = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
    if (ilascl) {
      eigHermitianStandard(A_data, A_size, tmp_data, V_size);
      for (int i = 0; i < *V_size; i++) {
        V_data[i].re = tmp_data[i];
        V_data[i].im = 0.0;
      }
    } else {
      double a;
      double anrm;
      double anrmto;
      double cto1;
      double ctoc;
      double stemp_im;
      int i;
      int k;
      signed char z_size_idx_0;
      bool notdone;
      At_size[0] = A_size[0];
      At_size[1] = A_size[1];
      j = A_size[0] * A_size[1];
      for (i = 0; i < j; i++) {
        At_data[i].re = A_data[i];
        At_data[i].im = 0.0;
      }
      anrm = 0.0;
      j = A_size[0] * A_size[1];
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k <= j - 1)) {
        absxk = rt_hypotd_snf(At_data[k].re, At_data[k].im);
        if (rtIsNaN(absxk)) {
          anrm = rtNaN;
          exitg2 = true;
        } else {
          if (absxk > anrm) {
            anrm = absxk;
          }
          k++;
        }
      }
      if (rtIsInf(anrm) || rtIsNaN(anrm)) {
        *V_size = A_size[0];
        j = A_size[0];
        for (i = 0; i < j; i++) {
          V_data[i].re = rtNaN;
          V_data[i].im = 0.0;
        }
        b_i = A_size[0];
        j = A_size[0];
        for (i = 0; i < j; i++) {
          beta1_data[i].re = rtNaN;
          beta1_data[i].im = 0.0;
        }
      } else {
        int n;
        bool guard1 = false;
        ilascl = false;
        anrmto = anrm;
        guard1 = false;
        if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
          anrmto = 6.7178761075670888E-139;
          ilascl = true;
          guard1 = true;
        } else if (anrm > 1.4885657073574029E+138) {
          anrmto = 1.4885657073574029E+138;
          ilascl = true;
          guard1 = true;
        }
        if (guard1) {
          absxk = anrm;
          ctoc = anrmto;
          notdone = true;
          while (notdone) {
            stemp_im = absxk * 2.0041683600089728E-292;
            cto1 = ctoc / 4.9896007738368E+291;
            if ((stemp_im > ctoc) && (ctoc != 0.0)) {
              a = 2.0041683600089728E-292;
              absxk = stemp_im;
            } else if (cto1 > absxk) {
              a = 4.9896007738368E+291;
              ctoc = cto1;
            } else {
              a = ctoc / absxk;
              notdone = false;
            }
            for (i = 0; i < j; i++) {
              At_data[i].re *= a;
              At_data[i].im *= a;
            }
          }
        }
        internal::reflapack::xzggbal(At_data, At_size, &ilo, &ihi, rscale_data,
                                     &j);
        n = At_size[0];
        if ((At_size[0] > 1) && (ihi >= ilo + 2)) {
          for (int jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
            int jcolp1;
            jcolp1 = jcol + 2;
            for (int jrow = ihi - 1; jrow + 1 > jcol + 2; jrow--) {
              internal::reflapack::xzlartg(
                  At_data[(jrow + At_size[0] * jcol) - 1],
                  At_data[jrow + At_size[0] * jcol], &absxk, &s,
                  &At_data[(jrow + At_size[0] * jcol) - 1]);
              i = jrow + At_size[0] * jcol;
              At_data[i].re = 0.0;
              At_data[i].im = 0.0;
              for (j = jcolp1; j <= n; j++) {
                k = jrow + At_size[0] * (j - 1);
                ctoc = absxk * At_data[k - 1].re +
                       (s.re * At_data[k].re - s.im * At_data[k].im);
                stemp_im = absxk * At_data[k - 1].im +
                           (s.re * At_data[k].im + s.im * At_data[k].re);
                cto1 = At_data[k - 1].re;
                At_data[k].re =
                    absxk * At_data[k].re -
                    (s.re * At_data[k - 1].re + s.im * At_data[k - 1].im);
                At_data[k].im = absxk * At_data[k].im -
                                (s.re * At_data[k - 1].im - s.im * cto1);
                At_data[k - 1].re = ctoc;
                At_data[k - 1].im = stemp_im;
              }
              s.re = -s.re;
              s.im = -s.im;
              if ((1 <= ihi) && (ihi > 2147483646)) {
                check_forloop_overflow_error();
              }
              for (b_i = 1; b_i <= ihi; b_i++) {
                k = (b_i + At_size[0] * (jrow - 1)) - 1;
                j = (b_i + At_size[0] * jrow) - 1;
                ctoc = absxk * At_data[j].re +
                       (s.re * At_data[k].re - s.im * At_data[k].im);
                stemp_im = absxk * At_data[j].im +
                           (s.re * At_data[k].im + s.im * At_data[k].re);
                cto1 = At_data[j].re;
                At_data[k].re = absxk * At_data[k].re -
                                (s.re * At_data[j].re + s.im * At_data[j].im);
                At_data[k].im = absxk * At_data[k].im -
                                (s.re * At_data[j].im - s.im * cto1);
                At_data[j].re = ctoc;
                At_data[j].im = stemp_im;
              }
            }
          }
        }
        internal::reflapack::xzhgeqz(At_data, At_size, ilo, ihi, &j, V_data,
                                     V_size, beta1_data, &b_i);
        if ((j == 0) && ilascl) {
          notdone = true;
          while (notdone) {
            stemp_im = anrmto * 2.0041683600089728E-292;
            cto1 = anrm / 4.9896007738368E+291;
            if ((stemp_im > anrm) && (anrm != 0.0)) {
              a = 2.0041683600089728E-292;
              anrmto = stemp_im;
            } else if (cto1 > anrmto) {
              a = 4.9896007738368E+291;
              anrm = cto1;
            } else {
              a = anrm / anrmto;
              notdone = false;
            }
            for (i = 0; i < *V_size; i++) {
              V_data[i].re *= a;
              V_data[i].im *= a;
            }
          }
        }
      }
      if (*V_size <= b_i) {
        z_size_idx_0 = static_cast<signed char>(*V_size);
      } else {
        z_size_idx_0 = static_cast<signed char>(b_i);
      }
      ilascl = true;
      notdone = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 2)) {
        if (k + 1 <= 1) {
          i = z_size_idx_0;
          j = *V_size;
        } else {
          i = 1;
          j = 1;
        }
        if (i != j) {
          notdone = false;
          exitg2 = true;
        } else {
          k++;
        }
      }
      if (notdone) {
        notdone = true;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k < 2)) {
          if (k + 1 <= 1) {
            i = z_size_idx_0;
            j = b_i;
          } else {
            i = 1;
            j = 1;
          }
          if (i != j) {
            notdone = false;
            exitg2 = true;
          } else {
            k++;
          }
        }
        if (!notdone) {
          ilascl = false;
        }
      } else {
        ilascl = false;
      }
      if (!ilascl) {
        m_rtErrorWithMessageID(p_emlrtRTEI.fName, p_emlrtRTEI.lineNo);
      }
      if (*V_size <= b_i) {
        z_size_idx_0 = static_cast<signed char>(*V_size);
      } else {
        z_size_idx_0 = static_cast<signed char>(b_i);
      }
      ilascl = true;
      notdone = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 2)) {
        if (k + 1 <= 1) {
          i = z_size_idx_0;
          j = *V_size;
        } else {
          i = 1;
          j = 1;
        }
        if (i != j) {
          notdone = false;
          exitg2 = true;
        } else {
          k++;
        }
      }
      if (notdone) {
        notdone = true;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k < 2)) {
          if (k + 1 <= 1) {
            i = z_size_idx_0;
            j = b_i;
          } else {
            i = 1;
            j = 1;
          }
          if (i != j) {
            notdone = false;
            exitg2 = true;
          } else {
            k++;
          }
        }
        if (!notdone) {
          ilascl = false;
        }
      } else {
        ilascl = false;
      }
      if (!ilascl) {
        m_rtErrorWithMessageID(p_emlrtRTEI.fName, p_emlrtRTEI.lineNo);
      }
      for (i = 0; i < *V_size; i++) {
        double ai;
        anrm = V_data[i].re;
        ai = V_data[i].im;
        ctoc = beta1_data[i].re;
        cto1 = beta1_data[i].im;
        if (cto1 == 0.0) {
          if (ai == 0.0) {
            anrmto = anrm / ctoc;
            absxk = 0.0;
          } else if (anrm == 0.0) {
            anrmto = 0.0;
            absxk = ai / ctoc;
          } else {
            anrmto = anrm / ctoc;
            absxk = ai / ctoc;
          }
        } else if (ctoc == 0.0) {
          if (anrm == 0.0) {
            anrmto = ai / cto1;
            absxk = 0.0;
          } else if (ai == 0.0) {
            anrmto = 0.0;
            absxk = -(anrm / cto1);
          } else {
            anrmto = ai / cto1;
            absxk = -(anrm / cto1);
          }
        } else {
          a = std::abs(ctoc);
          absxk = std::abs(cto1);
          if (a > absxk) {
            stemp_im = cto1 / ctoc;
            absxk = ctoc + stemp_im * cto1;
            anrmto = (anrm + stemp_im * ai) / absxk;
            absxk = (ai - stemp_im * anrm) / absxk;
          } else if (absxk == a) {
            if (ctoc > 0.0) {
              stemp_im = 0.5;
            } else {
              stemp_im = -0.5;
            }
            if (cto1 > 0.0) {
              absxk = 0.5;
            } else {
              absxk = -0.5;
            }
            anrmto = (anrm * stemp_im + ai * absxk) / a;
            absxk = (ai * stemp_im - anrm * absxk) / a;
          } else {
            stemp_im = ctoc / cto1;
            absxk = cto1 + stemp_im * ctoc;
            anrmto = (stemp_im * anrm + ai) / absxk;
            absxk = (stemp_im * ai - anrm) / absxk;
          }
        }
        V_data[i].re = anrmto;
        V_data[i].im = absxk;
      }
    }
  }
}

} // namespace coder

// End of code generation (eig.cpp)

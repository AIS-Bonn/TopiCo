//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzhgeqz.cpp
//
// Code generation for function 'xzhgeqz'
//

// Include files
#include "xzhgeqz.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "xzlanhs.h"
#include "xzlartg.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzhgeqz(const creal_T A_data[], const int A_size[2], int ilo, int ihi,
             int *info, creal_T alpha1_data[], int *alpha1_size,
             creal_T beta1_data[], int *beta1_size)
{
  creal_T b_A_data[36];
  creal_T b_ascale;
  creal_T ctemp;
  creal_T shift;
  double anorm;
  double ascale;
  double b_atol;
  double bscale;
  double eshift_im;
  double eshift_re;
  double temp;
  int A_size_idx_0;
  int ctemp_tmp;
  int ilast;
  int j;
  int jm1;
  int n;
  bool failed;
  bool guard1 = false;
  bool guard2 = false;
  A_size_idx_0 = A_size[0];
  jm1 = A_size[0] * A_size[1];
  if (0 <= jm1 - 1) {
    std::copy(&A_data[0], &A_data[jm1], &b_A_data[0]);
  }
  *info = 0;
  if ((A_size[0] == 1) && (A_size[1] == 1)) {
    ihi = 1;
  }
  n = A_size[0];
  *alpha1_size = A_size[0];
  jm1 = A_size[0];
  if (0 <= jm1 - 1) {
    std::memset(&alpha1_data[0], 0, jm1 * sizeof(creal_T));
  }
  *beta1_size = A_size[0];
  jm1 = A_size[0];
  for (ctemp_tmp = 0; ctemp_tmp < jm1; ctemp_tmp++) {
    beta1_data[ctemp_tmp].re = 1.0;
    beta1_data[ctemp_tmp].im = 0.0;
  }
  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = xzlanhs(A_data, A_size, ilo, ihi);
  temp = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (temp > 2.2250738585072014E-308) {
    b_atol = temp;
  }
  temp = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    temp = anorm;
  }
  ascale = 1.0 / temp;
  bscale = 1.0 / std::sqrt(static_cast<double>(A_size[0]));
  failed = true;
  jm1 = ihi + 1;
  for (j = jm1; j <= n; j++) {
    alpha1_data[j - 1] = A_data[(j + A_size[0] * (j - 1)) - 1];
  }
  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    int ifirst;
    int iiter;
    int ilastm;
    int ilastm1;
    int istart;
    int jiter;
    int maxit;
    bool goto60;
    bool goto70;
    bool goto90;
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    ilastm = ihi;
    iiter = 0;
    maxit = 30 * ((ihi - ilo) + 1);
    goto60 = false;
    goto70 = false;
    goto90 = false;
    if ((1 <= maxit) && (maxit > 2147483646)) {
      check_forloop_overflow_error();
    }
    jiter = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (jiter <= maxit - 1) {
        bool b_guard1 = false;
        bool exitg2;
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else {
          ctemp_tmp = ilast + A_size_idx_0 * ilastm1;
          if (std::abs(b_A_data[ctemp_tmp].re) +
                  std::abs(b_A_data[ctemp_tmp].im) <=
              b_atol) {
            b_A_data[ctemp_tmp].re = 0.0;
            b_A_data[ctemp_tmp].im = 0.0;
            goto60 = true;
            b_guard1 = true;
          } else {
            bool guard3 = false;
            j = ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (j + 1 >= ilo)) {
              if (j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                ctemp_tmp = j + A_size_idx_0 * (j - 1);
                if (std::abs(b_A_data[ctemp_tmp].re) +
                        std::abs(b_A_data[ctemp_tmp].im) <=
                    b_atol) {
                  b_A_data[ctemp_tmp].re = 0.0;
                  b_A_data[ctemp_tmp].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  j--;
                  guard3 = false;
                }
              }
            }
            if (guard3) {
              ifirst = j + 1;
              goto70 = true;
            }
            if (goto70) {
              b_guard1 = true;
            } else {
              for (ctemp_tmp = 0; ctemp_tmp < *alpha1_size; ctemp_tmp++) {
                alpha1_data[ctemp_tmp].re = rtNaN;
                alpha1_data[ctemp_tmp].im = 0.0;
              }
              for (ctemp_tmp = 0; ctemp_tmp < *beta1_size; ctemp_tmp++) {
                beta1_data[ctemp_tmp].re = rtNaN;
                beta1_data[ctemp_tmp].im = 0.0;
              }
              *info = 1;
              exitg1 = 1;
            }
          }
        }
        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1_data[ilast] = b_A_data[ilast + A_size_idx_0 * ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              failed = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              ilastm = ilast + 1;
              jiter++;
            }
          } else {
            if (goto70) {
              double ad22_im;
              double ad22_re;
              double tempr;
              goto70 = false;
              iiter++;
              if (iiter - iiter / 10 * 10 != 0) {
                double ascale_im;
                double ascale_re;
                double b_ascale_re;
                double t1_im;
                double t1_im_tmp;
                double t1_re;
                jm1 = ilastm1 + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                temp = ascale * b_A_data[jm1].im;
                if (temp == 0.0) {
                  shift.re = anorm / bscale;
                  shift.im = 0.0;
                } else if (anorm == 0.0) {
                  shift.re = 0.0;
                  shift.im = temp / bscale;
                } else {
                  shift.re = anorm / bscale;
                  shift.im = temp / bscale;
                }
                jm1 = ilast + A_size_idx_0 * ilast;
                anorm = ascale * b_A_data[jm1].re;
                temp = ascale * b_A_data[jm1].im;
                if (temp == 0.0) {
                  ad22_re = anorm / bscale;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = temp / bscale;
                } else {
                  ad22_re = anorm / bscale;
                  ad22_im = temp / bscale;
                }
                t1_re = 0.5 * (shift.re + ad22_re);
                t1_im = 0.5 * (shift.im + ad22_im);
                t1_im_tmp = t1_re * t1_im;
                jm1 = ilastm1 + A_size_idx_0 * ilast;
                anorm = ascale * b_A_data[jm1].re;
                temp = ascale * b_A_data[jm1].im;
                if (temp == 0.0) {
                  ascale_re = anorm / bscale;
                  ascale_im = 0.0;
                } else if (anorm == 0.0) {
                  ascale_re = 0.0;
                  ascale_im = temp / bscale;
                } else {
                  ascale_re = anorm / bscale;
                  ascale_im = temp / bscale;
                }
                jm1 = ilast + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                temp = ascale * b_A_data[jm1].im;
                if (temp == 0.0) {
                  b_ascale_re = anorm / bscale;
                  anorm = 0.0;
                } else if (anorm == 0.0) {
                  b_ascale_re = 0.0;
                  anorm = temp / bscale;
                } else {
                  b_ascale_re = anorm / bscale;
                  anorm = temp / bscale;
                }
                temp = shift.re * ad22_re - shift.im * ad22_im;
                tempr = shift.re * ad22_im + shift.im * ad22_re;
                shift.re = ((t1_re * t1_re - t1_im * t1_im) +
                            (ascale_re * b_ascale_re - ascale_im * anorm)) -
                           temp;
                shift.im = ((t1_im_tmp + t1_im_tmp) +
                            (ascale_re * anorm + ascale_im * b_ascale_re)) -
                           tempr;
                scalar::b_sqrt(&shift);
                if ((t1_re - ad22_re) * shift.re +
                        (t1_im - ad22_im) * shift.im <=
                    0.0) {
                  shift.re += t1_re;
                  shift.im += t1_im;
                } else {
                  shift.re = t1_re - shift.re;
                  shift.im = t1_im - shift.im;
                }
              } else {
                double ascale_im;
                double ascale_re;
                jm1 = ilast + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                temp = ascale * b_A_data[jm1].im;
                if (temp == 0.0) {
                  ascale_re = anorm / bscale;
                  ascale_im = 0.0;
                } else if (anorm == 0.0) {
                  ascale_re = 0.0;
                  ascale_im = temp / bscale;
                } else {
                  ascale_re = anorm / bscale;
                  ascale_im = temp / bscale;
                }
                eshift_re += ascale_re;
                eshift_im += ascale_im;
                shift.re = eshift_re;
                shift.im = eshift_im;
              }
              j = ilastm1;
              n = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                jm1 = A_size_idx_0 * j;
                ctemp_tmp = j + jm1;
                ctemp.re = ascale * b_A_data[ctemp_tmp].re - shift.re * bscale;
                ctemp.im = ascale * b_A_data[ctemp_tmp].im - shift.im * bscale;
                temp = std::abs(ctemp.re) + std::abs(ctemp.im);
                jm1 += n;
                anorm = ascale * (std::abs(b_A_data[jm1].re) +
                                  std::abs(b_A_data[jm1].im));
                tempr = temp;
                if (anorm > temp) {
                  tempr = anorm;
                }
                if ((tempr < 1.0) && (tempr != 0.0)) {
                  temp /= tempr;
                  anorm /= tempr;
                }
                ctemp_tmp = j + A_size_idx_0 * (j - 1);
                if ((std::abs(b_A_data[ctemp_tmp].re) +
                     std::abs(b_A_data[ctemp_tmp].im)) *
                        anorm <=
                    temp * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  n = j;
                  j--;
                }
              }
              if (!goto90) {
                istart = ifirst;
                ctemp_tmp = (ifirst + A_size_idx_0 * (ifirst - 1)) - 1;
                ctemp.re = ascale * b_A_data[ctemp_tmp].re - shift.re * bscale;
                ctemp.im = ascale * b_A_data[ctemp_tmp].im - shift.im * bscale;
              }
              goto90 = false;
              jm1 = istart + A_size_idx_0 * (istart - 1);
              b_ascale.re = ascale * b_A_data[jm1].re;
              b_ascale.im = ascale * b_A_data[jm1].im;
              xzlartg(ctemp, b_ascale, &anorm, &shift);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  xzlartg(b_A_data[(j + A_size_idx_0 * jm1) - 1],
                          b_A_data[j + A_size_idx_0 * jm1], &anorm, &shift,
                          &b_A_data[(j + A_size_idx_0 * jm1) - 1]);
                  ctemp_tmp = j + A_size_idx_0 * jm1;
                  b_A_data[ctemp_tmp].re = 0.0;
                  b_A_data[ctemp_tmp].im = 0.0;
                }
                if ((j <= ilastm) && (ilastm > 2147483646)) {
                  check_forloop_overflow_error();
                }
                for (n = j; n <= ilastm; n++) {
                  ctemp_tmp = j + A_size_idx_0 * (n - 1);
                  ad22_re = anorm * b_A_data[ctemp_tmp - 1].re +
                            (shift.re * b_A_data[ctemp_tmp].re -
                             shift.im * b_A_data[ctemp_tmp].im);
                  ad22_im = anorm * b_A_data[ctemp_tmp - 1].im +
                            (shift.re * b_A_data[ctemp_tmp].im +
                             shift.im * b_A_data[ctemp_tmp].re);
                  temp = b_A_data[ctemp_tmp - 1].re;
                  b_A_data[ctemp_tmp].re =
                      anorm * b_A_data[ctemp_tmp].re -
                      (shift.re * b_A_data[ctemp_tmp - 1].re +
                       shift.im * b_A_data[ctemp_tmp - 1].im);
                  b_A_data[ctemp_tmp].im =
                      anorm * b_A_data[ctemp_tmp].im -
                      (shift.re * b_A_data[ctemp_tmp - 1].im - shift.im * temp);
                  b_A_data[ctemp_tmp - 1].re = ad22_re;
                  b_A_data[ctemp_tmp - 1].im = ad22_im;
                }
                shift.re = -shift.re;
                shift.im = -shift.im;
                n = j + 2;
                if (ilast + 1 < j + 2) {
                  n = ilast + 1;
                }
                if ((ifirst <= n) && (n > 2147483646)) {
                  check_forloop_overflow_error();
                }
                for (int i = ifirst; i <= n; i++) {
                  ctemp_tmp = (i + A_size_idx_0 * (j - 1)) - 1;
                  jm1 = (i + A_size_idx_0 * j) - 1;
                  ad22_re = anorm * b_A_data[jm1].re +
                            (shift.re * b_A_data[ctemp_tmp].re -
                             shift.im * b_A_data[ctemp_tmp].im);
                  ad22_im = anorm * b_A_data[jm1].im +
                            (shift.re * b_A_data[ctemp_tmp].im +
                             shift.im * b_A_data[ctemp_tmp].re);
                  temp = b_A_data[jm1].re;
                  b_A_data[ctemp_tmp].re = anorm * b_A_data[ctemp_tmp].re -
                                           (shift.re * b_A_data[jm1].re +
                                            shift.im * b_A_data[jm1].im);
                  b_A_data[ctemp_tmp].im =
                      anorm * b_A_data[ctemp_tmp].im -
                      (shift.re * b_A_data[jm1].im - shift.im * temp);
                  b_A_data[jm1].re = ad22_re;
                  b_A_data[jm1].im = ad22_im;
                }
                jm1 = j - 1;
                j++;
              }
            }
            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }
  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      if ((1 <= ilast + 1) && (ilast + 1 > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (jm1 = 0; jm1 <= ilast; jm1++) {
        alpha1_data[jm1].re = rtNaN;
        alpha1_data[jm1].im = 0.0;
        beta1_data[jm1].re = rtNaN;
        beta1_data[jm1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1) {
    if ((1 <= ilo - 1) && (ilo - 1 > 2147483646)) {
      check_forloop_overflow_error();
    }
    for (j = 0; j <= ilo - 2; j++) {
      alpha1_data[j] = b_A_data[j + A_size_idx_0 * j];
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzhgeqz.cpp)

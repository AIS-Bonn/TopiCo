//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzggbal.cpp
//
// Code generation for function 'xzggbal'
//

// Include files
#include "xzggbal.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzggbal(creal_T A_data[], const int A_size[2], int *ilo, int *ihi,
             int rscale_data[], int *rscale_size)
{
  int i;
  int ii;
  *rscale_size = A_size[0];
  ii = A_size[0];
  for (i = 0; i < ii; i++) {
    rscale_data[i] = 1;
  }
  *ilo = 1;
  *ihi = A_size[0];
  if (A_size[0] <= 1) {
    *ihi = 1;
  } else {
    double atmp_im;
    double atmp_re;
    int b_i;
    int exitg2;
    int j;
    int jj;
    int k;
    int nzcount;
    bool exitg3;
    bool exitg4;
    bool found;
    do {
      exitg2 = 0;
      b_i = 0;
      j = 0;
      found = false;
      ii = *ihi;
      exitg3 = false;
      while ((!exitg3) && (ii > 0)) {
        nzcount = 0;
        b_i = ii;
        j = *ihi;
        if ((1 <= *ihi) && (*ihi > 2147483646)) {
          check_forloop_overflow_error();
        }
        jj = 0;
        exitg4 = false;
        while ((!exitg4) && (jj <= *ihi - 1)) {
          i = (ii + A_size[0] * jj) - 1;
          if ((A_data[i].re != 0.0) || (A_data[i].im != 0.0) ||
              (ii == jj + 1)) {
            if (nzcount == 0) {
              j = jj + 1;
              nzcount = 1;
              jj++;
            } else {
              nzcount = 2;
              exitg4 = true;
            }
          } else {
            jj++;
          }
        }
        if (nzcount < 2) {
          found = true;
          exitg3 = true;
        } else {
          ii--;
        }
      }
      if (!found) {
        exitg2 = 2;
      } else {
        nzcount = A_size[0];
        if (b_i != *ihi) {
          for (k = 1; k <= nzcount; k++) {
            ii = A_size[0] * (k - 1);
            jj = (b_i + ii) - 1;
            atmp_re = A_data[jj].re;
            atmp_im = A_data[jj].im;
            i = (*ihi + ii) - 1;
            A_data[jj] = A_data[i];
            A_data[i].re = atmp_re;
            A_data[i].im = atmp_im;
          }
        }
        if (j != *ihi) {
          if ((1 <= *ihi) && (*ihi > 2147483646)) {
            check_forloop_overflow_error();
          }
          for (k = 0; k < *ihi; k++) {
            jj = k + A_size[0] * (j - 1);
            atmp_re = A_data[jj].re;
            atmp_im = A_data[jj].im;
            i = k + A_size[0] * (*ihi - 1);
            A_data[jj] = A_data[i];
            A_data[i].re = atmp_re;
            A_data[i].im = atmp_im;
          }
        }
        rscale_data[*ihi - 1] = j;
        (*ihi)--;
        if (*ihi == 1) {
          rscale_data[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);
    if (exitg2 != 1) {
      int exitg1;
      do {
        exitg1 = 0;
        b_i = 0;
        j = 0;
        found = false;
        if ((*ilo <= *ihi) && (*ihi > 2147483646)) {
          check_forloop_overflow_error();
        }
        jj = *ilo;
        exitg3 = false;
        while ((!exitg3) && (jj <= *ihi)) {
          nzcount = 0;
          b_i = *ihi;
          j = jj;
          ii = *ilo;
          exitg4 = false;
          while ((!exitg4) && (ii <= *ihi)) {
            i = (ii + A_size[0] * (jj - 1)) - 1;
            if ((A_data[i].re != 0.0) || (A_data[i].im != 0.0) || (ii == jj)) {
              if (nzcount == 0) {
                b_i = ii;
                nzcount = 1;
                ii++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              ii++;
            }
          }
          if (nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            jj++;
          }
        }
        if (!found) {
          exitg1 = 1;
        } else {
          nzcount = A_size[0];
          if (b_i != *ilo) {
            for (k = *ilo; k <= nzcount; k++) {
              ii = A_size[0] * (k - 1);
              jj = (b_i + ii) - 1;
              atmp_re = A_data[jj].re;
              atmp_im = A_data[jj].im;
              i = (*ilo + ii) - 1;
              A_data[jj] = A_data[i];
              A_data[i].re = atmp_re;
              A_data[i].im = atmp_im;
            }
          }
          if (j != *ilo) {
            for (k = 0; k < *ihi; k++) {
              jj = k + A_size[0] * (j - 1);
              atmp_re = A_data[jj].re;
              atmp_im = A_data[jj].im;
              i = k + A_size[0] * (*ilo - 1);
              A_data[jj] = A_data[i];
              A_data[i].re = atmp_re;
              A_data[i].im = atmp_im;
            }
          }
          rscale_data[*ilo - 1] = j;
          (*ilo)++;
          if (*ilo == *ihi) {
            rscale_data[*ilo - 1] = *ilo;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzggbal.cpp)

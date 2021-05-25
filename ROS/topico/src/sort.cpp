//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sort.cpp
//
// Code generation for function 'sort'
//

// Include files
#include "sort.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
void b_sort(double x_data[], const int *x_size, int idx_data[], int *idx_size)
{
  double vwork_data[100];
  int iidx_data[100];
  int dim;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  *idx_size = *x_size;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  if ((1 <= vstride) && (vstride > 2147483646)) {
    check_forloop_overflow_error();
  }
  for (int j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, iidx_data, &dim);
    for (k = 0; k <= vlen; k++) {
      dim = j + k * vstride;
      x_data[dim] = vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
}

void sort(double x_data[], const int *x_size, int idx_data[], int *idx_size)
{
  double b_vwork_data[100];
  double vwork_data[100];
  double xwork_data[100];
  double x4[4];
  int b_iwork_data[100];
  int iidx_data[100];
  int iwork_data[100];
  int dim;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  signed char idx4[4];
  signed char perm[4];
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  *idx_size = *x_size;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  if ((1 <= vstride) && (vstride > 2147483646)) {
    check_forloop_overflow_error();
  }
  for (int j = 0; j < vstride; j++) {
    int loop_ub_tmp;
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    if (0 <= vwork_size - 1) {
      std::copy(&vwork_data[0], &vwork_data[vwork_size], &b_vwork_data[0]);
    }
    loop_ub_tmp = static_cast<signed char>(vwork_size);
    if (0 <= loop_ub_tmp - 1) {
      std::memset(&iidx_data[0], 0, loop_ub_tmp * sizeof(int));
    }
    if (vwork_size != 0) {
      int i1;
      int i2;
      int i3;
      int i4;
      int nNaNs;
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      if (0 <= loop_ub_tmp - 1) {
        std::memset(&iwork_data[0], 0, loop_ub_tmp * sizeof(int));
      }
      if (0 <= vwork_size - 1) {
        std::memset(&xwork_data[0], 0, vwork_size * sizeof(double));
      }
      nNaNs = 0;
      dim = -1;
      for (k = 0; k < vwork_size; k++) {
        if (rtIsNaN(b_vwork_data[k])) {
          i3 = (vwork_size - nNaNs) - 1;
          iidx_data[i3] = k + 1;
          xwork_data[i3] = b_vwork_data[k];
          nNaNs++;
        } else {
          dim++;
          idx4[dim] = static_cast<signed char>(k + 1);
          x4[dim] = b_vwork_data[k];
          if (dim + 1 == 4) {
            double d;
            double d1;
            dim = k - nNaNs;
            if (x4[0] <= x4[1]) {
              i1 = 1;
              i2 = 2;
            } else {
              i1 = 2;
              i2 = 1;
            }
            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }
            d = x4[i1 - 1];
            d1 = x4[i3 - 1];
            if (d <= d1) {
              d = x4[i2 - 1];
              if (d <= d1) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i2);
                perm[2] = static_cast<signed char>(i3);
                perm[3] = static_cast<signed char>(i4);
              } else if (d <= x4[i4 - 1]) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(i2);
                perm[3] = static_cast<signed char>(i4);
              } else {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(i4);
                perm[3] = static_cast<signed char>(i2);
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[i2 - 1] <= d1) {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(i2);
                  perm[3] = static_cast<signed char>(i4);
                } else {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(i4);
                  perm[3] = static_cast<signed char>(i2);
                }
              } else {
                perm[0] = static_cast<signed char>(i3);
                perm[1] = static_cast<signed char>(i4);
                perm[2] = static_cast<signed char>(i1);
                perm[3] = static_cast<signed char>(i2);
              }
            }
            iidx_data[dim - 3] = idx4[perm[0] - 1];
            iidx_data[dim - 2] = idx4[perm[1] - 1];
            iidx_data[dim - 1] = idx4[perm[2] - 1];
            iidx_data[dim] = idx4[perm[3] - 1];
            b_vwork_data[dim - 3] = x4[perm[0] - 1];
            b_vwork_data[dim - 2] = x4[perm[1] - 1];
            b_vwork_data[dim - 1] = x4[perm[2] - 1];
            b_vwork_data[dim] = x4[perm[3] - 1];
            dim = -1;
          }
        }
      }
      i4 = vwork_size - nNaNs;
      if (dim + 1 > 0) {
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (dim + 1 == 1) {
          perm[0] = 1;
        } else if (dim + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        if (dim + 1 > 2147483646) {
          check_forloop_overflow_error();
        }
        for (k = 0; k <= dim; k++) {
          i3 = perm[k] - 1;
          i1 = ((i4 - dim) + k) - 1;
          iidx_data[i1] = idx4[i3];
          b_vwork_data[i1] = x4[i3];
        }
      }
      dim = (nNaNs >> 1) + 1;
      for (k = 0; k <= dim - 2; k++) {
        i1 = i4 + k;
        i2 = iidx_data[i1];
        i3 = (vwork_size - k) - 1;
        iidx_data[i1] = iidx_data[i3];
        iidx_data[i3] = i2;
        b_vwork_data[i1] = xwork_data[i3];
        b_vwork_data[i3] = xwork_data[i1];
      }
      if ((nNaNs & 1) != 0) {
        dim = (i4 + dim) - 1;
        b_vwork_data[dim] = xwork_data[dim];
      }
      if (i4 > 1) {
        i3 = i4 >> 2;
        i2 = 4;
        while (i3 > 1) {
          if ((i3 & 1) != 0) {
            i3--;
            dim = i2 * i3;
            i1 = i4 - dim;
            if (i1 > i2) {
              merge(iidx_data, b_vwork_data, dim, i2, i1 - i2, iwork_data,
                    xwork_data);
            }
          }
          dim = i2 << 1;
          i3 >>= 1;
          for (k = 0; k < i3; k++) {
            merge(iidx_data, b_vwork_data, k * dim, i2, i2, iwork_data,
                  xwork_data);
          }
          i2 = dim;
        }
        if (i4 > i2) {
          if (0 <= loop_ub_tmp - 1) {
            std::copy(&iwork_data[0], &iwork_data[loop_ub_tmp],
                      &b_iwork_data[0]);
          }
          if (0 <= vwork_size - 1) {
            std::copy(&xwork_data[0], &xwork_data[vwork_size], &vwork_data[0]);
          }
          merge(iidx_data, b_vwork_data, 0, i2, i4 - i2, b_iwork_data,
                vwork_data);
        }
      }
    }
    if (0 <= vwork_size - 1) {
      std::copy(&b_vwork_data[0], &b_vwork_data[vwork_size], &vwork_data[0]);
    }
    for (k = 0; k <= vlen; k++) {
      dim = j + k * vstride;
      x_data[dim] = b_vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (sort.cpp)

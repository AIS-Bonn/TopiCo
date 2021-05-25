//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// circshift.cpp
//
// Code generation for function 'circshift'
//

// Include files
#include "circshift.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
void circshift(double a_data[], const int a_size[2])
{
  double buffer_data[50];
  int dim;
  dim = (a_size[0] == 1);
  if (a_size[0] != 0) {
    int i;
    int k;
    int npages;
    int ns;
    int nv;
    int pagesize;
    int stride;
    int u0;
    bool shiftright;
    ns = 0;
    shiftright = false;
    if (1 > a_size[dim]) {
      d_rtErrorWithMessageID(f_emlrtRTEI.fName, f_emlrtRTEI.lineNo);
    }
    if (1 > (a_size[dim] >> 1)) {
      ns = a_size[dim] - 2;
      shiftright = true;
    }
    u0 = a_size[0];
    if (u0 <= 7) {
      u0 = 7;
    }
    if (a_size[0] == 0) {
      u0 = 0;
    }
    u0 = static_cast<signed char>(std::floor(static_cast<double>(u0) / 2.0));
    if (0 <= u0 - 1) {
      std::memset(&buffer_data[0], 0, u0 * sizeof(double));
    }
    i = a_size[dim] - 1;
    nv = a_size[dim];
    stride = 1;
    for (k = 0; k < dim; k++) {
      stride *= a_size[0];
    }
    npages = 1;
    u0 = dim + 2;
    for (k = u0; k < 3; k++) {
      npages *= 7;
    }
    pagesize = stride * a_size[dim];
    if ((a_size[dim] > 1) && (ns + 1 > 0)) {
      if ((1 <= npages) && (npages > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (int b_i = 0; b_i < npages; b_i++) {
        dim = b_i * pagesize;
        if ((1 <= stride) && (stride > 2147483646)) {
          check_forloop_overflow_error();
        }
        for (int j = 0; j < stride; j++) {
          int i1;
          i1 = dim + j;
          if (shiftright) {
            for (k = 0; k <= ns; k++) {
              buffer_data[k] = a_data[i1 + ((k + i) - ns) * stride];
            }
            u0 = ns + 2;
            for (k = nv; k >= u0; k--) {
              a_data[i1 + (k - 1) * stride] =
                  a_data[i1 + ((k - ns) - 2) * stride];
            }
            for (k = 0; k <= ns; k++) {
              a_data[i1 + k * stride] = buffer_data[k];
            }
          } else {
            for (k = 0; k <= ns; k++) {
              buffer_data[k] = a_data[i1 + k * stride];
            }
            u0 = (i - ns) - 1;
            for (k = 0; k <= u0; k++) {
              a_data[i1 + k * stride] = a_data[i1 + ((k + ns) + 1) * stride];
            }
            for (k = 0; k <= ns; k++) {
              a_data[i1 + ((k + i) - ns) * stride] = buffer_data[k];
            }
          }
        }
      }
    }
  }
}

void circshift(int a_data[], const int *a_size)
{
  int buffer_data[50];
  int dim;
  dim = 2;
  if (*a_size != 1) {
    dim = 1;
  }
  if ((*a_size != 0) && (*a_size != 1)) {
    int k;
    int loop_ub;
    int ns;
    int nv;
    int stride;
    bool shiftright;
    ns = 0;
    shiftright = false;
    if (dim <= 1) {
      loop_ub = *a_size;
    } else {
      loop_ub = 1;
    }
    if (1 > (loop_ub >> 1)) {
      ns = loop_ub - 2;
      shiftright = true;
    }
    loop_ub = static_cast<signed char>(
        std::floor(static_cast<double>(*a_size) / 2.0));
    if (0 <= loop_ub - 1) {
      std::memset(&buffer_data[0], 0, loop_ub * sizeof(int));
    }
    if (dim <= 1) {
      nv = *a_size;
    } else {
      nv = 1;
    }
    stride = 1;
    for (k = 0; k <= dim - 2; k++) {
      stride *= *a_size;
    }
    if ((nv > 1) && (ns + 1 > 0)) {
      if ((1 <= stride) && (stride > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (dim = 0; dim < stride; dim++) {
        if (shiftright) {
          for (k = 0; k <= ns; k++) {
            buffer_data[k] = a_data[dim + (((k + nv) - ns) - 1) * stride];
          }
          loop_ub = ns + 2;
          for (k = nv; k >= loop_ub; k--) {
            a_data[dim + (k - 1) * stride] =
                a_data[dim + ((k - ns) - 2) * stride];
          }
          for (k = 0; k <= ns; k++) {
            a_data[dim + k * stride] = buffer_data[k];
          }
        } else {
          for (k = 0; k <= ns; k++) {
            buffer_data[k] = a_data[dim + k * stride];
          }
          loop_ub = nv - ns;
          for (k = 0; k <= loop_ub - 2; k++) {
            a_data[dim + k * stride] = a_data[dim + ((k + ns) + 1) * stride];
          }
          for (k = 0; k <= ns; k++) {
            a_data[dim + (((k + nv) - ns) - 1) * stride] = buffer_data[k];
          }
        }
      }
    }
  }
}

} // namespace coder

// End of code generation (circshift.cpp)

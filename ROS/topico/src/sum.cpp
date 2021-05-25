//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sum.cpp
//
// Code generation for function 'sum'
//

// Include files
#include "sum.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>

// Function Definitions
namespace coder {
void b_sum(const double x_data[], const int x_size[2], double y_data[],
           int *y_size)
{
  if (x_size[0] == 0) {
    *y_size = 0;
  } else {
    int vstride;
    vstride = x_size[0];
    *y_size = static_cast<signed char>(x_size[0]);
    if (0 <= vstride - 1) {
      std::copy(&x_data[0], &x_data[vstride], &y_data[0]);
    }
    for (int k = 0; k < 10; k++) {
      int xoffset;
      xoffset = (k + 1) * vstride;
      for (int xj = 0; xj < vstride; xj++) {
        y_data[xj] += x_data[xoffset + xj];
      }
    }
  }
}

void sum(const double x_data[], const int x_size[2], double y_data[],
         int *y_size)
{
  if (x_size[0] == 0) {
    *y_size = 0;
  } else {
    int vstride;
    vstride = x_size[0];
    *y_size = static_cast<signed char>(x_size[0]);
    if (0 <= vstride - 1) {
      std::copy(&x_data[0], &x_data[vstride], &y_data[0]);
    }
    for (int k = 0; k < 6; k++) {
      int xoffset;
      xoffset = (k + 1) * vstride;
      for (int xj = 0; xj < vstride; xj++) {
        y_data[xj] += x_data[xoffset + xj];
      }
    }
  }
}

void sum(const int x_data[], int x_size, double y_data[], int *y_size)
{
  if (x_size == 0) {
    *y_size = 0;
  } else {
    *y_size = x_size;
    for (int xj = 0; xj < x_size; xj++) {
      y_data[xj] = x_data[xj];
    }
  }
}

void sum(const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y)
{
  array<double, 1U> bsum;
  if ((x.size(0) == 0) || (x.size(1) == 0)) {
    int firstBlockLength;
    y.set_size(x.size(0));
    firstBlockLength = x.size(0);
    for (int xblockoffset = 0; xblockoffset < firstBlockLength;
         xblockoffset++) {
      y[xblockoffset] = 0.0;
    }
  } else {
    int bvstride;
    int firstBlockLength;
    int k;
    int lastBlockLength;
    int nblocks;
    int vstride;
    int xj;
    int xoffset;
    vstride = x.size(0);
    bvstride = x.size(0) << 10;
    y.set_size(x.size(0));
    bsum.set_size(x.size(0));
    if (x.size(1) <= 1024) {
      firstBlockLength = x.size(1);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = x.size(1) / 1024;
      lastBlockLength = x.size(1) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    if (x.size(0) > 2147483646) {
      check_forloop_overflow_error();
    }
    for (xj = 0; xj < vstride; xj++) {
      y[xj] = x[xj];
      bsum[xj] = 0.0;
    }
    for (k = 2; k <= firstBlockLength; k++) {
      xoffset = (k - 1) * vstride;
      if (vstride > 2147483646) {
        check_forloop_overflow_error();
      }
      for (xj = 0; xj < vstride; xj++) {
        y[xj] = y[xj] + x[xoffset + xj];
      }
    }
    for (int ib = 2; ib <= nblocks; ib++) {
      int xblockoffset;
      xblockoffset = (ib - 1) * bvstride;
      if (vstride > 2147483646) {
        check_forloop_overflow_error();
      }
      for (xj = 0; xj < vstride; xj++) {
        bsum[xj] = x[xblockoffset + xj];
      }
      if (ib == nblocks) {
        firstBlockLength = lastBlockLength;
      } else {
        firstBlockLength = 1024;
      }
      if ((2 <= firstBlockLength) && (firstBlockLength > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (k = 2; k <= firstBlockLength; k++) {
        xoffset = xblockoffset + (k - 1) * vstride;
        for (xj = 0; xj < vstride; xj++) {
          bsum[xj] = bsum[xj] + x[xoffset + xj];
        }
      }
      for (xj = 0; xj < vstride; xj++) {
        y[xj] = y[xj] + bsum[xj];
      }
    }
  }
}

} // namespace coder

// End of code generation (sum.cpp)

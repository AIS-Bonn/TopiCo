//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// combineVectorElements.cpp
//
// Code generation for function 'combineVectorElements'
//

// Include files
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
double b_combineVectorElements(const ::coder::array<double, 2U> &x)
{
  double y;
  if (x.size(1) == 0) {
    y = 0.0;
  } else {
    int firstBlockLength;
    int k;
    int lastBlockLength;
    int nblocks;
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
    y = x[0];
    for (k = 2; k <= firstBlockLength; k++) {
      y += x[k - 1];
    }
    for (int ib = 2; ib <= nblocks; ib++) {
      double bsum;
      int hi;
      firstBlockLength = (ib - 1) << 10;
      bsum = x[firstBlockLength];
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      if ((2 <= hi) && (hi > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (k = 2; k <= hi; k++) {
        bsum += x[(firstBlockLength + k) - 1];
      }
      y += bsum;
    }
  }
  return y;
}

void c_combineVectorElements(const ::coder::array<bool, 2U> &x,
                             ::coder::array<int, 1U> &y)
{
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    int j;
    int vstride;
    vstride = x.size(0);
    y.set_size(x.size(0));
    if (x.size(0) > 2147483646) {
      check_forloop_overflow_error();
    }
    for (j = 0; j < vstride; j++) {
      y[j] = x[j];
    }
    for (int k = 0; k < 2; k++) {
      int xoffset;
      xoffset = (k + 1) * vstride;
      for (j = 0; j < vstride; j++) {
        y[j] = y[j] + x[xoffset + j];
      }
    }
  }
}

void combineVectorElements(const ::coder::array<bool, 2U> &x,
                           ::coder::array<int, 2U> &y)
{
  int vlen;
  vlen = x.size(0);
  if ((x.size(0) == 0) || (x.size(1) == 0)) {
    y.set_size(1, x.size(1));
    vlen = x.size(1);
    for (int npages = 0; npages < vlen; npages++) {
      y[npages] = 0;
    }
  } else {
    int npages;
    npages = x.size(1);
    y.set_size(1, x.size(1));
    if (x.size(1) > 2147483646) {
      check_forloop_overflow_error();
    }
    for (int i = 0; i < npages; i++) {
      int xpageoffset;
      xpageoffset = i * x.size(0);
      y[i] = x[xpageoffset];
      if ((2 <= vlen) && (vlen > 2147483646)) {
        check_forloop_overflow_error();
      }
      for (int k = 2; k <= vlen; k++) {
        y[i] = y[i] + x[(xpageoffset + k) - 1];
      }
    }
  }
}

} // namespace coder

// End of code generation (combineVectorElements.cpp)

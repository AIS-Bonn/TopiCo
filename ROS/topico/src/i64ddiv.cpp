//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// i64ddiv.cpp
//
// Code generation for function 'i64ddiv'
//

// Include files
#include "i64ddiv.h"
#include "rt_nonfinite.h"
#include <math.h>

// Function Definitions
namespace coder {
namespace internal {
unsigned long i64ddiv(unsigned long x)
{
  unsigned long z;
  int xexp;
  if (x == 0UL) {
    z = 0UL;
  } else {
    unsigned long n;
    frexp(10.0, &xexp);
    xexp = -49;
    z = x / 5629499534213120UL;
    n = x - x / 5629499534213120UL * 5629499534213120UL;
    int exitg1;
    do {
      exitg1 = 0;
      if (xexp < 0) {
        int shiftAmount;
        shiftAmount = -xexp;
        if (shiftAmount >= 11) {
          shiftAmount = 11;
        }
        if ((z >> (64 - shiftAmount)) > 0UL) {
          z = MAX_uint64_T;
          exitg1 = 1;
        } else {
          unsigned long t;
          z <<= shiftAmount;
          n <<= shiftAmount;
          xexp += shiftAmount;
          t = n / 5629499534213120UL;
          if (MAX_uint64_T - t <= z) {
            z = MAX_uint64_T;
            exitg1 = 1;
          } else {
            z += t;
            n -= n / 5629499534213120UL * 5629499534213120UL;
          }
        }
      } else {
        if ((n << 1) >= 5629499534213120UL) {
          z++;
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return z;
}

} // namespace internal
} // namespace coder

// End of code generation (i64ddiv.cpp)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mod.cpp
//
// Code generation for function 'mod'
//

// Include files
#include "mod.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double b_mod(double x)
{
  double r;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = std::fmod(x, 2.0);
    if (r == 0.0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 2.0;
    }
  }
  return r;
}

} // namespace coder

// End of code generation (mod.cpp)

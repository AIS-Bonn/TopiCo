//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// minOrMax.h
//
// Code generation for function 'minOrMax'
//

#ifndef MINORMAX_H
#define MINORMAX_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void maximum(const ::coder::array<double, 1U> &x, double *ex, int *idx);

double maximum(const ::coder::array<double, 1U> &x);

} // namespace internal
} // namespace coder

#endif
// End of code generation (minOrMax.h)

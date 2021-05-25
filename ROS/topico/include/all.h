//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// all.h
//
// Code generation for function 'all'
//

#ifndef ALL_H
#define ALL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
bool all(const bool x_data[], const int x_size[2]);

bool b_all(const ::coder::array<bool, 1U> &x);

} // namespace coder

#endif
// End of code generation (all.h)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compensate_global.h
//
// Code generation for function 'compensate_global'
//

#ifndef COMPENSATE_GLOBAL_H
#define COMPENSATE_GLOBAL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void compensate_global(coder::array<double, 2U> &State_in,
                       const coder::array<double, 1U> &A_global,
                       const coder::array<double, 2U> &A_max_in,
                       const coder::array<double, 2U> &A_min_in,
                       coder::array<double, 2U> &A_max_out,
                       coder::array<double, 2U> &A_min_out);

#endif
// End of code generation (compensate_global.h)

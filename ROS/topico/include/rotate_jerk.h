//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rotate_jerk.h
//
// Code generation for function 'rotate_jerk'
//

#ifndef ROTATE_JERK_H
#define ROTATE_JERK_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void rotate_jerk(double alpha, const coder::array<double, 2U> &t_1,
                 const coder::array<double, 2U> &J_1,
                 const coder::array<double, 2U> &t_2,
                 const coder::array<double, 2U> &J_2,
                 coder::array<double, 2U> &t_out_1,
                 coder::array<double, 2U> &J_out_1,
                 coder::array<double, 2U> &t_out_2,
                 coder::array<double, 2U> &J_out_2);

#endif
// End of code generation (rotate_jerk.h)

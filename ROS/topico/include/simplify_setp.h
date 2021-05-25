//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// simplify_setp.h
//
// Code generation for function 'simplify_setp'
//

#ifndef SIMPLIFY_SETP_H
#define SIMPLIFY_SETP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void simplify_setp(const coder::array<double, 2U> &t_in,
                   const coder::array<double, 2U> &J_in,
                   coder::array<double, 2U> &t_out,
                   coder::array<double, 2U> &J_out);

void simplify_setp(const double t_in[7], const double J_in[7],
                   double t_out_data[], int t_out_size[2], double J_out_data[],
                   int J_out_size[2]);

#endif
// End of code generation (simplify_setp.h)

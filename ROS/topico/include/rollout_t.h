//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rollout_t.h
//
// Code generation for function 'rollout_t'
//

#ifndef ROLLOUT_T_H
#define ROLLOUT_T_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

// Function Declarations
void rollout_t(const coder::array<double, 1U> &P_init,
               const coder::array<double, 1U> &V_init,
               const coder::array<double, 1U> &A_init,
               const coder::array<struct0_T, 2U> &J_setp_struct, double ts,
               coder::array<double, 2U> &P, coder::array<double, 2U> &V,
               coder::array<double, 2U> &A, coder::array<double, 2U> &J,
               coder::array<double, 2U> &t);

#endif
// End of code generation (rollout_t.h)

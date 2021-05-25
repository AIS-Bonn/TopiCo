//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// check_inputs.h
//
// Code generation for function 'check_inputs'
//

#ifndef CHECK_INPUTS_H
#define CHECK_INPUTS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void check_inputs(const coder::array<double, 2U> &State_start,
                  const coder::array<double, 3U> &Waypoints,
                  const coder::array<double, 2U> &V_max,
                  const coder::array<double, 2U> &V_min,
                  const coder::array<double, 2U> &A_max,
                  const coder::array<double, 2U> &A_min,
                  const coder::array<double, 2U> &J_max,
                  const coder::array<double, 2U> &J_min,
                  const coder::array<double, 1U> &A_global,
                  const coder::array<bool, 2U> &b_sync_V,
                  const coder::array<bool, 2U> &b_sync_A,
                  const coder::array<bool, 2U> &b_sync_J,
                  const coder::array<bool, 2U> &b_sync_W,
                  const coder::array<bool, 2U> &b_rotate,
                  const coder::array<bool, 2U> &b_hard_V_lim,
                  const coder::array<bool, 2U> &b_catch_up,
                  const coder::array<signed char, 2U> &direction,
                  double ts_rollout);

#endif
// End of code generation (check_inputs.h)

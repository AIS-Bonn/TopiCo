//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// topico_wrapper.h
//
// Code generation for function 'topico_wrapper'
//

#ifndef TOPICO_WRAPPER_H
#define TOPICO_WRAPPER_H

// Include files
#include "rtwtypes.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void
topico_wrapper(const coder::array<double, 2U> &State_start,
               const coder::array<double, 3U> &Waypoints,
               const coder::array<double, 2U> &V_max_in,
               const coder::array<double, 2U> &V_min_in,
               const coder::array<double, 2U> &A_max_in,
               const coder::array<double, 2U> &A_min_in,
               const coder::array<double, 2U> &J_max_in,
               const coder::array<double, 2U> &J_min_in,
               const coder::array<double, 1U> &A_global,
               const coder::array<bool, 2U> &b_sync_V_in,
               const coder::array<bool, 2U> &b_sync_A_in,
               const coder::array<bool, 2U> &b_sync_J_in,
               const coder::array<bool, 2U> &b_sync_W_in,
               const coder::array<bool, 2U> &b_rotate_in,
               const coder::array<bool, 2U> &b_hard_V_lim_in,
               const coder::array<bool, 2U> &b_catch_up_in,
               const coder::array<signed char, 2U> &direction_in,
               double ts_rollout, coder::array<struct0_T, 2U> &J_setp_struct,
               coder::array<int, 2U> &solution_out,
               coder::array<double, 2U> &T_waypoints,
               coder::array<double, 2U> &P, coder::array<double, 2U> &V,
               coder::array<double, 2U> &A, coder::array<double, 2U> &J,
               coder::array<double, 2U> &t);

#endif
// End of code generation (topico_wrapper.h)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// synchronize_trajectory.h
//
// Code generation for function 'synchronize_trajectory'
//

#ifndef SYNCHRONIZE_TRAJECTORY_H
#define SYNCHRONIZE_TRAJECTORY_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_0;

// Function Declarations
void synchronize_trajectory(
    coder::array<double, 1U> &P_init, coder::array<double, 1U> &V_init,
    coder::array<double, 1U> &A_init, coder::array<double, 1U> &P_wayp,
    coder::array<double, 1U> &V_wayp, coder::array<double, 1U> &A_wayp,
    const coder::array<double, 1U> &V_max,
    const coder::array<double, 1U> &V_min,
    const coder::array<double, 1U> &A_max, coder::array<double, 1U> &A_min,
    coder::array<double, 1U> &J_max, coder::array<double, 1U> &J_min,
    const coder::array<bool, 1U> &b_sync_V,
    const coder::array<bool, 1U> &b_sync_A,
    const coder::array<bool, 1U> &b_sync_J,
    const coder::array<bool, 1U> &b_sync_W,
    const coder::array<bool, 1U> &b_hard_V_lim,
    const coder::array<double, 1U> &T_catch_up,
    const coder::array<signed char, 1U> &direction,
    coder::array<cell_wrap_0, 1U> &t_out, coder::array<cell_wrap_0, 1U> &J_out,
    coder::array<int, 1U> &solution_out);

#endif
// End of code generation (synchronize_trajectory.h)

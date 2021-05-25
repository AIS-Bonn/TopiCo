//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// solve_T.h
//
// Code generation for function 'solve_T'
//

#ifndef SOLVE_T_H
#define SOLVE_T_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void solve_T(double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             double t_sync, bool b_sync_V, bool b_sync_A, bool b_sync_J,
             bool b_sync_W, signed char direction, double t_out_data[],
             int t_out_size[2], double J_out_data[], int J_out_size[2],
             int solution_out_data[], int *solution_out_size);

#endif
// End of code generation (solve_T.h)

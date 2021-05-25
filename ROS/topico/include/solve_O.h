//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// solve_O.h
//
// Code generation for function 'solve_O'
//

#ifndef SOLVE_O_H
#define SOLVE_O_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void b_solve_O(double P_init, double V_init, double A_init, double V_wayp,
               double V_max, double A_max, double A_min, double J_max,
               double J_min, double t_out_data[], int t_out_size[2],
               double J_out_data[], int J_out_size[2], int solution_out_data[],
               int *solution_out_size);

void c_solve_O(double P_init, double V_init, double A_init, double V_max,
               double V_min, double A_max, double A_min, double J_max,
               double J_min, double t_out_data[], int t_out_size[2],
               double J_out_data[], int J_out_size[2], int solution_out_data[],
               int *solution_out_size);

void solve_O(double P_init, double V_init, double A_init, double V_wayp,
             double V_min, double A_max, double A_min, double J_max,
             double J_min, double t_out_data[], int t_out_size[2],
             double J_out_data[], int J_out_size[2], int solution_out_data[],
             int *solution_out_size);

void solve_O(double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             double t_out_data[], int t_out_size[2], double J_out_data[],
             int J_out_size[2], int solution_out_data[],
             int *solution_out_size);

#endif
// End of code generation (solve_O.h)

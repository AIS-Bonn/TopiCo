//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// check.h
//
// Code generation for function 'check'
//

#ifndef CHECK_H
#define CHECK_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void b_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double V_init, double A_init, double V_wayp, double V_max,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2]);

void c_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2]);

void check(const creal_T t_data[], const int t_size[2], const double J[7],
           double V_init, double A_init, double V_wayp, double V_min,
           double A_max, double A_min, double J_max, double J_min,
           bool valid_data[], int valid_size[2]);

void d_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double V_init, double A_init, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2]);

void e_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             double t_sync, bool valid_data[], int valid_size[2]);

#endif
// End of code generation (check.h)

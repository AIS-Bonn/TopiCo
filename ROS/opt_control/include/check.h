/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * check.h
 *
 * Code generation for function 'check'
 *
 */

#ifndef CHECK_H
#define CHECK_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_check(const creal_T t_data[], const int t_size[2], const double J
                    [7], double P_init, double V_init, double A_init, double
                    P_wayp, double V_wayp, double A_wayp, double V_max, double
                    V_min, double A_max, double A_min, double J_max, double
                    J_min, bool valid_data[], int valid_size[2]);
extern void c_check(const creal_T t_data[], const int t_size[2], const double J
                    [7], double P_init, double V_init, double A_init, double
                    P_wayp, double V_wayp, double A_wayp, double V_max, double
                    V_min, double A_max, double A_min, double J_max, double
                    J_min, bool valid_data[], int valid_size[2]);
extern void check(const creal_T t_data[], const int t_size[2], const double J[7],
                  double P_init, double V_init, double A_init, double V_wayp,
                  double V_max, double V_min, double A_max, double A_min, double
                  J_max, double J_min, bool valid_data[], int valid_size[2]);

#endif

/* End of code generation (check.h) */

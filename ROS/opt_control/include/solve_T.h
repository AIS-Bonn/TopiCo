/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * solve_T.h
 *
 * Code generation for function 'solve_T'
 *
 */

#ifndef SOLVE_T_H
#define SOLVE_T_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void solve_T(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double V_max, double V_min,
                    double A_max, double A_min, double J_max, double J_min,
                    double t_sync, bool b_sync_V, bool b_sync_A, bool b_sync_J,
                    bool b_best_solution, bool b_hard_vel_limit, short
                    solution_in, double t[11], double J[11], short *solution_out);

#endif

/* End of code generation (solve_T.h) */
